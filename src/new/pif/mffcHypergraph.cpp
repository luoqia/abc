#include "mffcHypergraph.h"
#include "partNtkFuncs.h"
#include "yaig.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <map>
#include <numeric>
#include <queue>
#include <set>

namespace ymc
{
	// Task 18 Stage 2: opt-in, behavior-neutral scale profiler.
	// Gated by the PIF_HG_PROFILE environment variable (disabled by
	// default). It only reads clocks and counters; no selection state is
	// read or written, so assignments are byte-identical with and without
	// profiling. Rows are emitted with ylog to stderr.
	struct HgProfiler
	{
		bool on = false;
		bool initDone = false;
		int64_t t0 = 0;
		int64_t levelScanIts = 0;	   // conn-scan inner iterations (pins visited)
		int64_t levelSortIts = 0;	   // conn entries consumed by sort+dedupe
		int64_t levelRebuildEdges = 0; // edges projected during rebuild
		int64_t levelRebuildPins = 0;  // pins written during rebuild
		int64_t levelTouchedPins = 0;  // pins of edges incident to matched vertices
		int64_t levelPairs = 0;
		int64_t levelSingles = 0;
		int64_t levelScanNs = 0;
		int64_t levelSortNs = 0;
		int64_t levelRebuildNs = 0;
		int64_t nLevels = 0;
		int64_t tProjectNs = 0, tCoarsenNs = 0, tFmNs = 0, tRepairNs = 0;
		int64_t tSplitNs = 0, tBisectNs = 0;
		int64_t scanItsBisect = 0, rebuildPinsBisect = 0, touchedPinsBisect = 0;
		int64_t nBisections = 0;
		int64_t tBuildNs = 0;
		// global accumulators (resetBisect at recursion entry must not
		// destroy totals; accumulated at each bisect-row emission)
		int64_t gTotLevels = 0, gTotScanIts = 0, gTotRebuildPins = 0, gTotTouchedPins = 0;
		// FM-internal counters (per level; emitted in the fm row)
		int64_t fmGainOfCalls = 0, fmHeapPushes = 0, fmHeapPops = 0, fmPinIters = 0;

		int64_t nowNs() const
		{
			return std::chrono::steady_clock::now().time_since_epoch().count();
		}
		void maybeInit()
		{
			if (!initDone)
			{
				on = getenv("PIF_HG_PROFILE") != nullptr;
				if (on)
					t0 = nowNs();
				initDone = true;
			}
		}
		void resetLevel()
		{
			levelScanIts = levelSortIts = levelRebuildEdges = levelRebuildPins = 0;
			levelTouchedPins = levelPairs = levelSingles = 0;
			levelScanNs = levelSortNs = levelRebuildNs = 0;
		}
		void resetBisect()
		{
			nLevels = 0;
			tProjectNs = tCoarsenNs = tFmNs = tRepairNs = tSplitNs = tBisectNs = 0;
			scanItsBisect = rebuildPinsBisect = touchedPinsBisect = 0;
		}
		int64_t hwmKb()
		{
			FILE *f = fopen("/proc/self/status", "r");
			if (!f)
				return -1;
			char line[256];
			int64_t hwm = -1;
			while (fgets(line, sizeof(line), f))
				if (sscanf(line, "VmHWM: %lld kB", &hwm) == 1)
					break;
			fclose(f);
			return hwm;
		}
	};
	static HgProfiler gProf;

	namespace
	{
		// Yaig reserves node 0 = NONE, node 1 = CONST (see yaig.h).
		const int32_t ID_OFFSET = 2;

		// FNV-1a 64: deterministic membership hash over sorted node ids.
		uint64_t fnv1a64(const std::vector<int32_t> &ids)
		{
			uint64_t h = 14695981039346656037ULL;
			for (int32_t id : ids)
			{
				for (int b = 0; b < 4; b++)
				{
					h ^= (uint64_t)((uint32_t)id >> (8 * b)) & 0xff;
					h *= 1099511628211ULL;
				}
			}
			return h;
		}
	} // namespace

	void MffcHypergraph::clear()
	{
		m_units.clear();
		m_edges.clear();
		m_nPinsTotal = 0;
		m_nInternalEdges = 0;
		m_nPiEdges = 0;
		m_nPoEdges = 0;
		m_nConstEdges = 0;
		m_nSingletonEdges = 0;
		m_nCutNets = 0;
		m_nConnectivityCost = 0;
		m_nCrossPartitionPins = 0;
		m_nPredictedInterfaces = 0;
		m_partCapacity.clear();
	}

	void MffcHypergraph::build(const std::vector<Node> &nodes,
							   const std::vector<int32_t> &node2unit,
							   const std::vector<MffcUnit> &units,
							   const std::vector<std::vector<int32_t>> &origins,
							   const std::vector<bool> &splitFlags)
	{
		gProf.maybeInit();
		int64_t tBuild0 = gProf.nowNs();
		clear();

		// ---- vertex table: post-preprocessing PartitionUnits ----
		m_units.reserve(units.size());
		for (size_t ui = 0; ui < units.size(); ui++)
		{
			const MffcUnit &m = units[ui];
			PartitionUnitRec rec;
			rec.unitId = m.iId;
			rec.rootNodeId = m.iRootId;
			rec.nNodes = m.nNodes;
			rec.levelMin = INT32_MAX;
			rec.levelMax = 0;
			rec.nOrigins = 0;
			rec.splitFragment = false;
			rec.partitionId = -1;
			rec.nodeIds = m.vNodeIds;
			std::sort(rec.nodeIds.begin(), rec.nodeIds.end());
			for (int32_t nid : rec.nodeIds)
			{
				if (nid >= 0 && nid < (int32_t)nodes.size())
				{
					int32_t lv = nodes[nid].iLevel;
					if (lv < rec.levelMin)
						rec.levelMin = lv;
					if (lv > rec.levelMax)
						rec.levelMax = lv;
				}
			}
			if (rec.levelMin == INT32_MAX)
				rec.levelMin = 0;
			if (ui < origins.size())
				rec.nOrigins = (int32_t)origins[ui].size();
			if (ui < splitFlags.size())
				rec.splitFragment = splitFlags[ui];
			rec.membershipHash = fnv1a64(rec.nodeIds);
			m_units.push_back(rec);
		}

		// ---- hyperedges ----
		// consumers[sig] = every distinct unit that consumes the signal
		// (from fanin references of AND nodes). producer[sig] = the unit
		// that produces the signal (AND nodes only). PO-driven signals are
		// recorded so that a signal consumed only by POs still has an edge.
		std::map<int32_t, std::set<int32_t>> consumers;
		std::map<int32_t, int32_t> producer; // signal node -> unit
		std::set<int32_t> poFanins;			  // signals directly consumed by POs

		const int32_t nNodes = (int32_t)nodes.size();
		for (int32_t i = ID_OFFSET; i < nNodes; i++)
		{
			const Node &nd = nodes[i];
			if (nd.isPi() || nd.isPo())
				continue;
			// The consumer of a fanin signal is the unit of node i.
			// PI and const fanins have no unit; their edges are classified
			// from the signal node id below.
			if (node2unit[i] < 0)
				continue;
			int32_t f0 = nd.getFanin0Id();
			int32_t f1 = nd.getFanin1Id();
			if (f0 > 0 && f0 < nNodes)
				consumers[f0].insert(node2unit[i]);
			if (f1 > 0 && f1 < nNodes)
				consumers[f1].insert(node2unit[i]);
			producer[i] = node2unit[i];
		}
		// PO nodes are not part of any unit; record the signals they
		// consume for explicit PO accounting.
		for (int32_t i = ID_OFFSET; i < nNodes; i++)
		{
			const Node &nd = nodes[i];
			if (!nd.isPo())
				continue;
			int32_t f0 = nd.getFanin0Id();
			if (f0 >= 0 && f0 < nNodes)
				poFanins.insert(f0);
		}

		int32_t edgeId = 0;
		std::set<int32_t> seenSignals;
		for (const auto &kv : consumers)
		{
			int32_t sig = kv.first;
			if (seenSignals.count(sig))
				continue;
			seenSignals.insert(sig);
			int32_t type;
			int32_t producerUnit = -1;
			if (producer.count(sig))
			{
				type = HG_EDGE_INTERNAL;
				producerUnit = producer[sig];
			}
			else if (sig == 1)
				type = HG_EDGE_CONST; // constant signal (node id 1)
			else
				type = HG_EDGE_PI; // PI-driven signal (node id >= ID_OFFSET, no producer)

			HyperedgeRec e;
			e.edgeId = edgeId++;
			e.type = type;
			e.producerUnit = producerUnit;
			e.signalNodeId = sig;
			e.pins.assign(kv.second.begin(), kv.second.end());
			if (producerUnit >= 0)
				e.pins.push_back(producerUnit); // internal edge: producer + consumers
			std::sort(e.pins.begin(), e.pins.end());
			e.pins.erase(std::unique(e.pins.begin(), e.pins.end()), e.pins.end());
			e.fanout = (int32_t)e.pins.size();
			e.weight = 1;
			e.singleton = (e.fanout <= 1);
			e.lambda = 0;
			e.cutNet = 0;
			m_edges.push_back(e);
			if (type == HG_EDGE_INTERNAL)
				m_nInternalEdges++;
			else if (type == HG_EDGE_PI)
				m_nPiEdges++;
			else
				m_nConstEdges++;
			if (e.singleton)
				m_nSingletonEdges++;
			m_nPinsTotal += e.fanout;
		}
		// Signals consumed only by POs: no unit consumers, but the edge
		// exists (producer unit pin only) so the census is complete.
		for (int32_t sig : poFanins)
		{
			if (seenSignals.count(sig))
				continue;
			seenSignals.insert(sig);
			int32_t type;
			int32_t producerUnit = -1;
			if (producer.count(sig))
			{
				type = HG_EDGE_INTERNAL;
				producerUnit = producer[sig];
			}
			else if (sig == 1)
				type = HG_EDGE_CONST;
			else
				type = HG_EDGE_PI;
			HyperedgeRec e;
			e.edgeId = edgeId++;
			e.type = type;
			e.producerUnit = producerUnit;
			e.signalNodeId = sig;
			if (producerUnit >= 0)
				e.pins.push_back(producerUnit);
			e.fanout = (int32_t)e.pins.size();
			e.weight = 1;
			e.singleton = true;
			e.lambda = 0;
			e.cutNet = 0;
			m_edges.push_back(e);
			if (type == HG_EDGE_INTERNAL)
				m_nInternalEdges++;
			else if (type == HG_EDGE_PI)
				m_nPiEdges++;
			else
				m_nConstEdges++;
			m_nSingletonEdges++;
			m_nPinsTotal += e.fanout;
		}
		// PO edges: census only (each PO is driven by exactly one signal,
		// so it can never span parts; recorded for explicit accounting).
		for (int32_t i = ID_OFFSET; i < nNodes; i++)
		{
			const Node &nd = nodes[i];
			if (!nd.isPo())
				continue;
			HyperedgeRec e;
			e.edgeId = edgeId++;
			e.type = HG_EDGE_PO;
			e.producerUnit = -1;
			e.signalNodeId = i;
			int32_t f0 = nd.getFanin0Id();
			if (f0 >= 0 && f0 < (int32_t)node2unit.size() && node2unit[f0] >= 0)
				e.pins.push_back(node2unit[f0]);
			std::sort(e.pins.begin(), e.pins.end());
			e.pins.erase(std::unique(e.pins.begin(), e.pins.end()), e.pins.end());
			e.fanout = (int32_t)e.pins.size();
			e.weight = 1;
			e.singleton = true;
			e.lambda = 0;
			e.cutNet = 0;
			m_edges.push_back(e);
			m_nPoEdges++;
			m_nSingletonEdges++;
			m_nPinsTotal += e.fanout;
		}
		gProf.tBuildNs += gProf.nowNs() - tBuild0;
		if (gProf.on)
			ylog("[HG-PROF] build wallMs=%.3f units=%lld edges=%lld pins=%lld\n",
				 gProf.tBuildNs / 1e6, (long long)nUnits(), (long long)nEdges(),
				 (long long)nPinsTotal());
	}

	void MffcHypergraph::assignControl(const std::vector<int32_t> &unit2partition)
	{
		m_nCutNets = 0;
		m_nConnectivityCost = 0;
		m_nCrossPartitionPins = 0;
		m_nPredictedInterfaces = 0;
		m_partCapacity.clear();
		for (size_t i = 0; i < m_units.size(); i++)
		{
			int32_t pid = (i < unit2partition.size()) ? unit2partition[i] : -1;
			m_units[i].partitionId = pid;
			if (pid >= 0)
			{
				if (pid >= (int32_t)m_partCapacity.size())
					m_partCapacity.resize(pid + 1, 0);
				m_partCapacity[pid] += m_units[i].nNodes;
			}
		}
		for (auto &e : m_edges)
		{
			if (e.singleton)
			{
				e.lambda = 0;
				e.cutNet = 0;
				continue;
			}
			std::set<int32_t> parts;
			int64_t maxPerPart = 0;
			std::map<int32_t, int64_t> pinsPerPart;
			for (int32_t p : e.pins)
			{
				int32_t pid = (p >= 0 && p < (int32_t)unit2partition.size()) ? unit2partition[p] : -1;
				if (pid < 0)
					continue;
				parts.insert(pid);
				pinsPerPart[pid]++;
			}
			for (const auto &kv : pinsPerPart)
				if (kv.second > maxPerPart)
					maxPerPart = kv.second;
			e.lambda = (int64_t)parts.size();
			e.cutNet = (e.lambda >= 2) ? 1 : 0;
			if (e.cutNet)
			{
				m_nCutNets++;
				m_nConnectivityCost += (int64_t)e.weight * (e.lambda - 1);
				m_nPredictedInterfaces += (int64_t)e.weight * (e.lambda - 1);
				m_nCrossPartitionPins += (int64_t)e.pins.size() - maxPerPart;
			}
		}
	}

	int64_t MffcHypergraph::partCapacityMax() const
	{
		int64_t mx = 0;
		for (int64_t c : m_partCapacity)
			if (c > mx)
				mx = c;
		return mx;
	}

	double MffcHypergraph::partCapacityCv() const
	{
		if (m_partCapacity.empty())
			return 0.0;
		int64_t sum = 0, sum2 = 0;
		for (int64_t c : m_partCapacity)
		{
			sum += c;
			sum2 += c * c;
		}
		double mean = (double)sum / m_partCapacity.size();
		double sd = sqrt(std::max(0.0, (double)sum2 / m_partCapacity.size() - mean * mean));
		return (mean > 0) ? sd / mean : 0.0;
	}

	void MffcHypergraph::exportTelemetry() const
	{
		{
			char buf[512];
			for (const auto &u : m_units)
			{
				snprintf(buf, sizeof(buf), "%d\t%d\t%d\t%d\t%d\t%d\t%d\t%llu\t%d",
						 u.unitId, u.rootNodeId, (int)u.nNodes, (int)u.levelMin,
						 (int)u.levelMax, (int)u.nOrigins, u.splitFragment ? 1 : 0,
						 (unsigned long long)u.membershipHash, (int)u.partitionId);
				pifTelemetryRow("pif_hg_unit.tsv",
								"unitId\trootNodeId\tnNodes\tlevelMin\tlevelMax\tnOrigins\tsplitFragment\tmembershipHash\tpartitionId",
								buf);
			}
		}
		{
			// Rows are built in a std::string: a high-fanout edge can have
			// tens of thousands of pins and must never be truncated.
			for (const auto &e : m_edges)
			{
				std::string pins;
				for (size_t k = 0; k < e.pins.size(); k++)
				{
					if (k)
						pins += ",";
					pins += std::to_string(e.pins[k]);
				}
				std::string row;
				row.reserve(pins.size() + 64);
				row += std::to_string(e.edgeId);
				row += "\t" + std::to_string(e.type);
				row += "\t" + std::to_string(e.signalNodeId);
				row += "\t" + std::to_string((int)e.producerUnit);
				row += "\t" + std::to_string((int)e.fanout);
				row += "\t" + std::to_string((int)e.weight);
				row += "\t" + pins;
				row += "\t" + std::to_string(e.singleton ? 1 : 0);
				row += "\t" + std::to_string((long long)e.lambda);
				row += "\t" + std::to_string((int)e.cutNet);
				pifTelemetryRow("pif_hg_edge.tsv",
								"edgeId\ttype\tsignalNodeId\tproducerUnit\tfanout\tweight\tpins\tsingleton\tlambda\tcutNet",
								row.c_str());
			}
		}
		{
			char buf[512];
			snprintf(buf, sizeof(buf), "%lld\t%lld\t%lld\t%lld\t%lld\t%lld\t%lld\t%lld\t%lld\t%lld\t%lld\t%.6f\t%lld",
					 (long long)nUnits(), (long long)nEdges(), (long long)nPinsTotal(),
					 (long long)m_nInternalEdges, (long long)m_nPiEdges,
					 (long long)m_nPoEdges, (long long)m_nConstEdges,
					 (long long)m_nSingletonEdges, (long long)m_nCutNets,
					 (long long)m_nConnectivityCost, (long long)m_nCrossPartitionPins,
					 partCapacityCv(), (long long)m_nPredictedInterfaces);
			pifTelemetryRow("pif_hg_summary.tsv",
							"nUnits\tnEdges\tnPinsTotal\tnInternalEdges\tnPiEdges\tnPoEdges\tnConstEdges\tnSingletonEdges\tnCutNets\tconnectivityCost\tcrossPartitionPins\tpartCapacityCv\tpredictedInterfaces",
							buf);
		}
		{
			char buf[256];
			for (size_t i = 0; i < m_partCapacity.size(); i++)
			{
				snprintf(buf, sizeof(buf), "%zu\t%lld", i, (long long)m_partCapacity[i]);
				pifTelemetryRow("pif_hg_part.tsv",
								"partitionIdx\tcapacityNodes", buf);
			}
		}
	}

	// ================================================================
	// Task 17 Stage 4: deterministic internal multilevel partitioner.
	//
	// Structure (inspired by the multilevel decomposition associated with
	// hMETIS; no external code is copied, linked, or invoked):
	//   1. heavy-connectivity coarsening (vertex pairs with the largest
	//      shared hyperedge weight, stable id tie-break);
	//   2. balanced initial bisection on the coarsest hypergraph
	//      (weight-descending greedy, stable id tie-break);
	//   3. constrained FM-like refinement while uncoarsening (lazy-gain
	//      moves, balance bounds, locked vertices);
	//   4. recursive bisection to exactly K parts with capacity targets
	//      distributed proportionally (k1 = k/2, k2 = k - k1).
	// Every loop iterates ascending ids; every tie-break is by the smaller
	// id. No random state is used anywhere.
	namespace
	{
		struct HgVtx
		{
			int32_t id;
			int64_t weight;
			int32_t part; // 0/1 during a bisection
			std::vector<int32_t> children; // vertex indices at the finer level
		};
		// Task 18 Stage 3: flat compact level storage (assignment-identical).
		// The old HgEdge vector-of-vectors (one heap allocation per edge pin
		// list and per-vertex incidence list) cost 7-34 us per FM heap op at
		// XS1c scale because every dereference missed into DRAM. Edges and
		// incidence are now CSR arrays with the SAME iteration order, so
		// every access is contiguous and the decisions are byte-identical.
		struct HgLevel
		{
			std::vector<HgVtx> v;
			std::vector<int32_t> eOffsets; // size eCount()+1; ePins[eOffsets[ei]..eOffsets[ei+1])
			std::vector<int32_t> ePins;	// contiguous pin lists (sorted, unique)
			std::vector<int32_t> eWeight;	// uniform 1 (frozen objective)
			std::vector<int32_t> incOffsets; // size v.size()+1 (CSR)
			std::vector<int32_t> incList;	// contiguous incident edge ids
			size_t eCount() const { return eOffsets.size() - 1; }
			void buildIncidence()
			{
				incOffsets.assign(v.size() + 1, 0);
				for (size_t ei = 0; ei + 1 < eOffsets.size(); ei++)
					for (int32_t pi = eOffsets[ei]; pi < eOffsets[ei + 1]; pi++)
						incOffsets[ePins[pi] + 1]++;
				for (size_t i = 1; i < incOffsets.size(); i++)
					incOffsets[i] += incOffsets[i - 1];
				incList.assign(incOffsets.back(), 0);
				std::vector<int32_t> cur(incOffsets.begin(), incOffsets.end() - 1);
				for (size_t ei = 0; ei + 1 < eOffsets.size(); ei++)
					for (int32_t pi = eOffsets[ei]; pi < eOffsets[ei + 1]; pi++)
						incList[cur[ePins[pi]]++] = (int32_t)ei;
			}
		};

		int64_t levelWeight(const HgLevel &l, int32_t side)
		{
			int64_t w = 0;
			for (const auto &v : l.v)
				if (v.part == side)
					w += v.weight;
			return w;
		}

		// Heavy-connectivity matching on 'in'; writes the coarser level to
		// 'out'. Returns false when no contraction happened.
		bool hgCoarsen(const HgLevel &in, HgLevel &out, int64_t capWeight)
		{
			gProf.maybeInit();
			gProf.resetLevel();
			const int32_t n = (int32_t)in.v.size();
			std::vector<int32_t> match(n, -1);
			std::vector<bool> matched(n, false);
			// CSR incidence makes the shared-connectivity scan O(pins)
			// per level instead of O(|V| * |E| * avgPins). Connectivity is
			// accumulated into a plain vector and summed by a stable
			// sort+dedupe (a per-vertex std::map allocates a tree node per
			// pin and dominates the coarsening cost).
			int64_t tScan0 = gProf.nowNs();
			for (int32_t i = 0; i < n; i++)
			{
				if (matched[i])
					continue;
				std::vector<std::pair<int32_t, int64_t>> conn;
				for (int32_t ip = in.incOffsets[i]; ip < in.incOffsets[i + 1]; ip++)
				{
					const int32_t ei = in.incList[ip];
					for (int32_t pp = in.eOffsets[ei]; pp < in.eOffsets[ei + 1]; pp++)
					{
						const int32_t p = in.ePins[pp];
						gProf.levelScanIts++;
						if (p != i && !matched[p])
							conn.push_back({p, in.eWeight[ei]});
					}
				}
				int64_t tSort0 = gProf.nowNs();
				std::sort(conn.begin(), conn.end());
				gProf.levelSortIts += (int64_t)conn.size();
				gProf.levelSortNs += gProf.nowNs() - tSort0;
				int32_t best = -1;
				int64_t bestC = -1;
				for (size_t ci = 0; ci < conn.size();)
				{
					int64_t w = conn[ci].second;
					size_t cj = ci + 1;
					while (cj < conn.size() && conn[cj].first == conn[ci].first)
						w += conn[cj++].second;
					if (w > bestC || (w == bestC && conn[ci].first < best))
					{
						best = conn[ci].first;
						bestC = w;
					}
					ci = cj;
				}
				if (best >= 0 && in.v[i].weight + in.v[best].weight <= capWeight)
				{
					match[i] = best;
					match[best] = i;
					matched[i] = matched[best] = true;
				}
			}
			gProf.levelScanNs += gProf.nowNs() - tScan0;
			// exact incremental-work estimate: pins of edges incident to any
			// matched vertex (what an incremental update would touch). The
			// same pass marks the edges whose pin list must be resorted.
			std::vector<char> eTouched(in.eCount(), 0);
			{
				for (int32_t i = 0; i < n; i++)
					if (matched[i])
						for (int32_t ip = in.incOffsets[i]; ip < in.incOffsets[i + 1]; ip++)
						{
							const int32_t ei = in.incList[ip];
							eTouched[ei] = 1;
							if (gProf.on)
								gProf.levelTouchedPins +=
									(int64_t)(in.eOffsets[ei + 1] - in.eOffsets[ei]);
						}
			}
			std::vector<int32_t> toNew(n, -1);
			int32_t next = 0;
			for (int32_t i = 0; i < n; i++)
			{
				if (match[i] < 0)
				{
					toNew[i] = next++;
					gProf.levelSingles++;
					HgVtx v;
					v.id = in.v[i].id;
					v.weight = in.v[i].weight;
					v.part = -1;
					// Self-child: an unmatched vertex persists to the next
					// level, and the uncoarsening projection must still
					// assign the part of its finer-level copy. Without this,
					// every singleton-chain vertex keeps part -1 and falls
					// to the "else" side of the final split.
					v.children = {i};
					out.v.push_back(v);
				}
				else if (i < match[i])
				{
					toNew[i] = toNew[match[i]] = next++;
					gProf.levelPairs++;
					HgVtx v;
					v.id = std::min(in.v[i].id, in.v[match[i]].id);
					v.weight = in.v[i].weight + in.v[match[i]].weight;
					v.part = -1;
					v.children = {i, match[i]};
					out.v.push_back(v);
				}
			}
			if (out.v.size() >= in.v.size())
				return false;
			int64_t tReb0 = gProf.nowNs();
			out.eOffsets.push_back(0);
			// toNew is non-decreasing in the old index, so an edge with no
			// contracted pin renumbers pointwise into a list that is still
			// sorted and unique; only edges incident to a matched vertex need
			// the sort+unique (exact duplicate handling as before).
			for (size_t ei = 0; ei + 1 < in.eOffsets.size(); ei++)
			{
				gProf.levelRebuildEdges++;
				const int32_t lo = in.eOffsets[ei], hi = in.eOffsets[ei + 1];
				gProf.levelRebuildPins += (int64_t)(hi - lo);
				const int32_t base = (int32_t)out.ePins.size();
				if (!eTouched[ei])
				{
					for (int32_t pp = lo; pp < hi; pp++)
						out.ePins.push_back(toNew[in.ePins[pp]]);
				}
				else
				{
					for (int32_t pp = lo; pp < hi; pp++)
						out.ePins.push_back(toNew[in.ePins[pp]]);
					std::sort(out.ePins.begin() + base, out.ePins.end());
					out.ePins.erase(std::unique(out.ePins.begin() + base, out.ePins.end()),
									out.ePins.end());
				}
				if ((int32_t)out.ePins.size() - base >= 2)
				{
					out.eWeight.push_back(in.eWeight[ei]);
					out.eOffsets.push_back((int32_t)out.ePins.size());
				}
				else
				{
					out.ePins.resize(base);
				}
			}
			gProf.levelRebuildNs += gProf.nowNs() - tReb0;
			return true;
		}

		// Balanced initial bisection: weight-descending greedy to the
		// lighter side; ties broken by ascending vertex id.
		void hgInitialBisect(HgLevel &l)
		{
			std::vector<int32_t> order(l.v.size());
			std::iota(order.begin(), order.end(), 0);
			std::sort(order.begin(), order.end(), [&](int32_t a, int32_t b)
					  { return l.v[a].weight > l.v[b].weight ||
							   (l.v[a].weight == l.v[b].weight && l.v[a].id < l.v[b].id); });
			int64_t wA = 0, wB = 0;
			for (int32_t i : order)
			{
				if (wA <= wB)
				{
					l.v[i].part = 0;
					wA += l.v[i].weight;
				}
				else
				{
					l.v[i].part = 1;
					wB += l.v[i].weight;
				}
			}
		}

		// Constrained FM-like refinement: lazy-gain max-heap, balance bound
		// |wA - targetW1| <= slack, each vertex moves at most once, ties by
		// ascending id. Gain of moving v to the other side = total weight of
		// hyperedges where v is the sole pin of its side and the other side
		// has at least one pin (connectivity and cut-net both improve).
		int64_t hgFmRefine(HgLevel &l, int64_t targetW1, int64_t slack)
		{
			const int32_t n = (int32_t)l.v.size();
			gProf.fmGainOfCalls = gProf.fmHeapPushes = gProf.fmHeapPops = gProf.fmPinIters = 0;
			if (n < 2)
				return 0;
			int64_t wA = levelWeight(l, 0);
			int64_t wB = levelWeight(l, 1);
			// per-edge pin counts per side and the exact initial gains,
			// computed in one sequential pass over the edges (the previous
			// per-vertex gainOf re-read the same counts randomly and cost
			// ~10 us per call at XS1c scale). contribution(e,p) = w iff p is
			// the sole pin of its side and the other side is non-empty.
			std::vector<int32_t> pinsA(l.eCount(), 0), pinsB(l.eCount(), 0);
			std::vector<int64_t> gain(n, 0);
			for (size_t ei = 0; ei + 1 < l.eOffsets.size(); ei++)
			{
				int32_t a = 0, b = 0, pa = -1, pb = -1;
				for (int32_t pp = l.eOffsets[ei]; pp < l.eOffsets[ei + 1]; pp++)
				{
					const int32_t p = l.ePins[pp];
					if (l.v[p].part == 0)
					{
						a++;
						pa = p;
					}
					else
					{
						b++;
						pb = p;
					}
				}
				pinsA[ei] = a;
				pinsB[ei] = b;
				if (a == 1 && b >= 1)
					gain[pa] += l.eWeight[ei];
				if (b == 1 && a >= 1)
					gain[pb] += l.eWeight[ei];
			}
			// Exact max-gain / min-id bucket queue (Task 18 Stage 3): the
			// lazy-gain max-heap's strided random access cost 3-20 us per op
			// on this host once the heap outgrew the per-core cache. A gain
			// bucket array (index = gain + maxDeg) with O(1) delta updates
			// keeps the same extraction order: max gain first, then the
			// smaller id, because the gains only change by +/-w(e) on the
			// crossing pins and every other stored gain is fresh by the
			// exact-invalidation invariant.
			int32_t maxDeg = 0;
			for (int32_t i = 0; i < n; i++)
			{
				const int32_t d = l.incOffsets[i + 1] - l.incOffsets[i];
				if (d > maxDeg)
					maxDeg = d;
			}
			if (maxDeg < 1)
				maxDeg = 1;
			const int32_t off = maxDeg;
			std::vector<std::set<int32_t>> buckets(2 * (size_t)maxDeg + 1);
			std::vector<int32_t> storedGain(n, 0);
			std::vector<bool> locked(n, false);
			// discarded[v]: v was extracted and skipped by the balance bound
			// without locking; it is NOT in a bucket and must be re-inserted
			// when a neighbor moves (exact replication of the pre-refactor
			// neighbor re-push, which re-examined skipped vertices).
			std::vector<char> discarded(n, 0);
			int32_t ptr = -off;
			for (int32_t i = 0; i < n; i++)
			{
				const int32_t g = (int32_t)gain[i];
				storedGain[i] = g;
				buckets[g + off].insert(i);
				gProf.fmHeapPushes++;
				if (g > ptr)
					ptr = g;
			}
			auto bucketMove = [&](int32_t p, int32_t gNew)
			{
				// The old entry stays: the pre-refactor heap pops entries at
				// their push-time gain positions, so a stale position can
				// trigger the negative-gain break earlier than the true max.
				storedGain[p] = gNew;
				buckets[gNew + off].insert(p);
				gProf.fmHeapPushes++;
				if (gNew > ptr)
					ptr = gNew;
			};
			int moves = 0;
			while (moves < n)
			{
				while (ptr >= -off && buckets[ptr + off].empty())
					ptr--;
				if (ptr < 0)
					break; // no improving move
				auto &bk = buckets[ptr + off];
				const int32_t v = *bk.begin();
				bk.erase(bk.begin());
				gProf.fmHeapPops++;
				if (locked[v])
					continue;
				const int64_t cur = storedGain[v];
				if (cur < 0)
					break; // no improving move (decided at the stale position,
						   // exactly like the pre-refactor heap)
				// balance check: v moves from its side to the other
				const int64_t dw = l.v[v].weight;
				const int64_t newA = (l.v[v].part == 0) ? wA - dw : wA + dw;
				if (std::llabs(newA - targetW1) > slack)
				{
					discarded[v] = 1; // extracted without locking
					continue;
				}
				if (newA < 0 || newA > wA + wB)
				{
					discarded[v] = 1; // extracted without locking
					continue;
				}
				// apply the move
				const int32_t from = l.v[v].part;
				l.v[v].part = 1 - from;
				if (from == 0)
				{
					wA -= dw;
					wB += dw;
				}
				else
				{
					wB -= dw;
					wA += dw;
				}
				for (int32_t ip = l.incOffsets[v]; ip < l.incOffsets[v + 1]; ip++)
				{
					const int32_t ei = l.incList[ip];
					// counts of v's old and new side BEFORE the move
					const int64_t aOld = (from == 0) ? pinsA[ei] : pinsB[ei];
					const int64_t bOld = (from == 0) ? pinsB[ei] : pinsA[ei];
					if (from == 0)
					{
						pinsA[ei]--;
						pinsB[ei]++;
					}
					else
					{
						pinsB[ei]--;
						pinsA[ei]++;
					}
					// Exact incremental gain invalidation: a move changes the
					// contribution of a pin p != v only when its side count
					// crosses 1 (a==2: the remaining old-side pin becomes
					// sole, +w; b==1: the sole new-side pin stops being sole,
					// -w). All other gains are unchanged, so the delta is
					// exact and the bucket move is O(log).
					if (aOld == 2)
					{
						for (int32_t pp = l.eOffsets[ei]; pp < l.eOffsets[ei + 1]; pp++)
						{
							const int32_t p = l.ePins[pp];
							gProf.fmPinIters++;
							if (p != v && l.v[p].part == from)
							{
								if (!locked[p])
								{
									bucketMove(p, storedGain[p] + l.eWeight[ei]);
									discarded[p] = 0;
								}
								break;
							}
						}
					}
					if (bOld == 1)
					{
						for (int32_t pp = l.eOffsets[ei]; pp < l.eOffsets[ei + 1]; pp++)
						{
							const int32_t p = l.ePins[pp];
							gProf.fmPinIters++;
							if (p != v && l.v[p].part == 1 - from)
							{
								if (!locked[p])
								{
									bucketMove(p, storedGain[p] - l.eWeight[ei]);
									discarded[p] = 0;
								}
								break;
							}
						}
					}
					// Re-insert every discarded (balance-skipped) neighbor with
					// its cached gain: the pre-refactor code re-pushed every
					// neighbor on every move, which re-examined skipped
					// vertices after the side weights shifted. A cached insert
					// is exact because storedGain is fresh by the invariant.
					for (int32_t pp = l.eOffsets[ei]; pp < l.eOffsets[ei + 1]; pp++)
					{
						const int32_t p = l.ePins[pp];
						if (p != v && !locked[p] && discarded[p])
						{
							buckets[storedGain[p] + off].insert(p);
							gProf.fmHeapPushes++;
							discarded[p] = 0;
							if (storedGain[p] > ptr)
								ptr = storedGain[p];
						}
					}
				}
				locked[v] = true;
				moves++;
			}
			return moves;
		}
	} // namespace

	std::vector<int32_t> MffcHypergraph::partitionMultiway(int32_t K)
	{
		if (getenv("PIF_HG_DEBUG"))
			ylog("[HG-DBG] partitionMultiway K=%d units=%zu edges=%zu\n",
				 K, m_units.size(), m_edges.size());
		const int32_t nUnits = (int32_t)m_units.size();
		std::vector<int32_t> unit2part(nUnits, 0);
		if (K <= 1 || nUnits == 0)
			return unit2part;
		if (K > nUnits)
			K = nUnits;

		// Recursive bisection with proportional capacity targets.
		// The recursion carries the induced edge list of the current
		// subset; each level filters only its parent's edges instead of
		// rescanning the whole hypergraph (O(edges) per bisection would be
		// quadratic over the recursion tree).
		std::function<void(const std::vector<int32_t> &, const std::vector<std::vector<int32_t>> &, int32_t, int32_t, int32_t)> bisectRec;
		bisectRec = [&](const std::vector<int32_t> &subset,
						const std::vector<std::vector<int32_t>> &subsetEdges,
						int32_t k, int32_t basePart, int32_t depth)
		{
			gProf.maybeInit();
			gProf.resetBisect();
			gProf.nBisections++;
			int64_t tBisect0 = gProf.nowNs();
			if (getenv("PIF_HG_DEBUG"))
				ylog("[HG-DBG] bisect k=%d subset=%zu edges=%zu\n",
					 k, subset.size(), subsetEdges.size());
			if (k > (int32_t)subset.size())
				k = (int32_t)subset.size(); // exact-K feasibility: k <= |subset|
			if (k <= 1)
			{
				for (int32_t u : subset)
					unit2part[u] = basePart;
				return;
			}
			int64_t totalW = 0;
			int64_t maxW = 0;
			for (int32_t u : subset)
			{
				totalW += m_units[u].nNodes;
				if (m_units[u].nNodes > maxW)
					maxW = m_units[u].nNodes;
			}
			int32_t k1 = k / 2, k2 = k - k1;
			int64_t targetW1 = (totalW * k1) / k;

			// induced hypergraph on the subset
			HgLevel l0;
			{
				int64_t tProj0 = gProf.nowNs();
				std::unordered_map<int32_t, int32_t> u2idx;
				u2idx.reserve(subset.size() * 2);
				for (int32_t u : subset)
				{
					u2idx[u] = (int32_t)l0.v.size();
					HgVtx v;
					v.id = u;
					v.weight = m_units[u].nNodes;
					v.part = -1;
					l0.v.push_back(v);
				}
				l0.eOffsets.push_back(0);
				for (const auto &pins0 : subsetEdges)
				{
					std::vector<int32_t> pins;
					pins.reserve(pins0.size());
					for (int32_t p : pins0)
					{
						auto it = u2idx.find(p);
						if (it != u2idx.end())
							pins.push_back(it->second);
					}
					std::sort(pins.begin(), pins.end());
					pins.erase(std::unique(pins.begin(), pins.end()), pins.end());
					if (pins.size() >= 2)
					{
						const int32_t base = (int32_t)l0.ePins.size();
						l0.ePins.insert(l0.ePins.end(), pins.begin(), pins.end());
						l0.eWeight.push_back(1);
						l0.eOffsets.push_back((int32_t)l0.ePins.size());
					}
				}
				gProf.tProjectNs += gProf.nowNs() - tProj0;
			}

			// coarsen (cap keeps any coarsened vertex bisectable)
			std::vector<HgLevel> levels;
			l0.buildIncidence();
			levels.push_back(l0);
			int64_t capW = targetW1 + std::max<int64_t>(targetW1 / 20, maxW);
			while (levels.back().v.size() > 1)
			{
				HgLevel nl;
				int64_t tC0 = gProf.nowNs();
				bool contracted = hgCoarsen(levels.back(), nl, capW);
				gProf.tCoarsenNs += gProf.nowNs() - tC0;
				if (gProf.on)
				{
					int64_t maxWl = 0;
					const HgLevel &src = contracted ? nl : levels.back();
					for (const auto &v : src.v)
						if (v.weight > maxWl)
							maxWl = v.weight;
					int64_t pinsLv = 0;
					for (size_t ei = 0; ei + 1 < levels.back().eOffsets.size(); ei++)
						pinsLv += (int64_t)(levels.back().eOffsets[ei + 1] - levels.back().eOffsets[ei]);
					ylog("[HG-PROF] level d=%d k=%d li=%zu vIn=%zu vOut=%zu e=%zu pins=%lld "
						 "pairs=%lld singles=%lld ratio=%.6f maxW=%lld scanIts=%lld sortIts=%lld "
						 "touchedPins=%lld rebuildE=%lld rebuildPins=%lld scanMs=%.3f sortMs=%.3f "
						 "rebuildMs=%.3f reason=%s\n",
						 depth, k, levels.size(), levels.back().v.size(), src.v.size(),
						 levels.back().eCount(), (long long)pinsLv, (long long)gProf.levelPairs,
						 (long long)gProf.levelSingles,
						 src.v.size() ? (double)src.v.size() / levels.back().v.size() : 0.0,
						 (long long)maxWl, (long long)gProf.levelScanIts,
						 (long long)gProf.levelSortIts, (long long)gProf.levelTouchedPins,
						 (long long)gProf.levelRebuildEdges, (long long)gProf.levelRebuildPins,
						 gProf.levelScanNs / 1e6, gProf.levelSortNs / 1e6,
						 gProf.levelRebuildNs / 1e6,
						 contracted ? "ok" : "no_contraction");
				}
				if (!contracted)
					break;
				gProf.nLevels++;
				gProf.scanItsBisect += gProf.levelScanIts;
				gProf.rebuildPinsBisect += gProf.levelRebuildPins;
				gProf.touchedPinsBisect += gProf.levelTouchedPins;
				nl.buildIncidence();
				levels.push_back(nl);
				if (getenv("PIF_HG_DEBUG") && levels.size() <= 8)
					ylog("[HG-DBG] coarsen level %zu: %zu -> %zu\n",
						 levels.size(), levels[levels.size() - 2].v.size(), nl.v.size());
			}

			// initial bisection on the coarsest level
			hgInitialBisect(levels.back());

			// uncoarsen: project parts down, then constrained FM refinement.
			// The balance slack is fixed from the finest (original unit)
			// level: a coarsened vertex can exceed it (indivisible-unit
			// exception), but the FM must never move a vertex across the
			// cut when that would violate the finest-level bound, otherwise
			// a giant coarsened vertex silently unbalances the whole split.
			int64_t slack = std::max<int64_t>(targetW1 / 20, maxW);
			if (getenv("PIF_HG_DEBUG"))
				ylog("[HG-DBG] coarsened to %zu levels (finest %zu)\n",
					 levels.size(), levels[0].v.size());
			int64_t tFm0 = gProf.nowNs();
			for (int li = (int)levels.size() - 2; li >= 0; li--)
			{
				for (const auto &v : levels[li + 1].v)
					for (int32_t c : v.children)
						levels[li].v[c].part = v.part;
				int64_t tF1 = gProf.nowNs();
				int64_t fmMoves = hgFmRefine(levels[li], targetW1, slack);
				int64_t tF2 = gProf.nowNs();
				if (gProf.on)
					ylog("[HG-PROF] fm d=%d k=%d li=%d v=%zu e=%zu moves=%lld ms=%.3f "
						 "gainOf=%lld pushes=%lld pops=%lld pinIters=%lld\n",
						 depth, k, li, levels[li].v.size(), levels[li].eCount(),
						 (long long)fmMoves, (tF2 - tF1) / 1e6,
						 (long long)gProf.fmGainOfCalls, (long long)gProf.fmHeapPushes,
						 (long long)gProf.fmHeapPops, (long long)gProf.fmPinIters);
			}
			gProf.tFmNs += gProf.nowNs() - tFm0;

			// split the subset by the finest-level parts
			std::vector<int32_t> a, b;
			a.reserve(subset.size());
			b.reserve(subset.size());
			for (size_t i = 0; i < subset.size(); i++)
				if (levels[0].v[i].part == 0)
					a.push_back(subset[i]);
				else
					b.push_back(subset[i]);
			// Exact-K feasibility: each side needs at least its target part
			// count of units. Repair sizes deterministically by moving the
			// smallest units (weight, then id) from the over-large side.
			// Move the smallest units first so count repair disturbs the
			// weight balance as little as possible. One sort per repair:
			// sorting inside the move loop is O(n^2 log n) and took minutes
			// on the XS1c-scale hypergraph.
			auto sizeRepair = [&](std::vector<int32_t> &x, int32_t needX,
								  std::vector<int32_t> &y, int32_t needY)
			{
				if ((int32_t)x.size() >= needX && (int32_t)y.size() >= needY)
					return;
				auto byWeight = [&](int32_t u1, int32_t u2)
				{
					return m_units[u1].nNodes < m_units[u2].nNodes ||
						   (m_units[u1].nNodes == m_units[u2].nNodes && u1 < u2);
				};
				if ((int32_t)x.size() < needX && !y.empty())
				{
					std::sort(y.begin(), y.end(), byWeight);
					size_t take = std::min<size_t>(needX - x.size(), y.size());
					x.insert(x.end(), y.begin(), y.begin() + take);
					y.erase(y.begin(), y.begin() + take);
				}
				if ((int32_t)y.size() < needY && !x.empty())
				{
					std::sort(x.begin(), x.end(), byWeight);
					size_t take = std::min<size_t>(needY - y.size(), x.size());
					y.insert(y.end(), x.begin(), x.begin() + take);
					x.erase(x.begin(), x.begin() + take);
				}
				// the moved units keep the weight/connectivity they had; an
				// oversized indivisible unit on a side is the explicit
				// balance exception allowed by the packet
			};
			int64_t tRep0 = gProf.nowNs();
			sizeRepair(a, k1, b, k2);
			sizeRepair(b, k2, a, k1);
			gProf.tRepairNs += gProf.nowNs() - tRep0;
			if (a.empty() || b.empty())
			{
				for (int32_t u : subset)
					unit2part[u] = basePart;
				return;
			}
			// partition the parent's induced edges into the children's
			// induced edge lists (each edge keeps its unit-id pins)
			std::vector<std::vector<int32_t>> aEdges, bEdges;
			{
				int64_t tSp0 = gProf.nowNs();
				std::vector<char> inA(nUnits, 0);
				for (int32_t u : a)
					inA[u] = 1;
				for (const auto &pins0 : subsetEdges)
				{
					std::vector<int32_t> pa, pb;
					for (int32_t p : pins0)
						if (inA[p])
							pa.push_back(p);
						else
							pb.push_back(p);
					if (pa.size() >= 2)
						aEdges.push_back(std::move(pa));
					if (pb.size() >= 2)
						bEdges.push_back(std::move(pb));
				}
				gProf.tSplitNs += gProf.nowNs() - tSp0;
			}
			gProf.tBisectNs += gProf.nowNs() - tBisect0;
			if (gProf.on)
			{
				gProf.gTotLevels += gProf.nLevels;
				gProf.gTotScanIts += gProf.scanItsBisect;
				gProf.gTotRebuildPins += gProf.rebuildPinsBisect;
				gProf.gTotTouchedPins += gProf.touchedPinsBisect;
				ylog("[HG-PROF] bisect d=%d k=%d subset=%zu edges=%zu levels=%lld "
					 "wallMs=%.3f projectMs=%.3f coarsenMs=%.3f fmMs=%.3f repairMs=%.3f "
					 "splitMs=%.3f scanIts=%lld rebuildPins=%lld touchedPins=%lld rssKb=%lld\n",
					 depth, k, subset.size(), subsetEdges.size(),
					 (long long)gProf.nLevels,
					 gProf.tBisectNs / 1e6, gProf.tProjectNs / 1e6,
					 gProf.tCoarsenNs / 1e6, gProf.tFmNs / 1e6,
					 gProf.tRepairNs / 1e6, gProf.tSplitNs / 1e6,
					 (long long)gProf.scanItsBisect, (long long)gProf.rebuildPinsBisect,
					 (long long)gProf.touchedPinsBisect, (long long)gProf.hwmKb());
			}
			bisectRec(a, aEdges, k1, basePart, depth + 1);
			bisectRec(b, bEdges, k2, basePart + k1, depth + 1);
		};

		std::vector<int32_t> all(nUnits);
		std::iota(all.begin(), all.end(), 0);
		std::vector<std::vector<int32_t>> allEdges;
		allEdges.reserve(m_edges.size());
		for (const auto &e : m_edges)
			allEdges.push_back(e.pins);
		bisectRec(all, allEdges, K, 0, 0);
		if (gProf.on)
			ylog("[HG-PROF] summary bisections=%lld levels=%lld scanIts=%lld "
				 "rebuildPins=%lld touchedPins=%lld rssKb=%lld\n",
				 (long long)gProf.nBisections, (long long)gProf.gTotLevels,
				 (long long)gProf.gTotScanIts, (long long)gProf.gTotRebuildPins,
				 (long long)gProf.gTotTouchedPins, (long long)gProf.hwmKb());
		return unit2part;
	}

} // namespace ymc
