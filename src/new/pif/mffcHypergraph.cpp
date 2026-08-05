#include "mffcHypergraph.h"
#include "partNtkFuncs.h"
#include "yaig.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <map>
#include <numeric>
#include <queue>
#include <set>

namespace ymc
{
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
	// ================================================================
	namespace
	{
		struct HgVtx
		{
			int32_t id;
			int64_t weight;
			int32_t part; // 0/1 during a bisection
			std::vector<int32_t> children; // vertex indices at the finer level
		};
		struct HgEdge
		{
			int32_t id;
			std::vector<int32_t> pins; // vertex indices at this level
			int32_t weight;
		};
		struct HgLevel
		{
			std::vector<HgVtx> v;
			std::vector<HgEdge> e;
			std::vector<std::vector<int32_t>> inc; // vertex -> incident edge indices
			void buildIncidence()
			{
				inc.assign(v.size(), {});
				for (size_t ei = 0; ei < e.size(); ei++)
					for (int32_t p : e[ei].pins)
						inc[p].push_back((int32_t)ei);
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
			const int32_t n = (int32_t)in.v.size();
			std::vector<int32_t> match(n, -1);
			std::vector<bool> matched(n, false);
			// Incidence list makes the shared-connectivity scan O(pins)
			// per level instead of O(|V| * |E| * avgPins).
			for (int32_t i = 0; i < n; i++)
			{
				if (matched[i])
					continue;
				// shared connectivity of (i, neighbor) = total weight of
				// hyperedges containing both; iterate incident edges in
				// ascending edge order and accumulate into a sorted map so
				// the result is stable.
				std::map<int32_t, int64_t> conn;
				for (int32_t ei : in.inc[i])
				{
					const HgEdge &e = in.e[ei];
					for (int32_t p : e.pins)
						if (p != i && !matched[p])
							conn[p] += e.weight;
				}
				int32_t best = -1;
				int64_t bestC = -1;
				for (const auto &kv : conn)
					if (kv.second > bestC || (kv.second == bestC && kv.first < best))
					{
						best = kv.first;
						bestC = kv.second;
					}
				if (best >= 0 && in.v[i].weight + in.v[best].weight <= capWeight)
				{
					match[i] = best;
					match[best] = i;
					matched[i] = matched[best] = true;
				}
			}
			std::vector<int32_t> toNew(n, -1);
			int32_t next = 0;
			for (int32_t i = 0; i < n; i++)
			{
				if (match[i] < 0)
				{
					toNew[i] = next++;
					HgVtx v;
					v.id = in.v[i].id;
					v.weight = in.v[i].weight;
					v.part = -1;
					out.v.push_back(v);
				}
				else if (i < match[i])
				{
					toNew[i] = toNew[match[i]] = next++;
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
			for (const auto &e : in.e)
			{
				std::vector<int32_t> pins;
				pins.reserve(e.pins.size());
				for (int32_t p : e.pins)
					pins.push_back(toNew[p]);
				std::sort(pins.begin(), pins.end());
				pins.erase(std::unique(pins.begin(), pins.end()), pins.end());
				if (pins.size() >= 2)
				{
					HgEdge ne;
					ne.id = e.id;
					ne.pins = pins;
					ne.weight = e.weight;
					out.e.push_back(ne);
				}
			}
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
		void hgFmRefine(HgLevel &l, int64_t targetW1, int64_t slack)
		{
			const int32_t n = (int32_t)l.v.size();
			if (n < 2)
				return;
			int64_t wA = levelWeight(l, 0);
			int64_t wB = levelWeight(l, 1);
			// per-edge pin counts per side
			std::vector<int64_t> pinsA(l.e.size(), 0), pinsB(l.e.size(), 0);
			for (size_t ei = 0; ei < l.e.size(); ei++)
				for (int32_t p : l.e[ei].pins)
					if (l.v[p].part == 0)
						pinsA[ei]++;
					else
						pinsB[ei]++;

			auto gainOf = [&](int32_t vid) -> int64_t
			{
				int64_t g = 0;
				for (int32_t ei : l.inc[vid])
				{
					const auto &e = l.e[ei];
					bool sole = (l.v[vid].part == 0) ? (pinsA[ei] == 1) : (pinsB[ei] == 1);
					bool other = (l.v[vid].part == 0) ? (pinsB[ei] >= 1) : (pinsA[ei] >= 1);
					if (sole && other)
						g += e.weight;
				}
				return g;
			};

			struct HeapItem
			{
				int64_t gain;
				int32_t id;
			};
			struct HeapCmp
			{
				bool operator()(const HeapItem &a, const HeapItem &b) const
				{
					if (a.gain != b.gain)
						return a.gain < b.gain; // max gain first
					return a.id > b.id;			// smaller id first
				}
			};
			std::priority_queue<HeapItem, std::vector<HeapItem>, HeapCmp> heap;
			std::vector<int64_t> storedGain(n, 0);
			std::vector<bool> locked(n, false);
			for (int32_t i = 0; i < n; i++)
			{
				int64_t g = gainOf(i);
				storedGain[i] = g;
				heap.push({g, i});
			}
			int moves = 0;
			while (!heap.empty() && moves < n)
			{
				HeapItem top = heap.top();
				heap.pop();
				int32_t v = top.id;
				if (locked[v])
					continue;
				int64_t cur = gainOf(v);
				if (cur != storedGain[v])
				{
					storedGain[v] = cur;
					heap.push({cur, v});
					continue;
				}
				if (cur < 0)
					break; // no improving move
				// balance check: v moves from its side to the other
				int64_t dw = l.v[v].weight;
				int64_t newA = (l.v[v].part == 0) ? wA - dw : wA + dw;
				if (std::llabs(newA - targetW1) > slack)
					continue; // constrained: skip without locking
				// apply the move
				int32_t from = l.v[v].part;
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
				for (int32_t ei : l.inc[v])
				{
					const auto &e = l.e[ei];
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
					for (int32_t p : e.pins)
						if (p != v && !locked[p])
						{
							int64_t g2 = gainOf(p);
							storedGain[p] = g2;
							heap.push({g2, p});
						}
				}
				locked[v] = true;
				moves++;
			}
		}
	} // namespace

	std::vector<int32_t> MffcHypergraph::partitionMultiway(int32_t K)
	{
		const int32_t nUnits = (int32_t)m_units.size();
		std::vector<int32_t> unit2part(nUnits, 0);
		if (K <= 1 || nUnits == 0)
			return unit2part;
		if (K > nUnits)
			K = nUnits;

		// Recursive bisection with proportional capacity targets.
		std::function<void(const std::vector<int32_t> &, int32_t, int32_t)> bisectRec;
		bisectRec = [&](const std::vector<int32_t> &subset, int32_t k, int32_t basePart)
		{
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
				for (const auto &e : m_edges)
				{
					std::vector<int32_t> pins;
					for (int32_t p : e.pins)
					{
						auto it = u2idx.find(p);
						if (it != u2idx.end())
							pins.push_back(it->second);
					}
					std::sort(pins.begin(), pins.end());
					pins.erase(std::unique(pins.begin(), pins.end()), pins.end());
					if (pins.size() >= 2)
					{
						HgEdge ne;
						ne.id = e.edgeId;
						ne.pins = pins;
						ne.weight = e.weight;
						l0.e.push_back(ne);
					}
				}
			}

			// coarsen (cap keeps any coarsened vertex bisectable)
			std::vector<HgLevel> levels;
			l0.buildIncidence();
			levels.push_back(l0);
			int64_t capW = targetW1 + std::max<int64_t>(targetW1 / 20, maxW);
			while (levels.back().v.size() > 1)
			{
				HgLevel nl;
				if (!hgCoarsen(levels.back(), nl, capW))
					break;
				nl.buildIncidence();
				levels.push_back(nl);
			}

			// initial bisection on the coarsest level
			hgInitialBisect(levels.back());

			// uncoarsen: project parts down, then constrained FM refinement
			for (int li = (int)levels.size() - 2; li >= 0; li--)
			{
				for (const auto &v : levels[li + 1].v)
					for (int32_t c : v.children)
						levels[li].v[c].part = v.part;
				int64_t maxWl = 0;
				for (const auto &v : levels[li].v)
					if (v.weight > maxWl)
						maxWl = v.weight;
				int64_t slack = std::max<int64_t>(targetW1 / 20, maxWl);
				hgFmRefine(levels[li], targetW1, slack);
			}

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
			auto sizeRepair = [&](std::vector<int32_t> &x, int32_t needX,
								  std::vector<int32_t> &y, int32_t needY)
			{
				if ((int32_t)x.size() >= needX && (int32_t)y.size() >= needY)
					return;
				std::sort(x.begin(), x.end(), [&](int32_t u1, int32_t u2)
						  { return m_units[u1].nNodes < m_units[u2].nNodes ||
								   (m_units[u1].nNodes == m_units[u2].nNodes && u1 < u2); });
				std::sort(y.begin(), y.end(), [&](int32_t u1, int32_t u2)
						  { return m_units[u1].nNodes < m_units[u2].nNodes ||
								   (m_units[u1].nNodes == m_units[u2].nNodes && u1 < u2); });
				while ((int32_t)x.size() < needX && !y.empty())
				{
					x.push_back(y.back());
					y.pop_back();
				}
				while ((int32_t)y.size() < needY && !x.empty())
				{
					y.push_back(x.back());
					x.pop_back();
				}
				// the moved units keep the weight/connectivity they had; an
				// oversized indivisible unit on a side is the explicit
				// balance exception allowed by the packet
			};
			sizeRepair(a, k1, b, k2);
			sizeRepair(b, k2, a, k1);
			if (a.empty() || b.empty())
			{
				for (int32_t u : subset)
					unit2part[u] = basePart;
				return;
			}
			bisectRec(a, k1, basePart);
			bisectRec(b, k2, basePart + k1);
		};

		std::vector<int32_t> all(nUnits);
		std::iota(all.begin(), all.end(), 0);
		bisectRec(all, K, 0);
		return unit2part;
	}

} // namespace ymc
