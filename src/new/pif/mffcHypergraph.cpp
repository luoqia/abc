#include "mffcHypergraph.h"
#include "partNtkFuncs.h"
#include "yaig.h"

#include <algorithm>
#include <cmath>
#include <map>
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
			char buf[1024];
			for (const auto &e : m_edges)
			{
				std::string pins;
				for (size_t k = 0; k < e.pins.size(); k++)
				{
					if (k)
						pins += ",";
					pins += std::to_string(e.pins[k]);
				}
				snprintf(buf, sizeof(buf), "%d\t%d\t%d\t%d\t%d\t%d\t%s\t%d\t%lld\t%d",
						 e.edgeId, e.type, e.signalNodeId, (int)e.producerUnit,
						 (int)e.fanout, (int)e.weight, pins.c_str(),
						 e.singleton ? 1 : 0, (long long)e.lambda, (int)e.cutNet);
				pifTelemetryRow("pif_hg_edge.tsv",
								"edgeId\ttype\tsignalNodeId\tproducerUnit\tfanout\tweight\tpins\tsingleton\tlambda\tcutNet",
								buf);
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

} // namespace ymc
