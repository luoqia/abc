#pragma once
#include <cstdint>
#include <vector>

namespace pif
{
	class MffcUnit;
	class Node;

	// ================================================================
	// Task 17 Stage 2 behavior-neutral hypergraph representation.
	//
	// A post-preprocessing region is called a PartitionUnit (not an
	// MFFC): the oversized-region split explicitly breaks the strict
	// MFFC property. Units are the hypergraph vertices, with internal
	// AND/node count as the capacity weight.
	//
	// One hyperedge is built per internal signal (producer unit plus
	// every distinct consumer unit), per primary input (every distinct
	// consumer unit, external status recorded), per primary output
	// (its driving unit), and per constant signal (every distinct
	// consumer unit). Singleton hyperedges (fanout <= 1) are counted
	// in the census but excluded from the cut objective. Pins are
	// deduplicated and sorted; all IDs are stable; all totals use
	// 64-bit counters.
	// ================================================================

	struct PartitionUnitRec
	{
		int32_t unitId;
		int32_t rootNodeId;
		int32_t nNodes; // internal AND/node count = capacity weight
		int32_t levelMin, levelMax;
		int32_t nOrigins;   // number of strict (pre-merge) MFFC origins
		bool splitFragment; // unit is (part of) an oversized-region split
		uint64_t membershipHash; // FNV-1a 64 over sorted node ids
		int32_t partitionId;	 // control assignment (-1 when unassigned)
		std::vector<int32_t> nodeIds; // sorted
	};

	enum HyperedgeType : int32_t
	{
		HG_EDGE_INTERNAL = 0, // signal produced by an AND node
		HG_EDGE_PI = 1,		  // signal driven by a primary input
		HG_EDGE_PO = 2,		  // primary output (driven by a signal)
		HG_EDGE_CONST = 3	  // constant signal (node id 1)
	};

	struct HyperedgeRec
	{
		int32_t edgeId;
		int32_t type;		 // HyperedgeType
		int32_t producerUnit; // -1 for PI/PO/const edges
		int32_t signalNodeId; // signal node in the Yaig (PI/AND), -1 for PO/const
		std::vector<int32_t> pins; // sorted unique unit ids
		int32_t fanout;			  // number of consumer units
		int32_t weight;			  // uniform 1 (frozen in Stage 3)
		bool singleton;			  // fanout <= 1 -> census only
		int64_t lambda;			  // distinct parts spanned (0 when unassigned)
		int32_t cutNet;			  // 1 when lambda >= 2
	};

	class MffcHypergraph
	{
	public:
		void clear();
		// Behavior-neutral build. All inputs are read-only; nothing here
		// changes selection semantics. origins[ui] lists the strict
		// (pre-merge) MFFC ids inside unit ui; splitFlags[ui] marks units
		// produced by an oversized-region split.
		void build(const std::vector<Node> &nodes,
				   const std::vector<int32_t> &node2unit,
				   const std::vector<MffcUnit> &units,
				   const std::vector<std::vector<int32_t>> &origins,
				   const std::vector<bool> &splitFlags);
		// Assign control partition ids (unit -> partition) and compute
		// the frozen-objective metrics over the control assignment.
		void assignControl(const std::vector<int32_t> &unit2partition);

		// Task 17 Stage 4: deterministic internal multilevel partitioner.
		// Heavy-connectivity coarsening, balanced initial bisection,
		// constrained FM-like refinement while uncoarsening, and recursive
		// bisection to exactly K parts (K <= nUnits). Capacity targets are
		// distributed proportionally over the recursion. All tie-breaks use
		// stable ascending unit ids; there is no random start.
		// Returns unit -> part id in [0, K).
		std::vector<int32_t> partitionMultiway(int32_t K);

		int64_t nUnits() const { return (int64_t)m_units.size(); }
		int64_t nEdges() const { return (int64_t)m_edges.size(); }
		int64_t nPinsTotal() const { return m_nPinsTotal; }
		int64_t nInternalEdges() const { return m_nInternalEdges; }
		int64_t nPiEdges() const { return m_nPiEdges; }
		int64_t nPoEdges() const { return m_nPoEdges; }
		int64_t nConstEdges() const { return m_nConstEdges; }
		int64_t nSingletonEdges() const { return m_nSingletonEdges; }
		int64_t nCutNets() const { return m_nCutNets; }
		int64_t connectivityCost() const { return m_nConnectivityCost; }
		int64_t crossPartitionPins() const { return m_nCrossPartitionPins; }
		int64_t predictedInterfaces() const { return m_nPredictedInterfaces; }
		const std::vector<int64_t> &partCapacity() const { return m_partCapacity; }
		int64_t partCapacityMax() const;
		double partCapacityCv() const;
		const std::vector<PartitionUnitRec> &units() const { return m_units; }
		const std::vector<HyperedgeRec> &edges() const { return m_edges; }

		// Write pif_hg_unit.tsv / pif_hg_unit_origin.tsv / pif_hg_edge.tsv /
		// pif_hg_summary.tsv under PIF_TELEMETRY_DIR (no-op when unset).
		void exportTelemetry() const;

	private:
		std::vector<PartitionUnitRec> m_units;
		std::vector<HyperedgeRec> m_edges;
		int64_t m_nPinsTotal = 0;
		int64_t m_nInternalEdges = 0;
		int64_t m_nPiEdges = 0;
		int64_t m_nPoEdges = 0;
		int64_t m_nConstEdges = 0;
		int64_t m_nSingletonEdges = 0;
		int64_t m_nCutNets = 0;
		int64_t m_nConnectivityCost = 0;
		int64_t m_nCrossPartitionPins = 0;
		int64_t m_nPredictedInterfaces = 0;
		std::vector<int64_t> m_partCapacity;
	};

} // namespace pif
