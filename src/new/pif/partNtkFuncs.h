#pragma once

#include "base/abc/abc.h"
#include "map/if/if.h"
#include "map/mio/mio.h"
#include "base/main/main.h"
#include "base/io/ioAbc.h"
#include "aig/hop/hop.h"
#include <sys/time.h>
#include "yaig.h"
#include <map>
#include <utility>
#include <cmath>

ABC_NAMESPACE_USING_NAMESPACE

namespace ymc
{
    struct PartitionConfig
    {
        int32_t targetK = 0;
        int64_t avgWorkload = 0;
        int64_t workloadLimit = 0;
    };

    class MetisGraph : public Graph // for partitioning
    {
    public:
        MetisGraph() = delete;
        ~MetisGraph() = default;
        MetisGraph(Abc_Ntk_t *pNtk, bool fMetis);
        void metisGraphInit(int32_t nNodes, bool fMetis);
        int createGraphFromNtk(Abc_Ntk_t *pNtk, bool fMetis);
        void initWeightVector()
        {
            yassert(m_nNodes);
            yassert(!m_vEdges.empty());
            m_vNodeWeights.resize(m_nNodes, 1);
            m_vEdgeWeights.resize(m_vEdges.size(), WALL_EDGE_WEIGHT);
        }

        Abc_Ntk_t *initOneSubNtk(int index);
        int createSubNtksFromPartition(vector<Abc_Ntk_t *> &vSubNtks);

        void tmp();
        int computeNodeWeightByLevel(int leftChildLevel, int rightChildLevel);
        int computeNodeWeightByDegree(int degree);
        int computeEdgeWeight(int faninLevel, int faninReqTime, int level, int reqTime);

        void setEdgeWeight(int fanin, int fanout, int weight);
        void setNodeWeight(int nodeId, int weight)
        {
            yassert(nodeId < m_nNodes);
            yassert(nodeId < m_vNodeWeights.size());
            m_vNodeWeights[nodeId] = weight;
        }
        void setNodePart(int id, int ipart)
        {
            if (m_vPartition[id] == -1)
            {
                m_vPartition[id] = ipart;
                return;
            }
            if (m_vPartition[id] != ipart)
            {
                return;
            }
        }
        void setPoPart();
        void fixUnpartitionedNodes();
        void set_sCluster(uint32_t size) { m_sCluster = size; };
        uint32_t get_sCluster() { return m_sCluster; };

    private:
        const int MAX_NODE_WEIGHT = 1;
        const int MAX_EDGE_WEIGHT_FOR_NODE = 150;
        const int MIN_EDGE_WEIGHT_FOR_NODE = 10;
        const int WALL_EDGE_WEIGHT = 10000; // wall to prevent metis to cut the edge
        const int WALL_THRESHOLD = 20;      // threshold of slack to generate wall
        const int METIS_UFACTOR = 30;
        Abc_Ntk_t *m_pNtkOrigin;
        vector<Abc_Obj_t *> m_vpObjs;    // nodeId -> pObj in origin Ntk
        vector<Abc_Obj_t *> m_vpObjsNew; // nodeId -> pObj in SubNtks
        vector<int32_t> m_vNodeWeights;
        vector<int32_t> m_vEdgeWeights;
        uint32_t m_sCluster;
    };

    class Edge
    {
    public:
        Edge(int32_t in, int32_t out) : iFaninId(in), iFanoutId(out) {};
        int32_t iFaninId;
        int32_t iFanoutId;
    };

    class Cone
    {
    public:
        Cone(int32_t lev, int32_t nodeId) : iMaxLevel(lev), iPoId(nodeId), iId(0) {};
        int32_t iMaxLevel;
        int32_t iPoId;
        int32_t iId;
        vector<Edge> vBoundaryEdges;
        set<int32_t> sAdjacentConeId;
    };

    // ============================================================
    // MFFC-based partition data structures
    // ============================================================

    // An MFFC (Maximum Fanout-Free Cone) unit represents a group of
    // AIG nodes whose logic is dedicated exclusively to driving
    // the root node. Cutting at MFFC boundaries guarantees functional
    // independence between partitions.
    class MffcUnit
    {
    public:
        MffcUnit() : iId(-1), iRootId(-1), iMaxLevel(0), nNodes(0), iWorkload(0) {};
        MffcUnit(int32_t id, int32_t rootId, int32_t maxLevel)
            : iId(id), iRootId(rootId), iMaxLevel(maxLevel), nNodes(0), iWorkload(0) {};

        int32_t iId;       // unique MFFC index
        int32_t iRootId;   // root node ID in Yaig (the node whose MFFC this is)
        int32_t iMaxLevel; // max logic depth of this MFFC
        int32_t nNodes;    // number of AND nodes in this MFFC
        int32_t iWorkload; // estimated workload

        vector<int32_t> vNodeIds;     // all node IDs belonging to this MFFC
        set<int32_t> sAdjacentMffcId; // neighboring MFFC IDs (share a boundary edge)
    };

    class Cluster
    {
    public:
        Cluster() = default;
        Cluster(int32_t id, int32_t lev) : iId(id), iMaxLevel(lev), iWorkload(0), nNodes(0), iPartitionId(-1) {};
        int32_t iId;
        int32_t iMaxLevel;
        int32_t iWorkload;
        int32_t nNodes; // excluding PI/PO
        int32_t iPartitionId;
        vector<int32_t> vConeIds;
    };

    class Partition
    {
    public:
        Partition() : iWorkload(0), nNodes(0) {};
        void addCluster(Cluster &cluster)
        {
            iWorkload += cluster.iWorkload;
            nNodes += cluster.nNodes;
            vClusterIds.push_back(cluster.iId);
        };
        int32_t iWorkload;
        int32_t nNodes;
        vector<int32_t> vClusterIds;
    };

    class MetisAig : public Yaig // for analysing the original NTK
    {
    public:
        MetisAig() : m_pMG(NULL) {};
        ~MetisAig() = default;
        void bindGraph(MetisGraph *pmg);
        const std::vector<int64_t> &getPartitionWorkloads() const { return m_vPartitionWorkload; }

        void parseAig(int32_t userK = 0);
        void parseAigMffc(int32_t userK = 0); // MFFC-based partition entry
        int32_t partitionAigMffc();

        // --- Legacy Cone-based methods ---
        void visitAllFaninFromNode(int32_t nodeId, Cone &cone);
        void findBoundaryEdges(Cone &cone);
        void findBoundaryEdges_rec(Cone &cone, int32_t nodeId, int32_t fCovered);
        void computeWorkLoad(Cluster &cluster);
        int32_t computeWorkLoad_rec(int32_t nodeId, int32_t &workload, int32_t &nNodes);
        int32_t computeConeWorkLoadPowerLaw(int32_t conePoId);
        int32_t decideNumParts();
        int32_t partBiggestClusterByPICut(int32_t clusterId);
        int32_t partBiggestCluster(int32_t clusterId, int32_t workLoadLimit);
        void cutOneBoundaryEdge(Edge &edge);
        int32_t tryPart();
        int32_t tryPart2();
        int32_t partitionAig();
        void mergeSmallClusters(uint32_t size);
        void setGraphPartition(Cluster &cluster, int32_t partId);
        void setNodePartition(int32_t nodeId, int32_t partId);
        void visitConeForEdgeWight(int32_t nodeId, int32_t coneId);

        void printCones();
        void printOneCone(int32_t coneId);
        void printOneCone_rec(int32_t nodeId);
        void printClusters();
        void checkClusters();
        void checkNodeCluster(int32_t nodeId, int32_t clusterId);

    private:
        const int32_t PI_WORK_LOAD = 1;
        const int32_t MAX_N_CUT = 8;

        static constexpr double WL_C = 0.00019046;
        static constexpr double WL_ALPHA = 1.0808;
        static constexpr double WL_BETA = 0.3562;
        static constexpr double WL_GAMMA = 0.0775;
        static constexpr double WL_MS_SCALE = 1000.0;

        const int32_t WL_NODE_BASE = 170;
        const int32_t WL_CUT_CLAMP = 0;
        const int32_t WL_FANOUT_WEIGHT = 80;
        const int32_t WL_PI_BASE = 5;
        const int32_t WL_PI_FANOUT_WEIGHT = 3;
        const int32_t MIN_N_PART = 2;
        const int32_t MAX_N_PART = 20;
        const int32_t METIS_N_PART = 4;
        const int32_t CRITICAL_PATH_FACTOR = 50;

        MetisGraph *m_pMG;
        vector<Cone> m_vCones;
        vector<Cluster> m_vClusters;
        int64_t m_iTotalWorkLoad;
        int64_t m_iMaxClusterWorkLoad;

        // Task 16 Stage 3 telemetry: per-partition predicted workload in
        // graph partition order (filled by partitionAigMffc/partitionAig;
        // read only by telemetry, never by selection code).
        std::vector<int64_t> m_vPartitionWorkload;

        vector<int> m_vConeId2ClusterId;

        // ============================================================
        // MFFC-based partition members
        // ============================================================
        vector<MffcUnit> m_vMffcs;       // all MFFC units
        vector<int32_t> m_vNode2MffcId;  // node ID -> MFFC ID (-1 if PI/PO/unassigned)
        vector<int> m_vMffcId2ClusterId; // MFFC ID -> Cluster ID
        bool m_useMffc = false;          // flag: using MFFC or legacy Cone
        int32_t m_forceK = 0;            // non-zero: user requests exactly K partitions

        // MFFC identification
        void identifyMffcs(); // identify all MFFCs in the Yaig
        int32_t computeMffcSize_rec(int32_t nodeId, vector<int32_t> &vMffcNodes);
        void computeMffcAdjacency(); // build adjacency between MFFCs
        int32_t computeMffcWorkload(const MffcUnit &mffc);

        // MFFC-based clustering (mirrors Cone-based but uses MffcUnit)
        void preprocessMffcs(vector<int32_t> &outMffcWorkloads);
        PartitionConfig determineMffcPartitionConfig(const vector<int32_t> &mffcWorkloads, int32_t userK);
        int32_t computeAdaptiveMffcTargetK(const vector<int32_t> &mffcWorkloads);
        void runMffcClusteringAlgorithm(const PartitionConfig &config, const vector<int32_t> &mffcWorkloads);
        int findBestClusterForMffc(int mffcIdx, const PartitionConfig &config,
                                   const vector<int> &mffcId2ClusterId,
                                   const vector<int32_t> &mffcWorkloads,
                                   double *pOutScore = nullptr);
        void assignOrphanMffc(int mffcIdx, vector<int> &mffcId2ClusterId,
                              const vector<int32_t> &mffcWorkloads);
        void postProcessMffcClusters();
        void rebalanceMffcClusters(vector<int32_t> &mffcWorkloads);
        void splitOversizedMffcClusters();

        // MFFC-based partition assignment
        void setGraphPartitionMffc(Cluster &cluster, int32_t partId);

        // ============================================================

        int32_t computeAdaptiveTargetK(const vector<int32_t> &coneWorkloads);
        void preprocessCones(vector<int32_t> &outConeWorkloads);
        void preprocessCones_fast(vector<int32_t> &outConeWorkloads);
        void splitOversizedCones(vector<int32_t> &coneWorkloads, int64_t maxAllowedWorkload);
        PartitionConfig determinePartitionConfig(const vector<int32_t> &coneWorkloads, int32_t userK);
        void runClusteringAlgorithm(const PartitionConfig &config, const vector<int32_t> &coneWorkloads);
        void postProcessClusters();

        int findBestClusterForCone(int coneIdx, const PartitionConfig &config,
                                   const vector<int> &coneId2ClusterId,
                                   const vector<int32_t> &coneWorkloads);
        void assignOrphanCone(int coneIdx, vector<int> &coneId2ClusterId,
                              const vector<int32_t> &coneWorkloads);

        void collectConeSimpleStats_rec(int32_t nodeId,
                                        int32_t &nNodes,
                                        int64_t &totalFanout,
                                        int32_t &maxLevel);

        void rebalanceClusters(vector<int32_t> &coneWorkloads);
        void splitOversizedClusters();
    };

    Abc_Ntk_t *Abc_NtkMerge(Abc_Ntk_t *pNtk, Vec_Ptr_t *pSubNtksOld, Vec_Ptr_t *pSubNtksNew);
    Abc_Ntk_t *Abc_NtkExtractCriticalPath(Abc_Ntk_t *pNtk, Abc_Obj_t *pCos);
    Abc_Obj_t *Abc_NtkPickCriticalPo(Abc_Ntk_t *pNtk);
    Abc_Ntk_t *Abc_NtkMergeMapped(Abc_Ntk_t *pNtkSkeleton, Vec_Ptr_t *pSubNtksRef, Vec_Ptr_t *pSubNtksMapped);

} // for namespace