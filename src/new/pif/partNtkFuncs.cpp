#include "partNtkFuncs.h"
#include <unistd.h>
#include <climits>
#include <unordered_map>
#include <string>
#include <cstdio>
#include <sys/types.h>

namespace pif
{

	MetisGraph::MetisGraph(Abc_Ntk_t *pNtk, bool fMetis)
	{
		yassert(Abc_NtkIsStrash(pNtk) && Abc_NtkHasAig(pNtk));
		int32_t nNodes = Abc_NtkObjNum(pNtk) - Abc_NtkBoxNum(pNtk) - 1; // the number of valid nodes

		metisGraphInit(nNodes, fMetis);
		createGraphFromNtk(pNtk, fMetis);
	}

	void MetisGraph::metisGraphInit(int32_t nNodes, bool fMetis)
	{
		m_nNodes = nNodes;
		m_nCon = 1;
		m_nPis = 0;
		m_vNodeIndices.reserve(nNodes + 2);
		m_vNodeIsPi.reserve(nNodes + 1);
		m_nParts = 0;
		m_nCutEdges = 0;
		m_vPartition.resize(nNodes + 1, -1);
		m_vpObjs.reserve(nNodes + 1);
		m_vpObjsNew.reserve(nNodes + 1);
		m_vNodeIndices.push_back(0);

		if (fMetis)
		{
			m_vNodeWeights.reserve(nNodes + 1);
			m_vEdgeWeights.reserve(4 * nNodes);
		}

		m_vEdges.reserve(4 * nNodes);
		m_vEdgeIsC.reserve(4 * nNodes);
	}

	int MetisGraph::computeNodeWeightByLevel(int leftChildLevel, int rightChildLevel)
	{
		int res = (leftChildLevel > rightChildLevel) ? leftChildLevel : rightChildLevel;
		if (res > MAX_NODE_WEIGHT - 1)
			res = MAX_NODE_WEIGHT - 1;
		// return 100;
		return res + 1;
	}

	int MetisGraph::computeNodeWeightByDegree(int degree)
	{
		int res = degree;
		if (res > MAX_NODE_WEIGHT)
			res = MAX_NODE_WEIGHT;
		return res;
	}

	int MetisGraph::computeEdgeWeight(int faninLevel, int faninReqTime, int level, int reqTime)
	{
		if (faninLevel == 0)
			return 1;
		if ((reqTime - level < WALL_THRESHOLD) || (faninReqTime - faninLevel < WALL_THRESHOLD))
			return WALL_EDGE_WEIGHT;
		int res = MAX_EDGE_WEIGHT_FOR_NODE;
		int slack = level - 1 - faninLevel;
		if (res - slack * 10 > MIN_EDGE_WEIGHT_FOR_NODE)
			res -= slack * 10;
		else
			res = MIN_EDGE_WEIGHT_FOR_NODE;
		return res;
	}

	void MetisGraph::setEdgeWeight(int fanin, int fanout, int weight)
	{
		yassert(fanin < fanout);
		yassert(fanout < m_nNodes);
		yassert(getNodeFanin0Id(fanout) == fanin || getNodeFanin1Id(fanout) == fanin);
		int nEdges = getNodeEdgeNum(fanin);
		int iEdge = m_vNodeIndices[fanin];
		bool status = 0;
		for (int i = 0; i < nEdges; i++)
		{
			if (m_vEdges[iEdge + i] == fanout)
			{
				m_vEdgeWeights[iEdge + i] = weight;
				status = 1;
				break;
			}
		}
		yassert(status);
		iEdge = m_vNodeIndices[fanout];
		if (getNodeFanin0Id(fanout) == fanin)
			m_vEdgeWeights[iEdge] = weight;
		else if (getNodeFanin1Id(fanout) == fanin)
			m_vEdgeWeights[iEdge + 1] = weight;
		else
			yassert(0);
	}

	int MetisGraph::createGraphFromNtk(Abc_Ntk_t *pNtk, bool fMetis)
	{
		Abc_Obj_t *pObj;
		Abc_Obj_t *pFanin0, *pFanin1, *pFanout;
		Vec_Ptr_t *vNodes;
		int iNode = 0, iEdgeBeg = 0, iEdgeEnd = 0;
		int iEdge;
		int i, j, weight;
		int maxLevel;
		Abc_NtkFillTemp(pNtk);
		map<int, unsigned int> nodeId2ReqTime;

		m_pNtkOrigin = pNtk;

		if (fMetis)
		{
			maxLevel = Abc_NtkLevelReverse(pNtk);
			Abc_NtkForEachNode(pNtk, pObj, i)
				nodeId2ReqTime.emplace(make_pair(pObj->Id, maxLevel + 1 - pObj->Level)); // BUG?: 2021/6/12: maxLevel - level!!!
			Abc_NtkLevel(pNtk);															 // mark level for each obj in pNtk
		}

		// Const1
		pObj = Abc_AigConst1(pNtk);
		iEdgeEnd += Abc_ObjFanoutNum(pObj);
		if (iEdgeEnd > iEdgeBeg) // check if the constant node is isolated
		{
			// printf("Const1 node added to the network.\n");
			pObj->iTemp = iNode++; // the corresponding index of the node in Metis Graph
			m_vNodeIsPi.push_back(true);
			m_vpObjs.push_back(pObj);
			m_vpObjsNew.push_back(NULL);
			m_vNodeIndices.push_back(iEdgeEnd);
			iEdgeBeg = iEdgeEnd;
			m_nNodes++;
			m_nPis++;

			if (fMetis)
				m_vNodeWeights.push_back(1);
		}
		// CIs
		Abc_NtkForEachCi(pNtk, pObj, i)
		{
			iEdgeEnd += Abc_ObjFanoutNum(pObj);
			if (iEdgeEnd > iEdgeBeg)
			{
				pObj->iTemp = iNode++;
				m_vNodeIsPi.push_back(true);
				m_vpObjs.push_back(pObj);
				m_vpObjsNew.push_back(NULL);
				m_vNodeIndices.push_back(iEdgeEnd);
				iEdgeBeg = iEdgeEnd;
				m_nPis++;

				if (fMetis)
					m_vNodeWeights.push_back(1);
			}
			else
			{
				// printf("Dangling PI detected.\n");
				m_nNodes--;
			}
		}
		// internal nodes
		vNodes = Abc_NtkDfsIter(pNtk, 0);
		Vec_PtrForEachEntry(Abc_Obj_t *, vNodes, pObj, i)
		{
			iEdgeEnd += Abc_ObjFaninNum(pObj);
			if (iEdgeEnd - iEdgeBeg != 2)
			{
				Abc_Print(-1, "An internal AIG node is supposed to have 2 fanins.\n ");
				return 1;
			}
			iEdgeBeg = iEdgeEnd;
			iEdgeEnd += Abc_ObjFanoutNum(pObj);
			if (iEdgeEnd == iEdgeBeg)
			{
				Abc_Print(-1, "An internal AIG node is supposed to have at least 1 fanout.\n ");
				return 1;
			}
			pObj->iTemp = iNode++;
			m_vNodeIsPi.push_back(false);
			m_vpObjs.push_back(pObj);
			m_vpObjsNew.push_back(NULL);
			m_vNodeIndices.push_back(iEdgeEnd);
			iEdgeBeg = iEdgeEnd;

			pFanin0 = Abc_ObjFanin0(pObj);
			pFanin1 = Abc_ObjFanin1(pObj);

			if (fMetis)
			{
				weight = computeNodeWeightByLevel(pFanin0->Level, pFanin1->Level);
				// weight = computeNodeWeightByDegree(Abc_ObjFanoutNum(pObj));
				m_vNodeWeights.push_back(weight);
			}
		}
		Vec_PtrFree(vNodes);
		// COs
		Abc_NtkForEachCo(pNtk, pObj, i)
		{
			iEdgeEnd += Abc_ObjFaninNum(pObj);
			if (iEdgeEnd - iEdgeBeg != 1)
			{
				Abc_Print(-1, "A Co is supposed to have 1 fanin.\n ");
				return 1;
			}
			pObj->iTemp = iNode++;
			m_vNodeIsPi.push_back(false);
			m_vpObjs.push_back(pObj);
			m_vpObjsNew.push_back(NULL);
			m_vNodeIndices.push_back(iEdgeEnd);
			iEdgeBeg = iEdgeEnd;

			if (fMetis)
				m_vNodeWeights.push_back(1);
		}
		/*wxx
		Vec_PtrForEachEntry(Abc_Obj_t *, pMetis->AbcObj , pObj, i)
		{
			printf("i: %d\ttype: %d\n", i ,Abc_ObjType(pObj));
		}
		*/

		yassert(m_nNodes == m_vNodeIndices.size() - 1);
		yassert(m_nNodes == m_vNodeIsPi.size());
		yassert(m_nNodes == m_vpObjs.size());
		yassert(m_nNodes == m_vpObjsNew.size());
		yassert(m_nNodes == iNode);
		//	yassert(m_nNodes == m_vNodeWeights.size());

		Abc_Print(-2, "\nThere are %d nodes in the initial graph to be partitioned.\n", m_nNodes);

		// add edges (m_vEdgeIsC is not prepared)
		for (i = 0; i < m_vpObjs.size(); i++)
		{
			pObj = m_vpObjs[i];
			if (isNodePi(i))
			{
				iEdge = m_vNodeIndices[i];
				Abc_ObjForEachFanout(pObj, pFanout, j)
				{
					m_vEdges.push_back(pFanout->iTemp);
					m_vEdgeIsC.push_back(false); // only first two edge for each non-Pi node has compl info
					iEdge++;

					if (fMetis)
						m_vEdgeWeights.push_back(1);
				}
				yassert(iEdge == m_vNodeIndices[i + 1]);
			}
			else if (isNodeAND(i))
			{
				iEdge = m_vNodeIndices[i];
				pFanin0 = Abc_ObjFanin0(pObj);
				m_vEdges.push_back(pFanin0->iTemp);
				m_vEdgeIsC.push_back(pObj->fCompl0);

				if (fMetis)
				{
					weight = computeEdgeWeight(pFanin0->Level, nodeId2ReqTime[pFanin0->Id], pObj->Level, nodeId2ReqTime[pObj->Id]);
					m_vEdgeWeights.push_back(weight);
				}

				pFanin1 = Abc_ObjFanin1(pObj);
				m_vEdges.push_back(pFanin1->iTemp);
				m_vEdgeIsC.push_back(pObj->fCompl1);

				if (fMetis)
				{
					weight = computeEdgeWeight(pFanin1->Level, nodeId2ReqTime[pFanin1->Id], pObj->Level, nodeId2ReqTime[pObj->Id]);
					m_vEdgeWeights.push_back(weight);
				}

				iEdge += 2;
				Abc_ObjForEachFanout(pObj, pFanout, j)
				{
					m_vEdges.push_back(pFanout->iTemp);
					m_vEdgeIsC.push_back(false);
					if (fMetis)
					{
						weight = computeEdgeWeight(pObj->Level, nodeId2ReqTime[pObj->Id], pFanout->Level, nodeId2ReqTime[pFanout->Id]);
						m_vEdgeWeights.push_back(weight);
					}
					iEdge++;
				}
				yassert(iEdge == m_vNodeIndices[i + 1]);
			}
			else if (isNodePo(i))
			{
				iEdge = m_vNodeIndices[i];
				pFanin0 = Abc_ObjFanin0(pObj);
				m_vEdges.push_back(pFanin0->iTemp);
				m_vEdgeIsC.push_back(pObj->fCompl0);
				iEdge++;
				// weight = computeEdgeWeight(pFanin0->Level, nodeId2ReqTime[pFanin0->Id], pObj->Level, nodeId2ReqTime[pObj->Id]);
				if (fMetis)
					m_vEdgeWeights.push_back(WALL_EDGE_WEIGHT);
				yassert(iEdge == m_vNodeIndices[i + 1]);
			}
			else
				yassert(0);
		}
		yassert(m_vEdges.size() == m_vEdgeIsC.size());
		// yassert(m_vEdges.size() == m_vEdgeWeights.size());

		ylog("Total number of edges in MetisGraph: %ld\n", m_vEdges.size() / 2);
		return 0;
	}

	/**Function*************************************************************

	  Synopsis    [Start a subnetwork using exsiting network.]

	  Description [Duplicate CI/COs from original network, leaving out internal nodes and newly generated PI/POs.]

	  SideEffects []

	  SeeAlso     []

	***********************************************************************/
	Abc_Ntk_t *MetisGraph::initOneSubNtk(int index)
	{
		Abc_Ntk_t *pNtkNew;
		Abc_Obj_t *pObj;
		int i, ipart;
		// int fCopyNames;

		if (m_pNtkOrigin == NULL)
			return NULL;
		yassert(index >= 0 && index < m_nParts);
		// decide whether to copy the names
		// fCopyNames = (pNtk->ntkType != ABC_NTK_NETLIST);

		// start the network
		pNtkNew = Abc_NtkAlloc(m_pNtkOrigin->ntkType, m_pNtkOrigin->ntkFunc, 1);
		pNtkNew->nConstrs = m_pNtkOrigin->nConstrs;
		pNtkNew->nBarBufs = m_pNtkOrigin->nBarBufs;

		// zkx
		pNtkNew->ntkType = m_pNtkOrigin->ntkType;

		//
		// duplicate the name and the spec
		pNtkNew->pName = Extra_UtilStrsav(m_pNtkOrigin->pName);
		pNtkNew->pSpec = Extra_UtilStrsav(m_pNtkOrigin->pSpec);
		// clone CI/CO Nodes
		for (i = 0; i < m_nNodes; i++)
		{
			ipart = m_vPartition[i];
			if (ipart != index)
				continue;
			pObj = m_vpObjs[i];
			if (Abc_ObjType(pObj) == ABC_OBJ_NODE)
				continue;
			// yassert( pObj->iTemp == i );
			if (Abc_ObjType(pObj) == ABC_OBJ_CONST1)
				pObj->pCopy = Abc_AigConst1(pNtkNew);
			else if (Abc_ObjIsCi(pObj))
			{
				pObj->pCopy = Abc_NtkCreatePi(pNtkNew);
				Abc_ObjAssignName(pObj->pCopy, Abc_ObjName(pObj), NULL);
				yassert(pObj->Level == 0);
				pObj->pCopy->Level = pObj->Level;
			}
			else if (Abc_ObjIsCo(pObj))
			{
				pObj->pCopy = Abc_NtkCreatePo(pNtkNew);
				Abc_ObjAssignName(pObj->pCopy, Abc_ObjName(pObj), NULL);
			}
			else
				yassert(0);
			pObj->pCopy->fCompl0 = pObj->fCompl0;
			pObj->pCopy->fCompl1 = pObj->fCompl1;
			// store the newly created AbcObj for corresponding node in Metis graph
			m_vpObjsNew[i] = pObj->pCopy;
		}
		Abc_ManTimeDup(m_pNtkOrigin, pNtkNew);
		pNtkNew->AndGateDelay = m_pNtkOrigin->AndGateDelay;

		if (pNtkNew->pManTime && Abc_FrameReadLibGen() && pNtkNew->AndGateDelay == 0.0)
		{
			pNtkNew->AndGateDelay = Mio_LibraryReadDelayAigNode((Mio_Library_t *)Abc_FrameReadLibGen());
		}
		return pNtkNew;
	}

	/**Function*************************************************************

	  Synopsis    [Generate subnetworks using hmetis partition results.]

	  Description [Create internal AND nodes and new PI/PO pairs for edges that straddle different partitions in DFS order(original pMetis->AbcObj order). Finish the connections as well.]

	  SideEffects []

	  SeeAlso     []

	***********************************************************************/
	int MetisGraph::createSubNtksFromPartition(vector<Abc_Ntk_t *> &vSubNtks)
	{
		Abc_Obj_t *pObj, *pFanin0, *pFanin1, *pFanout;
		Abc_Obj_t *pObjNew, *pPiNew, *pPoNew;
		Abc_Ntk_t *pNtk;
		int iFanin0, iFanin1, ipart, ipart0, ipart1;
		int i, j;
		cout << "vSubNtks.size() is " << vSubNtks.size() << " , m_nParts is " << m_nParts << endl;

		yassert(vSubNtks.size() == m_nParts);
		int nCutEdgeFromPi = 0;
		int nCutEdge = 0;

		// Task 17 Stage 2 ground truth: per signal, the producer/owner part
		// and the deduped set of consumer parts that actually received an
		// interface PI/PO in the emitted child networks. Read only by the
		// telemetry hook below; never affects selection.
		std::map<int32_t, std::pair<int32_t, std::set<int32_t>>> signalCuts;
		auto recordCut = [&](int32_t signalNodeId, int32_t consumerPart)
		{
			if (!pifTelemetryDir())
				return;
			auto &rec = signalCuts[signalNodeId];
			if (rec.second.empty())
				rec.first = m_vPartition[signalNodeId];
			rec.second.insert(consumerPart);
		};

		for (i = 0; i < vSubNtks.size(); i++)
		{
			vSubNtks[i] = initOneSubNtk(i);
		}

		// [优化] 引入缓存 Map，避免重复创建 PI
		// vPartPiMap[PartID][OriginalNodeID] -> Created_PI_Obj
		vector<map<int, Abc_Obj_t *>> vPartPiMap(m_nParts);

		// DFS order is expected.
		for (i = 0; i < m_vpObjs.size(); i++)
		{
			pObj = m_vpObjs[i];
			if (Abc_ObjIsCi(pObj) || pObj->Type == ABC_OBJ_CONST1)
				continue;
			ipart = m_vPartition[i];
			pNtk = vSubNtks[ipart]; // the pSubNtk that the current pObj belongs to

			// check fanin0 for CO and And nodes
			iFanin0 = getNodeFanin0Id(i);
			pFanin0 = m_vpObjsNew[iFanin0]; // pObj in pSubNtk
			if (pFanin0 == NULL)
			{
				Abc_Print(-1, "The AIG Nodes in metis array are expected to be in DFS order.\n ");
				return 1;
			}
			ipart0 = m_vPartition[iFanin0];
			if (ipart0 != ipart) // cut edge found!
			{
				nCutEdge++;
				recordCut(iFanin0, ipart);
				if (Abc_ObjIsPi(pFanin0)) // when the Ci has a fanout edge that is cut, don't generate cut-caused PO
				{
					nCutEdgeFromPi++;

					// [优化] 检查是否已为该 Fanin 创建过 PI
					if (vPartPiMap[ipart].count(iFanin0))
					{
						pPiNew = vPartPiMap[ipart][iFanin0];
					}
					else
					{
						pPiNew = Abc_NtkCreatePi(pNtk);
						Abc_ObjAssignName(pPiNew, Abc_ObjName(pPiNew), NULL);
						pPiNew->fMarkA = 1;
						pPiNew->pData = pFanin0;
						vPartPiMap[ipart][iFanin0] = pPiNew; // 记录
					}
					pFanin0 = pPiNew;
				}
				else
				{
					pPoNew = NULL;
					Abc_ObjForEachFanout(pFanin0, pFanout, j)
					{
						// If cut-caused PO already exists, use it directly, avoiding duplication.
						if (Abc_ObjIsPo(pFanout) && pFanout->fMarkA == 1)
						{
							pPoNew = pFanout;
							break;
						}
					}
					if (pPoNew == NULL) // create a new cut-caused PO
					{
						pPoNew = Abc_NtkCreatePo(vSubNtks[ipart0]);
						Abc_ObjAssignName(pPoNew, Abc_ObjName(pPoNew), NULL); // assign a temporary name
						pPoNew->fMarkA = 1;									  // mark as cut-caused PO
						Abc_ObjAddFanin(pPoNew, pFanin0);
					}

					// [优化] 检查是否已为该 Fanin 创建过 PI
					if (vPartPiMap[ipart].count(iFanin0))
					{
						pPiNew = vPartPiMap[ipart][iFanin0];
					}
					else
					{
						pPiNew = Abc_NtkCreatePi(pNtk);
						Abc_ObjAssignName(pPiNew, Abc_ObjName(pPiNew), NULL);
						pPiNew->fMarkA = 1;
						pPiNew->pData = pPoNew;
						vPartPiMap[ipart][iFanin0] = pPiNew; // 记录
					}
					pFanin0 = pPiNew;
				}
			}
			if (Abc_ObjIsCo(pObj))
			{
				pObjNew = m_vpObjsNew[i]; // pObjNew of CI/CO in pSubNtk has been added in Abc_SubNtkStartFrom()
				yassert(pObjNew != NULL);
				Abc_ObjAddFanin(pObjNew, pFanin0); // Connect
			}
			else if (Abc_ObjIsNode(pObj))
			{
				// fanin1
				iFanin1 = getNodeFanin1Id(i);
				pFanin1 = m_vpObjsNew[iFanin1]; // pObj in pSubNtk. In DFS, fanin should have been processed
				yassert(pFanin1);
				if (pFanin1 == NULL)
				{
					Abc_Print(-1, "The AIG Nodes in metis array are expected to be in DFS order.\n ");
					return 1;
				}
				ipart1 = m_vPartition[iFanin1];
				if (ipart1 != ipart)
				{
					nCutEdge++;
					recordCut(iFanin1, ipart);
					if (Abc_ObjIsPi(pFanin1)) // when the Ci has a fanout edge that is cut, don't generate cut-caused PO
					{
						nCutEdgeFromPi++;

						// [优化] 检查是否已为该 Fanin 创建过 PI
						if (vPartPiMap[ipart].count(iFanin1))
						{
							pPiNew = vPartPiMap[ipart][iFanin1];
						}
						else
						{
							pPiNew = Abc_NtkCreatePi(pNtk);
							Abc_ObjAssignName(pPiNew, Abc_ObjName(pPiNew), NULL);
							pPiNew->fMarkA = 1;
							pPiNew->pData = pFanin1;
							vPartPiMap[ipart][iFanin1] = pPiNew; // 记录
						}
						pFanin1 = pPiNew;
					}
					else
					{
						pPoNew = NULL;
						Abc_ObjForEachFanout(pFanin1, pFanout, j)
						{
							if (Abc_ObjIsPo(pFanout) && pFanout->fMarkA == 1)
							{
								pPoNew = pFanout;
								break;
							}
						}
						if (pPoNew == NULL)
						{
							pPoNew = Abc_NtkCreatePo(vSubNtks[ipart1]);
							Abc_ObjAssignName(pPoNew, Abc_ObjName(pPoNew), NULL);
							pPoNew->fMarkA = 1;
							Abc_ObjAddFanin(pPoNew, pFanin1);
						}

						// [优化] 检查是否已为该 Fanin 创建过 PI
						if (vPartPiMap[ipart].count(iFanin1))
						{
							pPiNew = vPartPiMap[ipart][iFanin1];
						}
						else
						{
							pPiNew = Abc_NtkCreatePi(pNtk);
							Abc_ObjAssignName(pPiNew, Abc_ObjName(pPiNew), NULL);
							pPiNew->fMarkA = 1;
							pPiNew->pData = pPoNew;
							vPartPiMap[ipart][iFanin1] = pPiNew; // 记录
						}
						pFanin1 = pPiNew;
					}
				}

				pObjNew = Abc_AigAnd(static_cast<Abc_Aig_t *>(pNtk->pManFunc), Abc_ObjNotCond(pFanin0, isNodeFanin0C(i)), Abc_ObjNotCond(pFanin1, isNodeFanin1C(i)));

				m_vpObjsNew[i] = pObjNew;
				pObj->pCopy = pObjNew;
			}
			else
				yassert(0);
		}

		ylog("Total number of CIs with cut fanout edges: %d\n", nCutEdgeFromPi);
		ylog("Total number of cut edges: %d\n", nCutEdge);

		// Task 17 Stage 2: emit the node-level interface ground truth.
		if (pifTelemetryDir())
		{
			char buf[512];
			for (const auto &kv : signalCuts)
			{
				int32_t sig = kv.first;
				const char *type = "const";
				if (sig >= 0 && sig < (int32_t)m_vpObjs.size() && m_vpObjs[sig])
				{
					if (Abc_ObjIsNode(m_vpObjs[sig]))
						type = "internal";
					else if (Abc_ObjIsPi(m_vpObjs[sig]))
						type = "pi";
				}
				std::string parts;
				for (int32_t p : kv.second.second)
				{
					if (!parts.empty())
						parts += ",";
					parts += std::to_string(p);
				}
				snprintf(buf, sizeof(buf), "%d\t%s\t%d\t%s\t%zu",
						 sig, type, (int)kv.second.first,
						 parts.c_str(), kv.second.second.size());
				pifTelemetryRow("pif_signal_cut.tsv",
								"signalNodeId\ttype\tproducerPart\tconsumerParts\tnConsumerParts",
								buf);
			}
		}

		return 0;
	}

	void MetisGraph::tmp()
	{
		Yaig y;
		struct timeval t1, t2;
		double time;
		gettimeofday(&t1, NULL);
		y.addSubGraph(*this);
		gettimeofday(&t2, NULL);
		time = t2.tv_sec - t1.tv_sec + (t2.tv_usec - t1.tv_usec) / 1000000.0;
		printf("yaig::addSubGraph spent time: %f\n", time);
	}

	void MetisGraph::setPoPart()
	{
		for (int i = 0; i < m_nNodes; i++)
		{
			if (m_vPartition[i] == -1)
			{
				if (!isNodePo(i))
				{
					if (isNodePi(i))
						ylog("Pi has no partition!!\n");
					else
						ylog("AND node[%d] has no partition!!\n", i);
					yassert(0);
				}
				int faninId = getNodeFanin0Id(i);
				yassert(m_vPartition[faninId] != -1);
				m_vPartition[i] = m_vPartition[faninId];
			}
		}
		return;
	}

	void MetisGraph::fixUnpartitionedNodes()
	{
		int fixedCount = 0;
		for (int i = 0; i < m_nNodes; i++)
		{
			if (m_vPartition[i] != -1)
				continue;
			if (isNodePo(i))
				continue; // PO handled by setPoPart

			// Try to inherit partition from a neighbor (fanout)
			int32_t assignedPart = -1;
			for (int e = m_vNodeIndices[i]; e < m_vNodeIndices[i + 1]; e++)
			{
				int32_t neighbor = m_vEdges[e];
				if (m_vPartition[neighbor] != -1)
				{
					assignedPart = m_vPartition[neighbor];
					break;
				}
			}

			// Fallback: assign to partition 0
			if (assignedPart == -1)
				assignedPart = 0;

			m_vPartition[i] = assignedPart;
			fixedCount++;
		}

		if (fixedCount > 0)
			ylog("[MFFC-Fix] Assigned partition to %d unpartitioned nodes\n", fixedCount);
	}

	void MetisAig::bindGraph(MetisGraph *pmg)
	{
		if (m_pMG)
		{
			clear();
			vector<Cone>().swap(m_vCones);
			vector<Cluster>().swap(m_vClusters);
			vector<int>().swap(m_vConeId2ClusterId);
			m_pMG = NULL;
		}
		addSubGraph(*pmg);
		m_pMG = pmg;
		return;
	}

	void MetisAig::printCones()
	{
		ylog("There are %ld cones\n", m_vCones.size());
#if 0
	for(auto& c : m_vCones)
	{
		if(c.vBoundaryEdges.size())
			ylog("m_vCones[%d] has %ld boundary edges, level: %d\n",c.iId, c.vBoundaryEdges.size(), c.iMaxLevel);
		for(auto e : c.vBoundaryEdges)
			ylog("edge: %d -> %d\n", e.iFaninId, e.iFanoutId);
		printOneCone(c.iId);
	}
#endif
	}

	void MetisAig::printOneCone(int32_t coneId)
	{
		ylog("Now print m_vCones[%d]:\n", coneId);
		auto &cone = m_vCones[coneId];
		ylog("iMaxLevel = %d, iPoId = %d, iId = %d\n", cone.iMaxLevel, cone.iPoId, cone.iId);
		ylog("NumBoundaryEdges = %ld, NumNeighborCone = %ld\n", cone.vBoundaryEdges.size(), cone.sAdjacentConeId.size());
	}

	void MetisAig::printClusters()
	{
		ylog("There are %ld clusters\n", m_vClusters.size());
		for (int i = 0; i < m_vClusters.size(); i++)
			// std::min(10, m_vClusters.size())
			ylog("cluster[%d]: workload = %d\tiMaxLevel = %d\tnNodes = %d\tnCones = %ld\n", i, m_vClusters[i].iWorkload, m_vClusters[i].iMaxLevel, m_vClusters[i].nNodes, m_vClusters[i].vConeIds.size());
#if 0
	for(auto& c : m_vClusters)
	{
		if(c.vConeIds.size())
			ylog("this cluster has %ld cones\n", c.vConeIds.size());
		for(auto coneId : c.vConeIds)
			ylog("coneId: %d\n", coneId);
	}
#endif
	}

	void MetisAig::parseAig(int32_t userK)
	{
		// 1. 预处理：构建 Cone，计算基础 Workload
		vector<int32_t> coneWorkloads;
		preprocessCones_fast(coneWorkloads);

		// 1.5 拆分超大 Cone (消除长尾)
		// 阈值: 总 workload 的 2%, 即如果一个 cone 超过总量的 2%,
		// 它会成为并行瓶颈, 需要拆分。
		// 比如 dft 总 workload 180 亿, 阈值约 3.6 亿。
		int64_t splitThreshold = m_iTotalWorkLoad / 50;
		if (splitThreshold < 1000000)
			splitThreshold = 1000000; // 最低门槛, 避免小电路过度拆分

		int nConesBeforeSplit = (int)m_vCones.size();
		// 在 splitOversizedCones 调用前加:
		int32_t maxConeWL = *std::max_element(coneWorkloads.begin(), coneWorkloads.end());
		ylog("[Debug] Before split: max cone workload = %d, threshold = %ld, nCones = %d\n",
			 maxConeWL, splitThreshold, (int)m_vCones.size());
		splitOversizedCones(coneWorkloads, splitThreshold);

		if ((int)m_vCones.size() > nConesBeforeSplit)
		{
			ylog("[ConeSplit] Cones: %d -> %d (split %d oversized cones)\n",
				 nConesBeforeSplit, (int)m_vCones.size(),
				 (int)m_vCones.size() - nConesBeforeSplit);
			ylog("[ConeSplit] New total workload: %ld\n", m_iTotalWorkLoad);
		}

		// 2. 决策：计算 K 和 Workload 限制
		PartitionConfig config = determinePartitionConfig(coneWorkloads, userK);

		// 3. 核心：执行聚类算法 (Seeding + Growing)
		runClusteringAlgorithm(config, coneWorkloads);

		// 4. 后处理：清理、重排、合并、统计
		postProcessClusters();
	}

	void MetisAig::splitOversizedCones(vector<int32_t> &coneWorkloads,
									   int64_t maxAllowedWorkload)
	{
		// 反复扫描, 直到没有超大 cone
		// 每轮拆分可能产生新 cone, 新 cone 可能还是超大的, 所以要循环
		bool changed = true;
		int totalSplits = 0;

		while (changed)
		{
			changed = false;
			int nCones = (int)m_vCones.size();

			for (int i = 0; i < nCones; i++)
			{
				if (coneWorkloads[i] <= maxAllowedWorkload)
					continue;

				auto &cone = m_vCones[i];

				// 找这个 cone 的所有 boundary edges 中可以切的
				// 条件: boundary edge 的 fanin 不属于本 cone
				// (和 partBiggestCluster 的逻辑相同)
				vector<Edge> cuttableEdges;
				for (auto &edge : cone.vBoundaryEdges)
				{
					auto &fanin = m_vNodes[edge.iFaninId];
					auto &fanout = m_vNodes[edge.iFanoutId];

					// fanin 属于本 cone (iConeId == cone.iId) → 不能切 (会破坏 cone 内部)
					if (fanin.iConeId == cone.iId)
						continue;
					// fanout 是 PO → 不切
					if (fanout.isPo())
						continue;

					cuttableEdges.push_back(edge);
				}

				if (cuttableEdges.empty())
					continue; // 无法拆分这个 cone

				// 按 level 差排序: 优先切 level 差大的边 (影响小)
				sort(cuttableEdges.begin(), cuttableEdges.end(),
					 [this](const Edge &a, const Edge &b)
					 {
						 int diffA = abs(m_vNodes[a.iFanoutId].iLevel -
										 m_vNodes[a.iFaninId].iLevel);
						 int diffB = abs(m_vNodes[b.iFanoutId].iLevel -
										 m_vNodes[b.iFaninId].iLevel);
						 return diffA > diffB;
					 });

				// 切割: 逐个切 boundary edge, 直到 workload 降到阈值以下
				// 每切一刀, cone 的一些节点就被截断 (变成新 PI),
				// 被截断的部分会在后续的 Cone 重建中归属到其他 cone
				int nCutEdges = 0;
				for (auto &edge : cuttableEdges)
				{
					cutOneBoundaryEdge(edge);
					nCutEdges++;

					// 切了一定数量的边后停止, 避免过度碎片化
					// 策略: 切 boundary edges 总数的一半
					if (nCutEdges >= (int)cuttableEdges.size() / 2 + 1)
						break;
				}

				if (nCutEdges == 0)
					continue;

				// 更新 cone 的 level 和 workload
				cone.iMaxLevel = computeNodeLevel(m_vNodes[cone.iPoId].getFanin0Id());

				int32_t newWorkload = computeConeWorkLoadPowerLaw(cone.iPoId);

				// 更新总 workload
				m_iTotalWorkLoad = m_iTotalWorkLoad - coneWorkloads[i] + newWorkload;
				coneWorkloads[i] = newWorkload;

				// 重新找 boundary edges (因为切割改变了连接关系)
				cone.vBoundaryEdges.clear();
				cone.sAdjacentConeId.clear();
				findBoundaryEdges(cone);

				totalSplits++;
				changed = true;

				ylog("[ConeSplit] Cone %d: workload %d -> %d (cut %d edges)\n",
					 i, (int)(m_iTotalWorkLoad - newWorkload + coneWorkloads[i]),
					 newWorkload, nCutEdges);
			}
		}

		if (totalSplits > 0)
		{
			ylog("[ConeSplit] Total split operations: %d\n", totalSplits);

			// 重新计算所有 cone 的 workload (切割可能影响了其他 cone)
			m_iTotalWorkLoad = 0;
			for (int i = 0; i < (int)m_vCones.size(); i++)
			{
				coneWorkloads[i] = computeConeWorkLoadPowerLaw(m_vCones[i].iPoId);
				m_iTotalWorkLoad += coneWorkloads[i];
			}
		}
	}

	void MetisAig::preprocessCones(vector<int32_t> &outConeWorkloads)
	{
		// 计算所有节点的 Level
		computeAllLevel();

		// 构建 Cone 对象
		m_vCones.clear();
		m_vCones.reserve(m_vPos.size()); // 优化内存分配
		for (auto poId : m_vPos)
		{
			m_vCones.emplace_back(m_vNodes[poId].iLevel, poId);
		}

		// 初始化 Workloads 容器
		outConeWorkloads.assign(m_vCones.size(), 0);
		m_iTotalWorkLoad = 0;

		// 排序 (按 Level 降序，这是为了后续 findBoundaryEdges 的正确性)
		sort(m_vCones.begin(), m_vCones.end(), [](const Cone &c1, const Cone &c2)
			 { return c1.iMaxLevel > c2.iMaxLevel; });

		// 计算每个 Cone 的详细信息 (Boundary, Workload)
		for (int i = 0; i < m_vCones.size(); i++)
		{
			auto &cone = m_vCones[i];
			cone.iId = i;

			findBoundaryEdges(cone);

			// 用幂律公式计算 cone workload (单位: 毫秒)
			outConeWorkloads[i] = computeConeWorkLoadPowerLaw(m_vCones[i].iPoId);
			m_iTotalWorkLoad += outConeWorkloads[i];
		}
	}

	void MetisAig::preprocessCones_fast(vector<int32_t> &outConeWorkloads)
	{
		// ── Step 0: Level 计算（不变）──────────────────────────────
		computeAllLevel();

		// ── Step 1: 建 Cone 列表，按 level 降序排列（不变）──────────
		m_vCones.clear();
		m_vCones.reserve(m_vPos.size());
		for (auto poId : m_vPos)
			m_vCones.emplace_back(m_vNodes[poId].iLevel, poId);

		sort(m_vCones.begin(), m_vCones.end(),
			 [](const Cone &a, const Cone &b)
			 { return a.iMaxLevel > b.iMaxLevel; });

		const int32_t nCones = (int32_t)m_vCones.size();
		for (int32_t i = 0; i < nCones; i++)
			m_vCones[i].iId = i;

		// ── Step 2: 建 poId -> coneId 快速查表 ────────────────────
		// PO 节点的 fanin0 是该 cone 的根 AND 节点
		// 用 unordered_map 因为 poId 不连续
		const int32_t nNodes = (int32_t)m_vNodes.size();

		// node2cone[i] = 该节点属于哪个 cone（-1 表示未分配）
		// 用独立数组而不是 m_vNodes[i].iConeId，避免干扰其他逻辑
		vector<int32_t> node2cone(nNodes, -1);

		// 先把每个 cone 的 PO 直接 fanin（即根 AND 节点）标记好
		for (int32_t ci = 0; ci < nCones; ci++)
		{
			auto &cone = m_vCones[ci];
			auto &poNode = m_vNodes[cone.iPoId];
			// PO 节点只有 fanin0，它就是这个 cone 的根
			int32_t rootId = poNode.getFanin0Id();
			if (!m_vNodes[rootId].isPi())
				node2cone[rootId] = ci;
			// 如果根是 PI，这个 cone 退化为只有一个 PI，不需要处理
		}

		// ── Step 3: 单次拓扑序扫描（从高 nodeId 到低，即 PO→PI 方向）
		//
		// 关键性质：AIG 中 nodeId 越大 level 越高（构建时拓扑序插入）
		// 因此倒序遍历 = 从输出端向输入端传播，保证处理一个节点时
		// 其所有 fanout 已经处理完毕。
		//
		// 每个节点做两件事：
		//   A. 把自己的 coneId 传播给两个 fanin
		//   B. 如果 fanin 已被另一个 cone 占据 → 记录 boundary edge
		// ──────────────────────────────────────────────────────────
		for (int32_t i = nNodes - 1; i >= 0; i--)
		{
			auto &node = m_vNodes[i];

			// 跳过 PO（PO 不是 AND 节点，不参与传播）
			if (node.isPo())
				continue;
			// 跳过未被任何 cone 覆盖的节点（孤立子图）
			if (node2cone[i] == -1)
				continue;

			int32_t myCone = node2cone[i];

			// PI 节点：它是叶子，不需要向 fanin 传播，但需要检查它
			// 是否是多个 cone 共享的（即它本身是 boundary）
			// PI 没有 fanin，直接跳过传播阶段
			if (node.isPi())
				continue;

			// 处理 fanin0 和 fanin1
			int32_t faninIds[2] = {node.getFanin0Id(), node.getFanin1Id()};

			for (int fi = 0; fi < 2; fi++)
			{
				int32_t fid = faninIds[fi];
				auto &fanin = m_vNodes[fid];

				if (fanin.isPi())
				{
					// PI 作为 fanin：它可以被多个 cone 共享
					// PI 本身不属于任何 cone，不记录 boundary edge
					// （与原来逻辑一致：PI 不进 vBoundaryEdges）
					continue;
				}

				if (node2cone[fid] == -1)
				{
					// fanin 尚未被分配：传播当前 cone
					node2cone[fid] = myCone;
				}
				else if (node2cone[fid] != myCone)
				{
					// fanin 已属于另一个 cone → 这条边是 boundary edge
					int32_t otherCone = node2cone[fid];

					// 记录到当前 cone 的 boundary edges
					m_vCones[myCone].vBoundaryEdges.push_back(Edge(fid, i));

					// 更新邻接关系（双向）
					m_vCones[myCone].sAdjacentConeId.insert(otherCone);
					m_vCones[otherCone].sAdjacentConeId.insert(myCone);

					// 注意：fanin 已经属于 otherCone，不改变它的归属
					// 这与原来 visitAllFaninFromNode 中
					// "level 更高的 cone 优先" 的逻辑一致：
					// 因为 cone 是按 level 降序排列的，ci=0 的 cone
					// level 最高，会最先传播，后续 cone 遇到已分配节点
					// 说明该节点已归属更高 level 的 cone，不覆盖。
				}
				// else: fanin 已属于同一 cone，正常传播，无需额外操作
			}
		}

		// ── Step 4: 把 node2cone 写回 m_vNodes[i].iConeId ──────────
		// 顺序写，cache 友好
		for (int32_t i = 0; i < nNodes; i++)
			m_vNodes[i].iConeId = node2cone[i];

		// ── Step 5: 计算每个 cone 的 workload（与原来相同）──────────
		outConeWorkloads.assign(nCones, 0);
		m_iTotalWorkLoad = 0;
		for (int32_t i = 0; i < nCones; i++)
		{
			outConeWorkloads[i] = computeConeWorkLoadPowerLaw(m_vCones[i].iPoId);
			m_iTotalWorkLoad += outConeWorkloads[i];
		}
	}

	PartitionConfig MetisAig::determinePartitionConfig(const vector<int32_t> &coneWorkloads, int32_t userK)
	{
		PartitionConfig config;

		if (userK > 0)
		{
			// 用户指定了 K
			if (userK <= coneWorkloads.size())
			{
				config.targetK = userK;
				ylog("User defined K = %d is used.\n", config.targetK);
			}
			else
			{
				// 用户指定的 K 太大（超过了 Cone 的总数），强制钳位
				config.targetK = (int32_t)coneWorkloads.size();
				ylog("[Warning] User K %d > Cone Count %ld. Clamped to %d.\n",
					 userK, coneWorkloads.size(), config.targetK);
			}
		}
		else
		{
			// 用户未指定 (userK == 0)，使用原有的自适应算法
			config.targetK = computeAdaptiveTargetK(coneWorkloads);
			ylog("Adaptive K = %d is used.\n", config.targetK);
		}

		// 计算平均负载和上限
		config.avgWorkload = (config.targetK > 0) ? (m_iTotalWorkLoad / config.targetK) : 0;
		{
			double sumWL = 0, sumWL2 = 0;
			for (auto wl : coneWorkloads)
			{
				sumWL += wl;
				sumWL2 += (double)wl * wl;
			}
			int32_t nCones = (int32_t)coneWorkloads.size();
			double meanWL = sumWL / std::max(nCones, 1);
			double varWL = sumWL2 / std::max(nCones, 1) - meanWL * meanWL;
			double stdWL = (varWL > 0) ? sqrt(varWL) : 0;
			double cv = (meanWL > 0) ? stdWL / meanWL : 0; // 变异系数

			// factor 在 [1.5, 3.0] 之间, 随 CV 线性增长
			double factor = 1.5 + std::min(cv, 3.0) * 0.5;
			if (factor > 3.0)
				factor = 3.0;

			config.workloadLimit = (int64_t)(config.avgWorkload * factor);
			ylog("  Limit factor: %.2f (cone CV=%.2f)\n", factor, cv);
		}

		// 特殊情况处理
		if (config.targetK == 1)
		{
			config.workloadLimit = m_iTotalWorkLoad;
		}

		ylog("Adaptive Partitioning Config:\n");
		ylog("  Target K:       %d\n", config.targetK);
		ylog("  Limit:          %ld\n", config.workloadLimit);

		return config;
	}

	int32_t MetisAig::computeAdaptiveTargetK(const vector<int32_t> &coneWorkloads)
	{
		// === 双指标 K 计算 ===
		// 指标 1: 总节点数 / 目标每分区节点数
		// 指标 2: 总 workload / 目标每分区 workload
		// 取两者较大值, 确保不会产生过大的分区

		const int32_t TARGET_NODES_PER_PART = 8000; // 每分区目标节点数
		const int32_t MAX_PARTITIONS = 200;
		const int32_t MIN_PARTITIONS = 2;

		if (m_iTotalWorkLoad <= 0)
			return 1;

		// 方法 A: 基于 workload (现在 WL ≈ nNodes, 所以和方法 B 很接近)
		int64_t targetWLPerPart = TARGET_NODES_PER_PART; // WL 单位 = 等效节点数
		int32_t k_by_wl = (int32_t)((m_iTotalWorkLoad + targetWLPerPart - 1) / targetWLPerPart);

		// 方法 B: 基于瓶颈 cone
		int64_t maxConeLoad = 0;
		for (auto wl : coneWorkloads)
			if (wl > maxConeLoad)
				maxConeLoad = wl;

		int32_t k_by_bottleneck = MAX_PARTITIONS;
		if (maxConeLoad > 0)
		{
			double theoretical_limit = (double)m_iTotalWorkLoad / maxConeLoad;
			k_by_bottleneck = (int32_t)(theoretical_limit * 2.0);
			if (k_by_bottleneck < 1)
				k_by_bottleneck = 1;
		}

		int32_t finalK = std::min(k_by_wl, k_by_bottleneck);

		if (finalK > MAX_PARTITIONS)
			finalK = MAX_PARTITIONS;
		if (finalK < MIN_PARTITIONS)
			finalK = MIN_PARTITIONS;

		// 不超过 cone 总数的 2/3
		int32_t nCones = (int32_t)coneWorkloads.size();
		if (nCones > 4 && finalK > (nCones * 2) / 3)
			finalK = (nCones * 2) / 3;
		if (finalK > nCones)
			finalK = nCones;

		ylog("[AdaptiveK] k_by_wl=%d, k_by_bottleneck=%d, totalWL=%lld -> K=%d\n",
			 k_by_wl, k_by_bottleneck, (long long)m_iTotalWorkLoad, finalK);

		const int32_t MIN_NODES_PER_PART = 3000;
		int64_t totalGraphNodes = 0;
		for (auto &node : m_vNodes)
		{
			if (!node.isPi() && !node.isPo())
				totalGraphNodes++;
		}
		int32_t k_by_nodes = (int32_t)(totalGraphNodes / MIN_NODES_PER_PART);
		if (k_by_nodes < 1)
			k_by_nodes = 1;
		if (finalK > k_by_nodes)
		{
			ylog("[AdaptiveK] Clamped by nNodes: %d -> %d (totalNodes=%lld, minPerPart=%d)\n",
				 finalK, k_by_nodes, (long long)totalGraphNodes, MIN_NODES_PER_PART);
			finalK = k_by_nodes;
		}

		// 加在 K 计算完之后，return 之前
		const int32_t MIN_CONES_PER_CLUSTER = 3;
		if (finalK > (int32_t)coneWorkloads.size() / MIN_CONES_PER_CLUSTER)
			finalK = std::max(2, (int32_t)coneWorkloads.size() / MIN_CONES_PER_CLUSTER);

		return finalK;
	}

	void MetisAig::runClusteringAlgorithm(const PartitionConfig &config,
										  const vector<int32_t> &coneWorkloads)
	{
		m_vClusters.clear();
		int nCones = (int)m_vCones.size();
		vector<int> coneId2ClusterId(nCones, -1);

		// ============================================================
		// A. Topology-aware Seeding
		//
		// 策略: 用 BFS-like 方式在 cone adjacency graph 上均匀撒种子
		//   1. 第一个种子: 选 WL 最大的 cone (作为锚点)
		//   2. 后续种子: 选离所有已有种子拓扑距离最远的 cone
		//      (近似: 选与已有种子没有共享边、且 WL 较大的 cone)
		//   3. 这样种子自然分散在电路的各个区域
		// ============================================================

		int targetK = config.targetK;
		if (targetK > nCones)
			targetK = nCones;

		// 标记哪些 cone 已被选为种子
		vector<bool> isSeed(nCones, false);
		// 每个 cone 到最近种子的"拓扑距离" (用共享边数的倒数近似)
		// 初始化为 "无穷远"
		vector<int> minDistToSeed(nCones, INT_MAX);

		int seedsFound = 0;

		// --- 第一个种子: WL 最大的 cone ---
		{
			int bestIdx = 0;
			for (int i = 1; i < nCones; i++)
			{
				if (coneWorkloads[i] > coneWorkloads[bestIdx])
					bestIdx = i;
			}

			Cluster c(seedsFound, m_vCones[bestIdx].iMaxLevel);
			c.vConeIds.push_back(bestIdx);
			c.iWorkload = coneWorkloads[bestIdx];
			m_vClusters.push_back(c);
			coneId2ClusterId[bestIdx] = seedsFound;
			isSeed[bestIdx] = true;
			seedsFound++;

			// 更新距离: 该种子的邻居距离设为 1
			for (auto nbrId : m_vCones[bestIdx].sAdjacentConeId)
			{
				if (nbrId >= 0 && nbrId < nCones)
					minDistToSeed[nbrId] = 1;
			}
			minDistToSeed[bestIdx] = 0;
		}

		// --- 后续种子: 选离已有种子最远的 cone ---
		while (seedsFound < targetK)
		{
			int bestIdx = -1;
			int bestDist = -1;
			int32_t bestWL = -1;

			for (int i = 0; i < nCones; i++)
			{
				if (isSeed[i])
					continue;

				// 优先选距离远的; 距离相同时选 WL 大的
				if (minDistToSeed[i] > bestDist ||
					(minDistToSeed[i] == bestDist && coneWorkloads[i] > bestWL))
				{
					bestIdx = i;
					bestDist = minDistToSeed[i];
					bestWL = coneWorkloads[i];
				}
			}

			if (bestIdx < 0)
				break;

			// 创建新 cluster
			Cluster c(seedsFound, m_vCones[bestIdx].iMaxLevel);
			c.vConeIds.push_back(bestIdx);
			c.iWorkload = coneWorkloads[bestIdx];
			m_vClusters.push_back(c);
			coneId2ClusterId[bestIdx] = seedsFound;
			isSeed[bestIdx] = true;
			seedsFound++;

			// 更新距离: BFS 扩散 (只扩散 2 层, 避免 O(n²))
			// 直接邻居距离 = 1, 邻居的邻居距离 = 2
			minDistToSeed[bestIdx] = 0;
			for (auto nbrId : m_vCones[bestIdx].sAdjacentConeId)
			{
				if (nbrId >= 0 && nbrId < nCones)
				{
					if (minDistToSeed[nbrId] > 1)
						minDistToSeed[nbrId] = 1;

					// 二阶邻居
					for (auto nbr2Id : m_vCones[nbrId].sAdjacentConeId)
					{
						if (nbr2Id >= 0 && nbr2Id < nCones)
						{
							if (minDistToSeed[nbr2Id] > 2)
								minDistToSeed[nbr2Id] = 2;
						}
					}
				}
			}
		}

		ylog("[Seeding] Placed %d seeds using topology-aware strategy\n", seedsFound);

		// ============================================================
		// B. Growing (与之前逻辑相同, 但按拓扑顺序遍历而非 WL 降序)
		//
		// 改进: 按 cone 的 level 降序遍历 (从 PO 向 PI 方向)
		// 这样先处理靠近输出的 cone, 它们更容易找到有共享边的种子
		// ============================================================

		// 按 level 降序排列未分配的 cone
		vector<int> growOrder(nCones);
		std::iota(growOrder.begin(), growOrder.end(), 0);
		std::sort(growOrder.begin(), growOrder.end(), [&](int a, int b)
				  { return m_vCones[a].iMaxLevel > m_vCones[b].iMaxLevel; });

		for (int idx : growOrder)
		{
			if (coneId2ClusterId[idx] != -1)
				continue; // 已分配 (种子或之前的 growing)

			int bestCluster = findBestClusterForCone(idx, config,
													 coneId2ClusterId,
													 coneWorkloads);

			if (bestCluster != -1)
			{
				coneId2ClusterId[idx] = bestCluster;
				m_vClusters[bestCluster].vConeIds.push_back(idx);
				m_vClusters[bestCluster].iWorkload += coneWorkloads[idx];
			}
			else
			{
				// 孤立处理
				assignOrphanCone(idx, coneId2ClusterId, coneWorkloads);
			}
		}

		m_vConeId2ClusterId = coneId2ClusterId;
	}

	int MetisAig::findBestClusterForCone(int coneIdx, const PartitionConfig &config,
										 const vector<int> &coneId2ClusterId,
										 const vector<int32_t> &coneWorkloads)
	{
		int bestCluster = -1;
		double maxScore = -1.0;
		auto &cone = m_vCones[coneIdx];
		int32_t coneWL = coneWorkloads[coneIdx];

		for (auto &cluster : m_vClusters)
		{
			// 1. 硬性限制 (不变)
			if (cluster.iWorkload + coneWL > config.workloadLimit)
				continue;

			// 2. 亲密度计算 (不变)
			int sharedEdges = 0;
			for (auto neighborId : cone.sAdjacentConeId)
			{
				if (coneId2ClusterId[neighborId] == cluster.iId)
					sharedEdges++;
			}

			// 3. 改进打分: 同时考虑亲密度和负载均衡
			//
			// 旧公式: score = sharedEdges / (1 + clusterWL/avgWL)
			//   问题: 当 clusterWL >> avgWL 时, penalty 太大, 导致孤立 cone 找不到归属
			//
			// 新公式: score = sharedEdges - λ × loadImbalance
			//   loadImbalance = |clusterWL + coneWL - avgWL| / avgWL
			//   λ 控制 balance 和 locality 的权衡
			//   sharedEdges > 0 时优先保持拓扑局部性
			//   sharedEdges == 0 时退化为纯负载均衡

			double avgWL = (double)std::max(config.avgWorkload, (int64_t)1);
			double newClusterWL = (double)cluster.iWorkload + coneWL;
			double loadImbalance = fabs(newClusterWL - avgWL) / avgWL;

			// λ = 0.3: 轻微惩罚不均衡, 优先保持局部性
			// 当 sharedEdges > 0 时, 局部性通常比完美均衡更重要
			double lambda = 0.3;
			double score;
			if (sharedEdges > 0)
			{
				score = (double)sharedEdges - lambda * loadImbalance;
			}
			else
			{
				// 没有共享边: 纯负载均衡 (选最轻的 cluster)
				// 用负的 loadImbalance 作为 score, 越轻越好
				score = -loadImbalance;
			}

			if (score > maxScore)
			{
				maxScore = score;
				bestCluster = cluster.iId;
			}
		}

		// 改进: 即使 maxScore <= 0, 只要有 cluster 没超 limit, 也应该放进去
		// (旧代码在 maxScore <= 0 时返回 -1, 导致 orphan 处理)
		if (bestCluster >= 0)
			return bestCluster;
		return -1;
	}

	void MetisAig::assignOrphanCone(int coneIdx, vector<int> &coneId2ClusterId,
									const vector<int32_t> &coneWorkloads)
	{
		int minLoadCluster = -1;
		// 使用 int64_t 防止溢出，虽然 workload 是 int32
		int64_t minLoad = std::numeric_limits<int64_t>::max();

		for (auto &c : m_vClusters)
		{
			if (c.iWorkload < minLoad)
			{
				minLoad = c.iWorkload;
				minLoadCluster = c.iId;
			}
		}

		if (minLoadCluster != -1)
		{
			coneId2ClusterId[coneIdx] = minLoadCluster;
			m_vClusters[minLoadCluster].vConeIds.push_back(coneIdx);
			m_vClusters[minLoadCluster].iWorkload += coneWorkloads[coneIdx];
		}
	}

	void MetisAig::rebalanceClusters(vector<int32_t> &coneWorkloads)
	{
		int nCones = (int)m_vCones.size();
		int nClusters = (int)m_vClusters.size();

		if (nClusters <= 1)
			return;

		// 重建 cone -> cluster 映射
		m_vConeId2ClusterId.assign(nCones, -1);
		for (int ci = 0; ci < nClusters; ci++)
		{
			for (auto coneId : m_vClusters[ci].vConeIds)
				m_vConeId2ClusterId[coneId] = ci;
		}

		// 计算当前 avgWL
		int64_t totalWL = 0;
		for (auto &c : m_vClusters)
			totalWL += c.iWorkload;
		double avgWL = (double)totalWL / nClusters;

		const double MERGE_THRESHOLD = 0.30; // WL < 15% avg → 太轻, 合并

		// ============ Phase 1: 合并过轻 cluster ============
		int mergeCount = 0;
		int64_t mergeLimit = (int64_t)(avgWL * MERGE_THRESHOLD);

		// 按 WL 升序处理 (最轻的先合并)
		vector<int> clusterOrder(nClusters);
		std::iota(clusterOrder.begin(), clusterOrder.end(), 0);
		std::sort(clusterOrder.begin(), clusterOrder.end(), [&](int a, int b)
				  { return m_vClusters[a].iWorkload < m_vClusters[b].iWorkload; });

		vector<bool> clusterAlive(nClusters, true);

		for (int ci : clusterOrder)
		{
			if (!clusterAlive[ci])
				continue;
			if (m_vClusters[ci].iWorkload > mergeLimit)
				break; // 后面的都更重, 不需要合并了

			// 找邻居 cluster (通过 cone adjacency)
			std::map<int, int> neighborClusterEdges; // clusterId -> 共享边数
			for (auto coneId : m_vClusters[ci].vConeIds)
			{
				for (auto nbrCone : m_vCones[coneId].sAdjacentConeId)
				{
					if (nbrCone < 0 || nbrCone >= nCones)
						continue;
					int nbrCluster = m_vConeId2ClusterId[nbrCone];
					if (nbrCluster >= 0 && nbrCluster != ci && clusterAlive[nbrCluster])
						neighborClusterEdges[nbrCluster]++;
				}
			}

			if (neighborClusterEdges.empty())
			{
				// 没有邻居: 找全局最轻的 cluster 合并
				int lightest = -1;
				int64_t lightestWL = INT64_MAX;
				for (int j = 0; j < nClusters; j++)
				{
					if (j != ci && clusterAlive[j] && m_vClusters[j].iWorkload < lightestWL)
					{
						lightest = j;
						lightestWL = m_vClusters[j].iWorkload;
					}
				}
				if (lightest >= 0)
					neighborClusterEdges[lightest] = 0;
			}

			if (neighborClusterEdges.empty())
				continue;

			// 选最轻的邻居 cluster 合并 (优先有共享边的)
			int bestTarget = -1;
			int64_t bestTargetWL = INT64_MAX;
			int bestEdges = -1;
			for (auto &[nci, edges] : neighborClusterEdges)
			{
				if (edges > bestEdges ||
					(edges == bestEdges && m_vClusters[nci].iWorkload < bestTargetWL))
				{
					bestTarget = nci;
					bestTargetWL = m_vClusters[nci].iWorkload;
					bestEdges = edges;
				}
			}

			if (bestTarget < 0)
				continue;

			// 执行合并
			for (auto coneId : m_vClusters[ci].vConeIds)
			{
				m_vClusters[bestTarget].vConeIds.push_back(coneId);
				m_vConeId2ClusterId[coneId] = bestTarget;
			}
			m_vClusters[bestTarget].iWorkload += m_vClusters[ci].iWorkload;
			m_vClusters[bestTarget].nNodes += m_vClusters[ci].nNodes;
			m_vClusters[ci].vConeIds.clear();
			m_vClusters[ci].iWorkload = 0;
			m_vClusters[ci].nNodes = 0;
			clusterAlive[ci] = false;
			mergeCount++;
		}

		if (mergeCount > 0)
			ylog("[Rebalance] Merged %d lightweight clusters\n", mergeCount);

		// ============ Phase 2: 从过重 cluster 移出边界 cone ============
		for (int i = 0; i < nClusters; i++)
		{
			if (!clusterAlive[i])
				continue;
			m_vClusters[i].nNodes = 0;
			setNextIter();
			for (auto coneId : m_vClusters[i].vConeIds)
			{
				int32_t tmpWL = 0;
				computeWorkLoad_rec(m_vCones[coneId].iPoId, tmpWL, m_vClusters[i].nNodes);
			}
		}
		int moveCount = 0;

		// 重新计算 avg (合并后 cluster 数量变了)
		int activeClusters = 0;
		totalWL = 0;
		for (int i = 0; i < nClusters; i++)
		{
			if (clusterAlive[i])
			{
				activeClusters++;
				totalWL += m_vClusters[i].iWorkload;
			}
		}
		avgWL = (activeClusters > 0) ? (double)totalWL / activeClusters : 1.0;
		int64_t totalNodes = 0;
		for (int i = 0; i < nClusters; i++)
		{
			if (clusterAlive[i])
				totalNodes += m_vClusters[i].nNodes;
		}
		int64_t avgNodes = (activeClusters > 0) ? totalNodes / activeClusters : 1;
		int64_t heavyNodeLimit = avgNodes * 2;

		ylog("[Rebalance] Phase 2: avgNodes=%lld, heavyLimit=%lld nodes\n",
			 (long long)avgNodes, (long long)heavyNodeLimit);

		for (int pass = 0; pass < 5; pass++) // 最多 5 轮
		{
			bool moved = false;

			for (int ci = 0; ci < nClusters; ci++)
			{
				if (!clusterAlive[ci])
					continue;
				if (m_vClusters[ci].nNodes <= heavyNodeLimit)
					continue;

				// 找边界 cone: 有邻居在其他 cluster 的 cone
				vector<std::pair<int, int>> boundaryCones; // (coneId, neighborCluster)
				for (auto coneId : m_vClusters[ci].vConeIds)
				{
					for (auto nbrCone : m_vCones[coneId].sAdjacentConeId)
					{
						if (nbrCone < 0 || nbrCone >= nCones)
							continue;
						int nbrCluster = m_vConeId2ClusterId[nbrCone];
						if (nbrCluster >= 0 && nbrCluster != ci && clusterAlive[nbrCluster])
						{
							boundaryCones.push_back({coneId, nbrCluster});
							break;
						}
					}
				}

				if (boundaryCones.empty())
					continue;

				// 按 coneWL 升序 (先移小 cone, 精细控制)
				std::sort(boundaryCones.begin(), boundaryCones.end(),
						  [&](const std::pair<int, int> &a, const std::pair<int, int> &b)
						  {
							  return coneWorkloads[a.first] < coneWorkloads[b.first];
						  });

				for (auto &[coneId, targetCluster] : boundaryCones)
				{
					if (m_vClusters[ci].nNodes <= heavyNodeLimit)
						break;
					if (m_vClusters[ci].vConeIds.size() <= 1)
						break; // 不能把最后一个 cone 移走

					// 目标 cluster 不能太重
					if (m_vClusters[targetCluster].nNodes > heavyNodeLimit)
						continue;

					// 移动
					auto &srcCones = m_vClusters[ci].vConeIds;
					srcCones.erase(std::remove(srcCones.begin(), srcCones.end(), coneId),
								   srcCones.end());
					m_vClusters[targetCluster].vConeIds.push_back(coneId);
					m_vConeId2ClusterId[coneId] = targetCluster;

					m_vClusters[ci].iWorkload -= coneWorkloads[coneId];
					m_vClusters[targetCluster].iWorkload += coneWorkloads[coneId];
					// 同步 nNodes (近似: 用 cone 的 workload 比例估算节点数)
					// 精确值需要遍历 cone, 这里用粗估
					int32_t estNodes = coneWorkloads[coneId];
					if (estNodes < 1)
						estNodes = 1;
					m_vClusters[ci].nNodes -= std::min(estNodes, m_vClusters[ci].nNodes - 1);
					m_vClusters[targetCluster].nNodes += estNodes;
					moveCount++;
					moved = true;
				}
			}

			if (!moved)
				break;
		}

		if (moveCount > 0)
			ylog("[Rebalance] Moved %d boundary cones from heavy clusters\n", moveCount);

		// Phase 3 结束后精确重算 nNodes
		for (int i = 0; i < nClusters; i++)
		{
			if (i < (int)clusterAlive.size() && !clusterAlive[i])
				continue;
			m_vClusters[i].nNodes = 0;
			setNextIter();
			for (auto coneId : m_vClusters[i].vConeIds)
			{
				int32_t tmpWL = 0;
				computeWorkLoad_rec(m_vCones[coneId].iPoId, tmpWL, m_vClusters[i].nNodes);
			}
		}

		// 清理: 移除空 cluster, 重新编号
		// (由 postProcessClusters 完成, 不在这里做)
	}

	void MetisAig::splitOversizedClusters()
	{
		// === 目标: 将节点数过多的 cluster 按拓扑 level 分裂 ===
		// 这是消除"长尾"的关键步骤: 如果一个 cluster 占了 50%+ 的节点,
		// 那它会成为并行瓶颈, 无论 workload 模型多准都无法改善。

		int maxCores = std::thread::hardware_concurrency();
		if (maxCores < 1)
			maxCores = 40;

		// --- 精确统计每个 cluster 的 nNodes ---
		for (auto &cluster : m_vClusters)
		{
			cluster.nNodes = 0;
			setNextIter();
			for (auto coneId : cluster.vConeIds)
			{
				int32_t tmpWL = 0;
				computeWorkLoad_rec(m_vCones[coneId].iPoId, tmpWL, cluster.nNodes);
			}
		}

		int64_t totalNodes = 0;
		for (auto &c : m_vClusters)
			totalNodes += c.nNodes;

		// --- 计算阈值 ---
		// 策略: 取 "totalNodes / (nCores/2)" 和绝对上限 50K 中的较小值
		// 这保证即使在小电路上也不会过度分裂
		int64_t nodeThresholdByCores = totalNodes / std::max(maxCores / 2, 1);
		const int64_t ABS_MAX_NODES = 50000;
		const int64_t ABS_MIN_NODES = 5000; // 避免把小 cluster 也拆碎
		int64_t maxNodesPerCluster = std::min(nodeThresholdByCores, ABS_MAX_NODES);
		if (maxNodesPerCluster < ABS_MIN_NODES)
			maxNodesPerCluster = ABS_MIN_NODES;

		ylog("[SplitOversized] totalNodes=%lld, maxCores=%d, threshold=%lld nodes\n",
			 (long long)totalNodes, maxCores, (long long)maxNodesPerCluster);

		// --- 找出需要分裂的 cluster ---
		int splitCount = 0;
		// 用 index 循环, 因为分裂过程中会 push_back 新 cluster
		int originalSize = (int)m_vClusters.size();

		for (int ci = 0; ci < originalSize; ci++)
		{
			if (m_vClusters[ci].nNodes <= maxNodesPerCluster)
				continue;
			if (m_vClusters[ci].vConeIds.size() <= 1)
				continue; // 只有 1 个 cone, 没法分裂

			int nSplit = (int)((m_vClusters[ci].nNodes + maxNodesPerCluster - 1) / maxNodesPerCluster);
			if (nSplit < 2)
				nSplit = 2;

			ylog("[SplitOversized] Cluster %d: nNodes=%d, nCones=%d -> splitting into %d\n",
				 ci, m_vClusters[ci].nNodes, (int)m_vClusters[ci].vConeIds.size(), nSplit);

			// 收集 cone 信息: (coneId, maxLevel, workload)
			struct ConeInfo
			{
				int coneId;
				int32_t maxLevel;
				int32_t workload;
			};
			vector<ConeInfo> coneInfos;
			coneInfos.reserve(m_vClusters[ci].vConeIds.size());
			for (auto coneId : m_vClusters[ci].vConeIds)
			{
				coneInfos.push_back({coneId,
									 m_vCones[coneId].iMaxLevel,
									 computeConeWorkLoadPowerLaw(m_vCones[coneId].iPoId)});
			}

			// 按 maxLevel 排序 (低 level -> 高 level)
			std::sort(coneInfos.begin(), coneInfos.end(),
					  [](const ConeInfo &a, const ConeInfo &b)
					  { return a.maxLevel < b.maxLevel; });

			// 计算总 workload 用于均分
			int64_t totalWL = 0;
			for (auto &ci_info : coneInfos)
				totalWL += ci_info.workload;
			int64_t targetWLPerSplit = totalWL / nSplit;
			if (targetWLPerSplit < 1)
				targetWLPerSplit = 1;

			// 按 workload 累加来切分
			vector<vector<int>> splitConeIds(nSplit);
			int currentSplit = 0;
			int64_t currentWL = 0;

			for (int j = 0; j < (int)coneInfos.size(); j++)
			{
				splitConeIds[currentSplit].push_back(coneInfos[j].coneId);
				currentWL += coneInfos[j].workload;

				// 切换到下一个 split (但要确保最后一个 split 收容剩余所有 cone)
				if (currentWL >= targetWLPerSplit &&
					currentSplit < nSplit - 1 &&
					j < (int)coneInfos.size() - 1)
				{
					currentSplit++;
					currentWL = 0;
				}
			}

			// 移除空 split
			splitConeIds.erase(
				std::remove_if(splitConeIds.begin(), splitConeIds.end(),
							   [](const vector<int> &v)
							   { return v.empty(); }),
				splitConeIds.end());

			if ((int)splitConeIds.size() < 2)
				continue; // 分裂失败 (所有 cone 都在同一个 split), 跳过

			// 第一个 split 复用原 cluster (直接用 index 访问, 避免引用失效)
			m_vClusters[ci].vConeIds = splitConeIds[0];
			m_vClusters[ci].iWorkload = 0;
			for (auto coneId : m_vClusters[ci].vConeIds)
				m_vClusters[ci].iWorkload += computeConeWorkLoadPowerLaw(m_vCones[coneId].iPoId);

			// 后续 split 创建新 cluster
			// 注意: push_back 可能导致 vector 重新分配, 不能持有 m_vClusters 的引用
			for (int s = 1; s < (int)splitConeIds.size(); s++)
			{
				Cluster newCluster;
				newCluster.iId = (int)m_vClusters.size(); // 临时 ID, 后续会重编号
				newCluster.iMaxLevel = 0;
				newCluster.iWorkload = 0;
				newCluster.nNodes = 0;
				newCluster.iPartitionId = -1;

				for (auto coneId : splitConeIds[s])
				{
					newCluster.vConeIds.push_back(coneId);
					newCluster.iWorkload += computeConeWorkLoadPowerLaw(m_vCones[coneId].iPoId);
					if (m_vCones[coneId].iMaxLevel > newCluster.iMaxLevel)
						newCluster.iMaxLevel = m_vCones[coneId].iMaxLevel;
				}

				m_vClusters.push_back(newCluster);
			}

			splitCount++;
		}

		if (splitCount == 0)
		{
			ylog("[SplitOversized] No oversized clusters found, skipping.\n");
			return;
		}

		// --- 精确重算所有 cluster 的 nNodes ---
		for (auto &cluster : m_vClusters)
		{
			cluster.nNodes = 0;
			setNextIter();
			for (auto coneId : cluster.vConeIds)
			{
				int32_t tmpWL = 0;
				computeWorkLoad_rec(m_vCones[coneId].iPoId, tmpWL, cluster.nNodes);
			}
			// 更新 maxLevel
			cluster.iMaxLevel = 0;
			for (auto coneId : cluster.vConeIds)
			{
				if (m_vCones[coneId].iMaxLevel > cluster.iMaxLevel)
					cluster.iMaxLevel = m_vCones[coneId].iMaxLevel;
			}
		}

		// --- 移除空 cluster, 重排 ID, 更新映射 ---
		auto it = std::remove_if(m_vClusters.begin(), m_vClusters.end(),
								 [](const Cluster &c)
								 { return c.vConeIds.empty(); });
		m_vClusters.erase(it, m_vClusters.end());

		// 按 workload 降序排序
		std::sort(m_vClusters.begin(), m_vClusters.end(),
				  [](const Cluster &a, const Cluster &b)
				  { return a.iWorkload > b.iWorkload; });

		// 重建 ID 和映射
		m_vConeId2ClusterId.assign(m_vCones.size(), -1);
		m_iTotalWorkLoad = 0;
		m_iMaxClusterWorkLoad = 0;
		for (int i = 0; i < (int)m_vClusters.size(); i++)
		{
			m_vClusters[i].iId = i;
			for (auto coneId : m_vClusters[i].vConeIds)
				m_vConeId2ClusterId[coneId] = i;
			m_iTotalWorkLoad += m_vClusters[i].iWorkload;
		}
		if (!m_vClusters.empty())
			m_iMaxClusterWorkLoad = m_vClusters[0].iWorkload;

		ylog("[SplitOversized] Split %d clusters. New total: %d clusters\n",
			 splitCount, (int)m_vClusters.size());

		// 打印分裂后最大的几个 cluster
		int printN = std::min(5, (int)m_vClusters.size());
		for (int i = 0; i < printN; i++)
		{
			ylog("  Top-%d: nNodes=%d, nCones=%d, workload=%d\n",
				 i, m_vClusters[i].nNodes, (int)m_vClusters[i].vConeIds.size(),
				 m_vClusters[i].iWorkload);
		}
	}

	void MetisAig::postProcessClusters()
	{
		// 0. Rebalance: 合并过轻 cluster + 移出过重 cluster 的边界 cone
		{
			// 需要 coneWorkloads, 重新计算一次
			vector<int32_t> coneWLs(m_vCones.size(), 0);
			for (int i = 0; i < (int)m_vCones.size(); i++)
				coneWLs[i] = computeConeWorkLoadPowerLaw(m_vCones[i].iPoId);
			rebalanceClusters(coneWLs);
		}

		// 1. 移除空 Cluster
		auto it = std::remove_if(m_vClusters.begin(), m_vClusters.end(),
								 [](const Cluster &c)
								 { return c.vConeIds.empty(); });
		m_vClusters.erase(it, m_vClusters.end());

		// 2. Recalculate Workload & Sanity Check
		m_iTotalWorkLoad = 0;
		m_iMaxClusterWorkLoad = 0;
		for (auto &cluster : m_vClusters)
		{
			// cluster workload = 其所有 cone workload 之和
			cluster.iWorkload = 0;
			cluster.nNodes = 0;
			for (auto coneId : cluster.vConeIds)
			{
				cluster.iWorkload += computeConeWorkLoadPowerLaw(m_vCones[coneId].iPoId);
			}
			// nNodes 仍需统计 (供 mergeSmallClusters 使用)
			setNextIter();
			int32_t tmpWl = 0;
			for (auto coneId : cluster.vConeIds)
				computeWorkLoad_rec(m_vCones[coneId].iPoId, tmpWl, cluster.nNodes);

			m_iTotalWorkLoad += cluster.iWorkload;
		}

		// 3. 排序 (按 Workload 降序)
		sort(m_vClusters.begin(), m_vClusters.end(), [](const Cluster &lhs, const Cluster &rhs)
			 { return lhs.iWorkload > rhs.iWorkload; });

		if (!m_vClusters.empty())
			m_iMaxClusterWorkLoad = m_vClusters[0].iWorkload;

		// 4. 更新 ID 和 映射表
		m_vConeId2ClusterId.assign(m_vCones.size(), -1);
		for (int i = 0; i < m_vClusters.size(); i++)
		{
			auto &cluster = m_vClusters[i];
			cluster.iId = i;
			for (auto coneId : cluster.vConeIds)
				m_vConeId2ClusterId[coneId] = cluster.iId;
		}

		// 5. 合并过小 Cluster
		uint32_t ClusterUpB = m_pMG->get_sCluster();
		if (ClusterUpB != 0)
			mergeSmallClusters(ClusterUpB);

		// 5.5 强制分裂超大 Cluster (消除长尾)
		splitOversizedClusters();

		{
			int maxCores = std::thread::hardware_concurrency();
			if (maxCores < 1)
				maxCores = 40; // fallback
			int maxParts = (int)(maxCores * 1.5);

			while ((int)m_vClusters.size() > maxParts && m_vClusters.size() > 1)
			{
				// 找最轻的 cluster
				int minIdx = 0;
				for (int i = 1; i < (int)m_vClusters.size(); i++)
				{
					if (m_vClusters[i].iWorkload < m_vClusters[minIdx].iWorkload)
						minIdx = i;
				}

				// 找它的最佳合并目标: 优先有共享边的邻居, 否则选最轻的
				int bestTarget = -1;
				int bestEdges = -1;
				int64_t bestTargetWL = INT64_MAX;

				// 检查邻居 (通过 cone adjacency)
				for (auto coneId : m_vClusters[minIdx].vConeIds)
				{
					for (auto nbrCone : m_vCones[coneId].sAdjacentConeId)
					{
						if (nbrCone < 0 || nbrCone >= (int)m_vCones.size())
							continue;
						int nbrCluster = m_vConeId2ClusterId[nbrCone];
						if (nbrCluster >= 0 && nbrCluster < (int)m_vClusters.size() && nbrCluster != minIdx)
						{
							int edges = 1; // 至少有 1 条共享边
							if (edges > bestEdges ||
								(edges == bestEdges && m_vClusters[nbrCluster].iWorkload < bestTargetWL))
							{
								bestTarget = nbrCluster;
								bestEdges = edges;
								bestTargetWL = m_vClusters[nbrCluster].iWorkload;
							}
						}
					}
				}

				// 没有邻居就选最轻的
				if (bestTarget < 0)
				{
					for (int i = 0; i < (int)m_vClusters.size(); i++)
					{
						if (i != minIdx && m_vClusters[i].iWorkload < bestTargetWL)
						{
							bestTarget = i;
							bestTargetWL = m_vClusters[i].iWorkload;
						}
					}
				}

				if (bestTarget < 0)
					break;

				// 合并 minIdx 到 bestTarget
				for (auto coneId : m_vClusters[minIdx].vConeIds)
				{
					m_vClusters[bestTarget].vConeIds.push_back(coneId);
					m_vConeId2ClusterId[coneId] = bestTarget;
				}
				m_vClusters[bestTarget].iWorkload += m_vClusters[minIdx].iWorkload;
				m_vClusters[bestTarget].nNodes += m_vClusters[minIdx].nNodes;

				m_vClusters.erase(m_vClusters.begin() + minIdx);

				// erase 后需要更新 coneId2ClusterId (index 变了)
				for (int i = 0; i < (int)m_vClusters.size(); i++)
				{
					m_vClusters[i].iId = i;
					for (auto coneId : m_vClusters[i].vConeIds)
						m_vConeId2ClusterId[coneId] = i;
				}
			}

			if (maxParts < 200)
				ylog("[PostProcess] Capped partitions to %d (cores=%d)\n",
					 (int)m_vClusters.size(), maxCores);
		}

		// 6. 打印
		printClusters();
		ylog("Final total work load: %ld\n", m_iTotalWorkLoad);
		ylog("Final max cluster work load: %ld\n", m_iMaxClusterWorkLoad);
		ylog("Final partition count: %ld\n", m_vClusters.size());
	}

	void MetisAig::mergeSmallClusters(uint32_t size)
	{
		sort(m_vClusters.begin(), m_vClusters.end(), [](const Cluster &lhs, const Cluster &rhs)
			 { return lhs.nNodes < rhs.nNodes; });
		vector<Cluster> merged_vSmallCluters;
		for (auto &cluster : m_vClusters)
		{
			if (merged_vSmallCluters.empty())
				merged_vSmallCluters.push_back(cluster);
			else
			{
				if (merged_vSmallCluters.back().nNodes + cluster.nNodes > size)
					merged_vSmallCluters.push_back(cluster);
				else
				{
					merged_vSmallCluters.back().iMaxLevel = max(merged_vSmallCluters.back().iMaxLevel, cluster.iMaxLevel);
					merged_vSmallCluters.back().iWorkload += cluster.iWorkload;
					merged_vSmallCluters.back().nNodes += cluster.nNodes;
					merged_vSmallCluters.back().vConeIds.insert(merged_vSmallCluters.back().vConeIds.end(), cluster.vConeIds.begin(), cluster.vConeIds.end());
				}
			}
		}
		m_vClusters = merged_vSmallCluters;
		sort(m_vClusters.begin(), m_vClusters.end(), [](const Cluster &lhs, const Cluster &rhs)
			 { return lhs.iWorkload > rhs.iWorkload; });
		m_iMaxClusterWorkLoad = m_vClusters[0].iWorkload;
		for (int i = 0; i < m_vClusters.size(); i++)
			m_vClusters[i].iId = i;
	}

	void MetisAig::checkClusters()
	{
		for (auto &cluster : m_vClusters)
		{
			setNextIter();
			for (auto coneId : cluster.vConeIds)
			{
				auto &cone = m_vCones[coneId];
				checkNodeCluster(cone.iPoId, cluster.iId);
			}
		}
	}

	void MetisAig::checkNodeCluster(int32_t nodeId, int32_t clusterId)
	{
		Node &node = m_vNodes[nodeId];
		if (node.isPo())
		{
			checkNodeCluster(node.getFanin0Id(), clusterId);
			return;
		}
		if (node.isPi())
			return;
		if (node.iIter == m_iGlobalIter)
			return;
		node.iIter = m_iGlobalIter;
		yassert(m_vConeId2ClusterId[node.iConeId] == clusterId);
		checkNodeCluster(node.getFanin0Id(), clusterId);
		checkNodeCluster(node.getFanin1Id(), clusterId);
		return;
	}

	void MetisAig::findBoundaryEdges(Cone &cone)
	{
		setNextIter();
		visitAllFaninFromNode(cone.iPoId, cone); // mark all the AND nodes in this cone and prepare nVisits of the nodes
		setNextIter();
		findBoundaryEdges_rec(cone, cone.iPoId, 0);
		return;
	}

	// 遍历以指定节点为起点的所有 fanin 节点，同时更新每个节点的访问状态和节点所属的cone区域
	void MetisAig::visitAllFaninFromNode(int32_t nodeId, Cone &cone)
	{
		Node &node = m_vNodes[nodeId];
		if (node.isPo())
		{
			visitAllFaninFromNode(node.getFanin0Id(), cone);
			return;
		}
		if (node.iIter == m_iGlobalIter)
		{
			yassert(node.nVisits);
			node.nVisits--;
			return;
		}

		/* 		if (node.isPi())
					return; */

		node.iIter = m_iGlobalIter;
		node.nVisits = node.nFanouts - 1;
		if (node.iConeId == -1)
			node.iConeId = cone.iId;
		else if (cone.iMaxLevel > m_vCones[node.iConeId].iMaxLevel)
			yassert(0);
		if (node.isPi())
			return;
		// node.iConeId = cone.iId;
		visitAllFaninFromNode(node.getFanin0Id(), cone);
		visitAllFaninFromNode(node.getFanin1Id(), cone);
		return;
	}

	// 在一个有向无环图中找到一个锥形子图的边界边
	void MetisAig::findBoundaryEdges_rec(Cone &cone, int32_t nodeId, int32_t fCovered)
	{
		Node &node = m_vNodes[nodeId];
		int fc = fCovered;

		// 检查是否为PO
		if (node.isPo())
		{
			// 递归访问它的输入节点，并检查是否为边界边
			Node &fanin = m_vNodes[node.getFanin0Id()];
			if (fanin.nVisits) // && !fanin.isPi())
			{
				Edge edge(fanin.id, nodeId);
				cone.vBoundaryEdges.push_back(move(edge));
				if (fanin.iConeId != cone.iId)
					cone.sAdjacentConeId.insert(fanin.iConeId);
				fc = 1;
			}
			findBoundaryEdges_rec(cone, node.getFanin0Id(), fc);
			return;
		}
		if (node.isPi())
			return;
		if (node.iIter == m_iGlobalIter)
			return;
		node.iIter = m_iGlobalIter;

		// 如果不是输出节点，那么它会继续访问两个输入节点，并对它们进行递归调用。
		Node &fanin0 = m_vNodes[node.getFanin0Id()];
		Node &fanin1 = m_vNodes[node.getFanin1Id()];

		// 如果该节点的一个输入节点已被访问过或它的多个输出节点不属于给定锥体(cone)，那么该节点就是边界节点。
		if (fanin0.nVisits || (fanin0.nFanouts > 1 && fanin0.iConeId != cone.iId)) // && !fanin0.isPi()) //has fanout node that belongs other cone
		{
			if (!fCovered)
			{
				// 每个边界边被保存到锥体的边界边列表中
				Edge edge(fanin0.id, nodeId);
				cone.vBoundaryEdges.push_back(move(edge));
			}
			if (fanin0.iConeId != cone.iId)
				// 与其相邻的其他锥体的ID被保存到锥体的相邻锥体ID集合中
				cone.sAdjacentConeId.insert(fanin0.iConeId);
			fc = 1;
		}
		findBoundaryEdges_rec(cone, fanin0.id, fc);

		fc = fCovered;
		if (fanin1.nVisits || (fanin1.nFanouts > 1 && fanin1.iConeId != cone.iId)) // && !fanin1.isPi()) //has fanout node that belongs other cone
		{
			if (!fCovered)
			{
				Edge edge(fanin1.id, nodeId);
				cone.vBoundaryEdges.push_back(move(edge));
			}
			if (fanin1.iConeId != cone.iId)
				cone.sAdjacentConeId.insert(fanin1.iConeId);
			fc = 1;
		}
		findBoundaryEdges_rec(cone, fanin1.id, fc);

		return;
	}

	void MetisAig::computeWorkLoad(Cluster &cluster)
	{
		cluster.iWorkload = 0;
		cluster.nNodes = 0;
		setNextIter();
		for (auto coneId : cluster.vConeIds)
			computeWorkLoad_rec(m_vCones[coneId].iPoId, cluster.iWorkload, cluster.nNodes);
	}

	int32_t MetisAig::computeWorkLoad_rec(int32_t nodeId, int32_t &workload, int32_t &nNodes)
	{
		Node &node = m_vNodes[nodeId];

		// PO: 透传到其 fanin
		if (node.isPo())
		{
			computeWorkLoad_rec(node.getFanin0Id(), workload, nNodes);
			return workload;
		}

		// PI: 基础贡献 + 扇出影响
		if (node.isPi())
		{
			workload += WL_PI_BASE + WL_PI_FANOUT_WEIGHT * node.nFanouts;
			return 1;
		}

		// 已访问: 返回缓存
		if (node.iIter == m_iGlobalIter)
			return node.iNCuts;
		node.iIter = m_iGlobalIter;
		nNodes++;

		// AND 节点: 递归计算 fanin 的 cut 数
		int32_t nCutsLeft = computeWorkLoad_rec(node.getFanin0Id(), workload, nNodes);
		int32_t nCutsRight = computeWorkLoad_rec(node.getFanin1Id(), workload, nNodes);

		// Regression-calibrated: base + fanout proportional
		int32_t nodeContrib = WL_NODE_BASE + WL_FANOUT_WEIGHT * node.nFanouts;
		workload += nodeContrib;

		// 保留 iNCuts 缓存 (递归结构需要)
		int32_t nCuts = nCutsLeft * nCutsRight;
		int32_t cutVal = (nCuts > 64) ? 64 : nCuts; // 内部缓存, 不用于 workload
		node.iNCuts = cutVal + 1;
		return node.iNCuts;
	}

	void MetisAig::collectConeSimpleStats_rec(int32_t nodeId,
											  int32_t &nNodes,
											  int64_t &totalFanout,
											  int32_t &maxLevel)
	{
		Node &node = m_vNodes[nodeId];

		if (node.isPo())
		{
			collectConeSimpleStats_rec(node.getFanin0Id(),
									   nNodes, totalFanout, maxLevel);
			return;
		}
		if (node.isPi())
			return;
		if (node.iIter == m_iGlobalIter)
			return;
		node.iIter = m_iGlobalIter;

		// 统计 AND 节点
		nNodes++;
		totalFanout += node.nFanouts;
		if (node.iLevel > maxLevel)
			maxLevel = node.iLevel;

		collectConeSimpleStats_rec(node.getFanin0Id(),
								   nNodes, totalFanout, maxLevel);
		collectConeSimpleStats_rec(node.getFanin1Id(),
								   nNodes, totalFanout, maxLevel);
	}

	int32_t MetisAig::computeConeWorkLoadPowerLaw(int32_t conePoId)
	{
		int32_t nNodes = 0;
		int64_t totalFanout = 0;
		int32_t maxLevel = 0;

		setNextIter();
		collectConeSimpleStats_rec(conePoId, nNodes, totalFanout, maxLevel);

		if (nNodes < 1)
			return 1;

		// === nNodes 主导, avgFanout 做小幅修正 ===
		// 实测: 时间和 nNodes 高度线性 (R² > 0.98)
		// avgFanout 的影响在 dft 等流水线电路上很小
		// 但在高扇出电路 (如 voter, arbiter) 上有 30-50% 的影响
		// 所以保留一个弱修正: WL = nNodes × (1 + 0.2 × (avgFanout - 1.5))
		// 当 avgFanout=1.5 (典型值) 时, 修正项为 0
		// 当 avgFanout=3.0 时, WL 增加 30%
		// 当 avgFanout=1.0 时, WL 减少 10%

		double avgFanout = (double)totalFanout / nNodes;
		double fanoutCorrection = 1.0 + 0.2 * (avgFanout - 1.5);
		if (fanoutCorrection < 0.5)
			fanoutCorrection = 0.5;
		if (fanoutCorrection > 3.0)
			fanoutCorrection = 3.0;

		// WL 单位: 直接就是 "等效节点数"
		// 好处: 1 WL ≈ 1 个节点的处理时间 ≈ 0.3ms
		//       5000 WL ≈ 5000 节点 ≈ 1.5 秒
		//       直观, 且和 nNodes 天然对齐
		double wl = (double)nNodes * fanoutCorrection;

		int32_t result = (int32_t)wl;
		if (result < 1)
			result = 1;
		return result;
	}

	int32_t MetisAig::decideNumParts()
	{
		int nParts = m_iTotalWorkLoad / m_iMaxClusterWorkLoad;
		ylog("Max nParts = %d\n", nParts);
		if (nParts < MIN_N_PART)
		{
			// nParts = tryPart();
			// nParts = tryPart2();
			ylog("This graph cannot be partitioned naturally. Metis will be used\n");
			return -1;
		}
		else
		{
			while (nParts > MAX_N_PART)
				nParts = nParts / 2;
		}
		ylog("At last, nParts = %d\n", nParts);
		return nParts;
	}

	int32_t MetisAig::tryPart2()
	{
		int clusterId = 0;
		int newClusterId = m_vClusters.size();
		sort(m_vClusters[clusterId].vConeIds.begin(), m_vClusters[clusterId].vConeIds.end(), [this](int32_t lhs, int32_t rhs)
			 { return m_vCones[lhs].iId < m_vCones[rhs].iId; });
		for (int i = 0; i < 100; i++)
			printOneCone(m_vClusters[clusterId].vConeIds[i]);

		vector<int>(m_vCones.size(), -1).swap(m_vConeId2ClusterId);
		for (auto coneId : m_vClusters[clusterId].vConeIds)
			m_vConeId2ClusterId[coneId] = m_vClusters[clusterId].iId;
		vector<int> coneIdForCut;

		auto &coneCritical = m_vCones[m_vClusters[clusterId].vConeIds[0]];
		coneIdForCut.push_back(coneCritical.iId);
		m_vConeId2ClusterId[coneCritical.iId] = newClusterId;
		// Not finished. This method is abandoned.

		return 2;
	}

	int32_t MetisAig::tryPart()
	{
		int workLoadLimit = m_iTotalWorkLoad / MIN_N_PART;
		int iCluster = 0;
		int nConesCut;
		int nCluster = m_vClusters.size();
		// try PI-Cut
		while (m_iMaxClusterWorkLoad > workLoadLimit)
		{
			// Cluster& cluster = m_vClusters[iCluster];
			nConesCut = partBiggestClusterByPICut(iCluster);
			if (nConesCut)
			{
				// if the next cluster becomes the biggest cluster:
				if (m_iMaxClusterWorkLoad == m_vClusters[++iCluster].iWorkload)
					workLoadLimit = m_iTotalWorkLoad / MIN_N_PART;
				else // if after Pi-Cut, the current cluster is still the biggest cluster:
					break;
			}
			else // The biggest cluster cannot be cut by PI-Cut
			{
				ylog("No Pi-Cut available for cluster[%d]\n", iCluster);
				break;
			}
		}
		if (nCluster < m_vClusters.size()) // if m_Clusters is changed, update the order and iId field.
		{
			sort(m_vClusters.begin(), m_vClusters.end(), [](const Cluster &lhs, const Cluster &rhs)
				 { return lhs.iWorkload > rhs.iWorkload; });
			for (int i = 0; i < m_vClusters.size(); i++)
				m_vClusters[i].iId = i;
			int totalwl = 0;
			for (auto &cluster : m_vClusters)
				totalwl += cluster.iWorkload;
			yassert(totalwl == m_iTotalWorkLoad);
			ylog("totalwl = %d\n", totalwl);
		}
		yassert(m_iMaxClusterWorkLoad == m_vClusters[0].iWorkload);
		printClusters();

		// return 2;

		iCluster = 0;
		// try Edge-Cut
		while (m_iMaxClusterWorkLoad > workLoadLimit)
		{
			nConesCut = partBiggestCluster(iCluster, workLoadLimit);
			printClusters();
			sort(m_vClusters.begin(), m_vClusters.end(), [](const Cluster &lhs, const Cluster &rhs)
				 { return lhs.iWorkload > rhs.iWorkload; });
			m_vConeId2ClusterId.resize(m_vCones.size(), -1);
			for (int i = 0; i < m_vClusters.size(); i++)
			{
				Cluster &cluster = m_vClusters[i];
				m_vClusters[i].iId = i;
				for (auto coneId : cluster.vConeIds)
				{
					yassert(m_vConeId2ClusterId[coneId] == -1);
					m_vConeId2ClusterId[coneId] = cluster.iId;
				}
			}
			checkClusters();

			return 2;
			yassert(0);
			break;
		}
		return 2;
	}

	int32_t MetisAig::partBiggestCluster(int32_t clusterId, int32_t workLoadLimit)
	{
		if (m_vClusters[clusterId].iWorkload <= workLoadLimit)
			return 0;
		yassert(m_vClusters[clusterId].iWorkload == m_iMaxClusterWorkLoad);
		sort(m_vClusters[clusterId].vConeIds.begin(), m_vClusters[clusterId].vConeIds.end(), [this](int32_t lhs, int32_t rhs)
			 { return m_vCones[lhs].sAdjacentConeId.size() > m_vCones[rhs].sAdjacentConeId.size(); });
		int nConesCut = 0;
		int workloadNew = 0;
		int workloadOld = m_vClusters[clusterId].iWorkload;
		int workloadNewMax = 0;
		for (int i = 0; i < m_vClusters[clusterId].vConeIds.size(); i++)
		{
			auto &cone = m_vCones[m_vClusters[clusterId].vConeIds[i]];
			bool fCut = 1;
			for (auto edge : cone.vBoundaryEdges)
			{
				auto &fanin = m_vNodes[edge.iFaninId];
				auto &fanout = m_vNodes[edge.iFanoutId];
#if 0
			if(fanout.iLevel - fanin.iLevel > 129)
			{
				if(fanout.iLevel - fanin.iLevel == 130)
				{
					ylog("===========\npif: This may be a critical cut\n");
					ylog("cone[%d]: edge: node[%d]->node[%d]\n", cone.iId, fanin.id, fanout.id);
					m_tmp0++;
				}

				fCut = 0;
				break;
			}
#endif
				if (fanin.iConeId == cone.iId)
				{
					fCut = 0;
					break;
				}
				if (fanout.isPo())
				{
					fCut = 0;
					break;
				}
			}
			if (fCut) // Cut this cone from cluster
			{
				for (auto edge : cone.vBoundaryEdges)
					cutOneBoundaryEdge(edge);
				// vector<Edge>().swap(cone.vBoundaryEdges);
				// set<int32_t>().swap(cone.sAdjacentConeId);
				cone.iMaxLevel = computeNodeLevel(m_vNodes[cone.iPoId].getFanin0Id());

				Cluster clusterNew(m_vClusters.size(), cone.iMaxLevel);
				clusterNew.vConeIds.push_back(cone.iId);
				computeWorkLoad(clusterNew);
				if (workloadNewMax < clusterNew.iWorkload)
					workloadNewMax = clusterNew.iWorkload;
				workloadNew += clusterNew.iWorkload;
				m_vClusters.push_back(move(clusterNew));
				nConesCut++;

				m_vClusters[clusterId].vConeIds[i] = -1;
			}
		}

		if (nConesCut) // EdgeCut happened!
		{
			ylog("In partCluster(): %d EdgeCut happened for cluster[%d]\n", nConesCut, clusterId);
			int maxLev = 0;
			sort(m_vClusters[clusterId].vConeIds.begin(), m_vClusters[clusterId].vConeIds.end(), [](int32_t lhs, int32_t rhs)
				 { return lhs > rhs; });
			for (int i = 0; i < m_vClusters[clusterId].vConeIds.size(); i++)
			{
				if (m_vClusters[clusterId].vConeIds[i] == -1) // delete cut cones
				{
					m_vClusters[clusterId].vConeIds.resize(i);
					break;
				}
				auto &cone = m_vCones[m_vClusters[clusterId].vConeIds[i]];
				if (maxLev < cone.iMaxLevel) // update maxlevel
					maxLev = cone.iMaxLevel;
			}
			if (m_vClusters[clusterId].iMaxLevel > maxLev) // critical cone is parted away
				m_vClusters[clusterId].iMaxLevel = maxLev;
			computeWorkLoad(m_vClusters[clusterId]);
			m_iTotalWorkLoad = m_iTotalWorkLoad - workloadOld + workloadNew + m_vClusters[clusterId].iWorkload;
			m_iMaxClusterWorkLoad = m_vClusters[clusterId].iWorkload; // assume the current cluster is still the biggest one.
			if (m_iMaxClusterWorkLoad < workloadNewMax)				  // cut-out cone may be the biggest cluster?
				m_iMaxClusterWorkLoad = workloadNewMax;
			if (m_iMaxClusterWorkLoad < m_vClusters[clusterId + 1].iWorkload) // After PI-Cut, next cluster may be bigger?
				m_iMaxClusterWorkLoad = m_vClusters[clusterId + 1].iWorkload;
		}

		if (!nConesCut)
			ylog("In partCluster(): No EdgeCut happened for cluster[%d]\n", clusterId);

		ylog("After partCluster(), m_iTotalwl = %ld, m_iMaxwl = %ld\n", m_iTotalWorkLoad, m_iMaxClusterWorkLoad);

		if (m_iMaxClusterWorkLoad > workLoadLimit)
			ylog("After partCluster(), the workLoadLimit can not be satisfied.\n");

		return nConesCut;
	}

	void MetisAig::cutOneBoundaryEdge(Edge &edge)
	{
		// Warning: this function may destroy the DFS order of m_vNodes, and invalidate the structure hash table.
		// The connectivity of nodes is modified directly
		Node &fanin = m_vNodes[edge.iFaninId];
		Node &fanout = m_vNodes[edge.iFanoutId];
		int32_t newPiId = addPi();
		Literal fanin0, fanin1;
		if (fanout.getFanin0Id() == fanin.id)
			fanin0 = Literal(newPiId, fanout.isC0());
		else
			fanin0 = fanout.attr.getFanin0();

		if (fanout.getFanin1Id() == fanin.id)
			fanin1 = Literal(newPiId, fanout.isC1());
		else
			fanin1 = fanout.attr.getFanin1();

		NodeAttribute nodeAttr(fanin0, fanin1);
		fanout.attr = nodeAttr;
	}

	int32_t MetisAig::partBiggestClusterByPICut(int32_t clusterId)
	{
		yassert(m_vClusters[clusterId].iWorkload == m_iMaxClusterWorkLoad);
		yassert(m_vClusters[clusterId].vConeIds.size() > 1);
		int32_t res = 0; // the number of the cut cones
		int32_t workloadNewMax = 0;
		bool fPiCut;
		for (auto i = 0; i < m_vClusters[clusterId].vConeIds.size(); i++)
		{
			auto &cone = m_vCones[m_vClusters[clusterId].vConeIds[i]];
			yassert(!cone.vBoundaryEdges.empty());
			fPiCut = true;
			for (auto edge : cone.vBoundaryEdges) // make sure all the boundary edges of the cone are fanouts of PI
			{
				auto fanin = m_vNodes[edge.iFaninId];
				if (!fanin.isPi())
				{
					fPiCut = false;
					break;
				}
			}
			if (fPiCut)
			{
				// generate new cluster from the cut cone
				Cluster clusterNew(m_vClusters.size(), cone.iMaxLevel);
				clusterNew.vConeIds.push_back(cone.iId);
				computeWorkLoad(clusterNew);
				// if(clusterNew.nNodes < 3) //prevent to cut small cone;
				// continue;
				if (workloadNewMax < clusterNew.iWorkload)
					workloadNewMax = clusterNew.iWorkload;
				m_vClusters.push_back(move(clusterNew));
				res++;

				// delete the cone from the cluster
				m_vClusters[clusterId].vConeIds[i] = -1;
			}
		}

		if (res) // PiCut happened!
		{
			ylog("In partClusterByPiCut(): %d PiCut happened for cluster[%d]\n", res, clusterId);
			int maxLev = 0;
			sort(m_vClusters[clusterId].vConeIds.begin(), m_vClusters[clusterId].vConeIds.end(), [](int32_t lhs, int32_t rhs)
				 { return lhs > rhs; });
			for (int i = 0; i < m_vClusters[clusterId].vConeIds.size(); i++)
			{
				if (m_vClusters[clusterId].vConeIds[i] == -1) // delete cut cones
				{
					m_vClusters[clusterId].vConeIds.resize(i);
					break;
				}
				auto &cone = m_vCones[m_vClusters[clusterId].vConeIds[i]];
				if (maxLev < cone.iMaxLevel) // update maxlevel
					maxLev = cone.iMaxLevel;
			}
			if (m_vClusters[clusterId].iMaxLevel > maxLev) // critical cone is parted away
				m_vClusters[clusterId].iMaxLevel = maxLev;
			computeWorkLoad(m_vClusters[clusterId]);
			m_iMaxClusterWorkLoad = m_vClusters[clusterId].iWorkload; // assume the current cluster is still the biggest one.
			if (m_iMaxClusterWorkLoad < workloadNewMax)				  // cut-out cone may be the biggest cluster?
				m_iMaxClusterWorkLoad = workloadNewMax;
			if (m_iMaxClusterWorkLoad < m_vClusters[clusterId + 1].iWorkload) // After PI-Cut, next cluster may be bigger?
				m_iMaxClusterWorkLoad = m_vClusters[clusterId + 1].iWorkload;
		}

		if (!res)
			ylog("In partClusterByPiCut(): No PiCut happened for cluster[%d]\n", clusterId);
		return res;
	}

	void MetisAig::visitConeForEdgeWight(int32_t nodeId, int32_t coneId)
	{
		Node &node = m_vNodes[nodeId];
		if (node.isPo())
		{
			visitConeForEdgeWight(node.getFanin0Id(), coneId);
			return;
		}
		if (node.iIter == m_iGlobalIter)
			return;
		if (node.isPi())
			return;
		node.iIter = m_iGlobalIter;
		Cone &coneNode = m_vCones[node.iConeId];

		Node &fanin0 = m_vNodes[node.getFanin0Id()];
		Node &fanin1 = m_vNodes[node.getFanin1Id()];
		yassert((fanin0.iData != -1) && (fanin1.iData != -1) && (node.iData != -1));

		if (coneNode.iMaxLevel < m_iMaxLevel * CRITICAL_PATH_FACTOR / 100) // not critical
		{
			if (fanin0.isPi() && fanin0.nFanouts > 1)
				m_pMG->setEdgeWeight(fanin0.iData, node.iData, 1);
			else
				m_pMG->setEdgeWeight(fanin0.iData, node.iData, 100);

			if (fanin1.isPi() && fanin1.nFanouts > 1)
				m_pMG->setEdgeWeight(fanin1.iData, node.iData, 1);
			else
				m_pMG->setEdgeWeight(fanin1.iData, node.iData, 100);
		}
		else // critical path
		{
			if (fanin0.isPi() && fanin0.nFanouts > 1)
				m_pMG->setEdgeWeight(fanin0.iData, node.iData, 1);
			else if (m_vCones[fanin0.iConeId].iMaxLevel > coneNode.iMaxLevel)
				m_pMG->setEdgeWeight(fanin0.iData, node.iData, 100);

			if (fanin1.isPi() && fanin1.nFanouts > 1)
				m_pMG->setEdgeWeight(fanin1.iData, node.iData, 1);
			else if (m_vCones[fanin1.iConeId].iMaxLevel > coneNode.iMaxLevel)
				m_pMG->setEdgeWeight(fanin1.iData, node.iData, 100);
		}

		visitConeForEdgeWight(fanin0.id, coneId);
		visitConeForEdgeWight(fanin1.id, coneId);

		return;
	}

	// 划分AIG
	// 调用 decideNumParts()函数，决定 AIG 图的分区数量。如果返回值为 -1，则使用 Metis 算法进行分区，否则使用确定好的分区数量。
	int32_t MetisAig::partitionAig()
	{
		// int32_t nParts = decideNumParts();
		int32_t nParts = m_vClusters.size();
		//  	if(nParts == -1)
		//  	{
		//  		//Metis is used. Prepare weight vectors
		//  		m_pMG->initWeightVector();
		//  		setNextIter();
		//  		for(auto& cone : m_vCones)
		//  		{
		//  			visitConeForEdgeWight(cone.iPoId, cone.iId);
		//  			//对 AIG 图中的节点进行遍历，并计算节点间的边权重和节点权重
		//  #if 0
		//  			for(auto& edge : cone.vBoundaryEdges)
		//  			{

		// 				Node fanin = m_vNodes[edge.iFaninId];
		// 				Node fanout = m_vNodes[edge.iFanoutId];
		// 				if(fanout.isPo())
		// 					continue;
		// 				if(fanin.isPi())
		// 					m_pMG->setEdgeWeight(fanin.iData, fanout.iData, 1);
		// 				else
		// 					m_pMG->setEdgeWeight(fanin.iData, fanout.iData, 10);
		// 			}
		// #endif
		// 		}
		// 		for(auto& node : m_vNodes)
		// 		{
		// 			if(node.isPi() || node.isPo())
		// 			{
		// 				if(node.iData != -1)
		// 					m_pMG->setNodeWeight(node.iData, 1);
		// 			}
		// 			else
		// 				if(node.iData != -1)
		// 					m_pMG->setNodeWeight(node.iData, 100);
		// 		}
		// 		return METIS_N_PART;
		// 	}
		// yassert(nParts * m_iMaxClusterWorkLoad <= m_iTotalWorkLoad);

		// 如果不适用metis，直接将 AIG 图分为指定数量的分区。
		vector<Partition> partitions(nParts);
		for (auto &cluster : m_vClusters)
		{
			partitions[0].addCluster(cluster);
			sort(partitions.begin(), partitions.end(), [](const Partition &lhs, const Partition &rhs)
				 { return lhs.iWorkload < rhs.iWorkload; });
		}

		/*
		for(auto& part : partitions)
		{
			ylog("This part: workload = %d\tnNodes = %d\n", part.iWorkload, part.nNodes);
		}
		*/

		for (int i = 0; i < partitions.size(); i++)
		{
			auto &part = partitions[i];
			for (auto clusterId : part.vClusterIds)
			{
				auto &cluster = m_vClusters[clusterId];
				setGraphPartition(cluster, i);
			}
		}

		m_pMG->setPoPart();
		m_pMG->setNumParts(nParts);
		return nParts;
	}

	void MetisAig::setGraphPartition(Cluster &cluster, int32_t partId)
	{
		setNextIter();
		for (auto coneId : cluster.vConeIds)
			setNodePartition(m_vCones[coneId].iPoId, partId);
	}

	void MetisAig::setNodePartition(int32_t nodeId, int32_t partId)
	{
		Node &node = m_vNodes[nodeId];
		if (node.isPo())
		{
			setNodePartition(node.getFanin0Id(), partId);
			return;
		}
		if (node.isPi())
		{
			if (node.iData != -1)
				m_pMG->setNodePart(node.iData, partId);
			return;
		}
		if (node.iIter == m_iGlobalIter)
			return;
		node.iIter = m_iGlobalIter;
		if (node.iData != -1)
			m_pMG->setNodePart(node.iData, partId);
		setNodePartition(node.getFanin0Id(), partId);
		setNodePartition(node.getFanin1Id(), partId);
		return;
	}

	static inline int Abc_ObjIsConst1Safe(Abc_Obj_t *pObj)
	{
		if (pObj == NULL)
			return 0;

		// 1. 检查标准结构类型 (这是标准的 ABC 写法)
		if (Abc_ObjType(pObj) == ABC_OBJ_CONST1)
			return 1;

		// 2. 检查是否为映射后的 Const1 门 (针对 Mapped 网表)
		if (Abc_ObjIsNode(pObj) && pObj->pNtk->ntkFunc == ABC_FUNC_MAP && pObj->pNtk->pManFunc)
		{
			Mio_Library_t *pLib = (Mio_Library_t *)pObj->pNtk->pManFunc;
			Mio_Gate_t *pGateC1 = Mio_LibraryReadConst1(pLib);
			// 只有当 pData 指向库里的 Const1 门时，才认为是逻辑 1
			if (pGateC1 && pObj->pData == pGateC1)
				return 1;
		}
		return 0;
	}

	// 安全判断是否为 Const0
	// Mapped 网表中，逻辑 0 通常由一个映射为 Const0 门的普通节点表示
	static inline int Abc_ObjIsConst0Safe(Abc_Obj_t *pObj)
	{
		if (pObj == NULL)
			return 0;

		// 检查是否为映射后的 Const0 门
		if (Abc_ObjIsNode(pObj) && pObj->pNtk->ntkFunc == ABC_FUNC_MAP && pObj->pNtk->pManFunc)
		{
			Mio_Library_t *pLib = (Mio_Library_t *)pObj->pNtk->pManFunc;
			Mio_Gate_t *pGateC0 = Mio_LibraryReadConst0(pLib);
			if (pGateC0 && pObj->pData == pGateC0)
				return 1;
		}
		return 0;
	}

	// 创建一个用于充当 Const0 的节点
	// 因为 ABC 没有原生的 Const0 节点，我们需要手动创建一个指向 Const0 门的节点
	static Abc_Obj_t *Abc_NtkCreateNodeConst0_Helper(Abc_Ntk_t *pNtk)
	{
		// 1. 创建一个普通的新节点
		Abc_Obj_t *pNode = Abc_NtkCreateNode(pNtk);

		// 2. 如果有工艺库(Mapping 网络)，赋予它 Const0 门的数据。
		//    仅当网络是 ABC_FUNC_MAP 时 pManFunc 才是 Mio_Library；
		//    对 AIG/SOP 网络 pManFunc 是 Hop_Man_t/Mem_Flex_t，强制转换会越界读
		if (pNtk->pManFunc && Abc_NtkHasMapping(pNtk))
		{
			Mio_Library_t *pLib = (Mio_Library_t *)pNtk->pManFunc;
			Mio_Gate_t *pGateC0 = Mio_LibraryReadConst0(pLib);

			// 容错：如果库里没有 Const0，尝试用 Const1 顶替（防止空指针崩）
			if (!pGateC0)
				pGateC0 = Mio_LibraryReadConst1(pLib);

			if (pGateC0)
			{
				pNode->pData = pGateC0;
			}
		}
		return pNode;
	}

	// 在 pif/partNtkFuncs.cpp 顶部添加

	// 创建一个用于充当 Const1 的普通节点 (替代 Abc_NtkCreateNodeConst1)
	static Abc_Obj_t *Abc_NtkCreateNodeConst1_Helper(Abc_Ntk_t *pNtk)
	{
		// 1. 创建一个普通的新节点 (ABC_OBJ_NODE)
		Abc_Obj_t *pNode = Abc_NtkCreateNode(pNtk);

		// 2. 如果有工艺库(Mapping 网络)，赋予它 Const1 门的数据。
		//    仅当网络是 ABC_FUNC_MAP 时 pManFunc 才是 Mio_Library；
		//    对 AIG/SOP 网络 pManFunc 是 Hop_Man_t/Mem_Flex_t，强制转换会越界读
		if (pNtk->pManFunc && Abc_NtkHasMapping(pNtk))
		{
			Mio_Library_t *pLib = (Mio_Library_t *)pNtk->pManFunc;
			Mio_Gate_t *pGateC1 = Mio_LibraryReadConst1(pLib);

			// 容错：如果库里没有 Const1，尝试用 Const0 顶替
			if (!pGateC1)
				pGateC1 = Mio_LibraryReadConst0(pLib);

			if (pGateC1)
			{
				pNode->pData = pGateC1;
			}
		}
		return pNode;
	}

	/**Function*************************************************************

	  Synopsis    [Merge several mapped networks.]

	  Description []

	  SideEffects []

	  SeeAlso     []

	***********************************************************************/
	// ===========================================================================
	// 递归构建 AIG 节点 (修正版 - 解决不完整类型报错)
	// 参数 pNtkRes: 目标网络 (Merged Network)
	// ===========================================================================
	static Abc_Obj_t *Abc_NtkMerge_Rec(Abc_Obj_t *pObj, Abc_Ntk_t *pNtkRes)
	{
		// 1. 缓存命中 (已创建)
		if (pObj->pCopy)
			return pObj->pCopy;

		// 2. 常量处理
		// [修复] 直接使用 pNtkRes 获取 Const1，这是 ABC 的标准 API
		if (Abc_AigNodeIsConst(pObj))
			return Abc_AigConst1(pNtkRes);

		// 3. PI 处理
		if (Abc_ObjIsPi(pObj))
		{
			// 3.1 Cut PI -> 递归找源头
			if (pObj->fMarkA == 1)
			{
				Abc_Obj_t *pSource = (Abc_Obj_t *)pObj->pData;
				if (!pSource)
				{
					// [修复] 异常处理：返回 Const0
					return Abc_ObjNotCond(Abc_AigConst1(pNtkRes), 1);
				}

				// 递归调用，传递 pNtkRes
				Abc_Obj_t *pRes = Abc_NtkMerge_Rec(pSource, pNtkRes);

				// 缓存 Regular 指针
				pObj->pCopy = pRes;

				return pRes;
			}

			// 3.2 Real PI -> 应该已初始化
			// [修复] 异常处理：返回 Const0
			// printf("[Error] Real PI %s pCopy is NULL during recursion.\n", Abc_ObjName(pObj));
			return Abc_ObjNotCond(Abc_AigConst1(pNtkRes), 1);
		}

		// 4. PO 处理 (Cut PO)
		if (Abc_ObjIsPo(pObj))
		{
			Abc_Obj_t *pDriver = Abc_ObjFanin0(pObj);
			Abc_Obj_t *pRes = Abc_NtkMerge_Rec(pDriver, pNtkRes);
			return Abc_ObjNotCond(pRes, Abc_ObjFaninC0(pObj));
		}

		// 5. 内部节点 (AND)
		if (Abc_ObjIsNode(pObj))
		{
			Abc_Obj_t *p0 = Abc_NtkMerge_Rec(Abc_ObjFanin0(pObj), pNtkRes);
			Abc_Obj_t *p1 = Abc_NtkMerge_Rec(Abc_ObjFanin1(pObj), pNtkRes);

			p0 = Abc_ObjNotCond(p0, Abc_ObjFaninC0(pObj));
			p1 = Abc_ObjNotCond(p1, Abc_ObjFaninC1(pObj));

			// [修复] 获取 AIG Manager: 从 pNtkRes->pManFunc 获取并强转
			// Abc_AigAnd 接受 Abc_Aig_t* 作为第一个参数
			Abc_Obj_t *pResNode = Abc_AigAnd((Abc_Aig_t *)pNtkRes->pManFunc, p0, p1);

			pObj->pCopy = pResNode;
			return pResNode;
		}

		return NULL;
	}

	// ===========================================================================
	// 主合并函数 (Recursive AIG Version)
	// ===========================================================================
	Abc_Ntk_t *Abc_NtkMerge(Abc_Ntk_t *pNtk, Vec_Ptr_t *pSubNtksOld, Vec_Ptr_t *pSubNtksNew)
	{
		Abc_Ntk_t *pNtkRes;
		Abc_Ntk_t *pSubNtk;
		Abc_Obj_t *pObj;
		Nm_Man_t *pManName;
		int i, j, iObjId;

		if (Vec_PtrSize(pSubNtksNew) == 0)
			return NULL;

		// 1. 初始化结果网络 (STRASH 类型)
		pNtkRes = Abc_NtkStartFrom(pNtk, ABC_NTK_STRASH, ABC_FUNC_AIG);
		pManName = pNtkRes->pManName;

		// 2. 清理 pCopy
		Vec_PtrForEachEntry(Abc_Ntk_t *, pSubNtksNew, pSubNtk, i)
			Abc_NtkCleanCopy(pSubNtk);

		// 3. 映射真实 PI (Base Case)
		Vec_PtrForEachEntry(Abc_Ntk_t *, pSubNtksNew, pSubNtk, i)
		{
			Abc_NtkForEachCi(pSubNtk, pObj, j)
			{
				if (pObj->fMarkA != 1)
				{ // Real PI
					iObjId = Nm_ManFindIdByName(pManName, Abc_ObjName(pObj), Abc_ObjType(pObj));
					if (iObjId == -1)
						iObjId = Nm_ManFindIdByName(pManName, Abc_ObjName(pObj), ABC_OBJ_BO);

					if (iObjId >= 0)
						pObj->pCopy = Abc_NtkObj(pNtkRes, iObjId);
					else
					{
						// [修复] 未映射 PI 连接到 Const0 (使用 pNtkRes)
						pObj->pCopy = Abc_ObjNotCond(Abc_AigConst1(pNtkRes), 1);
					}
				}
			}
		}

		// 4. 驱动递归 (从真实 PO 开始反向构建)
		Vec_PtrForEachEntry(Abc_Ntk_t *, pSubNtksNew, pSubNtk, i)
		{
			Abc_NtkForEachCo(pSubNtk, pObj, j)
			{
				if (pObj->fMarkA != 1)
				{ // Real PO
					iObjId = Nm_ManFindIdByName(pManName, Abc_ObjName(pObj), Abc_ObjType(pObj));
					if (iObjId == -1)
						iObjId = Nm_ManFindIdByName(pManName, Abc_ObjName(pObj), ABC_OBJ_BI);

					if (iObjId >= 0)
					{
						Abc_Obj_t *pResCo = Abc_NtkObj(pNtkRes, iObjId);

						if (Abc_ObjFaninNum(pResCo) > 0)
						{
							continue;
						}

						// [修复] 传递 pNtkRes 给递归函数，而不是 pManDest
						Abc_Obj_t *pDriverSub = Abc_ObjFanin0(pObj);
						Abc_Obj_t *pDriverRes = Abc_NtkMerge_Rec(pDriverSub, pNtkRes);

						Abc_ObjAddFanin(pResCo, Abc_ObjNotCond(pDriverRes, Abc_ObjFaninC0(pObj)));
					}
				}
			}
		}

		return pNtkRes;
	}

	// ===========================================================================
	// 递归构建 Mapped 节点 (核心修复逻辑)
	// 类似 AIG 的 Merge_Rec，但针对 Logic/Mapped 网表适配
	// ===========================================================================
	static Abc_Obj_t *Abc_NtkMergeMapped_Rec(Abc_Obj_t *pObj, Abc_Ntk_t *pNtkRes, Abc_Obj_t *pConst0, Abc_Obj_t *pConst1)
	{
		if (pObj->pCopy)
			return pObj->pCopy;

		if (Abc_ObjIsConst1Safe(pObj))
			return pConst1;

		if (Abc_ObjIsPi(pObj))
		{
			if (pObj->fMarkA == 1) // 切割点 PI
			{
				Abc_Obj_t *pSource = (Abc_Obj_t *)pObj->pData;
				if (pSource == NULL)
					return pConst0;
				Abc_Obj_t *pResDriver = Abc_NtkMergeMapped_Rec(pSource, pNtkRes, pConst0, pConst1);
				pObj->pCopy = pResDriver;
				return pResDriver;
			}
			if (pObj->pCopy == NULL)
				return pConst0;
			return pObj->pCopy;
		}

		if (Abc_ObjIsPo(pObj))
		{
			return Abc_NtkMergeMapped_Rec(Abc_ObjFanin0(pObj), pNtkRes, pConst0, pConst1);
		}

		if (Abc_ObjIsNode(pObj))
		{
			// 先递归处理所有 fanin
			int nFanins = Abc_ObjFaninNum(pObj);
			std::vector<Abc_Obj_t *> vResFanins(nFanins);
			Abc_Obj_t *pFanin;
			int i;
			Abc_ObjForEachFanin(pObj, pFanin, i)
			{
				Abc_Obj_t *pResFanin = Abc_NtkMergeMapped_Rec(pFanin, pNtkRes, pConst0, pConst1);
				vResFanins[i] = pResFanin ? pResFanin : pConst0;
			}

			// DupObj 浅拷贝节点 (包括 pData 指针)
			// 对于 SOP: pData 指向源子网 Mem_Flex 中的字符串, 源子网存活期间有效
			// 对于 MAP: pData 指向全局 Mio gate, 始终有效
			Abc_Obj_t *pResNode = Abc_NtkDupObj(pNtkRes, pObj, 0);
			for (i = 0; i < nFanins; i++)
				Abc_ObjAddFanin(pResNode, vResFanins[i]);

			pObj->pCopy = pResNode;
			return pResNode;
		}
		return NULL;
	}

	// ===========================================================================
	// 主合并函数 (Recursive Mapped Version)
	// ===========================================================================
	Abc_Ntk_t *Abc_NtkMergeMapped(Abc_Ntk_t *pNtkSkeleton, Vec_Ptr_t *pSubNtksRef, Vec_Ptr_t *pSubNtksMapped)
	{
		Abc_Ntk_t *pNtkRes;
		Abc_Ntk_t *pSubNtk;
		Abc_Obj_t *pObj;
		int i, k, iObjId;

		if (Vec_PtrSize(pSubNtksMapped) == 0)
			return NULL;

		Abc_Ntk_t *pFirstSub = (Abc_Ntk_t *)Vec_PtrEntry(pSubNtksMapped, 0);
		bool isGateMapped = (pFirstSub->ntkFunc == ABC_FUNC_MAP);

		// StartFrom 会根据 ntkFunc 创建对应的 pManFunc:
		//   ABC_FUNC_SOP -> Mem_Flex_t (SOP manager)
		//   ABC_FUNC_MAP -> 不创建 (需要手动设)
		//   ABC_FUNC_AIG -> Hop_Man_t
		pNtkRes = Abc_NtkStartFrom(pNtkSkeleton, ABC_NTK_LOGIC, pFirstSub->ntkFunc);

		// ASIC 模式: Mio library 是全局的, 需要手动赋值
		if (isGateMapped && pFirstSub->pManFunc)
			pNtkRes->pManFunc = pFirstSub->pManFunc;

		// 准备 Const0/1
		Abc_Obj_t *pConst0 = Abc_NtkCreateNodeConst0_Helper(pNtkRes);
		Abc_Obj_t *pConst1 = Abc_NtkCreateNodeConst1_Helper(pNtkRes);
		// [FIX] 给常量赋名，防止 ID 冲突
		Abc_ObjAssignName(pConst0, "sys_logic_zero", NULL);
		Abc_ObjAssignName(pConst1, "sys_logic_one", NULL);

		// 2. 清理 pCopy
		Vec_PtrForEachEntry(Abc_Ntk_t *, pSubNtksMapped, pSubNtk, i)
			Abc_NtkCleanCopy(pSubNtk);

		// 3. 映射真实 PI (索引匹配为主, 名字匹配为 fallback)
		Vec_PtrForEachEntry(Abc_Ntk_t *, pSubNtksMapped, pSubNtk, i)
		{
			Abc_Ntk_t *pSubRef = (Abc_Ntk_t *)Vec_PtrEntry(pSubNtksRef, i);

			// 构建 mapped 子网的 PI 名字 -> 对象 映射表 (用于 fallback)
			std::unordered_map<std::string, Abc_Obj_t *> mappedPiByName;
			{
				Abc_Obj_t *pPi;
				int j;
				Abc_NtkForEachPi(pSubNtk, pPi, j)
				{
					char *n = Abc_ObjName(pPi);
					if (n)
						mappedPiByName[n] = pPi;
				}
			}

			Abc_NtkForEachPi(pSubRef, pObj, k)
			{
				char *pName = Abc_ObjName(pObj);
				if (strncmp(pName, "lc_", 3) != 0)
				{
					// 优先按索引匹配, 索引越界时 fallback 到名字匹配
					Abc_Obj_t *pPiMap = NULL;
					if (k < Abc_NtkPiNum(pSubNtk))
						pPiMap = Abc_NtkPi(pSubNtk, k);
					else
					{
						auto it = mappedPiByName.find(pName);
						if (it != mappedPiByName.end())
							pPiMap = it->second;
					}

					if (pPiMap)
					{
						iObjId = Nm_ManFindIdByName(pNtkRes->pManName, pName, ABC_OBJ_PI);
						if (iObjId >= 0)
							pPiMap->pCopy = Abc_NtkObj(pNtkRes, iObjId);
						else
						{
							iObjId = Nm_ManFindIdByName(pNtkRes->pManName, pName, ABC_OBJ_BO);
							if (iObjId >= 0)
								pPiMap->pCopy = Abc_NtkObj(pNtkRes, iObjId);
							else
								pPiMap->pCopy = pConst0;
						}
					}
				}
			}
		}

		// 4. 驱动递归 (索引匹配为主, 名字匹配为 fallback)
		Vec_PtrForEachEntry(Abc_Ntk_t *, pSubNtksMapped, pSubNtk, i)
		{
			Abc_Ntk_t *pSubRef = (Abc_Ntk_t *)Vec_PtrEntry(pSubNtksRef, i);

			// 构建 mapped 子网的 PO 名字 -> 对象 映射表 (用于 fallback)
			std::unordered_map<std::string, Abc_Obj_t *> mappedPoByName;
			{
				Abc_Obj_t *pPo;
				int j;
				Abc_NtkForEachPo(pSubNtk, pPo, j)
				{
					char *n = Abc_ObjName(pPo);
					if (n)
						mappedPoByName[n] = pPo;
				}
			}

			Abc_NtkForEachPo(pSubRef, pObj, k)
			{
				char *pName = Abc_ObjName(pObj);

				if (strncmp(pName, "lc_", 3) != 0)
				{
					iObjId = Nm_ManFindIdByName(pNtkRes->pManName, pName, ABC_OBJ_PO);
					if (iObjId == -1)
						iObjId = Nm_ManFindIdByName(pNtkRes->pManName, pName, ABC_OBJ_BI);

					if (iObjId >= 0)
					{
						Abc_Obj_t *pResCo = Abc_NtkObj(pNtkRes, iObjId);

						if (Abc_ObjFaninNum(pResCo) > 0)
							continue;

						// 优先按索引匹配, 索引越界时 fallback 到名字匹配
						Abc_Obj_t *pPoMap = NULL;
						if (k < Abc_NtkPoNum(pSubNtk))
							pPoMap = Abc_NtkPo(pSubNtk, k);
						else
						{
							auto it = mappedPoByName.find(pName);
							if (it != mappedPoByName.end())
								pPoMap = it->second;
						}

						if (pPoMap)
						{
							Abc_Obj_t *pDriverSub = Abc_ObjFanin0(pPoMap);

							Abc_Obj_t *pDriverRes = Abc_NtkMergeMapped_Rec(pDriverSub, pNtkRes, pConst0, pConst1);

							if (pDriverRes)
								Abc_ObjAddFanin(pResCo, pDriverRes);
							else
								Abc_ObjAddFanin(pResCo, pConst0);
						}
					}
				}
			}
		}

		// 5. 最终清理
		Abc_NtkForEachPo(pNtkRes, pObj, i)
		{
			if (Abc_ObjFaninNum(pObj) == 0)
				Abc_ObjAddFanin(pObj, pConst0);
		}
		// [FIX] 确保未连接的 Latch Input 也连上 0
		Abc_NtkForEachCo(pNtkRes, pObj, i)
		{
			if (Abc_ObjFaninNum(pObj) == 0)
				Abc_ObjAddFanin(pObj, pConst0);
		}

		// [FIX] 修复节点数据指针，防止 ToNetlist 崩溃
		if (isGateMapped && pNtkRes->pManFunc)
		{
			// ASIC 模式: 确保所有节点有 gate 绑定
			Mio_Library_t *pLib = (Mio_Library_t *)pNtkRes->pManFunc;
			void *pGate0 = Mio_LibraryReadConst0(pLib);
			void *pGate1 = Mio_LibraryReadConst1(pLib);
			Mio_Gate_t *pBufGate = Mio_LibraryReadBuf(pLib);

			if (pGate0 != NULL)
			{
				if (!pConst0->pData)
					pConst0->pData = pGate0;
				if (!pConst1->pData)
					pConst1->pData = pGate1 ? pGate1 : pGate0;

				Abc_Obj_t *pNode;
				Abc_NtkForEachNode(pNtkRes, pNode, i)
				{
					if (!pNode->pData)
					{
						// 没有 gate: 根据 fanin 数决定绑什么
						if (Abc_ObjFaninNum(pNode) == 0)
							pNode->pData = pGate0;
						else if (Abc_ObjFaninNum(pNode) == 1 && pBufGate)
							pNode->pData = pBufGate;
						else
						{
							// 多 fanin 但无 gate: 断开所有 fanin, 变成 const0
							while (Abc_ObjFaninNum(pNode) > 0)
								Abc_ObjDeleteFanin(pNode, Abc_ObjFanin(pNode, 0));
							pNode->pData = pGate0;
						}
					}
					else
					{
						// 有 pData: 验证是否是合法的 Mio gate (pin 数匹配 fanin 数)
						Mio_Gate_t *pGate = (Mio_Gate_t *)pNode->pData;
						int nPins = Mio_GateReadPinNum(pGate);
						int nFanins = Abc_ObjFaninNum(pNode);
						if (nPins != nFanins)
						{
							// gate 和 fanin 不匹配: 来自未映射子网的节点
							// 断开所有 fanin, 变成 const0
							while (Abc_ObjFaninNum(pNode) > 0)
								Abc_ObjDeleteFanin(pNode, Abc_ObjFanin(pNode, 0));
							pNode->pData = pGate0;
						}
					}
				}
			}
		}
		else if (pNtkRes->ntkFunc == ABC_FUNC_SOP)
		{
			// SOP 模式 (FPGA blif 读回来的): 修复所有 pData 为 NULL 的节点
			Mem_Flex_t *pMan = (Mem_Flex_t *)pNtkRes->pManFunc;
			Abc_Obj_t *pNode;
			int iNode;
			Abc_NtkForEachNode(pNtkRes, pNode, iNode)
			{
				if (!pNode->pData)
					pNode->pData = Abc_SopRegister(pMan, " 0\n");
			}
			if (!pConst0->pData)
				pConst0->pData = Abc_SopRegister(pMan, " 0\n");
			if (!pConst1->pData)
				pConst1->pData = Abc_SopRegister(pMan, " 1\n");
		}

		if (!Abc_NtkCheck(pNtkRes))
			printf("[Error] Merged network check failed!\n");

		return pNtkRes;
	}

	static Abc_Obj_t *Abc_ObjExtractSubNtk_rec(Abc_Obj_t *pObj, Abc_Ntk_t *pNtkNew)
	{
		if (Abc_NodeIsTravIdCurrent(pObj))
			return pObj->pCopy;
		Abc_NodeSetTravIdCurrent(pObj);
		if (Abc_ObjIsCi(pObj))
		{
			pObj->pCopy = Abc_NtkCreatePi(pNtkNew);
			Abc_ObjAssignName(pObj->pCopy, Abc_ObjName(pObj), NULL);
			return pObj->pCopy;
		}
		if (Abc_ObjIsNode(pObj))
		{
			Abc_Obj_t *pFanin0New = Abc_ObjExtractSubNtk_rec(Abc_ObjFanin0(pObj), pNtkNew);
			Abc_Obj_t *pFanin1New = Abc_ObjExtractSubNtk_rec(Abc_ObjFanin1(pObj), pNtkNew);
			pObj->pCopy = Abc_AigAnd(static_cast<Abc_Aig_t *>(pNtkNew->pManFunc), Abc_ObjNotCond(pFanin0New, Abc_ObjFaninC0(pObj)), Abc_ObjNotCond(pFanin1New, Abc_ObjFaninC1(pObj)));
			return pObj->pCopy;
		}
		if (Abc_ObjIsCo(pObj))
		{
			Abc_Obj_t *pFaninNew = Abc_ObjExtractSubNtk_rec(Abc_ObjFanin0(pObj), pNtkNew);
			pObj->pCopy = Abc_NtkCreatePo(pNtkNew);
			Abc_ObjAssignName(pObj->pCopy, Abc_ObjName(pObj), NULL);
			Abc_ObjAddFanin(pObj->pCopy, pFaninNew);
			return pObj->pCopy;
		}
		yassert(0);
	}

	Abc_Ntk_t *Abc_NtkExtractCriticalPath(Abc_Ntk_t *pNtk, Abc_Obj_t *pCo)
	{
		yassert(pNtk->ntkType == ABC_NTK_STRASH);
		yassert(pCo->pNtk == pNtk);
		yassert(Abc_ObjIsCo(pCo));
		Abc_Ntk_t *pNtkRes = Abc_NtkAlloc(pNtk->ntkType, pNtk->ntkFunc, 1);
		pNtkRes->nConstrs = pNtk->nConstrs;
		pNtkRes->nBarBufs = pNtk->nBarBufs;
		// duplicate the name and the spec
		pNtkRes->pName = Extra_UtilStrsav(pNtk->pName);
		pNtkRes->pSpec = Extra_UtilStrsav(pNtk->pSpec);

		Abc_NtkIncrementTravId(pNtk);
		Abc_ObjExtractSubNtk_rec(pCo, pNtkRes);
		printf("Num of Pi: %d\n", Abc_NtkPiNum(pNtkRes));
		printf("Num of Po: %d\n", Abc_NtkPoNum(pNtkRes));
		printf("Num of AND Node: %d\n", Abc_NtkNodeNum(pNtkRes));
		return pNtkRes;
	}

	Abc_Obj_t *Abc_NtkPickCriticalPo(Abc_Ntk_t *pNtk)
	{
		printf("Now pick Po\n");
		yassert(pNtk->ntkType == ABC_NTK_LOGIC);
		int levelMax = Abc_NtkLevel(pNtk);
		printf("levelMax = %d\n", levelMax);
		Abc_Obj_t *pObj;
		int i;
		Abc_NtkForEachCo(pNtk, pObj, i)
		{
			Abc_Obj_t *pDriver = Abc_ObjFanin0(pObj);
			if (pDriver->Level == levelMax)
			{
				printf("Name of Po: %s\n", Abc_ObjName(Abc_ObjFanout0(pObj)));
				printf("Type of Co's fanout: %d\n", Abc_ObjType(Abc_ObjFanout0(pObj)));
				printf("Level of Po: %d\n", levelMax);
				printf("i of Po : %d\n", i);
				return pObj;
			}
		}
		return NULL;
	}

	// ================================================================
	// MFFC-based Partition Implementation
	// ================================================================

	void MetisAig::identifyMffcs()
	{
		const int32_t nNodes = (int32_t)m_vNodes.size();
		m_vNode2MffcId.assign(nNodes, -1);
		m_vMffcs.clear();

		// mffcRoot[i] = the MFFC root node ID for AND node i
		// -1 = not applicable (PI/PO), -2 = pending assignment
		vector<int32_t> mffcRoot(nNodes, -1);

		// Step 1: Classify AND nodes
		for (int32_t i = nNodes - 1; i >= ID_OFFSET; i--)
		{
			auto &node = m_vNodes[i];
			if (node.isPi() || node.isPo())
				continue;
			// AND node: root if nFanouts != 1, pending otherwise
			mffcRoot[i] = (node.nFanouts == 1) ? -2 : i;
		}

		// Step 2: Propagate root IDs from roots to their single-fanout fanins
		for (int32_t i = nNodes - 1; i >= ID_OFFSET; i--)
		{
			if (mffcRoot[i] != i)
				continue; // not an MFFC root

			// DFS to assign fanins with nFanouts == 1
			vector<int32_t> stack;
			stack.push_back(i);
			while (!stack.empty())
			{
				int32_t cur = stack.back();
				stack.pop_back();

				if (cur != i && mffcRoot[cur] == i)
					continue; // already assigned
				if (cur != i && mffcRoot[cur] >= 0 && mffcRoot[cur] != -2)
					continue; // other root

				mffcRoot[cur] = i;
				auto &curNode = m_vNodes[cur];
				if (curNode.isPi() || curNode.isPo())
					continue;

				int32_t f0 = curNode.getFanin0Id();
				if (f0 >= ID_OFFSET && !m_vNodes[f0].isPi() && !m_vNodes[f0].isPo() && m_vNodes[f0].nFanouts == 1 && mffcRoot[f0] == -2)
					stack.push_back(f0);

				int32_t f1 = curNode.getFanin1Id();
				if (f1 >= ID_OFFSET && !m_vNodes[f1].isPi() && !m_vNodes[f1].isPo() && m_vNodes[f1].nFanouts == 1 && mffcRoot[f1] == -2)
					stack.push_back(f1);
			}
		}

		// Handle remaining unassigned nodes
		for (int32_t i = ID_OFFSET; i < nNodes; i++)
			if (mffcRoot[i] == -2)
				mffcRoot[i] = i;

		// Step 3: Build MffcUnit objects
		unordered_map<int32_t, int32_t> rootToIdx;
		for (int32_t i = ID_OFFSET; i < nNodes; i++)
		{
			auto &node = m_vNodes[i];
			if (node.isPi() || node.isPo())
				continue;

			int32_t rootId = mffcRoot[i];
			if (rootId < 0)
				continue;

			auto it = rootToIdx.find(rootId);
			int32_t mffcIdx;
			if (it == rootToIdx.end())
			{
				mffcIdx = (int32_t)m_vMffcs.size();
				m_vMffcs.emplace_back(mffcIdx, rootId, m_vNodes[rootId].iLevel);
				rootToIdx[rootId] = mffcIdx;
			}
			else
				mffcIdx = it->second;

			m_vNode2MffcId[i] = mffcIdx;
			m_vMffcs[mffcIdx].vNodeIds.push_back(i);
			m_vMffcs[mffcIdx].nNodes++;
			if (m_vNodes[i].iLevel > m_vMffcs[mffcIdx].iMaxLevel)
				m_vMffcs[mffcIdx].iMaxLevel = m_vNodes[i].iLevel;
		}

		ylog("[MFFC] Identified %d MFFCs from %d AND nodes\n",
			 (int)m_vMffcs.size(), nNodes - ID_OFFSET);

		int32_t maxSz = 0, minSz = INT_MAX;
		int64_t totalN = 0;
		for (auto &m : m_vMffcs)
		{
			if (m.nNodes > maxSz)
				maxSz = m.nNodes;
			if (m.nNodes < minSz)
				minSz = m.nNodes;
			totalN += m.nNodes;
		}
		ylog("[MFFC] Size stats: min=%d, max=%d, avg=%.1f\n",
			 minSz, maxSz, m_vMffcs.empty() ? 0.0 : (double)totalN / m_vMffcs.size());
	}

	void MetisAig::computeMffcAdjacency()
	{
		const int32_t nNodes = (int32_t)m_vNodes.size();
		for (int32_t i = ID_OFFSET; i < nNodes; i++)
		{
			auto &node = m_vNodes[i];
			if (node.isPi() || node.isPo())
				continue;
			int32_t myMffc = m_vNode2MffcId[i];
			if (myMffc < 0)
				continue;

			for (int fi = 0; fi < 2; fi++)
			{
				int32_t fid = (fi == 0) ? node.getFanin0Id() : node.getFanin1Id();
				if (fid >= ID_OFFSET && m_vNode2MffcId[fid] >= 0 && m_vNode2MffcId[fid] != myMffc)
				{
					m_vMffcs[myMffc].sAdjacentMffcId.insert(m_vNode2MffcId[fid]);
					m_vMffcs[m_vNode2MffcId[fid]].sAdjacentMffcId.insert(myMffc);
				}
			}
		}
		int64_t totalAdj = 0;
		for (auto &m : m_vMffcs)
			totalAdj += m.sAdjacentMffcId.size();
		ylog("[MFFC] Adjacency: %lld edges (avg %.1f/MFFC)\n",
			 (long long)totalAdj / 2, m_vMffcs.empty() ? 0.0 : (double)totalAdj / m_vMffcs.size());
	}

	int32_t MetisAig::computeMffcWorkload(const MffcUnit &mffc)
	{
		if (mffc.nNodes < 1)
			return 1;
		int64_t totalFanout = 0;
		for (auto nid : mffc.vNodeIds)
			totalFanout += m_vNodes[nid].nFanouts;
		double avg = (double)totalFanout / mffc.nNodes;
		double corr = 1.0 + 0.2 * (avg - 1.5);
		corr = std::max(0.5, std::min(3.0, corr));
		int32_t r = (int32_t)((double)mffc.nNodes * corr);
		return std::max(r, 1);
	}

	void MetisAig::preprocessMffcs(vector<int32_t> &outMffcWorkloads)
	{
		computeAllLevel();
		identifyMffcs();
		computeMffcAdjacency();

		// Task 17 Stage 2: track strict (pre-merge) MFFC origins through
		// coalescing and splitting so every post-preprocessing unit can be
		// labeled with its origin set and split-fragment flag. Telemetry
		// state only; it never affects selection semantics.
		m_vUnitOrigins.assign(m_vMffcs.size(), {});
		m_vUnitSplit.assign(m_vMffcs.size(), false);
		for (size_t i = 0; i < m_vMffcs.size(); i++)
			m_vUnitOrigins[i].push_back((int32_t)i);

		// Telemetry: original (pre-coalescing) MFFC table.
		{
			char buf[256];
			for (size_t i = 0; i < m_vMffcs.size(); i++)
			{
				const MffcUnit &m = m_vMffcs[i];
				snprintf(buf, sizeof(buf), "%d\t%d\t%d\t%d\t%zu",
						 m.iId, m.iRootId, (int)m.nNodes, (int)m.iMaxLevel,
						 m.sAdjacentMffcId.size());
				pifTelemetryRow("pif_mffc_orig.tsv",
								"mffcIdx\trootId\tnNodes\tiMaxLevel\tadjCount", buf);
			}
		}

		// ============================================================
		// Pre-merge: coalesce tiny MFFCs along fanin chains
		//
		// Problem: most MFFCs contain only 1-3 nodes (because most AND
		// nodes have nFanouts > 1). This creates too many partition
		// boundaries, destroying optimization quality.
		//
		// Solution: greedily merge small MFFCs into their largest
		// adjacent MFFC, as long as the merged unit stays below a
		// size limit. This preserves the MFFC's key property
		// (functional independence at boundaries) while reducing
		// fragmentation.
		// ============================================================
		const int32_t MERGE_SIZE_LIMIT = 50;	// max nodes in a merged super-MFFC
		const int32_t SMALL_MFFC_THRESHOLD = 5; // MFFCs with <= 5 nodes are candidates

		// Sort MFFC indices by size ascending (merge smallest first)
		int nMffcs = (int)m_vMffcs.size();
		vector<int> mergeOrder(nMffcs);
		std::iota(mergeOrder.begin(), mergeOrder.end(), 0);
		std::sort(mergeOrder.begin(), mergeOrder.end(), [&](int a, int b)
				  { return m_vMffcs[a].nNodes < m_vMffcs[b].nNodes; });

		// mergeTarget[i] = the MFFC that MFFC i was merged into (-1 = self)
		vector<int32_t> mergeTarget(nMffcs, -1);

		// Find the canonical root of an MFFC (follow merge chains)
		auto findRoot = [&](int idx) -> int
		{
			while (mergeTarget[idx] != -1)
				idx = mergeTarget[idx];
			return idx;
		};

		int mergeCount = 0;
		for (int idx : mergeOrder)
		{
			int root = findRoot(idx);
			if (m_vMffcs[root].nNodes > SMALL_MFFC_THRESHOLD)
				continue; // already big enough after previous merges

			// Find the best adjacent MFFC to merge into
			// Priority: prefer the adjacent MFFC that is on the fanin path
			// (i.e., the MFFC whose root is a fanin of this MFFC's nodes)
			int bestNeighbor = -1;
			int bestSharedEdges = 0;
			int32_t bestNeighborSize = INT_MAX;

			for (auto adjId : m_vMffcs[root].sAdjacentMffcId)
			{
				int adjRoot = findRoot(adjId);
				if (adjRoot == root)
					continue; // already merged together
				if (m_vMffcs[adjRoot].nNodes + m_vMffcs[root].nNodes > MERGE_SIZE_LIMIT)
					continue; // would exceed size limit

				// Count shared boundary edges between root and adjRoot
				int shared = 0;
				for (auto nodeId : m_vMffcs[root].vNodeIds)
				{
					auto &nd = m_vNodes[nodeId];
					if (nd.isPi() || nd.isPo())
						continue;
					int32_t f0 = nd.getFanin0Id();
					int32_t f1 = nd.getFanin1Id();
					if (f0 >= ID_OFFSET && m_vNode2MffcId[f0] >= 0 && findRoot(m_vNode2MffcId[f0]) == adjRoot)
						shared++;
					if (f1 >= ID_OFFSET && m_vNode2MffcId[f1] >= 0 && findRoot(m_vNode2MffcId[f1]) == adjRoot)
						shared++;
				}

				if (shared > bestSharedEdges ||
					(shared == bestSharedEdges && m_vMffcs[adjRoot].nNodes < bestNeighborSize))
				{
					bestNeighbor = adjRoot;
					bestSharedEdges = shared;
					bestNeighborSize = m_vMffcs[adjRoot].nNodes;
				}
			}

			if (bestNeighbor < 0)
				continue;

			// Merge root into bestNeighbor
			mergeTarget[root] = bestNeighbor;
			{
				char buf[256];
				snprintf(buf, sizeof(buf), "%d\t%d\t%d\t%d\t%d",
						 root, bestNeighbor, (int)m_vMffcs[root].nNodes,
						 (int)m_vMffcs[bestNeighbor].nNodes, bestSharedEdges);
				pifTelemetryRow("pif_mffc_merge.tsv",
								"mergedIdx\ttargetIdx\tmergedNodes\ttargetNodes\tsharedEdges", buf);
			}

			// Transfer nodes
			for (auto nodeId : m_vMffcs[root].vNodeIds)
			{
				m_vMffcs[bestNeighbor].vNodeIds.push_back(nodeId);
				m_vNode2MffcId[nodeId] = bestNeighbor;
			}
			m_vMffcs[bestNeighbor].nNodes += m_vMffcs[root].nNodes;
			if (m_vMffcs[root].iMaxLevel > m_vMffcs[bestNeighbor].iMaxLevel)
				m_vMffcs[bestNeighbor].iMaxLevel = m_vMffcs[root].iMaxLevel;

			// Transfer strict-MFFC origins (telemetry state).
			for (auto o : m_vUnitOrigins[root])
				m_vUnitOrigins[bestNeighbor].push_back(o);
			m_vUnitOrigins[root].clear();

			// Transfer adjacency (merge neighbor sets)
			for (auto adjId : m_vMffcs[root].sAdjacentMffcId)
			{
				int adjRoot2 = findRoot(adjId);
				if (adjRoot2 != bestNeighbor)
				{
					m_vMffcs[bestNeighbor].sAdjacentMffcId.insert(adjRoot2);
					m_vMffcs[adjRoot2].sAdjacentMffcId.insert(bestNeighbor);
				}
				m_vMffcs[adjRoot2].sAdjacentMffcId.erase(root);
			}
			m_vMffcs[bestNeighbor].sAdjacentMffcId.erase(root);

			// Clear the merged MFFC
			m_vMffcs[root].vNodeIds.clear();
			m_vMffcs[root].nNodes = 0;
			m_vMffcs[root].sAdjacentMffcId.clear();
			mergeCount++;
		}

		if (mergeCount > 0)
		{
			// Compact: remove empty MFFCs, rebuild indices
			vector<MffcUnit> compacted;
			vector<int32_t> oldToNew(nMffcs, -1);
			for (int i = 0; i < nMffcs; i++)
			{
				if (m_vMffcs[i].nNodes == 0)
					continue;
				oldToNew[i] = (int32_t)compacted.size();
				m_vMffcs[i].iId = (int32_t)compacted.size();
				compacted.push_back(std::move(m_vMffcs[i]));
			}
			m_vMffcs = std::move(compacted);

			// Remap strict-MFFC origins and split flags (telemetry state).
			{
				vector<vector<int32_t>> newOrigins(m_vMffcs.size());
				vector<bool> newSplit(m_vMffcs.size(), false);
				for (int i = 0; i < nMffcs; i++)
				{
					if (oldToNew[i] < 0)
						continue;
					newOrigins[oldToNew[i]] = std::move(m_vUnitOrigins[i]);
					newSplit[oldToNew[i]] = m_vUnitSplit[i];
				}
				m_vUnitOrigins = std::move(newOrigins);
				m_vUnitSplit = std::move(newSplit);
			}

			// Rebuild node->MFFC mapping
			for (int32_t i = 0; i < (int32_t)m_vNode2MffcId.size(); i++)
			{
				if (m_vNode2MffcId[i] >= 0 && m_vNode2MffcId[i] < nMffcs)
					m_vNode2MffcId[i] = oldToNew[m_vNode2MffcId[i]];
			}

			// Rebuild adjacency with new indices
			for (auto &mffc : m_vMffcs)
			{
				set<int32_t> newAdj;
				for (auto adj : mffc.sAdjacentMffcId)
				{
					if (adj >= 0 && adj < nMffcs && oldToNew[adj] >= 0)
						newAdj.insert(oldToNew[adj]);
				}
				// Remove self-reference
				newAdj.erase(mffc.iId);
				mffc.sAdjacentMffcId = newAdj;
			}

			ylog("[MFFC-PreMerge] Merged %d small MFFCs: %d -> %d MFFCs\n",
				 mergeCount, nMffcs, (int)m_vMffcs.size());

			// Print updated stats
			int32_t maxSz = 0, minSz = INT_MAX;
			int64_t totalN = 0;
			for (auto &m : m_vMffcs)
			{
				if (m.nNodes > maxSz)
					maxSz = m.nNodes;
				if (m.nNodes < minSz)
					minSz = m.nNodes;
				totalN += m.nNodes;
			}
			ylog("[MFFC-PreMerge] New size stats: min=%d, max=%d, avg=%.1f\n",
				 minSz, maxSz, m_vMffcs.empty() ? 0.0 : (double)totalN / m_vMffcs.size());
		}

		// ============================================================
		// Split oversized MFFCs
		//
		// Problem: In circuits like hyp (arithmetic/multiplier), long
		// single-fanout chains create MFFCs with hundreds of thousands
		// of nodes. These become indivisible bottlenecks that destroy
		// parallelism (a single MFFC = a single partition = 0 speedup).
		//
		// Solution: Split any MFFC larger than SPLIT_THRESHOLD into
		// multiple smaller MFFCs by cutting along level boundaries.
		// Nodes are sorted by level, then divided into chunks.
		// This breaks the MFFC property (the pieces are no longer
		// fanout-free cones), but maintains good locality and allows
		// the clustering algorithm to distribute workload evenly.
		// ============================================================
		const int32_t SPLIT_THRESHOLD = 500; // max nodes per MFFC unit

		int splitCount = 0;
		int origMffcCount = (int)m_vMffcs.size();

		for (int mi = 0; mi < origMffcCount; mi++)
		{
			if (m_vMffcs[mi].nNodes <= SPLIT_THRESHOLD)
				continue;

			int nSplit = (m_vMffcs[mi].nNodes + SPLIT_THRESHOLD - 1) / SPLIT_THRESHOLD;
			if (nSplit < 2)
				continue;

			ylog("[MFFC-Split] MFFC %d (root=%d): %d nodes -> splitting into %d\n",
				 mi, m_vMffcs[mi].iRootId, m_vMffcs[mi].nNodes, nSplit);

			// Sort nodes by level for locality-preserving split
			auto &nodes = m_vMffcs[mi].vNodeIds;
			std::sort(nodes.begin(), nodes.end(), [this](int32_t a, int32_t b)
					  { return m_vNodes[a].iLevel < m_vNodes[b].iLevel; });

			int nodesPerChunk = (int)nodes.size() / nSplit;
			if (nodesPerChunk < 1)
				nodesPerChunk = 1;

			// First chunk stays in the original MFFC
			vector<vector<int32_t>> chunks(nSplit);
			for (int i = 0; i < (int)nodes.size(); i++)
			{
				int chunkIdx = i / nodesPerChunk;
				if (chunkIdx >= nSplit)
					chunkIdx = nSplit - 1;
				chunks[chunkIdx].push_back(nodes[i]);
			}

			// Update original MFFC with first chunk
			m_vMffcs[mi].vNodeIds = chunks[0];
			m_vMffcs[mi].nNodes = (int32_t)chunks[0].size();
			m_vMffcs[mi].iMaxLevel = 0;
			m_vUnitSplit[mi] = true; // the split breaks the strict MFFC property
			for (auto nid : chunks[0])
			{
				m_vNode2MffcId[nid] = mi;
				if (m_vNodes[nid].iLevel > m_vMffcs[mi].iMaxLevel)
					m_vMffcs[mi].iMaxLevel = m_vNodes[nid].iLevel;
			}

			// Create new MFFCs for remaining chunks
			for (int c = 1; c < nSplit; c++)
			{
				if (chunks[c].empty())
					continue;

				int32_t newIdx = (int32_t)m_vMffcs.size();
				int32_t newRoot = chunks[c].back(); // highest-level node as root
				int32_t newMaxLevel = 0;
				for (auto nid : chunks[c])
				{
					if (m_vNodes[nid].iLevel > newMaxLevel)
						newMaxLevel = m_vNodes[nid].iLevel;
				}

				MffcUnit newMffc(newIdx, newRoot, newMaxLevel);
				newMffc.vNodeIds = chunks[c];
				newMffc.nNodes = (int32_t)chunks[c].size();

				// Update node->MFFC mapping
				for (auto nid : chunks[c])
					m_vNode2MffcId[nid] = newIdx;

				m_vMffcs.push_back(std::move(newMffc));
				// Fragments inherit the origins of the split unit and are
				// labeled split fragments (telemetry state).
				m_vUnitOrigins.push_back(m_vUnitOrigins[mi]);
				m_vUnitSplit.push_back(true);
			}

			splitCount++;
		}

		if (splitCount > 0)
		{
			// Rebuild adjacency for all MFFCs (since splits changed everything)
			for (auto &mffc : m_vMffcs)
				mffc.sAdjacentMffcId.clear();
			computeMffcAdjacency();

			ylog("[MFFC-Split] Split %d oversized MFFCs: %d -> %d MFFCs\n",
				 splitCount, origMffcCount, (int)m_vMffcs.size());

			// Print stats
			int32_t maxSz2 = 0, minSz2 = INT_MAX;
			int64_t totalN2 = 0;
			for (auto &m : m_vMffcs)
			{
				if (m.nNodes > maxSz2)
					maxSz2 = m.nNodes;
				if (m.nNodes < minSz2)
					minSz2 = m.nNodes;
				totalN2 += m.nNodes;
			}
			ylog("[MFFC-Split] New size stats: min=%d, max=%d, avg=%.1f\n",
				 minSz2, maxSz2, m_vMffcs.empty() ? 0.0 : (double)totalN2 / m_vMffcs.size());
		}

		// Compute workloads
		int32_t n = (int32_t)m_vMffcs.size();
		outMffcWorkloads.resize(n);
		m_iTotalWorkLoad = 0;
		for (int32_t i = 0; i < n; i++)
		{
			outMffcWorkloads[i] = computeMffcWorkload(m_vMffcs[i]);
			m_vMffcs[i].iWorkload = outMffcWorkloads[i];
			m_iTotalWorkLoad += outMffcWorkloads[i];
		}
		ylog("[MFFC] Total workload: %lld\n", (long long)m_iTotalWorkLoad);

		// Telemetry: coalesced (post-merge) MFFC table with the active
		// workload estimate, i.e. exactly the clustering input.
		{
			char buf[256];
			for (int32_t i = 0; i < n; i++)
			{
				const MffcUnit &m = m_vMffcs[i];
				snprintf(buf, sizeof(buf), "%d\t%d\t%d\t%d\t%d\t%zu",
						 m.iId, (int)m.nNodes, (int)m.iMaxLevel,
						 (int)outMffcWorkloads[i], (int)m.iWorkload,
						 m.sAdjacentMffcId.size());
				pifTelemetryRow("pif_mffc.tsv",
								"mffcIdx\tnNodes\tiMaxLevel\tworkload\tiWorkload\tadjCount", buf);
			}
		}
	}

	int32_t MetisAig::computeAdaptiveMffcTargetK(const vector<int32_t> &mffcWorkloads)
	{
		const int32_t TARGET_PER_PART = 8000, MAX_P = 200, MIN_P = 2;
		if (m_iTotalWorkLoad <= 0)
			return 1;

		int32_t k_wl = (int32_t)((m_iTotalWorkLoad + TARGET_PER_PART - 1) / TARGET_PER_PART);
		int64_t maxL = 0;
		for (auto w : mffcWorkloads)
			if (w > maxL)
				maxL = w;
		int32_t k_bn = (maxL > 0) ? (int32_t)((double)m_iTotalWorkLoad / maxL * 2.0) : MAX_P;
		if (k_bn < 1)
			k_bn = 1;

		int32_t K = std::min(k_wl, k_bn);
		K = std::max(MIN_P, std::min(K, MAX_P));

		int32_t nM = (int32_t)mffcWorkloads.size();
		if (nM > 4 && K > (nM * 2) / 3)
			K = (nM * 2) / 3;
		if (K > nM)
			K = nM;

		int64_t totalN = 0;
		for (auto &m : m_vMffcs)
			totalN += m.nNodes;
		int32_t k_n = std::max(1, (int32_t)(totalN / 3000));
		if (K > k_n)
			K = k_n;
		if (K > nM / 3)
			K = std::max(2, nM / 3);

		ylog("[MFFC-K] k_wl=%d, k_bn=%d -> K=%d\n", k_wl, k_bn, K);
		{
			char buf[256];
			snprintf(buf, sizeof(buf), "%lld\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d",
					 (long long)m_iTotalWorkLoad, (int)mffcWorkloads.size(), k_wl, k_bn,
					 (int)std::max(1, (int32_t)(totalN / 3000)), (int)nM, K,
					 TARGET_PER_PART, MAX_P, MIN_P,
					 (int)((int64_t)totalN / 3000), (nM > 4) ? (nM * 2) / 3 : -1);
			pifTelemetryRow("pif_k.tsv",
							"totalWL\tnMffc\tk_wl\tk_bn\tk_n\tnM\tK\tTARGET_PER_PART\tMAX_P\tMIN_P\tk_n2\tcap23",
							buf);
		}
		return K;
	}

	PartitionConfig MetisAig::determineMffcPartitionConfig(const vector<int32_t> &mffcWorkloads, int32_t userK, int32_t effectiveJ)
	{
		PartitionConfig config;
		// Effective child concurrency J, shared with the child scheduler:
		// explicit pif -j wins, otherwise Linux sched_getaffinity
		// (cpuset/affinity aware) with a hardware-concurrency fallback,
		// default half the allowed CPUs with a minimum of one.
		int32_t j = effectiveJ > 0 ? effectiveJ : pifResolveEffectiveJ(0);
		if (userK > 0)
		{
			config.targetK = std::min(userK, (int32_t)mffcWorkloads.size());
		}
		else
		{
			// Task 18 Stage 5 / Task 19 Stage 3: deterministic dynamic-K
			// rule (J-based). K = ceil(J/2) with the same resolved J the
			// scheduler consumes. The Stage 5 screening table measured the
			// total cost (partition + child makespan + merge) as monotonic
			// in K on all six frozen designs, with the minimum at the
			// smallest feasible K and no clstm level movement; explicit
			// pif -N always overrides this rule.
			config.targetK = std::min((j + 1) / 2, (int32_t)mffcWorkloads.size());
		}
		config.avgWorkload = (config.targetK > 0) ? (m_iTotalWorkLoad / config.targetK) : 0;

		double sum = 0, sum2 = 0;
		for (auto w : mffcWorkloads)
		{
			sum += w;
			sum2 += (double)w * w;
		}
		int32_t n = (int32_t)mffcWorkloads.size();
		double mean = sum / std::max(n, 1);
		double sd = sqrt(std::max(0.0, sum2 / std::max(n, 1) - mean * mean));
		double cv = (mean > 0) ? sd / mean : 0;
		double factor = std::min(3.0, 1.5 + std::min(cv, 3.0) * 0.5);
		config.workloadLimit = (config.targetK == 1) ? m_iTotalWorkLoad : (int64_t)(config.avgWorkload * factor);

		ylog("[MFFC] Config: K=%d, J=%d, avgWL=%lld, limit=%lld\n",
			 config.targetK, j, (long long)config.avgWorkload, (long long)config.workloadLimit);
		{
			char buf[256];
			snprintf(buf, sizeof(buf), "%d\t%d\t%lld\t%lld\t%.4f\t%.4f\t%.4f\t%.4f\t%d",
					 config.targetK, userK, (long long)config.avgWorkload,
					 (long long)config.workloadLimit, cv, factor, mean, sd, n);
			pifTelemetryRow("pif_config.tsv",
							"targetK\tuserK\tavgWorkload\tworkloadLimit\tcv\tfactor\tmean\tsd\tnMffc",
							buf);
		}
		return config;
	}

	void MetisAig::runMffcClusteringAlgorithm(const PartitionConfig &config, const vector<int32_t> &mffcWorkloads)
	{
		m_vClusters.clear();
		int nM = (int)m_vMffcs.size();
		vector<int> id2cl(nM, -1);
		int targetK = std::min(config.targetK, nM);

		// Seeding
		vector<bool> isSeed(nM, false);
		vector<int> dist(nM, INT_MAX);
		int seeds = 0;

		auto placeSeed = [&](int idx)
		{
			Cluster c(seeds, m_vMffcs[idx].iMaxLevel);
			c.vConeIds.push_back(idx);
			c.iWorkload = mffcWorkloads[idx];
			m_vClusters.push_back(c);
			id2cl[idx] = seeds;
			isSeed[idx] = true;
			dist[idx] = 0;
			for (auto nb : m_vMffcs[idx].sAdjacentMffcId)
				if (nb >= 0 && nb < nM)
				{
					if (dist[nb] > 1)
						dist[nb] = 1;
					for (auto nb2 : m_vMffcs[nb].sAdjacentMffcId)
						if (nb2 >= 0 && nb2 < nM && dist[nb2] > 2)
							dist[nb2] = 2;
				}
			seeds++;
			{
				char buf[256];
				snprintf(buf, sizeof(buf), "%d\t%d\t%d\t%d", idx, (int)mffcWorkloads[idx], dist[idx], seeds - 1);
				pifTelemetryRow("pif_seed.tsv",
								"mffcIdx\tworkload\tdist\tseedOrder", buf);
			}
		};

		int best = 0;
		for (int i = 1; i < nM; i++)
			if (mffcWorkloads[i] > mffcWorkloads[best])
				best = i;
		placeSeed(best);

		while (seeds < targetK)
		{
			best = -1;
			int bestD = -1;
			int32_t bestW = -1;
			for (int i = 0; i < nM; i++)
			{
				if (isSeed[i])
					continue;
				if (dist[i] > bestD || (dist[i] == bestD && mffcWorkloads[i] > bestW))
				{
					best = i;
					bestD = dist[i];
					bestW = mffcWorkloads[i];
				}
			}
			if (best < 0)
				break;
			placeSeed(best);
		}
		ylog("[MFFC-Seed] %d seeds placed\n", seeds);

		// Growing
		vector<int> order(nM);
		std::iota(order.begin(), order.end(), 0);
		std::sort(order.begin(), order.end(), [&](int a, int b)
				  { return m_vMffcs[a].iMaxLevel > m_vMffcs[b].iMaxLevel; });

		for (int idx : order)
		{
			if (id2cl[idx] != -1)
				continue;
			double score = 0.0;
			int bc = findBestClusterForMffc(idx, config, id2cl, mffcWorkloads, &score);
			if (bc != -1)
			{
				id2cl[idx] = bc;
				m_vClusters[bc].vConeIds.push_back(idx);
				m_vClusters[bc].iWorkload += mffcWorkloads[idx];
				{
					char buf[256];
					snprintf(buf, sizeof(buf), "%d\t%d\t%d\t%.4f\t0", idx,
							 bc, (int)mffcWorkloads[idx], score);
					pifTelemetryRow("pif_grow.tsv",
									"mffcIdx\tclusterId\tworkload\tscore\torphan", buf);
				}
			}
			else
			{
				assignOrphanMffc(idx, id2cl, mffcWorkloads);
				{
					char buf[256];
					snprintf(buf, sizeof(buf), "%d\t%d\t%d\t%.4f\t1", idx,
							 id2cl[idx], (int)mffcWorkloads[idx], score);
					pifTelemetryRow("pif_grow.tsv",
									"mffcIdx\tclusterId\tworkload\tscore\torphan", buf);
				}
			}
		}
		m_vMffcId2ClusterId = id2cl;
	}

	int MetisAig::findBestClusterForMffc(int mIdx, const PartitionConfig &config,
										 const vector<int> &id2cl, const vector<int32_t> &wls,
										 double *pOutScore)
	{
		int best = -1;
		double maxS = -1e18;
		int32_t mffcWL = wls[mIdx];
		if (pOutScore)
			*pOutScore = 0.0;

		// Count total boundary edges of this MFFC (proxy for how many
		// cut edges placing it in the wrong cluster would create)
		int totalBoundaryEdges = (int)m_vMffcs[mIdx].sAdjacentMffcId.size();

		for (auto &cl : m_vClusters)
		{
			if (cl.iWorkload + mffcWL > config.workloadLimit)
				continue;

			// Count how many adjacent MFFCs are already in this cluster
			int shared = 0;
			for (auto nb : m_vMffcs[mIdx].sAdjacentMffcId)
				if (id2cl[nb] == cl.iId)
					shared++;

			// Cut edges if placed in this cluster = totalBoundaryEdges - shared
			int cutEdges = totalBoundaryEdges - shared;

			double avg = (double)std::max(config.avgWorkload, (int64_t)1);
			double imb = fabs((double)cl.iWorkload + mffcWL - avg) / avg;

			// New scoring: strongly reward shared edges, penalize cut edges
			// shared edges = edges that WON'T be cut (good)
			// cutEdges = edges that WILL be cut (bad for QoR)
			// lambda_balance = penalty for load imbalance
			// lambda_cut = penalty for each cut edge
			double lambda_balance = 0.2;
			double lambda_cut = 0.5;

			double score;
			if (shared > 0)
				score = (double)shared * 2.0 - lambda_cut * cutEdges - lambda_balance * imb;
			else
				score = -lambda_cut * cutEdges - imb;

			if (score > maxS)
			{
				maxS = score;
				best = cl.iId;
			}
		}
		if (pOutScore)
			*pOutScore = (best >= 0) ? maxS : -1e18;
		return best;
	}

	void MetisAig::assignOrphanMffc(int mIdx, vector<int> &id2cl, const vector<int32_t> &wls)
	{
		int best = -1;
		int64_t minL = INT64_MAX;
		for (auto &c : m_vClusters)
			if (c.iWorkload < minL)
			{
				minL = c.iWorkload;
				best = c.iId;
			}
		if (best >= 0)
		{
			id2cl[mIdx] = best;
			m_vClusters[best].vConeIds.push_back(mIdx);
			m_vClusters[best].iWorkload += wls[mIdx];
		}
	}

	void MetisAig::postProcessMffcClusters()
	{
		{ // Rebalance (skip when user requests exact K)
			if (m_forceK <= 0)
			{
				vector<int32_t> wls(m_vMffcs.size());
				for (int i = 0; i < (int)m_vMffcs.size(); i++)
					wls[i] = m_vMffcs[i].iWorkload;
				rebalanceMffcClusters(wls);
			}
			else
				ylog("[MFFC] Skipping rebalance (user requested K=%d)\n", m_forceK);
		}

		// Remove empty, recalc
		m_vClusters.erase(std::remove_if(m_vClusters.begin(), m_vClusters.end(),
										 [](const Cluster &c)
										 { return c.vConeIds.empty(); }),
						  m_vClusters.end());

		m_iTotalWorkLoad = 0;
		m_iMaxClusterWorkLoad = 0;
		for (auto &cl : m_vClusters)
		{
			cl.iWorkload = 0;
			cl.nNodes = 0;
			for (auto mid : cl.vConeIds)
			{
				cl.iWorkload += m_vMffcs[mid].iWorkload;
				cl.nNodes += m_vMffcs[mid].nNodes;
			}
			m_iTotalWorkLoad += cl.iWorkload;
		}

		std::sort(m_vClusters.begin(), m_vClusters.end(),
				  [](const Cluster &a, const Cluster &b)
				  { return a.iWorkload > b.iWorkload; });
		if (!m_vClusters.empty())
			m_iMaxClusterWorkLoad = m_vClusters[0].iWorkload;

		m_vMffcId2ClusterId.assign(m_vMffcs.size(), -1);
		for (int i = 0; i < (int)m_vClusters.size(); i++)
		{
			m_vClusters[i].iId = i;
			for (auto mid : m_vClusters[i].vConeIds)
				m_vMffcId2ClusterId[mid] = i;
		}

		uint32_t upB = m_pMG->get_sCluster();
		if (upB != 0 && m_forceK <= 0)
			mergeSmallClusters(upB);

		if (m_forceK <= 0)
			splitOversizedMffcClusters();
		else
			ylog("[MFFC] Skipping split (user requested K=%d)\n", m_forceK);

		// Cap partitions (skip when user requests exact K)
		if (m_forceK <= 0)
		{
			int maxC = std::thread::hardware_concurrency();
			if (maxC < 1)
				maxC = 40;
			int maxP = (int)(maxC * 1.5);
			while ((int)m_vClusters.size() > maxP && m_vClusters.size() > 1)
			{
				int mi = 0;
				for (int i = 1; i < (int)m_vClusters.size(); i++)
					if (m_vClusters[i].iWorkload < m_vClusters[mi].iWorkload)
						mi = i;
				int bt = -1;
				int64_t bw = INT64_MAX;
				for (int i = 0; i < (int)m_vClusters.size(); i++)
					if (i != mi && m_vClusters[i].iWorkload < bw)
					{
						bt = i;
						bw = m_vClusters[i].iWorkload;
					}
				if (bt < 0)
					break;
				{
					char buf[256];
					snprintf(buf, sizeof(buf), "%d\t%d\t%d", mi, bt,
							 (int)m_vClusters[mi].iWorkload);
					pifTelemetryRow("pif_cap.tsv",
									"clusterId\ttargetId\tworkload", buf);
				}
				for (auto mid : m_vClusters[mi].vConeIds)
					m_vClusters[bt].vConeIds.push_back(mid);
				m_vClusters[bt].iWorkload += m_vClusters[mi].iWorkload;
				m_vClusters[bt].nNodes += m_vClusters[mi].nNodes;
				m_vClusters.erase(m_vClusters.begin() + mi);
				for (int i = 0; i < (int)m_vClusters.size(); i++)
					m_vClusters[i].iId = i;
			}
		}
		else
			ylog("[MFFC] Skipping partition cap (user requested K=%d)\n", m_forceK);

		m_vMffcId2ClusterId.assign(m_vMffcs.size(), -1);
		for (int i = 0; i < (int)m_vClusters.size(); i++)
		{
			m_vClusters[i].iId = i;
			for (auto mid : m_vClusters[i].vConeIds)
				m_vMffcId2ClusterId[mid] = i;
		}

		ylog("[MFFC] Final: %d partitions, totalWL=%lld\n", (int)m_vClusters.size(), (long long)m_iTotalWorkLoad);
		printClusters();

		// Telemetry: final cluster table (cluster id, predicted workload,
		// node count, cone/MFFC count, max level) and the predicted
		// critical cluster.
		{
			char buf[256];
			int critIdx = -1;
			int64_t critWl = -1;
			for (int i = 0; i < (int)m_vClusters.size(); i++)
			{
				const Cluster &c = m_vClusters[i];
				snprintf(buf, sizeof(buf), "%d\t%d\t%d\t%zu\t%d",
						 c.iId, (int)c.iWorkload, (int)c.nNodes,
						 c.vConeIds.size(), (int)c.iMaxLevel);
				pifTelemetryRow("pif_final.tsv",
								"clusterId\tworkload\tnNodes\tnMffc\tiMaxLevel", buf);
				if ((int64_t)c.iWorkload > critWl)
				{
					critWl = c.iWorkload;
					critIdx = i;
				}
			}
			snprintf(buf, sizeof(buf), "critical\t%d\t%lld", critIdx, (long long)critWl);
			pifTelemetryRow("pif_final.tsv",
							"clusterId\tworkload\tnNodes\tnMffc\tiMaxLevel", buf);
		}
	}

	void MetisAig::rebalanceMffcClusters(vector<int32_t> &mffcWorkloads)
	{
		int nM = (int)m_vMffcs.size(), nC = (int)m_vClusters.size();
		if (nC <= 1)
			return;

		m_vMffcId2ClusterId.assign(nM, -1);
		for (int ci = 0; ci < nC; ci++)
			for (auto mid : m_vClusters[ci].vConeIds)
				m_vMffcId2ClusterId[mid] = ci;

		int64_t totalWL = 0;
		for (auto &c : m_vClusters)
			totalWL += c.iWorkload;
		double avgWL = (double)totalWL / nC;
		int64_t mergeLimit = (int64_t)(avgWL * 0.30);

		vector<int> ord(nC);
		std::iota(ord.begin(), ord.end(), 0);
		std::sort(ord.begin(), ord.end(), [&](int a, int b)
				  { return m_vClusters[a].iWorkload < m_vClusters[b].iWorkload; });

		vector<bool> alive(nC, true);
		int mc = 0;
		for (int ci : ord)
		{
			if (!alive[ci] || m_vClusters[ci].iWorkload > mergeLimit)
				break;

			std::map<int, int> nbEdges;
			for (auto mid : m_vClusters[ci].vConeIds)
				for (auto nb : m_vMffcs[mid].sAdjacentMffcId)
					if (nb >= 0 && nb < nM)
					{
						int nc = m_vMffcId2ClusterId[nb];
						if (nc >= 0 && nc != ci && alive[nc])
							nbEdges[nc]++;
					}
			if (nbEdges.empty())
			{
				int li = -1;
				int64_t lw = INT64_MAX;
				for (int j = 0; j < nC; j++)
					if (j != ci && alive[j] && m_vClusters[j].iWorkload < lw)
					{
						li = j;
						lw = m_vClusters[j].iWorkload;
					}
				if (li >= 0)
					nbEdges[li] = 0;
			}
			if (nbEdges.empty())
				continue;

			int bt = -1;
			int be = -1;
			int64_t bw = INT64_MAX;
			for (auto &[nc, e] : nbEdges)
				if (e > be || (e == be && m_vClusters[nc].iWorkload < bw))
				{
					bt = nc;
					bw = m_vClusters[nc].iWorkload;
					be = e;
				}
			if (bt < 0)
				continue;

			for (auto mid : m_vClusters[ci].vConeIds)
			{
				m_vClusters[bt].vConeIds.push_back(mid);
				m_vMffcId2ClusterId[mid] = bt;
			}
			{
				char buf[256];
				snprintf(buf, sizeof(buf), "%d\t%d\t%d\t%d", ci, bt, be,
						 (int)m_vClusters[ci].iWorkload);
				pifTelemetryRow("pif_rebalance.tsv",
								"clusterId\ttargetId\tedges\tworkload", buf);
			}
			m_vClusters[bt].iWorkload += m_vClusters[ci].iWorkload;
			m_vClusters[bt].nNodes += m_vClusters[ci].nNodes;
			m_vClusters[ci].vConeIds.clear();
			m_vClusters[ci].iWorkload = 0;
			m_vClusters[ci].nNodes = 0;
			alive[ci] = false;
			mc++;
		}
		if (mc > 0)
			ylog("[MFFC-Rebal] Merged %d lightweight clusters\n", mc);
	}

	void MetisAig::splitOversizedMffcClusters()
	{
		int maxC = std::thread::hardware_concurrency();
		if (maxC < 1)
			maxC = 40;
		int64_t totalN = 0;
		for (auto &c : m_vClusters)
			totalN += c.nNodes;
		int64_t thr = std::min((int64_t)50000, totalN / std::max(maxC / 2, 1));
		if (thr < 5000)
			thr = 5000;

		int origSz = (int)m_vClusters.size(), sc = 0;
		for (int ci = 0; ci < origSz; ci++)
		{
			if (m_vClusters[ci].nNodes <= thr || m_vClusters[ci].vConeIds.size() <= 1)
				continue;
			int nS = std::max(2, (int)((m_vClusters[ci].nNodes + thr - 1) / thr));
			{
				char buf[256];
				snprintf(buf, sizeof(buf), "%d\t%lld\t%d\t%lld\t%d\t%lld",
						 ci, (long long)m_vClusters[ci].nNodes, (int)thr, nS,
						 (long long)m_vClusters[ci].iWorkload,
						 (long long)(m_vClusters[ci].iWorkload / std::max(nS, 1)));
				pifTelemetryRow("pif_split.tsv",
								"clusterId\tnNodes\tthr\tnSplits\tworkload\tperSplit", buf);
			}

			vector<pair<int, int32_t>> infos;
			for (auto mid : m_vClusters[ci].vConeIds)
				infos.push_back({mid, m_vMffcs[mid].iWorkload});
			std::sort(infos.begin(), infos.end(),
					  [&](const pair<int, int32_t> &a, const pair<int, int32_t> &b)
					  { return m_vMffcs[a.first].iMaxLevel < m_vMffcs[b.first].iMaxLevel; });

			int64_t tw = 0;
			for (auto &[id, w] : infos)
				tw += w;
			int64_t perS = std::max((int64_t)1, tw / nS);

			vector<vector<int>> splits(nS);
			int cs = 0;
			int64_t cw = 0;
			for (int j = 0; j < (int)infos.size(); j++)
			{
				splits[cs].push_back(infos[j].first);
				cw += infos[j].second;
				if (cw >= perS && cs < nS - 1 && j < (int)infos.size() - 1)
				{
					cs++;
					cw = 0;
				}
			}
			splits.erase(
				std::remove_if(splits.begin(), splits.end(),
							   [](const std::vector<int> &v)
							   { return v.empty(); }),
				splits.end());
			if ((int)splits.size() < 2)
				continue;

			m_vClusters[ci].vConeIds = splits[0];
			m_vClusters[ci].iWorkload = 0;
			m_vClusters[ci].nNodes = 0;
			for (auto mid : m_vClusters[ci].vConeIds)
			{
				m_vClusters[ci].iWorkload += m_vMffcs[mid].iWorkload;
				m_vClusters[ci].nNodes += m_vMffcs[mid].nNodes;
			}
			for (int s = 1; s < (int)splits.size(); s++)
			{
				Cluster nc;
				nc.iId = (int)m_vClusters.size();
				nc.iMaxLevel = 0;
				nc.iWorkload = 0;
				nc.nNodes = 0;
				nc.iPartitionId = -1;
				for (auto mid : splits[s])
				{
					nc.vConeIds.push_back(mid);
					nc.iWorkload += m_vMffcs[mid].iWorkload;
					nc.nNodes += m_vMffcs[mid].nNodes;
					if (m_vMffcs[mid].iMaxLevel > nc.iMaxLevel)
						nc.iMaxLevel = m_vMffcs[mid].iMaxLevel;
				}
				m_vClusters.push_back(nc);
			}
			sc++;
		}
		if (sc > 0)
		{
			m_vClusters.erase(std::remove_if(m_vClusters.begin(), m_vClusters.end(),
											 [](const Cluster &c)
											 { return c.vConeIds.empty(); }),
							  m_vClusters.end());
			std::sort(m_vClusters.begin(), m_vClusters.end(),
					  [](const Cluster &a, const Cluster &b)
					  { return a.iWorkload > b.iWorkload; });
			for (int i = 0; i < (int)m_vClusters.size(); i++)
				m_vClusters[i].iId = i;
			ylog("[MFFC-Split] Split %d clusters -> %d total\n", sc, (int)m_vClusters.size());
		}
	}

	int32_t MetisAig::partitionAigMffc()
	{
		int32_t nParts = (int32_t)m_vClusters.size();
		vector<Partition> partitions(nParts);
		for (auto &cluster : m_vClusters)
		{
			partitions[0].addCluster(cluster);
			sort(partitions.begin(), partitions.end(),
				 [](const Partition &a, const Partition &b)
				 { return a.iWorkload < b.iWorkload; });
		}

		m_vCluster2PartitionId.assign(m_vClusters.size(), -1);
		for (int i = 0; i < (int)partitions.size(); i++)
			for (auto cid : partitions[i].vClusterIds)
			{
				setGraphPartitionMffc(m_vClusters[cid], i);
				if (cid >= 0 && cid < (int32_t)m_vCluster2PartitionId.size())
					m_vCluster2PartitionId[cid] = i;
			}

		// Task 17 Stage 2: behavior-neutral PartitionUnit/hypergraph
		// telemetry over the control assignment (gated by
		// PIF_TELEMETRY_DIR; never affects selection).
		buildHypergraphTelemetry();

		// Telemetry: per-partition predicted workload in graph partition order.
		m_vPartitionWorkload.clear();
		m_vPartitionWorkload.reserve(partitions.size());
		for (const auto &p : partitions)
			m_vPartitionWorkload.push_back(p.iWorkload);
		{
			int critIdx = -1;
			int64_t critWl = -1;
			char buf[256];
			for (int i = 0; i < (int)m_vPartitionWorkload.size(); i++)
			{
				snprintf(buf, sizeof(buf), "part\t%d\t%lld", i,
						 (long long)m_vPartitionWorkload[i]);
				pifTelemetryRow("pif_partition.tsv",
								"kind\tpartitionIdx\tworkload", buf);
				if (m_vPartitionWorkload[i] > critWl)
				{
					critWl = m_vPartitionWorkload[i];
					critIdx = i;
				}
			}
			snprintf(buf, sizeof(buf), "critical\t%d\t%lld", critIdx, (long long)critWl);
			pifTelemetryRow("pif_partition.tsv",
							"kind\tpartitionIdx\tworkload", buf);
		}

		// Safety net: assign all remaining unpartitioned non-PO nodes.
		// MFFC only covers AND nodes; PI/Const nodes that are not
		// fanins of any AND node in the same partition would be missed.
		// Also handles: dangling PIs, PI->PO direct paths, Const1 node.
		m_pMG->fixUnpartitionedNodes();

		m_pMG->setPoPart();
		m_pMG->setNumParts(nParts);
		return nParts;
	}

	void MetisAig::setGraphPartitionMffc(Cluster &cluster, int32_t partId)
	{
		for (auto mffcId : cluster.vConeIds)
		{
			auto &mffc = m_vMffcs[mffcId];
			for (auto nodeId : mffc.vNodeIds)
			{
				// Set partition for this AND node
				if (m_vNodes[nodeId].iData != -1)
					m_pMG->setNodePart(m_vNodes[nodeId].iData, partId);

				auto &nd = m_vNodes[nodeId];
				if (!nd.isPi() && !nd.isPo())
				{
					// Set partition for ALL fanins (PI or AND)
					// For PI fanins: they can be shared by multiple partitions,
					// setNodePart handles this (first-write-wins)
					// For AND fanins: they should already have been set by their
					// own MFFC, but if not (edge case), set them here as fallback
					int32_t f0 = nd.getFanin0Id();
					if (f0 >= 0 && (int)f0 < (int)m_vNodes.size() && m_vNodes[f0].iData != -1)
						m_pMG->setNodePart(m_vNodes[f0].iData, partId);
					int32_t f1 = nd.getFanin1Id();
					if (f1 >= 0 && (int)f1 < (int)m_vNodes.size() && m_vNodes[f1].iData != -1)
						m_pMG->setNodePart(m_vNodes[f1].iData, partId);
				}
			}
		}
	}

	void MetisAig::buildHypergraphTelemetry()
	{
		if (!pifTelemetryDir())
			return;

		// unit -> partition from the control clustering.
		std::vector<int32_t> unit2part(m_vMffcs.size(), -1);
		for (size_t ui = 0; ui < m_vMffcs.size(); ui++)
		{
			int32_t cid = (ui < m_vMffcId2ClusterId.size()) ? m_vMffcId2ClusterId[ui] : -1;
			if (cid >= 0 && cid < (int32_t)m_vCluster2PartitionId.size())
				unit2part[ui] = m_vCluster2PartitionId[cid];
		}

		m_hg.build(m_vNodes, m_vNode2MffcId, m_vMffcs, m_vUnitOrigins, m_vUnitSplit);
		m_hg.assignControl(unit2part);

		// Strict (pre-merge) MFFC origins per post-preprocessing unit.
		{
			char buf[256];
			for (size_t ui = 0; ui < m_vUnitOrigins.size(); ui++)
				for (int32_t o : m_vUnitOrigins[ui])
				{
					snprintf(buf, sizeof(buf), "%zu\t%d", ui, (int)o);
					pifTelemetryRow("pif_unit_origin.tsv",
									"unitId\toriginMffcId", buf);
				}
		}
		m_hg.exportTelemetry();

		ylog("[MFFC-HG] %lld units, %lld edges (%lld internal, %lld PI, %lld PO, "
			 "%lld const), %lld singleton, %lld cut nets, connectivity=%lld, "
			 "crossPartPins=%lld, predInterfaces=%lld, partCapMax=%lld (CV %.3f)\n",
			 (long long)m_hg.nUnits(), (long long)m_hg.nEdges(),
			 (long long)m_hg.nInternalEdges(), (long long)m_hg.nPiEdges(),
			 (long long)m_hg.nPoEdges(), (long long)m_hg.nConstEdges(),
			 (long long)m_hg.nSingletonEdges(), (long long)m_hg.nCutNets(),
			 (long long)m_hg.connectivityCost(), (long long)m_hg.crossPartitionPins(),
			 (long long)m_hg.predictedInterfaces(), (long long)m_hg.partCapacityMax(),
			 m_hg.partCapacityCv());
	}

	void MetisAig::runHypergraphPartitioning(const PartitionConfig &config,
											 const vector<int32_t> &mffcWorkloads)
	{
		// Deterministic internal multilevel partitioner over the
		// post-preprocessing PartitionUnits. Target K comes from the same
		// explicit-K or adaptive-K calculation as the control; exactly K
		// non-empty parts are produced (K = min(targetK, nM)).
		if (getenv("PIF_HG_DEBUG"))
			ylog("[HG-DBG] runHypergraphPartitioning: build start\n");
		m_hg.build(m_vNodes, m_vNode2MffcId, m_vMffcs, m_vUnitOrigins, m_vUnitSplit);
		if (getenv("PIF_HG_DEBUG"))
			ylog("[HG-DBG] hypergraph built: units=%lld edges=%lld\n",
				 (long long)m_hg.nUnits(), (long long)m_hg.nEdges());
		int32_t K = std::min(config.targetK, (int32_t)m_vMffcs.size());
		if (K < 1)
			K = 1;
		std::vector<int32_t> unit2part = m_hg.partitionMultiway(K);

		// One cluster per part; every unit is assigned exactly once.
		m_vClusters.clear();
		m_vClusters.reserve(K);
		for (int32_t p = 0; p < K; p++)
			m_vClusters.emplace_back(p, 0);
		for (size_t ui = 0; ui < m_vMffcs.size(); ui++)
		{
			int32_t p = (ui < unit2part.size()) ? unit2part[ui] : -1;
			if (p < 0 || p >= K)
				p = 0; // safety net: never lose or duplicate a unit
			m_vClusters[p].vConeIds.push_back((int32_t)ui);
			m_vClusters[p].iWorkload += mffcWorkloads[ui];
			m_vClusters[p].nNodes += m_vMffcs[ui].nNodes;
			if (m_vMffcs[ui].iMaxLevel > m_vClusters[p].iMaxLevel)
				m_vClusters[p].iMaxLevel = m_vMffcs[ui].iMaxLevel;
		}
		m_vMffcId2ClusterId.assign(m_vMffcs.size(), -1);
		for (int32_t p = 0; p < K; p++)
			for (auto mid : m_vClusters[p].vConeIds)
				m_vMffcId2ClusterId[mid] = p;

		m_iTotalWorkLoad = 0;
		m_iMaxClusterWorkLoad = 0;
		for (auto &cl : m_vClusters)
		{
			m_iTotalWorkLoad += cl.iWorkload;
			if (cl.iWorkload > m_iMaxClusterWorkLoad)
				m_iMaxClusterWorkLoad = cl.iWorkload;
		}
		ylog("[MFFC] Final: %d partitions, totalWL=%lld\n",
			 (int)m_vClusters.size(), (long long)m_iTotalWorkLoad);
		printClusters();
	}

	void MetisAig::parseAigMffc(int32_t userK, int32_t effectiveJ)
	{
		m_useMffc = true;
		m_forceK = userK;
		vector<int32_t> mffcWorkloads;
		preprocessMffcs(mffcWorkloads);
		PartitionConfig config = determineMffcPartitionConfig(mffcWorkloads, userK, effectiveJ);
		runHypergraphPartitioning(config, mffcWorkloads);
	}

} // for namespace
