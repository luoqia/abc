/**CFile****************************************************************

  FileName    [lpkPar.c]

  PackageName [Fast Boolean matching for LUT structures.]

  Synopsis    [Private-window parallel lutpack evaluation/commit engine.]

  Description [Task 79 candidate mode (env LUTPACK_WORKERS>1).  Each outer
  saturation pass is split into a parallel read-only evaluation phase and a
  serial validated commit phase.  A per-pivot evaluation uses per-worker
  private scratch only (drain/union marks, cut storage, truth tables, DSD
  manager); it never writes shared network state, so the network is a
  consistent read epoch.  Commits are applied serially in the original
  pivot order.  A pivot whose evaluation context (window liveness, leaf
  levels, required level) drifted since the parallel evaluation, or that
  became offered only after mid-pass deletions, is re-evaluated with the
  original serial machinery on the live network.  The parallel mode is
  therefore result-identical to the serial mode pass by pass.]

  SideEffects []

  SeeAlso     [lpkCore.c, lpkCut.c, lpkAbcDec.c]

***********************************************************************/

#include "lpkInt.h"
#include "bool/kit/cloud.h"
#include "base/main/main.h"
#include <stdlib.h>
#include <string.h>

#ifdef ABC_USE_PTHREADS
#include <pthread.h>
#endif

ABC_NAMESPACE_IMPL_START

extern int         Lpk_ResynthesizeNodeNew( Lpk_Man_t * p );
extern int         Lpk_NodeHasChanged( Lpk_Man_t * p, int iNode );
extern Abc_Obj_t * Lpk_Implement( Lpk_Man_t * pMan, Abc_Ntk_t * pNtk, Vec_Ptr_t * vLeaves, int nLeavesOld );
extern void        Lpk_DecomposeClean( Vec_Ptr_t * vLeaves, int nLeavesOld );
extern int         Lpk_Decompose_rec( Lpk_Man_t * pMan, Lpk_Fun_t * p );
extern void        Lpk_NodeCutSignature( Lpk_Cut_t * pCut );
extern int         Lpk_NodeCutsOneFilter( Lpk_Man_t * p, Lpk_Cut_t * pCutNew );
extern Abc_Ntk_t * Abc_NtkDecFromTruth( word * pTruth, int nVars, int nLutSize );
extern Abc_Obj_t * Abc_NtkLutMinDecompose( Abc_Ntk_t * pNtk, Vec_Ptr_t * vLeaves, word * pTruth, int nLutSize, int Required );

/* serial per-cut fallback gate table (mirror of Lpk_ResynthesizeNodeNew) */
static const int Lpk_ParNodeCounts[16] = { 0, 0, 0, 0, 1, 3, 6, 14, 26, 57, 106, 230, 425, 1000000, 1000000, 1000000 };

#ifdef ABC_USE_PTHREADS
static pthread_mutex_t lpk_par_lutmin_lock = PTHREAD_MUTEX_INITIALIZER;
#endif

#define LPK_PAR_CHUNK_PIVOTS  2048
#define LPK_PAR_MEMO_SLOTS    65536

typedef struct Lpk_ParMemo_t_
{
    void *       pKey;
    unsigned *   pVal;
    unsigned     Epoch;
} Lpk_ParMemo_t;

/* per-pivot evaluation record (created only for pivots with MFFC > 1) */
typedef struct Lpk_ParRec_t_
{
    int          fPlan;        // 1: candidate plan exists (fLutMin selects kind)
    int          fLutMin;      // plan = deferred K==4 lutmin fallback
    int          fConsumed;    // plan was materialized at commit
    int          Required;     // pivot required level at evaluation
    int          nLeaves;      // plan cut leaf count
    int          nNodes;       // plan cut window node count
    int          nNodesDup;
    int          nFuns;        // decomposition plan Fun entry count
    Lpk_Fun_t ** ppFuns;       // owned Fun entries
    int *        pLeafIds;     // plan leaf ids
    Vec_Int_t *  vWin;         // window interior ids read by enumeration
    Vec_Int_t *  vLeaves;      // (id, level) pairs of every enumerated leaf
    unsigned *   pTruth;       // lutmin plan: truth words
} Lpk_ParRec_t;

typedef struct Lpk_ParCtx_t_ Lpk_ParCtx_t;
struct Lpk_ParCtx_t_
{
    int         iWorker;
    Lpk_Man_t * pShared;     // shared manager (vVisited etc.)
    Lpk_Man_t * pMan;
    Abc_Ntk_t * pNtk;
    int *       pCntGen;
    int *       pCnt;
    int *       pUG;
    int         nMarkSize;
    int         GenDrain;
    int         GenU;
    Vec_Int_t * vStack;
    Vec_Int_t * vWinCur;
    Vec_Int_t * vLeafCur;
    Vec_Ptr_t * vNT;           // (nodeId, truth) pairs of evaluated window nodes
    Lpk_ParMemo_t * Memo;
    int         nMemo;
    unsigned    MemoEpoch;
    // Task 80 F3 closure: per-worker debug counters (the rev-1 code
    // incremented process-global counters from worker threads).
    int         nDbgEvals;
    int         nDbgGate;
    int         nDbgDec;
};

typedef struct Lpk_ParPass_t_
{
    Lpk_Man_t *    p;
    Abc_Ntk_t *    pNtk;
    int *          vPivots;     // phase-A offered pivot ids
    char *         pOffered;    // per-node-id offered bitmap (phase A list)
    int            nNodes;
    Lpk_ParRec_t ** ppRecs;     // per-node-id records
    int            nPivots;
    int            nWorkers;
    int            nChunks;
    int            NextChunk;   // atomic
    int            nDbgEvals;   // summed per-worker counters (F3)
    int            nDbgGate;
    int            nDbgDec;
} Lpk_ParPass_t;

////////////////////////////////////////////////////////////////////////
///            pure helpers (mirrors of lpkCut.c/lpkCore.c)          ///
////////////////////////////////////////////////////////////////////////

static inline int Lpk_ParIsMember( Lpk_ParCtx_t * pC, int v )
{
    return pC->pCntGen[v] == pC->GenDrain && pC->pCnt[v] == 0;
}

static inline int Lpk_ParCutHasLeafInWindow( Lpk_Cut_t * pCut )
{
    int i, k;
    for ( i = 0; i < (int)pCut->nNodes; i++ )
        for ( k = 0; k < (int)pCut->nLeaves; k++ )
            if ( pCut->pNodes[i] == pCut->pLeaves[k] )
                return 1;
    return 0;
}

/* Private drain: exact simulation of Abc_NodeDeref_rec using private
   counts; never writes network fanout counts or traversal marks.  If
   pLeafIds is non-NULL the listed leaves receive one phantom fanout (the
   serial eval-time gate).  Returns the number of drained internal nodes
   including the pivot (CIs never count). */
static int Lpk_ParDrain( Lpk_ParCtx_t * pC, int vPivot, int * pLeafIds, int nLeaves )
{
    Abc_Ntk_t * pNtk = pC->pNtk;
    int Counter = 1, v, k, m, iFanin;
    Abc_Obj_t * pObj, * pFanin;
    pC->GenDrain++;
    if ( pC->GenDrain == 0 )
    {
        memset( pC->pCntGen, 0, sizeof(int) * pC->nMarkSize );
        pC->GenDrain = 1;
    }
    Vec_IntClear( pC->vStack );
    Vec_IntPush( pC->vStack, vPivot );
    while ( Vec_IntSize(pC->vStack) > 0 )
    {
        v = Vec_IntPop( pC->vStack );
        pObj = Abc_NtkObj( pNtk, v );
        if ( Abc_ObjIsCi(pObj) )
            continue;
        assert( Abc_ObjIsNode(pObj) );
        Abc_ObjForEachFanin( pObj, pFanin, k )
        {
            iFanin = pFanin->Id;
            if ( pC->pCntGen[iFanin] != pC->GenDrain )
            {
                pC->pCntGen[iFanin] = pC->GenDrain;
                pC->pCnt[iFanin] = Abc_ObjFanoutNum(pFanin);
                if ( pLeafIds )
                    for ( m = 0; m < nLeaves; m++ )
                        if ( pLeafIds[m] == iFanin )
                        {
                            pC->pCnt[iFanin]++;
                            break;
                        }
            }
            assert( pC->pCnt[iFanin] > 0 );
            if ( --pC->pCnt[iFanin] == 0 && !Abc_ObjIsCi(pFanin) )
            {
                Vec_IntPush( pC->vStack, iFanin );
                Counter++;
            }
        }
    }
    return Counter;
}

/* mirror of Lpk_NodeCutsCheckDsd without network mark writes */
static int Lpk_ParCheckDsd( Lpk_Man_t * p, Lpk_Cut_t * pCut )
{
    Abc_Obj_t * pObj, * pFanin;
    int i, k, m, nCands, fLeavesOnly, RetValue, iLeaf;
    assert( pCut->nLeaves > 0 );
    memset( p->pRefs, 0, sizeof(int) * pCut->nLeaves );
    nCands = 0;
    Lpk_CutForEachNode( p->pNtk, pCut, pObj, i )
    {
        fLeavesOnly = 1;
        Abc_ObjForEachFanin( pObj, pFanin, k )
        {
            iLeaf = -1;
            for ( m = 0; m < (int)pCut->nLeaves; m++ )
                if ( pCut->pLeaves[m] == pFanin->Id )
                {
                    iLeaf = m;
                    break;
                }
            if ( iLeaf >= 0 )
                p->pRefs[iLeaf]++;
            else
                fLeavesOnly = 0;
        }
        if ( fLeavesOnly )
            p->pCands[nCands++] = pObj->Id;
    }
    RetValue = 0;
    for ( i = 0; i < nCands; i++ )
    {
        pObj = Abc_NtkObj( p->pNtk, p->pCands[i] );
        Abc_ObjForEachFanin( pObj, pFanin, k )
        {
            iLeaf = -1;
            for ( m = 0; m < (int)pCut->nLeaves; m++ )
                if ( pCut->pLeaves[m] == pFanin->Id )
                {
                    iLeaf = m;
                    break;
                }
            assert( iLeaf >= 0 );
            if ( p->pRefs[iLeaf] > 1 )
                break;
        }
        if ( k == Abc_ObjFaninNum(pObj) )
        {
            RetValue = 1;
            break;
        }
    }
    return RetValue;
}

/* mirror of Lpk_NodeCutsOne with private membership marks */
static void Lpk_ParNodeCutsOne( Lpk_Man_t * p, Lpk_ParCtx_t * pC, Lpk_Cut_t * pCut, int Node )
{
    Lpk_Cut_t * pCutNew;
    Abc_Obj_t * pObj, * pFanin;
    int i, k, j, nLeavesNew;
    if ( pCut->nNodes == LPK_SIZE_MAX )
        return;
    pObj = Abc_NtkObj( p->pNtk, Node );
    if ( Abc_ObjIsCi(pObj) )
        return;
    assert( Abc_ObjIsNode(pObj) );
    if ( !Lpk_ParIsMember( pC, Node ) )
    {
        if ( (int)pCut->nNodesDup == p->pPars->nLutsOver )
            return;
        assert( (int)pCut->nNodesDup < p->pPars->nLutsOver );
    }
    nLeavesNew = pCut->nLeaves - 1;
    Abc_ObjForEachFanin( pObj, pFanin, i )
    {
        if ( (pCut->uSign[(pFanin->Id & 32) > 0] & (1 << (pFanin->Id & 31))) )
            continue;
        if ( ++nLeavesNew > p->pPars->nVarsMax )
            return;
    }
    assert( p->nCuts < LPK_CUTS_MAX );
    pCutNew = p->pCuts + p->nCuts;
    pCutNew->nLeaves = 0;
    for ( i = 0; i < (int)pCut->nLeaves; i++ )
        if ( pCut->pLeaves[i] != Node )
            pCutNew->pLeaves[pCutNew->nLeaves++] = pCut->pLeaves[i];
    Abc_ObjForEachFanin( pObj, pFanin, i )
    {
        for ( k = 0; k < (int)pCutNew->nLeaves; k++ )
            if ( pCutNew->pLeaves[k] >= pFanin->Id )
                break;
        if ( k < (int)pCutNew->nLeaves && pCutNew->pLeaves[k] == pFanin->Id )
            continue;
        if ( (int)pCutNew->nLeaves == p->pPars->nVarsMax )
            return;
        for ( j = pCutNew->nLeaves; j > k; j-- )
            pCutNew->pLeaves[j] = pCutNew->pLeaves[j-1];
        pCutNew->pLeaves[k] = pFanin->Id;
        pCutNew->nLeaves++;
        assert( pCutNew->nLeaves <= LPK_SIZE_MAX );
    }
    Lpk_NodeCutSignature( pCutNew );
    if ( Lpk_NodeCutsOneFilter( p, pCutNew ) )
        return;
    assert( pCut->nNodes < LPK_SIZE_MAX );
    memcpy( pCutNew->pNodes, pCut->pNodes, pCut->nNodes * sizeof(int) );
    pCutNew->nNodes = pCut->nNodes;
    pCutNew->nNodesDup = pCut->nNodesDup;
    for ( i = 0; i < (int)pCutNew->nNodes; i++ )
        if ( pCutNew->pNodes[i] == Node )
        {
            for ( k = i; k < (int)pCutNew->nNodes - 1; k++ )
                pCutNew->pNodes[k] = pCutNew->pNodes[k+1];
            pCutNew->pNodes[k] = Node;
            break;
        }
    if ( i == (int)pCutNew->nNodes )
    {
        pCutNew->pNodes[ pCutNew->nNodes++ ] = Node;
        pCutNew->nNodesDup += !Lpk_ParIsMember( pC, Node );
    }
    assert( pCutNew->nNodes <= p->nMffc + pCutNew->nNodesDup );
    assert( p->nCuts < LPK_CUTS_MAX );
    p->nCuts++;
    assert( p->nLiveCuts < LPK_CUTS_MAX );
    p->pLiveCuts[p->nLiveCuts++] = p->nCuts - 1;
}

/* Private cut enumeration.  Returns 1 when enumeration happened (MFFC>1).
   Collects the evaluation-context record (window interior ids, leaf ids
   with levels) into the ctx vectors and, under saturation, replaces the
   pivot's vVisited impact list like the serial Lpk_NodeRecordImpact. */
static int Lpk_ParNodeCuts( Lpk_ParCtx_t * pC, Lpk_Man_t * p )
{
    Lpk_Cut_t * pCut, * pCut2;
    int i, k, Temp, fChanges, nMffc, v;
    Abc_Obj_t * pNode;
    Vec_Ptr_t * vNodes;
    nMffc = Lpk_ParDrain( pC, p->pObj->Id, NULL, 0 );
    // the serial traversal marks include the pivot itself; the private
    // drain never decrements the pivot, so mark it explicitly
    pC->pCntGen[p->pObj->Id] = pC->GenDrain;
    pC->pCnt[p->pObj->Id] = 0;
    p->nMffc = nMffc;
    if ( getenv("LUTPACK_DEBUG2") && pC->iWorker == 0 && nMffc > 1 )
    {
        extern int Abc_NodeMffcLabel( Abc_Obj_t * pNode, Vec_Ptr_t * vNodes );
        static int fProbed = 0;
        if ( !fProbed )
        {
            int nMffcS = Abc_NodeMffcLabel( p->pObj, NULL );
            if ( nMffcS != nMffc )
                printf( "[lpkpar] MFFC mismatch pivot=%d serial=%d par=%d\n", p->pObj->Id, nMffcS, nMffc );
            else
                printf( "[lpkpar] MFFC match pivot=%d serial=%d par=%d\n", p->pObj->Id, nMffcS, nMffc );
            fProbed = 1;
        }
    }
    if ( nMffc == 1 )
        return 0;
    // initialize the first cut
    pCut = p->pCuts; p->nCuts = 1;
    pCut->nNodes = 0;
    pCut->nNodesDup = 0;
    pCut->nLeaves = 1;
    pCut->pLeaves[0] = p->pObj->Id;
    Lpk_NodeCutSignature( pCut );
    p->nLiveCuts = 1;
    p->pLiveCuts[0] = 0;
    // cut computation
    for ( i = 0; i < p->nCuts; i++ )
    {
        pCut = p->pCuts + i;
        if ( pCut->nLeaves == 0 )
            continue;
        for ( k = 0; k < (int)pCut->nLeaves; k++ )
        {
            Lpk_ParNodeCutsOne( p, pC, pCut, pCut->pLeaves[k] );
            if ( p->nCuts == LPK_CUTS_MAX )
                break;
        }
        if ( p->nCuts == LPK_CUTS_MAX )
            break;
    }
    if ( p->nCuts == LPK_CUTS_MAX )
        p->nNodesOver++;
    // compress the cuts
    p->nEvals = 0;
    for ( i = 0; i < p->nCuts; i++ )
    {
        pCut = p->pCuts + i;
        if ( pCut->nLeaves < 2 )
            continue;
        if ( Lpk_ParCutHasLeafInWindow( pCut ) )
            continue;
        pCut->nLuts = Lpk_LutNumLuts( pCut->nLeaves, p->pPars->nLutSize );
        pCut->Weight = (float)1.0 * (pCut->nNodes - pCut->nNodesDup) / pCut->nLuts;
        if ( pCut->Weight <= 1.001 )
            continue;
        pCut->fHasDsd = Lpk_ParCheckDsd( p, pCut );
        if ( pCut->fHasDsd )
            continue;
        p->pEvals[p->nEvals++] = i;
    }
    // sort pEvals by Weight descending
    do {
        fChanges = 0;
        for ( i = 0; i < p->nEvals - 1; i++ )
        {
            pCut = p->pCuts + p->pEvals[i];
            pCut2 = p->pCuts + p->pEvals[i+1];
            if ( pCut->Weight >= pCut2->Weight - 0.001 )
                continue;
            Temp = p->pEvals[i];
            p->pEvals[i] = p->pEvals[i+1];
            p->pEvals[i+1] = Temp;
            fChanges = 1;
        }
    } while ( fChanges );
    // evaluation-context record: leaf (id,level) pairs (tag GenU+1) and the
    // window interior union (tag GenU+2); the tags are separate because a
    // node can be interior to one cut and a leaf of another
    pC->GenU++;
    if ( pC->GenU == 0 )
    {
        memset( pC->pUG, 0, sizeof(int) * pC->nMarkSize );
        pC->GenU = 1;
    }
    Vec_IntClear( pC->vLeafCur );
    for ( i = 0; i < p->nCuts; i++ )
    {
        pCut = p->pCuts + i;
        if ( pCut->nLeaves == 0 )
            continue;
        for ( k = 0; k < (int)pCut->nLeaves; k++ )
        {
            v = pCut->pLeaves[k];
            if ( pC->pUG[v] != pC->GenU )
            {
                pC->pUG[v] = pC->GenU;
                pNode = Abc_NtkObj( p->pNtk, v );
                Vec_IntPush( pC->vLeafCur, v );
                Vec_IntPush( pC->vLeafCur, Abc_ObjLevel(pNode) );
            }
        }
    }
    pC->GenU++;
    if ( pC->GenU == 0 )
    {
        memset( pC->pUG, 0, sizeof(int) * pC->nMarkSize );
        pC->GenU = 1;
    }
    Vec_IntClear( pC->vWinCur );
    for ( i = 0; i < p->nCuts; i++ )
    {
        pCut = p->pCuts + i;
        if ( pCut->nLeaves == 0 )
            continue;
        for ( k = 0; k < (int)pCut->nNodes; k++ )
        {
            v = pCut->pNodes[k];
            if ( pC->pUG[v] != pC->GenU )
            {
                pC->pUG[v] = pC->GenU;
                Vec_IntPush( pC->vWinCur, v );
            }
        }
        for ( k = 0; k < (int)pCut->nLeaves; k++ )
        {
            v = pCut->pLeaves[k];
            if ( pC->pUG[v] != pC->GenU )
            {
                pC->pUG[v] = pC->GenU;
                Vec_IntPush( pC->vWinCur, v );
            }
        }
    }
    // saturation impact: replace vVisited[pivot] with the unique leaves.
    // Task 80 F2 closure: this must NOT clear vWinCur (the full window
    // union recorded above); the leaf impact list is built from vLeafCur
    // (already deduplicated, with (id, level) pairs) into a separate list.
    if ( p->pPars->fSatur )
    {
        vNodes = Vec_VecEntry( pC->pShared->vVisited, p->pObj->Id );
        Vec_PtrClear( vNodes );
        for ( i = 0; i < Vec_IntSize(pC->vLeafCur); i += 2 )
        {
            v = Vec_IntEntry( pC->vLeafCur, i );
            pNode = Abc_NtkObj( p->pNtk, v );
            assert( pNode != NULL );
            Vec_PtrPush( vNodes, (void *)(ABC_PTRUINT_T)pNode->Id );
            Vec_PtrPush( vNodes, (void *)(ABC_PTRUINT_T)Abc_ObjFanoutNum(pNode) );
        }
    }
    return 1;
}

////////////////////////////////////////////////////////////////////////
///        private truth simulation (no Hop/network writes)          ///
////////////////////////////////////////////////////////////////////////

typedef struct Lpk_ParSim_t_
{
    Lpk_Man_t *    p;
    Lpk_ParCtx_t * pC;
    Lpk_Cut_t *    pCut;
    int            iCount;      // pool slots consumed in this cut truth call
    Abc_Obj_t *    pRoot;       // window node whose cone is simulated
    int            fInv;        // inverse variable order
    Vec_Ptr_t *    vNodeTruth;  // (nodeId, truth) of evaluated window nodes
} Lpk_ParSim_t;

/* truth of one window-node fanin given by ABC node id */
static unsigned * Lpk_ParFaninTruth( Lpk_ParSim_t * s, int vFanin )
{
    int i, k, nLeafPos;
    nLeafPos = -1;
    for ( k = 0; k < (int)s->pCut->nLeaves; k++ )
        if ( s->pCut->pLeaves[k] == vFanin )
        {
            nLeafPos = k;
            break;
        }
    if ( nLeafPos >= 0 )
        return (unsigned *)Vec_PtrEntry( s->p->vTtElems, s->fInv ? s->pCut->nLeaves - 1 - nLeafPos : nLeafPos );
    for ( i = 0; i < Vec_PtrSize(s->vNodeTruth); i += 2 )
        if ( (int)(ABC_PTRUINT_T)Vec_PtrEntry(s->vNodeTruth, i) == vFanin )
            return (unsigned *)Vec_PtrEntry( s->vNodeTruth, i + 1 );
    assert( 0 );
    return NULL;
}

/* simulate the local Hop cone of the root window node.  Hop PIs are bound
   to the truths of the root's fanins (leaf positions become variables).
   Returns the plain truth of the regular cone root; the root literal's
   phase is handled by the caller.  Memo is per window-node simulation. */
static unsigned * Lpk_ParSim_rec( Lpk_ParSim_t * s, Hop_Obj_t * pLit )
{
    Hop_Obj_t * pObjR = Hop_Regular( pLit );
    unsigned * pTruth, * pT0, * pT1;
    unsigned h;
    int k;
    Abc_Obj_t * pFan;
    if ( Hop_ObjIsPi(pObjR) )
    {
        // find the manager PI index of this object
        for ( k = 0; ; k++ )
        {
            assert( k < Hop_ManPiNum((Hop_Man_t *)s->p->pNtk->pManFunc) );
            if ( Hop_ManPi((Hop_Man_t *)s->p->pNtk->pManFunc, k) == pObjR )
                break;
        }
        // bind the k-th fanin of the root window node
        pFan = NULL;
        {
            int j = 0;
            Abc_ObjForEachFanin( s->pRoot, pFan, j )
                if ( j == k )
                    break;
        }
        assert( pFan != NULL );
        assert( !Abc_ObjIsComplement(pFan) );
        return Lpk_ParFaninTruth( s, pFan->Id );
    }
    // Task 80 F4 closure: entries whose Epoch is stale are dead and their
    // slot is immediately reusable.  Only entries created in the CURRENT
    // window-node simulation epoch are live, and one window-node cone is
    // bounded by LPK_SIZE_MAX (<= 100) objects, far below nMemo, so the
    // probe terminates at the first reusable slot (NULL key or stale epoch)
    // and can never loop without bound on a full table of stale keys.
    h = (unsigned)((ABC_PTRUINT_T)pObjR >> 4) & (s->pC->nMemo - 1);
    for ( ; ; )
    {
        if ( s->pC->Memo[h].pKey == NULL || s->pC->Memo[h].Epoch != s->pC->MemoEpoch )
            break;
        if ( s->pC->Memo[h].pKey == pObjR )
            return s->pC->Memo[h].pVal;
        h = (h + 1) & (s->pC->nMemo - 1);
    }
    if ( Hop_ObjIsConst1(pObjR) )
    {
        pTruth = (unsigned *)Vec_PtrEntry( s->p->vTtNodes, s->iCount++ );
        Kit_TruthFill( pTruth, s->pCut->nLeaves );
    }
    else
    {
        assert( Hop_ObjIsAnd(pObjR) );
        // Task 80 F1 closure: the child phase belongs to the RAW child edge,
        // not to the regularized fanin.  Hop_ObjFanin0/1 regularize, so
        // Hop_IsComplement(Hop_ObjFanin0(p)) was always 0 and complemented
        // AND-input edges lost their polarity.  Mirror the serial reference
        // lpkCut.c:155-158: recurse on the regular fanin, pass the raw-edge
        // complement flags (Hop_ObjFaninC0/C1) to Kit_TruthAndPhase.
        pT0 = Lpk_ParSim_rec( s, Hop_ObjFanin0(pObjR) );
        pT1 = Lpk_ParSim_rec( s, Hop_ObjFanin1(pObjR) );
        pTruth = (unsigned *)Vec_PtrEntry( s->p->vTtNodes, s->iCount++ );
        Kit_TruthAndPhase( pTruth, pT0, pT1, s->pCut->nLeaves,
            Hop_ObjFaninC0(pObjR), Hop_ObjFaninC1(pObjR) );
    }
    s->pC->Memo[h].pKey = pObjR;
    s->pC->Memo[h].pVal = pTruth;
    s->pC->Memo[h].Epoch = s->pC->MemoEpoch;
    return pTruth;
}

/* Evaluate all window nodes of a cut (reverse level order, mirroring the
   serial topological fix).  Returns the pivot truth: for fInv==0 the
   direct truth is copied to a private slot beyond every allocated slot
   (serial contract); for fInv==1 no copy is made and the pivot's pool slot
   is returned. */
static unsigned * Lpk_ParCutTruth( Lpk_Man_t * p, Lpk_ParCtx_t * pC, Lpk_Cut_t * pCut, int fInv )
{
    unsigned * pTruth, * pPiv;
    Abc_Obj_t * pWin;
    int i;
    Lpk_ParSim_t Sim;
    if ( pCut->nNodes > 1 )
    {
        int fSwap = 1;
        while ( fSwap )
        {
            fSwap = 0;
            for ( i = 0; i < (int)pCut->nNodes - 1; i++ )
            {
                Abc_Obj_t * pN1 = Abc_NtkObj( p->pNtk, pCut->pNodes[i] );
                Abc_Obj_t * pN2 = Abc_NtkObj( p->pNtk, pCut->pNodes[i+1] );
                if ( pN1->Level < pN2->Level )
                {
                    int Temp = pCut->pNodes[i];
                    pCut->pNodes[i] = pCut->pNodes[i+1];
                    pCut->pNodes[i+1] = Temp;
                    fSwap = 1;
                }
            }
        }
    }
    Sim.p = p;
    Sim.pC = pC;
    Sim.pCut = pCut;
    Sim.iCount = 0;
    Sim.fInv = fInv;
    Sim.vNodeTruth = pC->vNT;
    Vec_PtrClear( Sim.vNodeTruth );
    // evaluate window nodes from the lowest level to the pivot
    for ( i = (int)pCut->nNodes - 1; i >= 0; i-- )
    {
        int v, fCompl;
        unsigned * pWinTruth;
        pWin = Abc_NtkObj( p->pNtk, pCut->pNodes[i] );
        Sim.pRoot = pWin;
        pC->MemoEpoch++;
        if ( pC->MemoEpoch == 0 )
        {
            memset( pC->Memo, 0, sizeof(pC->Memo[0]) * pC->nMemo );
            pC->MemoEpoch = 1;
        }
        fCompl = Hop_IsComplement((Hop_Obj_t *)pWin->pData);
        pWinTruth = Lpk_ParSim_rec( &Sim, Hop_Regular((Hop_Obj_t *)pWin->pData) );
        if ( fCompl )
        {
            unsigned * pInv = (unsigned *)Vec_PtrEntry( p->vTtNodes, Sim.iCount++ );
            Kit_TruthCopy( pInv, pWinTruth, pCut->nLeaves );
            Kit_TruthNot( pInv, pInv, pCut->nLeaves );
            pWinTruth = pInv;
        }
        v = pWin->Id;
        Vec_PtrPush( Sim.vNodeTruth, (void *)(ABC_PTRUINT_T)v );
        Vec_PtrPush( Sim.vNodeTruth, pWinTruth );
    }
    // the last processed window node is the pivot (sorted first, highest level)
    assert( Vec_PtrSize(Sim.vNodeTruth) >= 2 );
    pPiv = (unsigned *)Vec_PtrEntry( Sim.vNodeTruth, Vec_PtrSize(Sim.vNodeTruth) - 1 );
    if ( fInv )
    {
        assert( Sim.iCount <= Vec_PtrSize(p->vTtNodes) );
        return pPiv;
    }
    pTruth = (unsigned *)Vec_PtrEntry( p->vTtNodes, Sim.iCount++ );
    Kit_TruthCopy( pTruth, pPiv, pCut->nLeaves );
    assert( Sim.iCount <= Vec_PtrSize(p->vTtNodes) );
    return pTruth;
}

/* mirror of Lpk_ComputeSupports with the private truth call */
static void Lpk_ParComputeSupports( Lpk_Man_t * p, Lpk_ParCtx_t * pC, Lpk_Cut_t * pCut, unsigned * pTruth )
{
    unsigned * pTruthInv;
    int RetValue1, RetValue2;
    pTruthInv = Lpk_ParCutTruth( p, pC, pCut, 1 );
    RetValue1 = Kit_CreateCloudFromTruth( p->pDsdMan->dd, pTruth, pCut->nLeaves, p->vBddDir );
    RetValue2 = Kit_CreateCloudFromTruth( p->pDsdMan->dd, pTruthInv, pCut->nLeaves, p->vBddInv );
    if ( RetValue1 && RetValue2 && Vec_IntSize(p->vBddDir) > 1 && Vec_IntSize(p->vBddInv) > 1 )
        Kit_TruthCofSupports( p->vBddDir, p->vBddInv, pCut->nLeaves, p->vMemory, p->puSupps );
    else
        p->puSupps[0] = p->puSupps[1] = 0;
}

////////////////////////////////////////////////////////////////////////
///            detached decomposition planning (no mutation)         ///
////////////////////////////////////////////////////////////////////////

/* mirror of Lpk_Decompose, but on success the Fun entries are transferred
   to the record instead of being implemented into the network. */
static int Lpk_ParDecomposePlan( Lpk_Man_t * p, Vec_Ptr_t * vLeaves, unsigned * pTruth,
    unsigned * puSupps, int nLutK, int AreaLim, int DelayLim, int nLeavesOld, Lpk_ParRec_t * pRec )
{
    Lpk_Fun_t * pFun;
    int nFuns, i, nLeavesPre;
    nLeavesPre = Vec_PtrSize(vLeaves);
    pFun = Lpk_FunCreate( p->pNtk, vLeaves, pTruth, nLutK, AreaLim, DelayLim );
    if ( puSupps[0] || puSupps[1] )
    {
        // puSupps holds two words per leaf; copy with the pre-create leaf
        // count (the mirror of the serial Lpk_Decompose)
        memcpy( pFun->puSupps, puSupps, sizeof(unsigned) * 2 * nLeavesPre );
        pFun->fSupports = 1;
    }
    Lpk_FunSuppMinimize( pFun );
    if ( pFun->nVars <= pFun->nLutK )
        nFuns = 1;
    else if ( !Lpk_Decompose_rec( p, pFun ) )
        nFuns = 0;
    else
        nFuns = Vec_PtrSize(vLeaves) - nLeavesOld;
    if ( nFuns == 0 )
    {
        Lpk_DecomposeClean( vLeaves, nLeavesOld );
        return 0;
    }
    pRec->fPlan = 1;
    pRec->fLutMin = 0;
    pRec->nFuns = nFuns;
    pRec->ppFuns = ABC_ALLOC( Lpk_Fun_t *, nFuns );
    for ( i = 0; i < nFuns; i++ )
        pRec->ppFuns[i] = (Lpk_Fun_t *)Vec_PtrEntry( vLeaves, nLeavesOld + i );
    Vec_PtrShrink( vLeaves, nLeavesOld );
    return 1;
}

/* Detached K==4 lutmin fallback decision: mirrors the feasibility part of
   Abc_NtkLutMinDecompose (scratch build + level check, no duplication).
   Runs under a global lock because the CUDD path is not reentrant. */
static int Lpk_ParLutMinDecide( Abc_Ntk_t * pNtk, Vec_Ptr_t * vLeaves, word * pTruth, int nLutSize, int Required )
{
    Abc_Ntk_t * pNew;
    Vec_Ptr_t * vNodes;
    Abc_Obj_t * pObj, * pFanin;
    int i, k, fOk;
#ifdef ABC_USE_PTHREADS
    pthread_mutex_lock( &lpk_par_lutmin_lock );
#endif
    pNew = Abc_NtkDecFromTruth( pTruth, Vec_PtrSize(vLeaves), nLutSize );
    fOk = 0;
    if ( pNew )
    {
        vNodes = Abc_NtkDfs( pNew, 0 );
        assert( Abc_NtkHasAig(pNew) );
        Vec_PtrForEachEntry( Abc_Obj_t *, vLeaves, pObj, i )
            Abc_NtkCi( pNew, i )->Level = pObj->Level;
        Vec_PtrForEachEntry( Abc_Obj_t *, vNodes, pObj, i )
        {
            pObj->Level = 0;
            Abc_ObjForEachFanin( pObj, pFanin, k )
                if ( pObj->Level < pFanin->Level )
                    pObj->Level = pFanin->Level;
            pObj->Level++;
        }
        fOk = ( (int)pObj->Level <= Required );
        Vec_PtrFree( vNodes );
        Abc_NtkDelete( pNew );
    }
#ifdef ABC_USE_PTHREADS
    pthread_mutex_unlock( &lpk_par_lutmin_lock );
#endif
    return fOk;
}

////////////////////////////////////////////////////////////////////////
///                   per-pivot evaluation                           ///
////////////////////////////////////////////////////////////////////////

static Lpk_ParRec_t * Lpk_ParRecAlloc( void );

/* mirror of Lpk_ResynthesizeNodeNew with all network mutations deferred;
   fills the evaluation record for the pivot p->pObj. */
static void Lpk_ParEvalPivot( Lpk_ParCtx_t * pC, Lpk_Man_t * p, Lpk_ParRec_t ** ppRecOut )
{
    Lpk_Cut_t * pCut;
    unsigned * pTruth;
    abctime clk;
    int i, k, Required, nCutNodes, nLeavesOld;
    Vec_Ptr_t * vLeaves = p->vLeaves;
    Lpk_ParRec_t * pRec = NULL;
    Required = Abc_ObjRequiredLevel(p->pObj);
    clk = Abc_Clock();
    if ( !Lpk_ParNodeCuts( pC, p ) )
    {
        p->timeCuts += Abc_Clock() - clk;
        return;
    }
    p->timeCuts += Abc_Clock() - clk;
    pRec = Lpk_ParRecAlloc();
    pRec->Required = Required;
    Vec_IntAppend( pRec->vWin, pC->vWinCur );
    Vec_IntAppend( pRec->vLeaves, pC->vLeafCur );
    p->nCutsTotal += p->nCuts;
    p->nCutsUseful += p->nEvals;
    if ( getenv("LUTPACK_DEBUG3") )
    {
        extern int Lpk_NodeCuts( Lpk_Man_t * p );
        static int fProbe = 0, nProbed = 0, nEnumSeen = 0;
        nEnumSeen++;
        if ( !fProbe && (nEnumSeen % 500 == 1) && nProbed < 60 )
        {
            Lpk_Man_t * pS = Lpk_ManStart( p->pPars );
            pS->pNtk = p->pNtk;
            pS->pObj = p->pObj;
            pS->pPars->fSatur = 0;
            pS->vTemp = Vec_PtrAlloc( 32 );
            Lpk_NodeCuts( pS );
            nProbed++;
            if ( pS->nEvals != p->nEvals || pS->nCuts != p->nCuts )
            {
                printf( "[lpkpar] ENUM-MISMATCH pivot=%d serial: mffc=%d cuts=%d evals=%d | par: mffc=%d cuts=%d evals=%d\n",
                    p->pObj->Id, pS->nMffc, pS->nCuts, pS->nEvals, p->nMffc, p->nCuts, p->nEvals );
                // compare membership: which nodes each side treats as MFFC members
                {
                    // serial: the traversal marks currently set by Lpk_NodeCuts on pS
                    Abc_Obj_t * pX;
                    int j2, nSer = 0, nPar = 0;
                    Vec_Ptr_t * vSer = Vec_PtrAlloc( 16 );
                    Vec_Ptr_t * vPar = Vec_PtrAlloc( 16 );
                    Abc_NtkForEachNode( p->pNtk, pX, j2 )
                    {
                        if ( Abc_NodeIsTravIdCurrent(pX) )
                        {
                            nSer++;
                            Vec_PtrPush( vSer, (void *)(ABC_PTRUINT_T)pX->Id );
                        }
                        if ( Lpk_ParIsMember( pC, pX->Id ) )
                        {
                            nPar++;
                            Vec_PtrPush( vPar, (void *)(ABC_PTRUINT_T)pX->Id );
                        }
                    }
                    printf( "[lpkpar] members serial=%d par=%d\n", nSer, nPar );
                    printf( "  serial: " );
                    Vec_PtrForEachEntry( void *, vSer, pX, j2 )
                        printf( "%d ", (int)(ABC_PTRUINT_T)pX );
                    printf( "\n  par:    " );
                    Vec_PtrForEachEntry( void *, vPar, pX, j2 )
                        printf( "%d ", (int)(ABC_PTRUINT_T)pX );
                    printf( "\n" );
                    Vec_PtrFree( vSer );
                    Vec_PtrFree( vPar );
                }
                fProbe = 1;
            }
            Lpk_ManStop( pS );
        }
    }
    // try the good cuts in weight order
    pC->nDbgEvals += p->nEvals;
    for ( i = 0; i < p->nEvals; i++ )
    {
        pCut = p->pCuts + p->pEvals[i];
        if ( p->pPars->fFirst && i == 1 )
            break;
        // serial live gate: the cut window must be the pivot MFFC given the
        // cut leaves as boundary
        nCutNodes = Lpk_ParDrain( pC, p->pObj->Id, pCut->pLeaves, pCut->nLeaves );
        if ( getenv("LUTPACK_DEBUG3") && i < 25 )
        {
            // serial gate reference: bump leaves, serial MFFC label, restore
            extern int Abc_NodeMffcLabel( Abc_Obj_t * pNode, Vec_Ptr_t * vNodes );
            int nCutNodesS, k2;
            for ( k2 = 0; k2 < (int)pCut->nLeaves; k2++ )
                Abc_NtkObj(p->pNtk, pCut->pLeaves[k2])->vFanouts.nSize++;
            nCutNodesS = Abc_NodeMffcLabel( p->pObj, NULL );
            for ( k2 = 0; k2 < (int)pCut->nLeaves; k2++ )
                Abc_NtkObj(p->pNtk, pCut->pLeaves[k2])->vFanouts.nSize--;
            printf( "[lpkpar] GATE pivot=%d cut=%d: serial=%d par=%d expect=%d (nodes=%d dup=%d leaves=%d)\n",
                p->pObj->Id, i, nCutNodesS, nCutNodes,
                (int)pCut->nNodes - (int)pCut->nNodesDup,
                pCut->nNodes, pCut->nNodesDup, pCut->nLeaves );
        }
        if ( nCutNodes != (int)pCut->nNodes - (int)pCut->nNodesDup )
            continue;
        pC->nDbgGate++;
        Vec_PtrClear( vLeaves );
        for ( k = 0; k < (int)pCut->nLeaves; k++ )
            Vec_PtrPush( vLeaves, Abc_NtkObj( p->pNtk, pCut->pLeaves[k]) );
        nLeavesOld = Vec_PtrSize(vLeaves);
        clk = Abc_Clock();
        pTruth = Lpk_ParCutTruth( p, pC, pCut, 0 );
        if ( getenv("LUTPACK_DEBUG4") )
        {
            extern unsigned * Lpk_CutTruth( Lpk_Man_t * p, Lpk_Cut_t * pCut, int fInv );
            static int fProbe = 0;
            if ( !fProbe )
            {
                unsigned * pTruthS = Lpk_CutTruth( p, pCut, 0 );
                int w, nDiff = 0, nW = Kit_TruthWordNum(pCut->nLeaves);
                for ( w = 0; w < nW; w++ )
                    if ( pTruthS[w] != pTruth[w] )
                        nDiff++;
                printf( "[lpkpar] TRUTH pivot=%d leaves=%d words=%d diffwords=%d\n",
                    p->pObj->Id, pCut->nLeaves, nW, nDiff );
                if ( nDiff )
                {
                    printf( "  serial:" );
                    for ( w = 0; w < nW && w < 8; w++ )
                        printf( " %08x", pTruthS[w] );
                    printf( "\n  par:   " );
                    for ( w = 0; w < nW && w < 8; w++ )
                        printf( " %08x", pTruth[w] );
                    printf( "\n" );
                }
                fProbe = 1;
            }
        }
        p->timeTruth += Abc_Clock() - clk;
        clk = Abc_Clock();
        Lpk_ParComputeSupports( p, pC, pCut, pTruth );
        p->timeSupps += Abc_Clock() - clk;
        clk = Abc_Clock();
        pC->nDbgDec++;
        if ( Lpk_ParDecomposePlan( p, vLeaves, pTruth, p->puSupps, p->pPars->nLutSize,
                (int)pCut->nNodes - (int)pCut->nNodesDup - 1 + (int)(p->pPars->fZeroCost > 0), Required, nLeavesOld, pRec ) )
        {
            pRec->nLeaves = pCut->nLeaves;
            pRec->nNodes = pCut->nNodes;
            pRec->nNodesDup = pCut->nNodesDup;
            pRec->pLeafIds = ABC_ALLOC( int, pCut->nLeaves );
            memcpy( pRec->pLeafIds, pCut->pLeaves, sizeof(int) * pCut->nLeaves );
            p->timeEval += Abc_Clock() - clk;
            *ppRecOut = pRec;
            return;
        }
        // K==4 lutmin fallback (mirror of the serial gate)
        if ( getenv("LUTPACK_NOLUTMIN") ) { }
        else if ( p->pPars->nLutSize == 4 && pCut->nLeaves <= 15 &&
             (int)pCut->nNodes > Lpk_ParNodeCounts[pCut->nLeaves] + !p->pPars->fZeroCost )
        {
            if ( Lpk_ParLutMinDecide( p->pNtk, vLeaves, (word *)pTruth, p->pPars->nLutSize, Required ) )
            {
                pRec->fPlan = 1;
                pRec->fLutMin = 1;
                pRec->nLeaves = pCut->nLeaves;
                pRec->nNodes = pCut->nNodes;
                pRec->nNodesDup = pCut->nNodesDup;
                pRec->pTruth = ABC_ALLOC( unsigned, Kit_TruthWordNum(pCut->nLeaves) );
                memcpy( pRec->pTruth, pTruth, sizeof(unsigned) * Kit_TruthWordNum(pCut->nLeaves) );
                pRec->pLeafIds = ABC_ALLOC( int, pCut->nLeaves );
                memcpy( pRec->pLeafIds, pCut->pLeaves, sizeof(int) * pCut->nLeaves );
                p->timeEval += Abc_Clock() - clk;
                *ppRecOut = pRec;
                return;
            }
        }
        p->timeEval += Abc_Clock() - clk;
    }
    *ppRecOut = pRec;
}

////////////////////////////////////////////////////////////////////////
///                    record lifecycle                              ///
////////////////////////////////////////////////////////////////////////

static Lpk_ParRec_t * Lpk_ParRecAlloc( void )
{
    Lpk_ParRec_t * pRec = ABC_ALLOC( Lpk_ParRec_t, 1 );
    memset( pRec, 0, sizeof(Lpk_ParRec_t) );
    pRec->vWin = Vec_IntAlloc( 64 );
    pRec->vLeaves = Vec_IntAlloc( 32 );
    return pRec;
}

static void Lpk_ParRecFreePlan( Lpk_ParRec_t * pRec )
{
    int i;
    if ( !pRec->fPlan )
        return;
    if ( pRec->fLutMin )
    {
        ABC_FREE( pRec->pTruth );
        ABC_FREE( pRec->pLeafIds );
    }
    else
    {
        for ( i = 0; i < pRec->nFuns; i++ )
            Lpk_FunFree( pRec->ppFuns[i] );
        ABC_FREE( pRec->ppFuns );
        ABC_FREE( pRec->pLeafIds );
    }
    pRec->fPlan = 0;
}

static void Lpk_ParRecFree( Lpk_ParRec_t * pRec )
{
    if ( pRec == NULL )
        return;
    Lpk_ParRecFreePlan( pRec );
    Vec_IntFree( pRec->vWin );
    Vec_IntFree( pRec->vLeaves );
    ABC_FREE( pRec );
}

////////////////////////////////////////////////////////////////////////
///               worker context and thread pool                     ///
////////////////////////////////////////////////////////////////////////

static Lpk_ParCtx_t * Lpk_ParCtxStart( int iWorker, Lpk_ParPass_t * pPass )
{
    Lpk_ParCtx_t * pC = ABC_ALLOC( Lpk_ParCtx_t, 1 );
    memset( pC, 0, sizeof(Lpk_ParCtx_t) );
    pC->iWorker = iWorker;
    pC->pNtk = pPass->pNtk;
    pC->pShared = pPass->p;
    pC->pMan = Lpk_ManStart( pPass->p->pPars );
    pC->pMan->pNtk = pPass->pNtk;
    pC->nMarkSize = Abc_NtkObjNumMax( pPass->pNtk ) + 1;
    pC->pCntGen = ABC_ALLOC( int, pC->nMarkSize );
    pC->pCnt = ABC_ALLOC( int, pC->nMarkSize );
    pC->pUG = ABC_ALLOC( int, pC->nMarkSize );
    memset( pC->pCntGen, 0, sizeof(int) * pC->nMarkSize );
    memset( pC->pCnt, 0, sizeof(int) * pC->nMarkSize );
    memset( pC->pUG, 0, sizeof(int) * pC->nMarkSize );
    pC->vStack = Vec_IntAlloc( 1024 );
    pC->vWinCur = Vec_IntAlloc( 64 );
    pC->vLeafCur = Vec_IntAlloc( 32 );
    pC->vNT = Vec_PtrAlloc( 64 );
    pC->nMemo = LPK_PAR_MEMO_SLOTS;
    pC->Memo = ABC_ALLOC( Lpk_ParMemo_t, pC->nMemo );
    memset( pC->Memo, 0, sizeof(pC->Memo[0]) * pC->nMemo );
    pC->MemoEpoch = 1;
    return pC;
}

static void Lpk_ParCtxStop( Lpk_ParCtx_t * pC )
{
    Lpk_ManStop( pC->pMan );
    ABC_FREE( pC->pCntGen );
    ABC_FREE( pC->pCnt );
    ABC_FREE( pC->pUG );
    Vec_IntFree( pC->vStack );
    Vec_IntFree( pC->vWinCur );
    Vec_IntFree( pC->vLeafCur );
    Vec_PtrFree( pC->vNT );
    ABC_FREE( pC->Memo );
    ABC_FREE( pC );
}

/* process one chunk of the pivot list (any worker) */
static void Lpk_ParWorkerChunk( Lpk_ParPass_t * pPass, Lpk_ParCtx_t * pC, int iChunk )
{
    int i, iStart, iStop, v;
    iStart = iChunk * LPK_PAR_CHUNK_PIVOTS;
    iStop = iStart + LPK_PAR_CHUNK_PIVOTS;
    if ( iStop > pPass->nPivots )
        iStop = pPass->nPivots;
    for ( i = iStart; i < iStop; i++ )
    {
        v = pPass->vPivots[i];
        pC->pMan->pObj = Abc_NtkObj( pPass->pNtk, v );
        assert( pC->pMan->pObj && Abc_ObjIsNode(pC->pMan->pObj) );
        Lpk_ParEvalPivot( pC, pC->pMan, &pPass->ppRecs[v] );
    }
}

typedef struct Lpk_ParThr_t_
{
    Lpk_ParPass_t * pPass;
    Lpk_ParCtx_t *  pC;
} Lpk_ParThr_t;

#ifdef ABC_USE_PTHREADS
static void * Lpk_ParWorkerThr( void * pArg )
{
    Lpk_ParThr_t * pThr = (Lpk_ParThr_t *)pArg;
    int iChunk;
    while ( 1 )
    {
        iChunk = __sync_fetch_and_add( &pThr->pPass->NextChunk, 1 );
        if ( iChunk >= pThr->pPass->nChunks )
            break;
        Lpk_ParWorkerChunk( pThr->pPass, pThr->pC, iChunk );
    }
    return NULL;
}
#endif

static void Lpk_ParAggregate( Lpk_ParPass_t * pPass, Lpk_ParCtx_t * pC )
{
    Lpk_Man_t * pW = pC->pMan;
    Lpk_Man_t * p = pPass->p;
    p->nCutsTotal += pW->nCutsTotal;
    p->nCutsUseful += pW->nCutsUseful;
    p->nNodesOver += pW->nNodesOver;
    p->timeCuts += pW->timeCuts;
    p->timeTruth += pW->timeTruth;
    p->timeSupps += pW->timeSupps;
    p->timeEval += pW->timeEval;
    pPass->nDbgEvals += pC->nDbgEvals;
    pPass->nDbgGate += pC->nDbgGate;
    pPass->nDbgDec += pC->nDbgDec;
}

/* run the parallel evaluation phase */
static void Lpk_ParEvalPhase( Lpk_ParPass_t * pPass )
{
    int nThr, i, iChunk;
    Lpk_ParCtx_t * pC0;
#ifdef ABC_USE_PTHREADS
    pthread_t * pThreads;
    Lpk_ParThr_t * pThrs;
#endif
    if ( pPass->nPivots == 0 )
        return;
    pPass->nChunks = (pPass->nPivots + LPK_PAR_CHUNK_PIVOTS - 1) / LPK_PAR_CHUNK_PIVOTS;
#ifdef ABC_USE_PTHREADS
    nThr = pPass->nWorkers - 1;
    if ( getenv("LUTPACK_DEBUG3") )
        nThr = 0;
    if ( nThr > pPass->nChunks - 1 )
        nThr = pPass->nChunks - 1;
    if ( nThr < 1 )
        nThr = 0;
    if ( nThr > 0 )
    {
        pThreads = ABC_ALLOC( pthread_t, nThr );
        pThrs = ABC_ALLOC( Lpk_ParThr_t, nThr );
        pPass->NextChunk = 0;
        for ( i = 0; i < nThr; i++ )
        {
            pThrs[i].pPass = pPass;
            pThrs[i].pC = Lpk_ParCtxStart( i + 1, pPass );
            if ( pthread_create( pThreads + i, NULL, Lpk_ParWorkerThr, pThrs + i ) != 0 )
                break;
        }
        nThr = i;
        // the calling thread is worker 0
        pC0 = Lpk_ParCtxStart( 0, pPass );
        while ( 1 )
        {
            iChunk = __sync_fetch_and_add( &pPass->NextChunk, 1 );
            if ( iChunk >= pPass->nChunks )
                break;
            Lpk_ParWorkerChunk( pPass, pC0, iChunk );
        }
        for ( i = 0; i < nThr; i++ )
            pthread_join( pThreads[i], NULL );
        Lpk_ParAggregate( pPass, pC0 );
        Lpk_ParCtxStop( pC0 );
        for ( i = 0; i < nThr; i++ )
        {
            Lpk_ParAggregate( pPass, pThrs[i].pC );
            Lpk_ParCtxStop( pThrs[i].pC );
        }
        ABC_FREE( pThreads );
        ABC_FREE( pThrs );
        return;
    }
#endif
    // serial fallback (single worker)
    pC0 = Lpk_ParCtxStart( 0, pPass );
    for ( iChunk = 0; iChunk < pPass->nChunks; iChunk++ )
        Lpk_ParWorkerChunk( pPass, pC0, iChunk );
    Lpk_ParAggregate( pPass, pC0 );
    Lpk_ParCtxStop( pC0 );
}

////////////////////////////////////////////////////////////////////////
///                 serial validated commit phase                    ///
////////////////////////////////////////////////////////////////////////

/* live-context drift test for an evaluated pivot */
static int Lpk_ParDrifted( Lpk_ParPass_t * pPass, Lpk_ParRec_t * pRec, int vPivot )
{
    Abc_Ntk_t * pNtk = pPass->pNtk;
    int i, v, nLevel;
    Abc_Obj_t * pObj;
    Vec_IntForEachEntry( pRec->vWin, v, i )
    {
        if ( v >= Abc_NtkObjNumMax(pNtk) )
            return 1;
        pObj = Abc_NtkObj( pNtk, v );
        if ( pObj == NULL )
            return 1;
    }
    for ( i = 0; i < Vec_IntSize(pRec->vLeaves); i += 2 )
    {
        v = Vec_IntEntry( pRec->vLeaves, i );
        nLevel = Vec_IntEntry( pRec->vLeaves, i + 1 );
        if ( v >= Abc_NtkObjNumMax(pNtk) )
            return 1;
        pObj = Abc_NtkObj( pNtk, v );
        if ( pObj == NULL || Abc_ObjLevel(pObj) != nLevel )
            return 1;
    }
    if ( vPivot >= Abc_NtkObjNumMax(pNtk) )
        return 1;
    pObj = Abc_NtkObj( pNtk, vPivot );
    if ( pObj == NULL || Abc_ObjRequiredLevel(pObj) != pRec->Required )
        return 1;
    // Task 80 F2 closure: even when every recorded id is alive, an earlier
    // serial commit can have attached NEW consumers to window-interior
    // nodes (new nodes get fresh ids, so the interior ids still exist).
    // The live MFFC-with-cut gate must therefore be re-run before a plan
    // commit: it is exactly the serial eval-time gate (leaf fanout bump +
    // Abc_NodeMffcLabel).  Interior nodes with an outside consumer survive
    // deletion, so a stale plan would otherwise overstate the gain.
    if ( pRec->fPlan )
    {
        extern int Abc_NodeMffcLabel( Abc_Obj_t * pNode, Vec_Ptr_t * vNodes );
        int nCutNodes, k;
        for ( k = 0; k < pRec->nLeaves; k++ )
            Abc_NtkObj(pNtk, pRec->pLeafIds[k])->vFanouts.nSize++;
        nCutNodes = Abc_NodeMffcLabel( pObj, NULL );
        for ( k = 0; k < pRec->nLeaves; k++ )
            Abc_NtkObj(pNtk, pRec->pLeafIds[k])->vFanouts.nSize--;
        if ( nCutNodes != pRec->nNodes - pRec->nNodesDup )
            return 1;
    }
    return 0;
}

/* materialize a validated plan (consumes the plan storage) */
static void Lpk_ParCommitPlan( Lpk_ParPass_t * pPass, Lpk_ParRec_t * pRec, int vPivot )
{
    Lpk_Man_t * p = pPass->p;
    Abc_Ntk_t * pNtk = pPass->pNtk;
    Abc_Obj_t * pObj, * pObjNew;
    int i, nGain, nNodesBef, nNodesAft;
    pObj = Abc_NtkObj( pNtk, vPivot );
    assert( pObj && Abc_ObjIsNode(pObj) );
    p->pObj = pObj;
    Vec_PtrClear( p->vLeaves );
    for ( i = 0; i < pRec->nLeaves; i++ )
        Vec_PtrPush( p->vLeaves, Abc_NtkObj( pNtk, pRec->pLeafIds[i]) );
    nNodesBef = Abc_NtkNodeNum( pNtk );
    if ( pRec->fLutMin )
    {
        pObjNew = Abc_NtkLutMinDecompose( pNtk, p->vLeaves, (word *)pRec->pTruth, p->pPars->nLutSize, pRec->Required );
        nNodesAft = Abc_NtkNodeNum( pNtk );
        if ( pObjNew == NULL )
            return; // level drift on the live network: leave unchanged
    }
    else
    {
        for ( i = 0; i < pRec->nFuns; i++ )
            Vec_PtrPush( p->vLeaves, pRec->ppFuns[i] );
        pObjNew = Lpk_Implement( p, pNtk, p->vLeaves, pRec->nLeaves );
        nNodesAft = Abc_NtkNodeNum( pNtk );
        assert( pObjNew != NULL );
        assert( nNodesAft - nNodesBef == pRec->nFuns );
        assert( Abc_ObjLevel(pObjNew) <= pRec->Required );
    }
    nGain = pRec->nNodes - pRec->nNodesDup - (nNodesAft - nNodesBef);
    p->nGainTotal += nGain;
    p->nChanges++;
    Abc_NtkUpdate( pObj, pObjNew, p->vLevels );
    // the plan storage is consumed (Fun entries were freed by Lpk_Implement)
    if ( pRec->fLutMin )
    {
        ABC_FREE( pRec->pTruth );
        ABC_FREE( pRec->pLeafIds );
    }
    else
    {
        ABC_FREE( pRec->ppFuns );
        ABC_FREE( pRec->pLeafIds );
    }
    pRec->fPlan = 0;
    pRec->fConsumed = 1;
}

/* serial commit/re-evaluation sweep over the full pivot range */
static void Lpk_ParCommitPhase( Lpk_ParPass_t * pPass )
{
    Lpk_Man_t * p = pPass->p;
    Abc_Ntk_t * pNtk = pPass->pNtk;
    int v;
    Abc_Obj_t * pObj;
    Lpk_ParRec_t * pRec;
    for ( v = 0; v < pPass->nNodes; v++ )
    {
        pObj = Abc_NtkObj( pNtk, v );
        if ( pObj == NULL || !Abc_ObjIsNode(pObj) )
            continue;
        if ( !pPass->pOffered[v] )
        {
            // serial evaluates any pivot whose saturation/first filter
            // passes at its turn; such pivots were not part of the phase-A
            // offered set only if their filter state changed mid-pass
            if ( p->pPars->fFirst )
            {
                if ( !Abc_ObjIsCo(Abc_ObjFanout0(pObj)) )
                    continue;
            }
            if ( p->pPars->fSatur && !Lpk_NodeHasChanged( p, v ) )
                continue;
            p->pObj = pObj;
            Lpk_ResynthesizeNodeNew( p );
            continue;
        }
        pRec = pPass->ppRecs[v];
        if ( pRec == NULL )
        {
            if ( getenv("LUTPACK_NOEVAL") )
            {
                p->pObj = pObj;
                Lpk_ResynthesizeNodeNew( p );
            }
            continue; // MFFC==1 pivot (or no-eval mode: handled above)
        }
        if ( !Lpk_ParDrifted( pPass, pRec, v ) )
        {
            // evaluation context intact: the parallel plan is exactly what
            // the serial evaluation would have chosen at this point
            if ( pRec->fPlan )
                Lpk_ParCommitPlan( pPass, pRec, v );
        }
        else
        {
            // context drifted: discard the plan and re-evaluate with the
            // serial machinery on the live network
            Lpk_ParRecFreePlan( pRec );
            p->pObj = pObj;
            Lpk_ResynthesizeNodeNew( p );
        }
    }
}

////////////////////////////////////////////////////////////////////////
///                    per-pass parallel driver                      ///
////////////////////////////////////////////////////////////////////////

/* Main entry: one outer saturation pass with private-window parallel
   evaluation.  Called from Lpk_Resynthesize when pPars->nWorkers > 1. */
void Lpk_PassPar( Lpk_Man_t * p, int nNodes, int nWorkers )
{
    Abc_Ntk_t * pNtk = p->pNtk;
    Lpk_ParPass_t Pass;
    int v, nPivots, i;
    Abc_Obj_t * pObj;
    memset( &Pass, 0, sizeof(Pass) );
    Pass.p = p;
    Pass.pNtk = pNtk;
    Pass.nNodes = nNodes;
    Pass.nWorkers = nWorkers;
    Pass.pOffered = ABC_ALLOC( char, nNodes );
    Pass.ppRecs = ABC_ALLOC( Lpk_ParRec_t *, nNodes );
    memset( Pass.pOffered, 0, sizeof(char) * nNodes );
    memset( Pass.ppRecs, 0, sizeof(Lpk_ParRec_t *) * nNodes );
    Pass.vPivots = NULL;
    Pass.nPivots = 0;
    Pass.NextChunk = 0;
    // build the phase-A offered pivot list (serial filter at pass start)
    {
        int nCap = 0;
        for ( v = 0; v < nNodes; v++ )
        {
            pObj = Abc_NtkObj( pNtk, v );
            if ( pObj == NULL || !Abc_ObjIsNode(pObj) )
                continue;
            if ( p->pPars->fFirst )
            {
                if ( !Abc_ObjIsCo(Abc_ObjFanout0(pObj)) )
                    continue;
            }
            if ( p->pPars->fSatur && !Lpk_NodeHasChanged( p, v ) )
                continue;
            nCap++;
        }
        if ( nCap == 0 )
        {
            ABC_FREE( Pass.pOffered );
            ABC_FREE( Pass.ppRecs );
            return;
        }
        Pass.vPivots = ABC_ALLOC( int, nCap );
        nPivots = 0;
        for ( v = 0; v < nNodes; v++ )
        {
            pObj = Abc_NtkObj( pNtk, v );
            if ( pObj == NULL || !Abc_ObjIsNode(pObj) )
                continue;
            if ( p->pPars->fFirst )
            {
                if ( !Abc_ObjIsCo(Abc_ObjFanout0(pObj)) )
                    continue;
            }
            if ( p->pPars->fSatur && !Lpk_NodeHasChanged( p, v ) )
                continue;
            Pass.vPivots[nPivots++] = v;
            Pass.pOffered[v] = 1;
        }
        Pass.nPivots = nPivots;
    }
    // parallel evaluation phase
    if ( !getenv("LUTPACK_NOEVAL") )
        Lpk_ParEvalPhase( &Pass );
    // serial validated commit / re-evaluation sweep
    Lpk_ParCommitPhase( &Pass );
    if ( getenv("LUTPACK_DEBUG") )
    {
        int nEnum = 0, nPlan = 0, nComm = 0;
        for ( v = 0; v < nNodes; v++ )
            if ( Pass.ppRecs[v] )
            {
                nEnum++;
                if ( Pass.ppRecs[v]->fPlan )
                    nPlan++;
                if ( Pass.ppRecs[v]->fConsumed )
                    nComm++;
            }
        printf( "[lpkpar] pass pivots=%d enum=%d plan=%d committed=%d evals=%d gate_ok=%d dec_try=%d cuts_total=%d cuts_useful=%d\n",
            Pass.nPivots, nEnum, nPlan, nComm, Pass.nDbgEvals, Pass.nDbgGate, Pass.nDbgDec,
            p->nCutsTotal, p->nCutsUseful );
    }
    // release records
    for ( v = 0; v < nNodes; v++ )
        if ( Pass.ppRecs[v] )
        {
            Lpk_ParRecFree( Pass.ppRecs[v] );
            Pass.ppRecs[v] = NULL;
        }
    ABC_FREE( Pass.ppRecs );
    ABC_FREE( Pass.pOffered );
    ABC_FREE( Pass.vPivots );
    p->nPivots += Pass.nPivots;
    (void)i;
}


ABC_NAMESPACE_IMPL_END
