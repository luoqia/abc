/**CFile****************************************************************

  FileName    [giaScript.c]

  SystemName  [ABC: Logic synthesis and verification system.]

  PackageName [Scalable AIG package.]

  Synopsis    [Various hardcoded scripts.]

  Author      [Alan Mishchenko]
  
  Affiliation [UC Berkeley]

  Date        [Ver. 1.0. Started - June 20, 2005.]

  Revision    [$Id: giaScript.c,v 1.00 2005/06/20 00:00:00 alanmi Exp $]

***********************************************************************/

#include "giaAig.h"
#include "base/main/main.h"
#include "base/cmd/cmd.h"
#include "proof/dch/dch.h"
#include "opt/dau/dau.h"
#ifndef _WIN32
#include <sys/mman.h>
#include <sys/wait.h>
#include <unistd.h>
#endif

ABC_NAMESPACE_IMPL_START


////////////////////////////////////////////////////////////////////////
///                        DECLARATIONS                              ///
////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////
///                     FUNCTION DEFINITIONS                         ///
////////////////////////////////////////////////////////////////////////

/**Function*************************************************************

  Synopsis    [Synthesis script.]

  Description []
               
  SideEffects []

  SeeAlso     []

***********************************************************************/
void Gia_ManAigPrintPiLevels( Gia_Man_t * p )
{
    Gia_Obj_t * pObj;
    int i;
    Gia_ManForEachPi( p, pObj, i )
        printf( "%d ", Gia_ObjLevel(p, pObj) );
    printf( "\n" );
}

/**Function*************************************************************

  Synopsis    [Synthesis script.]

  Description []
               
  SideEffects []

  SeeAlso     []

***********************************************************************/
Gia_Man_t * Gia_ManAigSyn2( Gia_Man_t * pInit, int fOldAlgo, int fCoarsen, int fCutMin, int nRelaxRatio, int fDelayMin, int fVerbose, int fVeryVerbose )
{
    Gia_Man_t * p, * pNew, * pTemp;
    Jf_Par_t Pars, * pPars = &Pars;
    if ( fOldAlgo )
    {
        Jf_ManSetDefaultPars( pPars );
        pPars->fCutMin     = fCutMin;
    }
    else
    {
        Lf_ManSetDefaultPars( pPars );
        pPars->fCutMin     = fCutMin;
        pPars->fCoarsen    = fCoarsen;
        pPars->nRelaxRatio = nRelaxRatio;
        pPars->nAreaTuner  = 1;
        pPars->nCutNum     = 4;
    }
    if ( fVerbose )  Gia_ManPrintStats( pInit, NULL );
    p = Gia_ManDup( pInit );
    Gia_ManTransferTiming( p, pInit );
    if ( Gia_ManAndNum(p) == 0 )
    {
        pNew = Gia_ManDup(p);
        Gia_ManTransferTiming( pNew, p );
        Gia_ManStop( p );
        return pNew;
    }
    // delay optimization
    if ( fDelayMin && p->pManTime == NULL )
    {
        int Area0, Area1, Delay0, Delay1;
        int fCutMin = pPars->fCutMin;
        int fCoarsen = pPars->fCoarsen;
        int nRelaxRatio = pPars->nRelaxRatio;
        pPars->fCutMin = 0;
        pPars->fCoarsen = 0;
        pPars->nRelaxRatio = 0;
        // perform mapping
        if ( fOldAlgo )
            Jf_ManPerformMapping( p, pPars );
        else
            Lf_ManPerformMapping( p, pPars );
        Area0  = (int)pPars->Area;
        Delay0 = (int)pPars->Delay;
        // perform balancing
        pNew = Gia_ManPerformDsdBalance( p, 6, 4, 0, 0 );
        // perform mapping again
        if ( fOldAlgo )
            Jf_ManPerformMapping( pNew, pPars );
        else
            Lf_ManPerformMapping( pNew, pPars );
        Area1  = (int)pPars->Area;
        Delay1 = (int)pPars->Delay;
        // choose the best result
        if ( Delay1 < Delay0 - 1 || (Delay1 == Delay0 + 1 && 100.0 * (Area1 - Area0) / Area1 < 3.0) )
        {
            Gia_ManStop( p );
            p = pNew;
        }
        else
        {
            Gia_ManStop( pNew );
            Vec_IntFreeP( &p->vMapping );
        }
        // reset params
        pPars->fCutMin = fCutMin;
        pPars->fCoarsen = fCoarsen;
        pPars->nRelaxRatio = nRelaxRatio;
    }
    // perform balancing
    pNew = Gia_ManAreaBalance( p, 0, ABC_INFINITY, fVeryVerbose, 0 );
    if ( fVerbose )     Gia_ManPrintStats( pNew, NULL );
    Gia_ManStop( p );
    // perform mapping
    if ( fOldAlgo )
        pNew = Jf_ManPerformMapping( pTemp = pNew, pPars );
    else
        pNew = Lf_ManPerformMapping( pTemp = pNew, pPars );
    if ( fVerbose )     Gia_ManPrintStats( pNew, NULL );
    if ( pTemp != pNew )
        Gia_ManStop( pTemp );
    // perform balancing
    pNew = Gia_ManAreaBalance( pTemp = pNew, 0, ABC_INFINITY, fVeryVerbose, 0 );
    if ( fVerbose )     Gia_ManPrintStats( pNew, NULL );
    Gia_ManStop( pTemp );
    return pNew;
}
Gia_Man_t * Gia_ManAigSyn3( Gia_Man_t * p, int fVerbose, int fVeryVerbose )
{
    Gia_Man_t * pNew, * pTemp;
    Jf_Par_t Pars, * pPars = &Pars;
    Jf_ManSetDefaultPars( pPars );
    pPars->nRelaxRatio = 40;
    if ( fVerbose )     Gia_ManPrintStats( p, NULL );
    if ( Gia_ManAndNum(p) == 0 )
    {
        pNew = Gia_ManDup(p);
        Gia_ManTransferTiming( pNew, p );
        //Gia_ManStop( p );
        return pNew;
    }
    // perform balancing
    pNew = Gia_ManAreaBalance( p, 0, ABC_INFINITY, fVeryVerbose, 0 );
    if ( fVerbose )     Gia_ManPrintStats( pNew, NULL );
    // perform mapping
    pPars->nLutSize = 6;
    pNew = Jf_ManPerformMapping( pTemp = pNew, pPars );
    if ( fVerbose )     Gia_ManPrintStats( pNew, NULL );
//    Gia_ManStop( pTemp );
    // perform balancing
    pNew = Gia_ManAreaBalance( pTemp = pNew, 0, ABC_INFINITY, fVeryVerbose, 0 );
    if ( fVerbose )     Gia_ManPrintStats( pNew, NULL );
    Gia_ManStop( pTemp );
    // perform mapping
    pPars->nLutSize = 4;
    pNew = Jf_ManPerformMapping( pTemp = pNew, pPars );
    if ( fVerbose )     Gia_ManPrintStats( pNew, NULL );
//    Gia_ManStop( pTemp );
    // perform balancing
    pNew = Gia_ManAreaBalance( pTemp = pNew, 0, ABC_INFINITY, fVeryVerbose, 0 );
    if ( fVerbose )     Gia_ManPrintStats( pNew, NULL );
    Gia_ManStop( pTemp );
    return pNew;
}
Gia_Man_t * Gia_ManAigSyn4( Gia_Man_t * p, int fVerbose, int fVeryVerbose )
{
    Gia_Man_t * pNew, * pTemp;
    Jf_Par_t Pars, * pPars = &Pars;
    Jf_ManSetDefaultPars( pPars );
    pPars->nRelaxRatio = 40;
    if ( fVerbose )     Gia_ManPrintStats( p, NULL );
    if ( Gia_ManAndNum(p) == 0 )
    {
        pNew = Gia_ManDup(p);
        Gia_ManTransferTiming( pNew, p );
        //Gia_ManStop( p );
        return pNew;
    }
//Gia_ManAigPrintPiLevels( p );
    // perform balancing
    pNew = Gia_ManAreaBalance( p, 0, ABC_INFINITY, fVeryVerbose, 0 );
    if ( fVerbose )     Gia_ManPrintStats( pNew, NULL );
    // perform mapping
    pPars->nLutSize = 7;
    pNew = Jf_ManPerformMapping( pTemp = pNew, pPars );
    if ( fVerbose )     Gia_ManPrintStats( pNew, NULL );
//    Gia_ManStop( pTemp );
    // perform extraction
    pNew = Gia_ManPerformFx( pTemp = pNew, ABC_INFINITY, 0, 0, fVeryVerbose, 0 );
    if ( fVerbose )     Gia_ManPrintStats( pNew, NULL );
    Gia_ManStop( pTemp );
    // perform balancing
    pNew = Gia_ManAreaBalance( pTemp = pNew, 0, ABC_INFINITY, fVeryVerbose, 0 );
    if ( fVerbose )     Gia_ManPrintStats( pNew, NULL );
    Gia_ManStop( pTemp );
    // perform mapping
    pPars->nLutSize = 5;
    pNew = Jf_ManPerformMapping( pTemp = pNew, pPars );
    if ( fVerbose )     Gia_ManPrintStats( pNew, NULL );
//    Gia_ManStop( pTemp );
    // perform extraction
    pNew = Gia_ManPerformFx( pTemp = pNew, ABC_INFINITY, 0, 0, fVeryVerbose, 0 );
    if ( fVerbose )     Gia_ManPrintStats( pNew, NULL );
    Gia_ManStop( pTemp );
    // perform balancing
    pNew = Gia_ManAreaBalance( pTemp = pNew, 0, ABC_INFINITY, fVeryVerbose, 0 );
    if ( fVerbose )     Gia_ManPrintStats( pNew, NULL );
    Gia_ManStop( pTemp );
//Gia_ManAigPrintPiLevels( pNew );
    return pNew;
}

/**Function*************************************************************

  Synopsis    [Duplicates the AIG manager.]

  Description [This duplicator works for AIGs with choices.]
               
  SideEffects []

  SeeAlso     []

***********************************************************************/
Vec_Ptr_t * Gia_ManOrderPios( Aig_Man_t * p, Gia_Man_t * pOrder )
{
    Vec_Ptr_t * vPios;
    Gia_Obj_t * pObj;
    int i;
    assert( Aig_ManCiNum(p) == Gia_ManCiNum(pOrder) );
    assert( Aig_ManCoNum(p) == Gia_ManCoNum(pOrder) );
    vPios = Vec_PtrAlloc( Aig_ManCiNum(p) + Aig_ManCoNum(p) );
    Gia_ManForEachObj( pOrder, pObj, i )
    {
        if ( Gia_ObjIsCi(pObj) )
            Vec_PtrPush( vPios, Aig_ManCi(p, Gia_ObjCioId(pObj)) );
        else if ( Gia_ObjIsCo(pObj) )
            Vec_PtrPush( vPios, Aig_ManCo(p, Gia_ObjCioId(pObj)) );
    }
    return vPios;
}

/**Function*************************************************************

  Synopsis    []

  Description []
               
  SideEffects []

  SeeAlso     []

***********************************************************************/
Gia_Man_t * Gia_ManDupFromBarBufs( Gia_Man_t * p )
{
    Vec_Int_t * vBufObjs;
    Gia_Man_t * pNew;
    Gia_Obj_t * pObj;
    int i, k = 0;
    assert( Gia_ManBufNum(p) > 0 );
    assert( Gia_ManRegNum(p) == 0 );
    assert( !Gia_ManHasChoices(p) );
    pNew = Gia_ManStart( Gia_ManObjNum(p) );
    pNew->pName = Abc_UtilStrsav( p->pName );
    pNew->pSpec = Abc_UtilStrsav( p->pSpec );
    Gia_ManFillValue(p);
    Gia_ManConst0(p)->Value = 0;
    Gia_ManForEachCi( p, pObj, i )
        pObj->Value = Gia_ManAppendCi( pNew );
    vBufObjs = Vec_IntAlloc( Gia_ManBufNum(p) );
    for ( i = 0; i < Gia_ManBufNum(p); i++ )
        Vec_IntPush( vBufObjs, Gia_ManAppendCi(pNew) );
    Gia_ManForEachAnd( p, pObj, i )
    {
        if ( Gia_ObjIsBuf(pObj) )
        {
            pObj->Value = Vec_IntEntry( vBufObjs, k );
            Vec_IntWriteEntry( vBufObjs, k++, Gia_ObjFanin0Copy(pObj) );
        }
        else 
            pObj->Value = Gia_ManAppendAnd( pNew, Gia_ObjFanin0Copy(pObj), Gia_ObjFanin1Copy(pObj) );
    }
    assert( k == Gia_ManBufNum(p) );
    for ( i = 0; i < Gia_ManBufNum(p); i++ )
        Gia_ManAppendCo( pNew, Vec_IntEntry(vBufObjs, i) );
    Vec_IntFree( vBufObjs );
    Gia_ManForEachCo( p, pObj, i )
        Gia_ManAppendCo( pNew, Gia_ObjFanin0Copy(pObj) );
    Gia_ManSetRegNum( pNew, Gia_ManRegNum(p) );
    return pNew;
}
Gia_Man_t * Gia_ManDupToBarBufs( Gia_Man_t * p, int nBarBufs )
{
    Gia_Man_t * pNew;
    Gia_Obj_t * pObj;
    int nPiReal = Gia_ManCiNum(p) - nBarBufs;
    int nPoReal = Gia_ManCoNum(p) - nBarBufs;
    int i, k = 0;
    assert( Gia_ManBufNum(p) == 0 );
    assert( Gia_ManRegNum(p) == 0 );
    assert( Gia_ManHasChoices(p) );
    pNew = Gia_ManStart( Gia_ManObjNum(p) );
    pNew->pName = Abc_UtilStrsav( p->pName );
    pNew->pSpec = Abc_UtilStrsav( p->pSpec );
    if ( Gia_ManHasChoices(p) )
        pNew->pSibls = ABC_CALLOC( int, Gia_ManObjNum(p) );
    Gia_ManFillValue(p);
    Gia_ManConst0(p)->Value = 0;
    for ( i = 0; i < nPiReal; i++ )
        Gia_ManCi(p, i)->Value = Gia_ManAppendCi( pNew );
    Gia_ManForEachAnd( p, pObj, i )
    {
        for ( ; k < nBarBufs; k++ )
            if ( ~Gia_ObjFanin0(Gia_ManCo(p, k))->Value )
                Gia_ManCi(p, nPiReal + k)->Value = Gia_ManAppendBuf( pNew, Gia_ObjFanin0Copy(Gia_ManCo(p, k)) );
            else
                break;
        pObj->Value = Gia_ManAppendAnd( pNew, Gia_ObjFanin0Copy(pObj), Gia_ObjFanin1Copy(pObj) );
        if ( Gia_ObjSibl(p, Gia_ObjId(p, pObj)) )
            pNew->pSibls[Abc_Lit2Var(pObj->Value)] = Abc_Lit2Var(Gia_ObjSiblObj(p, Gia_ObjId(p, pObj))->Value);  
    }
    for ( ; k < nBarBufs; k++ )
        if ( ~Gia_ObjFanin0Copy(Gia_ManCo(p, k)) )
            Gia_ManCi(p, nPiReal + k)->Value = Gia_ManAppendBuf( pNew, Gia_ObjFanin0Copy(Gia_ManCo(p, k)) );
    assert( k == nBarBufs );
    for ( i = 0; i < nPoReal; i++ )
        Gia_ManAppendCo( pNew, Gia_ObjFanin0Copy(Gia_ManCo(p, nBarBufs+i)) );
    Gia_ManSetRegNum( pNew, Gia_ManRegNum(p) );
    assert( Gia_ManBufNum(pNew) == nBarBufs );
    assert( Gia_ManCiNum(pNew) == nPiReal );
    assert( Gia_ManCoNum(pNew) == nPoReal );
    return pNew;
}

/**Function*************************************************************

  Synopsis    []

  Description []
               
  SideEffects []

  SeeAlso     []

***********************************************************************/
/**Function*************************************************************

  Synopsis    [Parallel choice computation for &synch2 -J.]

  Description [The choice miter holds nSnap snapshots of every output,
  output-major. Its outputs are split into contiguous groups of similar
  size; each group's cone, across all snapshots, is copied into its own
  AIG and swept by Dch in a forked child, which returns the proven
  equivalences as pairs of global node ids through shared memory. The
  parent unions them (the representative is the earliest node) and derives
  the choice AIG once, as the serial Dch_ComputeChoices does. An
  equivalence proven inside a cone holds globally, so correctness does not
  depend on the grouping; pairs that only meet across groups are missed.]

  SideEffects []

  SeeAlso     []

***********************************************************************/
#ifndef _WIN32
static void Gia_ManSynch2ParDup_rec( Aig_Man_t * pNew, Aig_Obj_t * pObj, Vec_Int_t * vMap )
{
    Aig_Obj_t * pObjNew;
    if ( pObj->pData )
        return;
    assert( Aig_ObjIsNode(pObj) );
    Gia_ManSynch2ParDup_rec( pNew, Aig_ObjFanin0(pObj), vMap );
    Gia_ManSynch2ParDup_rec( pNew, Aig_ObjFanin1(pObj), vMap );
    // no structural hashing: every copied node keeps its own id
    pObjNew = Aig_ObjCreate( pNew, Aig_ObjCreateGhost( pNew, Aig_ObjChild0Copy(pObj), Aig_ObjChild1Copy(pObj), AIG_OBJ_AND ) );
    pObj->pData = pObjNew;
    Vec_IntSetEntry( vMap, Aig_ObjId(pObjNew), Aig_ObjId(pObj) );
}
static int Gia_ManSynch2ParFind( int * pPar, int i )
{
    while ( pPar[i] != i )
        i = pPar[i] = pPar[pPar[i]];
    return i;
}
static Aig_Man_t * Gia_ManSynch2ChoicesPar( Gia_Man_t * pMiter, int nSnap, Dch_Pars_t * pPars )
{
    extern void Gia_ManToAig_rec( Aig_Man_t * pNew, Aig_Obj_t ** ppNodes, Gia_Man_t * p, Gia_Obj_t * pObj );
    extern Aig_Man_t * Dch_DeriveChoiceAig( Aig_Man_t * pAig, int fSkipRedSupps );
    Aig_Man_t * pMan, * pRes;
    Aig_Obj_t ** ppNodes, ** ppRoots;
    Gia_Obj_t * pObj;
    Vec_Int_t * vOwned;
    int * pPar, ** ppBufs, * pStart, * pPids;
    int i, k, g, nOuts, nGroups, nObjs, nRunning = 0, nNext = 0, nFailed = 0, nPairs = 0;
    long nCap;
    abctime clk = Abc_Clock();
    assert( Gia_ManCoNum(pMiter) % nSnap == 0 );
    nOuts = Gia_ManCoNum(pMiter) / nSnap;
    // Groups overlap in shared logic, which every group re-proves: on CVA6
    // (406k AND nodes) sixteen groups cost twice the work of one, but the
    // sweep is the longest serial step of the PIF tail (59 s with two groups,
    // 26 s with fourteen), so one group per ~25k original nodes trades that
    // work for wall time; -J caps the count. NULL: run serially.
    nGroups = Abc_MinInt( Abc_MinInt( pPars->nProcs, nOuts ), Abc_MaxInt( 1, (Gia_ManAndNum(pMiter) / nSnap + 12500) / 25000 ) );
    if ( nGroups < 2 )
        return NULL;
    // global AIG, built exactly as Gia_ManToAigSkip() does, keeping the map
    pMan = Aig_ManStart( Gia_ManAndNum(pMiter) );
    pMan->pName = Abc_UtilStrsav( pMiter->pName );
    pMan->pSpec = Abc_UtilStrsav( pMiter->pSpec );
    ppNodes = ABC_CALLOC( Aig_Obj_t *, Gia_ManObjNum(pMiter) );
    ppNodes[0] = Aig_ManConst0(pMan);
    Gia_ManForEachCi( pMiter, pObj, i )
        ppNodes[Gia_ObjId(pMiter, pObj)] = Aig_ObjCreateCi( pMan );
    ppRoots = ABC_CALLOC( Aig_Obj_t *, Gia_ManCoNum(pMiter) );
    vOwned = Vec_IntAlloc( nOuts + 1 );   // object count after each output
    Gia_ManForEachCo( pMiter, pObj, i )
    {
        Gia_ManToAig_rec( pMan, ppNodes, pMiter, Gia_ObjFanin0(pObj) );
        ppRoots[i] = Aig_NotCond( ppNodes[Gia_ObjFaninId0p(pMiter, pObj)], Gia_ObjFaninC0(pObj) );
        if ( i % nSnap == nSnap - 1 )
            Vec_IntPush( vOwned, Aig_ManObjNumMax(pMan) );
        if ( i % nSnap != 0 )
            continue;
        ppNodes[Gia_ObjId(pMiter, pObj)] = Aig_ObjCreateCo( pMan, ppRoots[i] );
    }
    Aig_ManSetRegNum( pMan, Gia_ManRegNum(pMiter) );
    ABC_FREE( ppNodes );
    nObjs = Aig_ManObjNumMax( pMan );
    // contiguous output groups of similar newly-created size

    pStart = ABC_ALLOC( int, nGroups + 1 );
    pStart[0] = 0;
    for ( g = 1, k = 0; g < nGroups; g++ )
    {
        int Target = (int)((long)nObjs * g / nGroups);
        while ( k < nOuts - 1 && Vec_IntEntry(vOwned, k) < Target )
            k++;
        pStart[g] = Abc_MaxInt( pStart[g-1], k );
    }
    pStart[nGroups] = nOuts;
    Vec_IntFree( vOwned );
    // shared result buffers: [0] = status (1 = done), [1] = pair count, pairs
    nCap = 2L * nObjs + 2;
    ppBufs = ABC_CALLOC( int *, nGroups );
    pPids = ABC_CALLOC( int, nGroups );
    for ( g = 0; g < nGroups; g++ )
    {
        ppBufs[g] = (int *)mmap( NULL, sizeof(int) * nCap, PROT_READ | PROT_WRITE, MAP_SHARED | MAP_ANONYMOUS, -1, 0 );
        if ( ppBufs[g] == MAP_FAILED )
            ppBufs[g] = NULL;
    }
    fflush( stdout );
    while ( nNext < nGroups || nRunning > 0 )
    {
        while ( nNext < nGroups && nRunning < pPars->nProcs )
        {
            g = nNext++;
            if ( ppBufs[g] == NULL || (pPids[g] = fork()) < 0 )
            {
                pPids[g] = 0;
                nFailed++;
                continue;
            }
            if ( pPids[g] == 0 )
            {
                // child: sweep the cone of this group's outputs
                Aig_Man_t * pSub = Aig_ManStart( 1000 );
                Vec_Int_t * vMap = Vec_IntStart( 1000 );
                Aig_Obj_t * pAigObj, * pRepr;
                Dch_Pars_t Pars = *pPars;
                int * pBuf = ppBufs[g], nOut = 0;
                Pars.fVerbose = 0;
                Aig_ManCleanData( pMan );
                Aig_ManConst1(pMan)->pData = Aig_ManConst1(pSub);
                Aig_ManForEachCi( pMan, pAigObj, i )
                {
                    pAigObj->pData = Aig_ObjCreateCi( pSub );
                    Vec_IntSetEntry( vMap, Aig_ObjId((Aig_Obj_t *)pAigObj->pData), Aig_ObjId(pAigObj) );
                }
                for ( k = pStart[g] * nSnap; k < pStart[g+1] * nSnap; k++ )
                {
                    Gia_ManSynch2ParDup_rec( pSub, Aig_Regular(ppRoots[k]), vMap );
                    Aig_ObjCreateCo( pSub, Aig_NotCond( (Aig_Obj_t *)Aig_Regular(ppRoots[k])->pData, Aig_IsComplement(ppRoots[k]) ) );
                }
                Dch_ComputeEquivalences( pSub, &Pars );
                Aig_ManForEachObj( pSub, pAigObj, i )
                {
                    if ( (pRepr = Aig_ObjRepr( pSub, pAigObj )) == NULL )
                        continue;
                    pBuf[2 + 2 * nOut]     = Vec_IntEntry( vMap, Aig_ObjId(pAigObj) );
                    pBuf[2 + 2 * nOut + 1] = Aig_ObjIsConst1(pRepr) ? 0 : Vec_IntEntry( vMap, Aig_ObjId(pRepr) );
                    nOut++;
                }
                pBuf[1] = nOut;
                pBuf[0] = 1;
                _exit( 0 );
            }
            nRunning++;
        }
        if ( nRunning > 0 )
        {
            int Status, Pid = wait( &Status );
            if ( Pid < 0 )
                break;
            for ( g = 0; g < nGroups; g++ )
                if ( pPids[g] == Pid )
                {
                    pPids[g] = 0;
                    if ( !WIFEXITED(Status) || WEXITSTATUS(Status) != 0 || ppBufs[g][0] != 1 )
                        nFailed++;
                    break;
                }
            nRunning--;
        }
    }
    ABC_FREE( ppRoots );
    if ( nFailed )
    {
        // a lost group would only lose choices, but keep the result identical
        // to what the serial engine guarantees: fall back to it
        printf( "&synch2 -J: %d of %d groups failed; computing choices serially.\n", nFailed, nGroups );
        for ( g = 0; g < nGroups; g++ )
            if ( ppBufs[g] )
                munmap( ppBufs[g], sizeof(int) * nCap );
        ABC_FREE( ppBufs ); ABC_FREE( pPids ); ABC_FREE( pStart );
        pRes = Dch_ComputeChoices( pMan, pPars );
        Aig_ManStop( pMan );
        return pRes;
    }
    // union the proven equivalences; the representative is the earliest node
    pPar = ABC_ALLOC( int, nObjs );
    for ( i = 0; i < nObjs; i++ )
        pPar[i] = i;
    for ( g = 0; g < nGroups; g++ )
    {
        int * pBuf = ppBufs[g];
        for ( k = 0; k < pBuf[1]; k++ )
        {
            int a = Gia_ManSynch2ParFind( pPar, pBuf[2 + 2*k] );
            int b = Gia_ManSynch2ParFind( pPar, pBuf[2 + 2*k + 1] );
            if ( a != b )
            {
                if ( a < b ) pPar[b] = a; else pPar[a] = b;
            }
        }
        nPairs += pBuf[1];
        munmap( ppBufs[g], sizeof(int) * nCap );
    }
    Aig_ManReprStart( pMan, nObjs );
    for ( i = 0; i < nObjs; i++ )
    {
        int r = Gia_ManSynch2ParFind( pPar, i );
        Aig_Obj_t * pAigObj = Aig_ManObj( pMan, i );
        if ( r == i || pAigObj == NULL || !Aig_ObjIsNode(pAigObj) )
            continue;
        Aig_ObjSetRepr( pMan, pAigObj, Aig_ManObj(pMan, r) );
    }
    ABC_FREE( pPar ); ABC_FREE( ppBufs ); ABC_FREE( pPids ); ABC_FREE( pStart );
    if ( pPars->fVerbose )
    {
        printf( "&synch2 -J %d: %d groups, %d proven pairs.  ", pPars->nProcs, nGroups, nPairs );
        Abc_PrintTime( 1, "Parallel sweep time", Abc_Clock() - clk );
    }
    ABC_FREE( pMan->pTable );
    pRes = Dch_DeriveChoiceAig( pMan, pPars->fSkipRedSupp );
    Aig_ManStop( pMan );
    return pRes;
}
#endif

Gia_Man_t * Gia_ManAigSynch2Choices( Gia_Man_t * pGia1, Gia_Man_t * pGia2, Gia_Man_t * pGia3, Dch_Pars_t * pPars )
{
    Aig_Man_t * pMan, * pTemp;
    Gia_Man_t * pGia, * pMiter;
    // derive miter
    Vec_Ptr_t * vPios, * vGias = Vec_PtrAlloc( 3 );
    if ( pGia3 ) Vec_PtrPush( vGias, pGia3 );
    if ( pGia2 ) Vec_PtrPush( vGias, pGia2 );
    if ( pGia1 ) Vec_PtrPush( vGias, pGia1 );
    pMiter = Gia_ManChoiceMiter( vGias );
    Vec_PtrFree( vGias );
    pMan = NULL;
#ifndef _WIN32
    if ( pPars->nProcs > 1 )
    {
        pMan = Gia_ManSynch2ChoicesPar( pMiter, 3, pPars );
    }
    if ( pMan != NULL )
        Gia_ManStop( pMiter );
    else
#endif
    {
    // transform into an AIG
    pMan = Gia_ManToAigSkip( pMiter, 3 );
    Gia_ManStop( pMiter );
    // compute choices
    pMan = Dch_ComputeChoices( pTemp = pMan, pPars );
    Aig_ManStop( pTemp );
    }
    // reconstruct the network
    vPios = Gia_ManOrderPios( pMan, pGia1 ); 
    pMan = Aig_ManDupDfsGuided( pTemp = pMan, vPios );
    Aig_ManStop( pTemp );
    Vec_PtrFree( vPios );
    // convert to GIA
    pGia = Gia_ManFromAigChoices( pMan );
    Aig_ManStop( pMan );
    return pGia;
}
Gia_Man_t * Gia_ManAigSynch2( Gia_Man_t * pInit, void * pPars0, int nLutSize, int nRelaxRatio )
{
    extern Gia_Man_t * Gia_ManLutBalance( Gia_Man_t * p, int nLutSize, int fUseMuxes, int fRecursive, int fOptArea, int fVerbose );
    Dch_Pars_t * pParsDch = (Dch_Pars_t *)pPars0;
    Gia_Man_t * pGia1, * pGia2, * pGia3, * pNew, * pTemp;
    int fVerbose = pParsDch->fVerbose;
    Jf_Par_t Pars, * pPars = &Pars;
    Lf_ManSetDefaultPars( pPars );
    pPars->fCutMin     = 1;
    pPars->fCoarsen    = 1;
    pPars->nRelaxRatio = nRelaxRatio;
    pPars->nAreaTuner  = 5;
    pPars->nCutNum     = 12;
    pPars->fVerbose    = fVerbose;
    if ( fVerbose )  Gia_ManPrintStats( pInit, NULL );
    pGia1 = Gia_ManDup( pInit );
    if ( Gia_ManAndNum(pGia1) == 0 )
    {
        Gia_ManTransferTiming( pGia1, pInit );
        return pGia1;
    }
    if ( pGia1->pManTime && pGia1->vLevels == NULL )
        Gia_ManLevelWithBoxes( pGia1 );
    // unmap if mapped
    if ( Gia_ManHasMapping(pInit) )
    {
        Gia_ManTransferMapping( pGia1, pInit );
        pGia1 = (Gia_Man_t *)Dsm_ManDeriveGia( pTemp = pGia1, 0 );
        Gia_ManStop( pTemp );
    }
    // perform balancing
    if ( Gia_ManBufNum(pGia1) || 1 )
        pGia2 = Gia_ManAreaBalance( pGia1, 0, ABC_INFINITY, 0, 0 );
    else
    {
        pGia2 = Gia_ManLutBalance( pGia1, nLutSize, 1, 1, 1, 0 );
        pGia2 = Gia_ManAreaBalance( pTemp = pGia2, 0, ABC_INFINITY, 0, 0 );
        Gia_ManStop( pTemp );
    }
    if ( fVerbose )     Gia_ManPrintStats( pGia2, NULL );
    // perform mapping
    pGia2 = Lf_ManPerformMapping( pTemp = pGia2, pPars );
    if ( fVerbose )     Gia_ManPrintStats( pGia2, NULL );
    if ( pTemp != pGia2 )
        Gia_ManStop( pTemp );
    // perform balancing
    if ( pParsDch->fLightSynth || Gia_ManBufNum(pGia2) )
        pGia3 = Gia_ManAreaBalance( pGia2, 0, ABC_INFINITY, 0, 0 );
    else
    {
        assert( Gia_ManBufNum(pGia2) == 0 );
        pGia2 = Gia_ManAreaBalance( pTemp = pGia2, 0, ABC_INFINITY, 0, 0 );
        if ( fVerbose )     Gia_ManPrintStats( pGia2, NULL );
        Gia_ManStop( pTemp );
        // perform DSD balancing
        pGia3 = Gia_ManPerformDsdBalance( pGia2, 6, 8, 0, 0 );
    }
    if ( fVerbose )     Gia_ManPrintStats( pGia3, NULL );
    // perform choice computation
    if ( Gia_ManBufNum(pInit) )
    {
        assert( Gia_ManBufNum(pInit) == Gia_ManBufNum(pGia1) );
        pGia1 = Gia_ManDupFromBarBufs( pTemp = pGia1 );
        Gia_ManStop( pTemp );
        assert( Gia_ManBufNum(pInit) == Gia_ManBufNum(pGia2) );
        pGia2 = Gia_ManDupFromBarBufs( pTemp = pGia2 );
        Gia_ManStop( pTemp );
        assert( Gia_ManBufNum(pInit) == Gia_ManBufNum(pGia3) );
        pGia3 = Gia_ManDupFromBarBufs( pTemp = pGia3 );
        Gia_ManStop( pTemp );
    }
    pNew = Gia_ManAigSynch2Choices( pGia1, pGia2, pGia3, pParsDch );
    Gia_ManStop( pGia1 );
    Gia_ManStop( pGia2 );
    Gia_ManStop( pGia3 );
    if ( Gia_ManBufNum(pInit) )
    {
        pNew = Gia_ManDupToBarBufs( pTemp = pNew, Gia_ManBufNum(pInit) );
        Gia_ManStop( pTemp );
    }
    // copy names
    ABC_FREE( pNew->pName );
    ABC_FREE( pNew->pSpec );
    pNew->pName = Abc_UtilStrsav(pInit->pName);
    pNew->pSpec = Abc_UtilStrsav(pInit->pSpec);
    Gia_ManTransferTiming( pNew, pInit );
    return pNew;
}

/**Function*************************************************************

  Synopsis    []

  Description []
               
  SideEffects []

  SeeAlso     []

***********************************************************************/
void Gia_ManPerformMap( int nAnds, int nLutSize, int nCutNum, int fMinAve, int fUseMfs, int fVerbose )
{
    char Command[200];
    sprintf( Command, "&unmap; &lf -K %d -C %d -k %s; &save", nLutSize, nCutNum, fMinAve?"-t":"" );
//    sprintf( Command, "&unmap; &if -K %d -C %d %s; &save", nLutSize, nCutNum, fMinAve?"-t":"" );
    Cmd_CommandExecute( Abc_FrameGetGlobalFrame(), Command );
    if ( fVerbose )
    {
        printf( "MAPPING:\n" );
        printf( "Mapping with &lf -k:\n" );
        Cmd_CommandExecute( Abc_FrameGetGlobalFrame(), "&ps" );
    }
    sprintf( Command, "&unmap; &lf -K %d -C %d %s; &save", nLutSize, nCutNum, fMinAve?"-t":"" );
    Cmd_CommandExecute( Abc_FrameGetGlobalFrame(), Command );
    if ( fVerbose )
    {
        printf( "Mapping with &lf:\n" );
        Cmd_CommandExecute( Abc_FrameGetGlobalFrame(), "&ps" );
    }
    if ( (nLutSize == 4 && nAnds < 100000) || (nLutSize == 6 && nAnds < 2000) )
    {
        sprintf( Command, "&unmap; &if -sz -S %d%d -K %d -C %d %s", nLutSize, nLutSize, 2*nLutSize-1, 2*nCutNum, fMinAve?"-t":"" );
        Cmd_CommandExecute( Abc_FrameGetGlobalFrame(), Command );
        Vec_IntFreeP( &Abc_FrameReadGia(Abc_FrameGetGlobalFrame())->vPacking );
        Cmd_CommandExecute( Abc_FrameGetGlobalFrame(), "&save" );
        if ( fVerbose )
        {
            printf( "Mapping with &if -sz -S %d%d -K %d -C %d %s:\n", nLutSize, nLutSize, 2*nLutSize-1, 2*nCutNum, fMinAve?"-t":"" );
            Cmd_CommandExecute( Abc_FrameGetGlobalFrame(), "&ps" );
        }
    }
    Cmd_CommandExecute( Abc_FrameGetGlobalFrame(), "&load" );
    if ( fUseMfs )
        Cmd_CommandExecute( Abc_FrameGetGlobalFrame(), "&put; mfs2 -W 4 -M 500 -C 7000; &get -m" );
    if ( fVerbose )
    {
        printf( "Mapping final:\n" );
        Cmd_CommandExecute( Abc_FrameGetGlobalFrame(), "&ps" );
    }
}
void Gia_ManPerformRound( int fIsMapped, int nAnds, int nLevels, int nLutSize, int nCutNum, int fMinAve, int fUseMfs, int fVerbose )
{
    char Command[200];

    // perform AIG-based synthesis
    if ( nAnds < 50000 )
    {
        Cmd_CommandExecute( Abc_FrameGetGlobalFrame(), "" );
        sprintf( Command, "&dsdb; &dch -C 500; &if -K %d -C %d %s; &save", nLutSize, nCutNum, fMinAve?"-t":"" );
        Cmd_CommandExecute( Abc_FrameGetGlobalFrame(), Command );
        if ( fVerbose )
        {
            printf( "Mapping with &dch -C 500; &if -K %d -C %d %s:\n", nLutSize, nCutNum, fMinAve?"-t":"" );
            Cmd_CommandExecute( Abc_FrameGetGlobalFrame(), "&ps" );
        }
        Cmd_CommandExecute( Abc_FrameGetGlobalFrame(), "&st" );
    }

    // perform AIG-based synthesis
    if ( nAnds < 20000 )
    {
        Cmd_CommandExecute( Abc_FrameGetGlobalFrame(), "" );
        sprintf( Command, "&dsdb; &dch -C 500; &if -K %d -C %d %s; &save", nLutSize, nCutNum, fMinAve?"-t":"" );
        Cmd_CommandExecute( Abc_FrameGetGlobalFrame(), Command );
        if ( fVerbose )
        {
            printf( "Mapping with &dch -C 500; &if -K %d -C %d %s:\n", nLutSize, nCutNum, fMinAve?"-t":"" );
            Cmd_CommandExecute( Abc_FrameGetGlobalFrame(), "&ps" );
        }
        Cmd_CommandExecute( Abc_FrameGetGlobalFrame(), "&st" );
    }

    // perform first round of mapping
    Gia_ManPerformMap( nAnds, nLutSize, nCutNum, fMinAve, fUseMfs, fVerbose );
    Cmd_CommandExecute( Abc_FrameGetGlobalFrame(), "&st" );

    // perform synthesis
    Cmd_CommandExecute( Abc_FrameGetGlobalFrame(), "&dsdb" );

    // perform second round of mapping
    Gia_ManPerformMap( nAnds, nLutSize, nCutNum, fMinAve, fUseMfs, fVerbose );
    Cmd_CommandExecute( Abc_FrameGetGlobalFrame(), "&st" );

    // perform synthesis
    Cmd_CommandExecute( Abc_FrameGetGlobalFrame(), "&syn2 -m -R 10; &dsdb" );

    // prepare for final mapping
    sprintf( Command, "&blut -a -K %d", nLutSize );
    Cmd_CommandExecute( Abc_FrameGetGlobalFrame(), Command );

    // perform third round of mapping
    Gia_ManPerformMap( nAnds, nLutSize, nCutNum, fMinAve, fUseMfs, fVerbose );
}
void Gia_ManPerformFlow( int fIsMapped, int nAnds, int nLevels, int nLutSize, int nCutNum, int fMinAve, int fUseMfs, int fVerbose )
{
    // remove comb equivs
    if ( fIsMapped )
        Cmd_CommandExecute( Abc_FrameGetGlobalFrame(), "&st" );
//    if ( Abc_FrameReadGia(Abc_FrameGetGlobalFrame())->pManTime )
//        Cmd_CommandExecute( Abc_FrameGetGlobalFrame(), "&sweep" );
//    else
//        Cmd_CommandExecute( Abc_FrameGetGlobalFrame(), "&fraig -c" );

    // perform first round
    Gia_ManPerformRound( fIsMapped, nAnds, nLevels, nLutSize, nCutNum, fMinAve, fUseMfs, fVerbose );

    // perform synthesis
    Cmd_CommandExecute( Abc_FrameGetGlobalFrame(), "&st; &sopb" );

    // perform first round
    Gia_ManPerformRound( fIsMapped, nAnds, nLevels, nLutSize, nCutNum, fMinAve, fUseMfs, fVerbose );
}

/**Function*************************************************************

  Synopsis    []

  Description []
               
  SideEffects []

  SeeAlso     []

***********************************************************************/
void Gia_ManPerformFlow2( int fIsMapped, int nAnds, int nLevels, int nLutSize, int nCutNum, int fBalance, int fMinAve, int fUseMfs, int fVerbose )
{
    char Comm1[1000], Comm2[1000], Comm3[1000], Comm4[1000];
    if ( nLutSize == 0 ) 
    {
        sprintf( Comm1, "&synch2 -K 6 -C 500;  &if -m%s       -C %d; %s &save", fMinAve?"t":"", nCutNum,   fUseMfs ? "&put; mfs2 -W 4 -M 500 -C 7000; &get -m;":"" );
        sprintf( Comm2, "&dch -C 500;          &if -m%s       -C %d; %s &save", fMinAve?"t":"", nCutNum+4, fUseMfs ? "&put; mfs2 -W 4 -M 500 -C 7000; &get -m;":"" );
        sprintf( Comm3, "&synch2 -K 6 -C 500;  &lf -m%s  -E 5 -C %d; %s &save", fMinAve?"t":"", nCutNum,   fUseMfs ? "&put; mfs2 -W 4 -M 500 -C 7000; &get -m;":"" );
        sprintf( Comm4, "&dch -C 500;          &lf -m%sk -E 5 -C %d; %s &save", fMinAve?"t":"", nCutNum+4, fUseMfs ? "&put; mfs2 -W 4 -M 500 -C 7000; &get -m;":"" );
    }
    else
    {
        sprintf( Comm1, "&synch2 -K %d -C 500; &if -m%s       -K %d -C %d; %s &save", nLutSize, fMinAve?"t":"", nLutSize, nCutNum,   fUseMfs ? "&put; mfs2 -W 4 -M 500 -C 7000; &get -m;":"" );
        sprintf( Comm2, "&dch -C 500;          &if -m%s       -K %d -C %d; %s &save",           fMinAve?"t":"", nLutSize, nCutNum+4, fUseMfs ? "&put; mfs2 -W 4 -M 500 -C 7000; &get -m;":"" );
        sprintf( Comm3, "&synch2 -K %d -C 500; &lf -m%s  -E 5 -K %d -C %d; %s &save", nLutSize, fMinAve?"t":"", nLutSize, nCutNum,   fUseMfs ? "&put; mfs2 -W 4 -M 500 -C 7000; &get -m;":"" );
        sprintf( Comm4, "&dch -C 500;          &lf -m%sk -E 5 -K %d -C %d; %s &save",           fMinAve?"t":"", nLutSize, nCutNum+4, fUseMfs ? "&put; mfs2 -W 4 -M 500 -C 7000; &get -m;":"" );
    }

    // perform synthesis
    if ( fVerbose )
        printf( "Trying synthesis...\n" );
    if ( fIsMapped )
    Cmd_CommandExecute( Abc_FrameGetGlobalFrame(), "&st" );
    Cmd_CommandExecute( Abc_FrameGetGlobalFrame(), Comm1 );
    if ( fVerbose )
    Cmd_CommandExecute( Abc_FrameGetGlobalFrame(), "&ps" );

    // perform synthesis
    Cmd_CommandExecute( Abc_FrameGetGlobalFrame(), "&st" );
    Cmd_CommandExecute( Abc_FrameGetGlobalFrame(), Comm2 );
    if ( fVerbose )
    Cmd_CommandExecute( Abc_FrameGetGlobalFrame(), "&ps" );

    // return the result
    Cmd_CommandExecute( Abc_FrameGetGlobalFrame(), "&load" );
    if ( fVerbose )
    Cmd_CommandExecute( Abc_FrameGetGlobalFrame(), "&ps" );


    // perform balancing
    if ( fBalance )
    {
        if ( fVerbose )
            printf( "Trying SOP balancing...\n" );
        Cmd_CommandExecute( Abc_FrameGetGlobalFrame(), "&st; &sopb -R 10 -C 4" );
    }


    // perform synthesis
    Cmd_CommandExecute( Abc_FrameGetGlobalFrame(), Comm3 );
    if ( fVerbose )
    Cmd_CommandExecute( Abc_FrameGetGlobalFrame(), "&ps" );

    // perform synthesis
    Cmd_CommandExecute( Abc_FrameGetGlobalFrame(), "&st" );
    Cmd_CommandExecute( Abc_FrameGetGlobalFrame(), Comm2 );
    if ( fVerbose )
    Cmd_CommandExecute( Abc_FrameGetGlobalFrame(), "&ps" );

    // return the result
    Cmd_CommandExecute( Abc_FrameGetGlobalFrame(), "&load" );
    if ( fVerbose )
    Cmd_CommandExecute( Abc_FrameGetGlobalFrame(), "&ps" );
    if ( nAnds > 100000 )
        return;


    // perform balancing
    if ( fBalance )
    {
        if ( fVerbose )
            printf( "Trying SOP balancing...\n" );
        Cmd_CommandExecute( Abc_FrameGetGlobalFrame(), "&st; &sopb -R 10" );
    }


    // perform synthesis
    Cmd_CommandExecute( Abc_FrameGetGlobalFrame(), Comm3 );
    if ( fVerbose )
    Cmd_CommandExecute( Abc_FrameGetGlobalFrame(), "&ps" );

    // perform synthesis
    Cmd_CommandExecute( Abc_FrameGetGlobalFrame(), "&st" );
    Cmd_CommandExecute( Abc_FrameGetGlobalFrame(), Comm2 );
    if ( fVerbose )
    Cmd_CommandExecute( Abc_FrameGetGlobalFrame(), "&ps" );

    // return the result
    Cmd_CommandExecute( Abc_FrameGetGlobalFrame(), "&load" );
    if ( fVerbose )
    Cmd_CommandExecute( Abc_FrameGetGlobalFrame(), "&ps" );
    if ( nAnds > 50000 )
        return;


    // perform balancing
    if ( fBalance )
    {
        if ( fVerbose )
            printf( "Trying SOP balancing...\n" );
        Cmd_CommandExecute( Abc_FrameGetGlobalFrame(), "&st; &sopb -R 10" );
    }


    // perform synthesis
    Cmd_CommandExecute( Abc_FrameGetGlobalFrame(), Comm3 );
    if ( fVerbose )
    Cmd_CommandExecute( Abc_FrameGetGlobalFrame(), "&ps" );

    // perform synthesis
    Cmd_CommandExecute( Abc_FrameGetGlobalFrame(), "&st" );
    Cmd_CommandExecute( Abc_FrameGetGlobalFrame(), Comm2 );
    if ( fVerbose )
    Cmd_CommandExecute( Abc_FrameGetGlobalFrame(), "&ps" );

    // return the result
    Cmd_CommandExecute( Abc_FrameGetGlobalFrame(), "&load" );
    if ( fVerbose )
    Cmd_CommandExecute( Abc_FrameGetGlobalFrame(), "&ps" );
}


/**Function*************************************************************

  Synopsis    []

  Description []
               
  SideEffects []

  SeeAlso     []

***********************************************************************/
void Gia_ManPerformFlow3( int nLutSize, int nCutNum, int fBalance, int fMinAve, int fUseMfs, int fUseLutLib, int fVerbose )
{
    char Comm1[200], Comm2[200], Comm3[200];
    if ( fUseLutLib )
        sprintf( Comm1, "&st; &if -C %d;       &save; &st; &syn2; &if -C %d;       &save; &load", nCutNum, nCutNum );
    else
        sprintf( Comm1, "&st; &if -C %d -K %d; &save; &st; &syn2; &if -C %d -K %d; &save; &load", nCutNum, nLutSize, nCutNum, nLutSize );
    if ( fUseLutLib )
        sprintf( Comm2, "&st; &if -%s -K 6; &dch -f; &if -C %d;       %s&save; &load", Abc_NtkRecIsRunning3() ? "y" : "g", nCutNum,           fUseMfs ? "&mfs; ":"" );
    else
        sprintf( Comm2, "&st; &if -%s -K 6; &dch -f; &if -C %d -K %d; %s&save; &load", Abc_NtkRecIsRunning3() ? "y" : "g", nCutNum, nLutSize, fUseMfs ? "&mfs; ":"" );
    if ( fUseLutLib )
        sprintf( Comm3, "&st; &if -%s -K 6; &synch2; &if -C %d;       %s&save; &load", Abc_NtkRecIsRunning3() ? "y" : "g", nCutNum,           fUseMfs ? "&mfs; ":"" );
    else
        sprintf( Comm3, "&st; &if -%s -K 6; &synch2; &if -C %d -K %d; %s&save; &load", Abc_NtkRecIsRunning3() ? "y" : "g", nCutNum, nLutSize, fUseMfs ? "&mfs; ":"" );

    if ( fVerbose ) printf( "Trying simple synthesis with %s...\n", Abc_NtkRecIsRunning3() ? "LMS" : "SOP balancing" );
    Cmd_CommandExecute( Abc_FrameGetGlobalFrame(), Comm1 );
    if ( fVerbose )
    Cmd_CommandExecute( Abc_FrameGetGlobalFrame(), "&ps" );

    if ( Gia_ManAndNum( Abc_FrameReadGia(Abc_FrameGetGlobalFrame()) ) < 200000 )
    {
        if ( fVerbose ) printf( "Trying medium synthesis...\n" );
        Cmd_CommandExecute( Abc_FrameGetGlobalFrame(), Comm2 );
        if ( fVerbose )
        Cmd_CommandExecute( Abc_FrameGetGlobalFrame(), "&ps" );
    }

    if ( Gia_ManAndNum( Abc_FrameReadGia(Abc_FrameGetGlobalFrame()) ) < 10000 )
    {
        if ( fVerbose ) printf( "Trying harder synthesis...\n" );
        Cmd_CommandExecute( Abc_FrameGetGlobalFrame(), Comm3 );
        if ( fVerbose )
        Cmd_CommandExecute( Abc_FrameGetGlobalFrame(), "&ps" );
    }

    if ( fVerbose ) printf( "Final result...\n" );
    if ( fVerbose )
    Cmd_CommandExecute( Abc_FrameGetGlobalFrame(), "&ps" );
}



////////////////////////////////////////////////////////////////////////
///                       END OF FILE                                ///
////////////////////////////////////////////////////////////////////////


ABC_NAMESPACE_IMPL_END

