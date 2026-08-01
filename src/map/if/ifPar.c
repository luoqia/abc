/**CFile****************************************************************

  FileName    [ifPar.c]

  SystemName  [ABC: Logic synthesis and verification system.]

  PackageName [FPGA mapping.]

  Synopsis    [Stage 1 parallel path support gate and metadata checks.]

  Author      [qingyu-sudo]

  Affiliation [复旦大学]

  Date        [Ver. 1.0. Started - May 1, 2026.]

  Revision    [$Id: ifPar.c,v 1.00 2026/05/01 00:00:00 alanmi Exp $]

***********************************************************************/

#include "if.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdint.h>

#ifdef ABC_USE_PTHREADS
#if defined(_WIN32) && !defined(__MINGW32__)
#include "../lib/pthread.h"
#else
#include <pthread.h>
#endif
#endif

ABC_NAMESPACE_IMPL_START

#define IF_PAR_STAGE55_NOMINAL_CHUNK 32
#define IF_PAR_STATIC_MIN_TASKS_PER_THREAD 2048
#define IF_PAR_REQUIRED_MIN_TASKS_PER_THREAD 16

typedef char If_ParRequiredFloatSizeCheck[(sizeof(float) == 4) ? 1 : -1];

#if defined(__GNUC__) || defined(__clang__)
static inline uint32_t If_ParAtomicU32Load( uint32_t * pValue )
{
    return __atomic_load_n( pValue, __ATOMIC_RELAXED );
}
static inline void If_ParAtomicU32Store( uint32_t * pValue, uint32_t Value )
{
    __atomic_store_n( pValue, Value, __ATOMIC_RELAXED );
}
static inline int If_ParAtomicU32CompareExchangeWeak( uint32_t * pValue, uint32_t * pExpected, uint32_t Desired )
{
    return __atomic_compare_exchange_n( pValue, pExpected, Desired, 1, __ATOMIC_RELAXED, __ATOMIC_RELAXED );
}
#else
#error "Stage 9 required-time CAS-min requires GCC/Clang __atomic builtins."
#endif

////////////////////////////////////////////////////////////////////////
///                        DECLARATIONS                              ///
////////////////////////////////////////////////////////////////////////

struct If_ParMeta_t_
{
    int        nLevels;
    int        nObjs;
    int        nAnds;
    int        nChoices;
    int        nProviders;
    int        nMaxFreeLevel;
    int *      pLevelCounts;
    int *      pLevelStarts;
    int *      pLevelObjs;
    int *      pFreeLevel;
    int        nCutSetPeak;
    int *      pCutSetSlot;
    int *      pReleaseStarts;
    int *      pReleaseObjs;
    int *      pChoiceReprs;
    int *      pChoiceLevelCounts;
    int *      pChoiceLevelStarts;
    int *      pChoiceLevelObjs;
    int *      pProviderStarts;
    int *      pProviders;
    char *     pIsChoiceProvider;
    abctime    SlotAssignTime;
    abctime    SlotValidateTime;
};

typedef enum If_ParJobKind_t_
{
    IF_PAR_JOB_NONE = 0,
    IF_PAR_JOB_SKELETON,
    IF_PAR_JOB_MODE0_AND,
    IF_PAR_JOB_MODE0_CHOICE,
    IF_PAR_JOB_MODE1_AND,
    IF_PAR_JOB_MODE1_CHOICE,
    IF_PAR_JOB_MODE2_AND,
    IF_PAR_JOB_MODE2_CHOICE,
    IF_PAR_JOB_CUTSET_ALLOC,
    IF_PAR_JOB_CUTSET_RELEASE,
    IF_PAR_JOB_REQUIRED_AND,
    IF_PAR_JOB_REQUIRED_STAGE9_AND,
    IF_PAR_JOB_REQUIRED_FLAT_SCAN,
    IF_PAR_JOB_REQUIRED_CI_TERMINAL,
    IF_PAR_JOB_REQUIRED_WRITEBACK,
    IF_PAR_JOB_IMPROVE_AND
} If_ParJobKind_t;

typedef struct If_ParMode2Metric_t_ If_ParMode2Metric_t;
struct If_ParMode2Metric_t_
{
    float      Area;
    float      Edge;
};

typedef struct If_ParThread_t_ If_ParThread_t;
struct If_ParThread_t_
{
    struct If_ParRuntime_t_ * pRuntime;
    int        iThread;
    int        iJob;
    int        iStart;
    int        iStop;
    int        nObjs;
    int        nChoices;
    int        nCutsMerged;
    int        nCutsTotal;
    int        nCandidateCount;
    int        nImproveRoots;
    int        nImproveCandidates;
    int        nImproveAccepted;
    int        nImproveRejectRule;
    int        nImproveRejectDelay;
    unsigned   Checksum;
    char *     pError;
    int        ErrorObj;
    int *      pMode1EvalDelta;
    int *      pMode1EvalTouched;
    char *     pMode1EvalMark;
    int        nMode1EvalTouched;
    int        nMode1EvalTouchedPeak;
    float *    pReqMin;
    int *      pReqTouched;
    char *     pReqMark;
    int        nReqTouched;
    int        nReqTouchedPeak;
    float      RequiredAreaGlo;
    int        RequiredNets;
    float      RequiredPower;
    int        nClaimedChunks;
    int        nEmptyClaims;
    int        nProcessedRoots;
    int        nProcessedReprs;
    abctime    ActiveTime;
    abctime    ClaimTime;
    abctime    WaitTime;
#ifdef ABC_USE_PTHREADS
    pthread_t  Thread;
#endif
};

struct If_ParRuntime_t_
{
    If_Man_t *       pMan;
    int              nThreads;
    int              nThreadsStarted;
    int              fStarted;
    int              fStop;
    int              iJob;
    If_ParJobKind_t  JobKind;
    int              nWorking;
    int              iLevel;
    int              Mode;
    int              fPreprocess;
    int              fFirst;
    int              nRoundLevels;
    int              nRoundObjs;
    int              nRoundChoices;
    unsigned         RoundChecksum;
    abctime          RoundTime;
    abctime          RoundFinalizeTime;
    abctime          RoundAllocTime;
    abctime          RoundReleaseTime;
    abctime          RoundRefCheckTime;
    abctime          RoundActiveTime;
    abctime          RoundClaimTime;
    abctime          RoundWaitTime;
    int              nRoundAndTasks;
    int              nRoundChoiceTasks;
    int              nRoundAndChunks;
    int              nRoundChoiceChunks;
    int              nRoundClaimedChunks;
    int              nRoundEmptyClaims;
    int              nRoundAllocCount;
    int              nRoundReleaseCount;
    int              nRoundChoiceSkippedLevels;
    int *            pJobObjs;
    int              nJobTasks;
    int              iJobNext;
    int              nJobNominalChunk;
    int              nJobEffectiveChunk;
    int              nJobChunks;
    int              nJobClaimedChunks;
    int              nJobEmptyClaims;
    int              fJobStop;
    FILE *           pProfileFile;
    int              fProfileRequested;
    int              fProfileEnabled;
    If_Set_t *       pMode0CutSets;
    int              nMode0CutSets;
    If_Cut_t *       pMode1OldCuts;
    If_Cut_t *       pMode1NewCuts;
    char *           pMode1Changed;
    char *           pMode1AllowFinalize;
    int *            pMode1Delta;
    int *            pMode1Touched;
    int *            pMode1Wave;
    int *            pMode1WaveDelta;
    char *           pMode1DeltaMark;
    int              nMode1Touched;
    int *            pMode1RefCheck;
    int *            pReqLevelTouched;
    char *           pReqLevelMark;
    int              nReqLevelTouched;
    uint32_t *       pReqKeys;
    char *           pImproveAccepted;
    int              nImproveRoots;
    int              nImproveCandidates;
    int              nImproveAccepted;
    int              nImproveRejectRule;
    int              nImproveRejectDelay;
    int              nImproveAreaDelta;
    If_ParThread_t * pThreads;
#ifdef ABC_USE_PTHREADS
    pthread_mutex_t  Mutex;
    pthread_cond_t   CondStart;
    pthread_cond_t   CondDone;
#endif
};

////////////////////////////////////////////////////////////////////////
///                     FUNCTION DEFINITIONS                         ///
////////////////////////////////////////////////////////////////////////

static int If_ParFloatEq( float Value, float Target )
{
    float Diff = Value - Target;
    return Diff > (float)-0.000001 && Diff < (float)0.000001;
}

static void If_ManParSetReason( char ** ppReason, char * pReason )
{
    if ( ppReason )
        *ppReason = pReason;
}

static int If_ManParCheckDefaultTiming( If_Man_t * p, char ** ppReason )
{
    If_Obj_t * pObj;
    int i;
    if ( p->pManTim != NULL )
    {
        If_ManParSetReason( ppReason, "timing_manager" );
        return 0;
    }
    if ( p->pPars->pTimesReq != NULL )
    {
        If_ManParSetReason( ppReason, "external_required_times" );
        return 0;
    }
    if ( p->pPars->pTimesArr == NULL )
        return 1;
    If_ManForEachCi( p, pObj, i )
    {
        if ( !If_ParFloatEq( p->pPars->pTimesArr[i], (float)0.0 ) )
        {
            If_ManParSetReason( ppReason, "external_arrival_times" );
            return 0;
        }
    }
    return 1;
}

int If_ManParIsSupported( If_Man_t * p, char ** ppReason )
{
    If_Par_t * pPars = p->pPars;
    if ( pPars->nParThreads < 1 )
        { If_ManParSetReason( ppReason, "invalid_thread_count" ); return 0; }
    if ( pPars->nParThreads > IF_PAR_THREAD_MAX )
        { If_ManParSetReason( ppReason, "thread_count_too_large" ); return 0; }
#ifndef ABC_USE_PTHREADS
    if ( pPars->nParThreads > 1 )
        { If_ManParSetReason( ppReason, "pthreads_disabled" ); return 0; }
#endif
    if ( pPars->nLutSize != 4 && pPars->nLutSize != 6 )
        { If_ManParSetReason( ppReason, "unsupported_lut_width" ); return 0; }
    if ( pPars->pLutLib != NULL )
        { If_ManParSetReason( ppReason, "lut_library" ); return 0; }
    if ( pPars->pCellLib != NULL )
        { If_ManParSetReason( ppReason, "cell_library" ); return 0; }
    if ( pPars->nCutsMax != 8 )
        { If_ManParSetReason( ppReason, "nondefault_cuts" ); return 0; }
    if ( pPars->nFlowIters != 1 )
        { If_ManParSetReason( ppReason, "nondefault_flow_iters" ); return 0; }
    if ( pPars->nAreaIters != 2 )
        { If_ManParSetReason( ppReason, "nondefault_area_iters" ); return 0; }
    if ( !If_ParFloatEq( pPars->DelayTarget, (float)-1.0 ) )
        { If_ManParSetReason( ppReason, "delay_target" ); return 0; }
    if ( !If_ParFloatEq( pPars->Epsilon, (float)0.005 ) )
        { If_ManParSetReason( ppReason, "epsilon" ); return 0; }
    if ( !If_ParFloatEq( pPars->WireDelay, (float)0.0 ) )
        { If_ManParSetReason( ppReason, "wire_delay" ); return 0; }
    if ( pPars->nRelaxRatio != 0 )
        { If_ManParSetReason( ppReason, "relax_ratio" ); return 0; }
    if ( pPars->fPreprocess != 1 )
        { If_ManParSetReason( ppReason, "disabled_preprocess" ); return 0; }
    if ( pPars->fArea != 0 )
        { If_ManParSetReason( ppReason, "area_only" ); return 0; }
    if ( pPars->fFancy != 0 )
        { If_ManParSetReason( ppReason, "fancy_mode" ); return 0; }
    if ( pPars->fEdge != 1 )
        { If_ManParSetReason( ppReason, "disabled_edge" ); return 0; }
    if ( pPars->fLut6Filter )
        { If_ManParSetReason( ppReason, "lut6_filter" ); return 0; }
    if ( pPars->fLatchPaths || pPars->nLatchesCi || pPars->nLatchesCo || pPars->nLatchesCiBox || pPars->nLatchesCoBox )
        { If_ManParSetReason( ppReason, "latch_path" ); return 0; }
    if ( !If_ManParCheckDefaultTiming( p, ppReason ) )
        return 0;
    if ( pPars->fPower )
        { If_ManParSetReason( ppReason, "power" ); return 0; }
    if ( pPars->fTruth || pPars->fUseTtPerm || pPars->fUsePerm )
        { If_ManParSetReason( ppReason, "truth_or_perm" ); return 0; }
    if ( pPars->fUseDsd || pPars->fUseDsdTune || pPars->fDsdBalance )
        { If_ManParSetReason( ppReason, "dsd" ); return 0; }
    if ( pPars->fUseCofVars || pPars->fUseAndVars )
        { If_ManParSetReason( ppReason, "cofactor_or_and_vars" ); return 0; }
    if ( pPars->fUseCheck1 || pPars->fUseCheck2 || pPars->fEnableCheck07 || pPars->fEnableCheck08 || pPars->fEnableCheck10 || pPars->fEnableCheck75 || pPars->fEnableCheck75u )
        { If_ManParSetReason( ppReason, "truth_check" ); return 0; }
    if ( pPars->fBidec || pPars->fUse34Spec || pPars->fCutMin || pPars->fUseBat )
        { If_ManParSetReason( ppReason, "special_cut_mode" ); return 0; }
    if ( pPars->pFuncCost || pPars->pFuncUser || pPars->pFuncCell || pPars->pFuncCell2 || pPars->pFuncWrite )
        { If_ManParSetReason( ppReason, "user_callback" ); return 0; }
    if ( pPars->fUserRecLib || pPars->fUserSesLib )
        { If_ManParSetReason( ppReason, "user_library" ); return 0; }
    if ( pPars->fUserLutDec || pPars->fUserLut2D || pPars->pLutStruct || pPars->nLutDecSize || pPars->fEnableStructN || pPars->fDeriveLuts )
        { If_ManParSetReason( ppReason, "lut_decomposition" ); return 0; }
    if ( pPars->nGateSize > 0 || pPars->nAndDelay > 0 || pPars->nAndArea > 0 )
        { If_ManParSetReason( ppReason, "and_gate_mapping" ); return 0; }
    if ( pPars->fDelayOpt || pPars->fDelayOptLut || pPars->fDelayOptCell )
        { If_ManParSetReason( ppReason, "delay_opt" ); return 0; }
    if ( pPars->fDumpFile )
        { If_ManParSetReason( ppReason, "dump_file" ); return 0; }
    if ( pPars->fUseBuffs )
        { If_ManParSetReason( ppReason, "output_buffers" ); return 0; }
    if ( pPars->fDoAverage )
        { If_ManParSetReason( ppReason, "average_required" ); return 0; }
    if ( pPars->fHashMapping )
        { If_ManParSetReason( ppReason, "hash_mapping" ); return 0; }
    if ( pPars->fVerboseTrace )
        { If_ManParSetReason( ppReason, "verbose_trace" ); return 0; }
    if ( pPars->nNonDecLimit || pPars->nStructType )
        { If_ManParSetReason( ppReason, "nondefault_structure" ); return 0; }
    if ( pPars->fExpRed == 0 && p->nChoices == 0 )
        { If_ManParSetReason( ppReason, "no_choice_expred_disabled" ); return 0; }
    if ( pPars->fExpRed != 0 && p->nChoices != 0 )
        { If_ManParSetReason( ppReason, "choice_expred_enabled" ); return 0; }
    if ( pPars->fSkipCutFilter || pPars->fAreaOnly || pPars->fLiftLeaves || pPars->fUseCoAttrs )
        { If_ManParSetReason( ppReason, "internal_mode" ); return 0; }
    if ( pPars->fUseBdds || pPars->fUseSops || pPars->fUseCnfs || pPars->fUseMv )
        { If_ManParSetReason( ppReason, "nonstandard_output" ); return 0; }
    if ( !If_ParFloatEq( pPars->DelayTargetNew, (float)0.0 ) )
        { If_ManParSetReason( ppReason, "delay_target_new" ); return 0; }
    if ( pPars->pReoMan )
        { If_ManParSetReason( ppReason, "reordering_manager" ); return 0; }
    If_ManParSetReason( ppReason, "supported" );
    return 1;
}

static void If_ManParFreeMeta( If_ParMeta_t * pMeta )
{
    if ( pMeta == NULL )
        return;
    ABC_FREE( pMeta->pLevelCounts );
    ABC_FREE( pMeta->pLevelStarts );
    ABC_FREE( pMeta->pLevelObjs );
    ABC_FREE( pMeta->pFreeLevel );
    ABC_FREE( pMeta->pCutSetSlot );
    ABC_FREE( pMeta->pReleaseStarts );
    ABC_FREE( pMeta->pReleaseObjs );
    ABC_FREE( pMeta->pChoiceReprs );
    ABC_FREE( pMeta->pChoiceLevelCounts );
    ABC_FREE( pMeta->pChoiceLevelStarts );
    ABC_FREE( pMeta->pChoiceLevelObjs );
    ABC_FREE( pMeta->pProviderStarts );
    ABC_FREE( pMeta->pProviders );
    ABC_FREE( pMeta->pIsChoiceProvider );
    ABC_FREE( pMeta );
}

static unsigned If_ManParHash( unsigned Hash, unsigned Value )
{
    return Hash ^ (Value + 0x9e3779b9u + (Hash << 6) + (Hash >> 2));
}

static int If_ParWordCountOnes( unsigned uWord )
{
    uWord = (uWord & 0x55555555) + ((uWord>>1) & 0x55555555);
    uWord = (uWord & 0x33333333) + ((uWord>>2) & 0x33333333);
    uWord = (uWord & 0x0F0F0F0F) + ((uWord>>4) & 0x0F0F0F0F);
    uWord = (uWord & 0x00FF00FF) + ((uWord>>8) & 0x00FF00FF);
    return  (uWord & 0x0000FFFF) + (uWord>>16);
}

static If_Set_t * If_ManParMode0CutSet( If_ParRuntime_t * pRuntime, int i )
{
    assert( i >= 0 && i < pRuntime->nMode0CutSets );
    return (If_Set_t *)((char *)pRuntime->pMode0CutSets + (size_t)i * pRuntime->pMan->nSetBytes);
}

static void If_ManParMode0PoolSetup( If_ParRuntime_t * pRuntime )
{
    If_Set_t * pSet;
    int i;
    for ( i = 0; i < pRuntime->nMode0CutSets; i++ )
    {
        pSet = If_ManParMode0CutSet( pRuntime, i );
        If_ManSetupSet( pRuntime->pMan, pSet );
        pSet->pNext = NULL;
    }
}

static void If_ManParRequestJobStop( If_ParThread_t * pThread )
{
    If_ParRuntime_t * pRuntime = pThread->pRuntime;
#ifdef ABC_USE_PTHREADS
    if ( pRuntime->nThreads > 1 )
    {
        pthread_mutex_lock( &pRuntime->Mutex );
        pRuntime->fJobStop = 1;
        pthread_mutex_unlock( &pRuntime->Mutex );
        return;
    }
#endif
    pRuntime->fJobStop = 1;
}

static void If_ManParThreadSetError( If_ParThread_t * pThread, char * pError, If_Obj_t * pObj )
{
    if ( pThread->pError == NULL )
    {
        pThread->pError = pError;
        pThread->ErrorObj = pObj ? pObj->Id : -1;
    }
    If_ManParRequestJobStop( pThread );
}

static int If_ManParCheckMode0Standard( If_Man_t * p, char ** ppReason )
{
    If_Par_t * pPars = p->pPars;
    if ( p->pManTim != NULL || pPars->pLutLib != NULL || pPars->pCellLib != NULL )
        { If_ManParSetReason( ppReason, "mode0_nonstandard_flag" ); return 0; }
    if ( pPars->fTruth || pPars->fUseTtPerm || pPars->fUsePerm || pPars->fPower )
        { If_ManParSetReason( ppReason, "mode0_nonstandard_flag" ); return 0; }
    if ( pPars->fDelayOpt || pPars->fDelayOptLut || pPars->fDelayOptCell || pPars->fDsdBalance )
        { If_ManParSetReason( ppReason, "mode0_nonstandard_flag" ); return 0; }
    if ( pPars->fUseDsd || pPars->fUseDsdTune || pPars->fUseCofVars || pPars->fUseAndVars )
        { If_ManParSetReason( ppReason, "mode0_nonstandard_flag" ); return 0; }
    if ( pPars->fUse34Spec || pPars->fCutMin || pPars->fUseBat || pPars->fSkipCutFilter )
        { If_ManParSetReason( ppReason, "mode0_nonstandard_flag" ); return 0; }
    if ( pPars->pFuncCost || pPars->pFuncUser || pPars->pFuncCell || pPars->pFuncCell2 || pPars->pFuncWrite )
        { If_ManParSetReason( ppReason, "mode0_nonstandard_flag" ); return 0; }
    if ( pPars->fUserRecLib || pPars->fUserSesLib || pPars->fUserLutDec || pPars->fUserLut2D )
        { If_ManParSetReason( ppReason, "mode0_nonstandard_flag" ); return 0; }
    if ( pPars->pLutStruct || pPars->nGateSize > 0 || pPars->nAndDelay > 0 || pPars->nAndArea > 0 )
        { If_ManParSetReason( ppReason, "mode0_nonstandard_flag" ); return 0; }
    if ( pPars->fLiftLeaves || pPars->fUseCoAttrs || p->vCuts != NULL )
        { If_ManParSetReason( ppReason, "mode0_nonstandard_flag" ); return 0; }
    If_ManParSetReason( ppReason, "ok" );
    return 1;
}

static void If_ManParThreadReset( If_ParThread_t * pThread )
{
    pThread->nObjs = 0;
    pThread->nChoices = 0;
    pThread->nCutsMerged = 0;
    pThread->nCutsTotal = 0;
    pThread->nCandidateCount = 0;
    pThread->nImproveRoots = 0;
    pThread->nImproveCandidates = 0;
    pThread->nImproveAccepted = 0;
    pThread->nImproveRejectRule = 0;
    pThread->nImproveRejectDelay = 0;
    pThread->Checksum = 0;
    pThread->pError = NULL;
    pThread->ErrorObj = -1;
    pThread->nMode1EvalTouchedPeak = 0;
    pThread->nReqTouchedPeak = 0;
    pThread->RequiredAreaGlo = 0.0;
    pThread->RequiredNets = 0;
    pThread->RequiredPower = 0.0;
    pThread->nClaimedChunks = 0;
    pThread->nEmptyClaims = 0;
    pThread->nProcessedRoots = 0;
    pThread->nProcessedReprs = 0;
    pThread->ActiveTime = 0;
    pThread->ClaimTime = 0;
    pThread->WaitTime = 0;
}

static void If_ManParThreadAddNodeSummary( If_ParThread_t * pThread, If_Obj_t * pObj )
{
    If_ParRuntime_t * pRuntime = pThread->pRuntime;
    If_Man_t * p = pRuntime->pMan;
    If_ParMeta_t * pMeta = p->pParMeta;
    If_Obj_t * pFanin;
    unsigned Hash;
    pThread->nObjs++;
    Hash = If_ManParHash( 2166136261u, (unsigned)pObj->Id );
    Hash = If_ManParHash( Hash, (unsigned)If_ObjLevel(pObj) );
    pFanin = If_ObjFanin0(pObj);
    Hash = If_ManParHash( Hash, (unsigned)If_ObjId(pFanin) );
    Hash = If_ManParHash( Hash, (unsigned)If_ObjLevel(pFanin) );
    pFanin = If_ObjFanin1(pObj);
    Hash = If_ManParHash( Hash, (unsigned)If_ObjId(pFanin) );
    Hash = If_ManParHash( Hash, (unsigned)If_ObjLevel(pFanin) );
    Hash = If_ManParHash( Hash, (unsigned)pMeta->pFreeLevel[pObj->Id] );
    pThread->Checksum += Hash;
}

static If_Cut_t * If_ManParMode1OldCut( If_ParRuntime_t * pRuntime, int ObjId )
{
    assert( ObjId >= 0 && ObjId < pRuntime->pMan->pParMeta->nObjs );
    return (If_Cut_t *)((char *)pRuntime->pMode1OldCuts + (size_t)ObjId * pRuntime->pMan->nCutBytes);
}

static If_Cut_t * If_ManParMode1NewCut( If_ParRuntime_t * pRuntime, int ObjId )
{
    assert( ObjId >= 0 && ObjId < pRuntime->pMan->pParMeta->nObjs );
    return (If_Cut_t *)((char *)pRuntime->pMode1NewCuts + (size_t)ObjId * pRuntime->pMan->nCutBytes);
}

static int If_ManParCutLeavesEqual( If_Cut_t * pCut0, If_Cut_t * pCut1 )
{
    if ( pCut0->nLeaves != pCut1->nLeaves )
        return 0;
    return memcmp( pCut0->pLeaves, pCut1->pLeaves, sizeof(int) * pCut0->nLeaves ) == 0;
}

static void If_ManParMode1EvalClear( If_ParThread_t * pThread )
{
    int i, ObjId;
    for ( i = 0; i < pThread->nMode1EvalTouched; i++ )
    {
        ObjId = pThread->pMode1EvalTouched[i];
        pThread->pMode1EvalDelta[ObjId] = 0;
        pThread->pMode1EvalMark[ObjId] = 0;
    }
    pThread->nMode1EvalTouched = 0;
}

static void If_ManParMode1EvalAdd( If_ParThread_t * pThread, If_Obj_t * pObj, int Delta )
{
    int ObjId = pObj->Id;
    if ( !pThread->pMode1EvalMark[ObjId] )
    {
        pThread->pMode1EvalMark[ObjId] = 1;
        pThread->pMode1EvalTouched[pThread->nMode1EvalTouched++] = ObjId;
        if ( pThread->nMode1EvalTouchedPeak < pThread->nMode1EvalTouched )
            pThread->nMode1EvalTouchedPeak = pThread->nMode1EvalTouched;
    }
    pThread->pMode1EvalDelta[ObjId] += Delta;
}

static int If_ManParMode1EvalRefs( If_ParThread_t * pThread, If_Obj_t * pObj )
{
    return pObj->nRefs + pThread->pMode1EvalDelta[pObj->Id];
}

static int If_ManParMode1DerefEvalCut( If_ParThread_t * pThread, If_Cut_t * pCut );

static If_Cut_t * If_ManParImproveOldCut( If_ParRuntime_t * pRuntime, int ObjId )
{
    return If_ManParMode1OldCut( pRuntime, ObjId );
}

static If_Cut_t * If_ManParImproveNewCut( If_ParRuntime_t * pRuntime, int ObjId )
{
    return If_ManParMode1NewCut( pRuntime, ObjId );
}

static void If_ManParImproveEvalClear( If_ParThread_t * pThread )
{
    If_ManParMode1EvalClear( pThread );
}

static int If_ManParImproveEvalRefs( If_ParThread_t * pThread, If_Obj_t * pObj )
{
    return If_ManParMode1EvalRefs( pThread, pObj );
}

static void If_ManParImproveFrontierClear( If_ParThread_t * pThread )
{
    int i, ObjId;
    for ( i = 0; i < pThread->nReqTouched; i++ )
    {
        ObjId = pThread->pReqTouched[i];
        pThread->pReqMark[ObjId] = 0;
    }
    pThread->nReqTouched = 0;
}

static int If_ManParImproveFrontierAdd( If_ParThread_t * pThread, If_Obj_t * pObj )
{
    int ObjId = pObj->Id;
    if ( ObjId < 0 || ObjId >= pThread->pRuntime->pMan->pParMeta->nObjs )
        { If_ManParThreadSetError( pThread, "improve_frontier_range", pObj ); return 0; }
    if ( pThread->pReqMark[ObjId] )
        return 1;
    pThread->pReqMark[ObjId] = 1;
    pThread->pReqTouched[pThread->nReqTouched++] = ObjId;
    if ( pThread->nReqTouchedPeak < pThread->nReqTouched )
        pThread->nReqTouchedPeak = pThread->nReqTouched;
    return 1;
}

static void If_ManParImproveFrontierRemove( If_ParThread_t * pThread, int iLeaf )
{
    int ObjId, i;
    assert( iLeaf >= 0 && iLeaf < pThread->nReqTouched );
    ObjId = pThread->pReqTouched[iLeaf];
    pThread->pReqMark[ObjId] = 0;
    for ( i = iLeaf; i < pThread->nReqTouched - 1; i++ )
        pThread->pReqTouched[i] = pThread->pReqTouched[i+1];
    pThread->nReqTouched--;
}

static int If_ManParImproveFaninAllowed( If_ParThread_t * pThread, If_Obj_t * pFanin )
{
    int Refs;
    if ( pThread->pReqMark[pFanin->Id] )
        return 1;
    if ( !If_ObjIsAnd(pFanin) )
        return 1;
    Refs = If_ManParImproveEvalRefs( pThread, pFanin );
    if ( Refs < 0 )
        { If_ManParThreadSetError( pThread, "improve_eval_negative_ref", pFanin ); return -1; }
    return Refs > 0;
}

static int If_ManParImproveLeafCanExpand( If_ParThread_t * pThread, If_Obj_t * pLeaf, int nLimit )
{
    If_Obj_t * pFanin0, * pFanin1;
    int Refs, nNewLeaves, fAdd0, fAdd1, Allowed;
    if ( !If_ObjIsAnd(pLeaf) )
        return 0;
    Refs = If_ManParImproveEvalRefs( pThread, pLeaf );
    if ( Refs < 0 )
        { If_ManParThreadSetError( pThread, "improve_eval_negative_ref", pLeaf ); return -1; }
    if ( Refs != 0 )
        return 0;
    pFanin0 = If_ObjFanin0(pLeaf);
    pFanin1 = If_ObjFanin1(pLeaf);
    fAdd0 = !pThread->pReqMark[pFanin0->Id];
    fAdd1 = !pThread->pReqMark[pFanin1->Id] && pFanin1->Id != pFanin0->Id;
    nNewLeaves = pThread->nReqTouched - 1 + fAdd0 + fAdd1;
    if ( nNewLeaves > nLimit )
        return 0;
    Allowed = If_ManParImproveFaninAllowed( pThread, pFanin0 );
    if ( Allowed <= 0 )
        return Allowed;
    Allowed = If_ManParImproveFaninAllowed( pThread, pFanin1 );
    if ( Allowed <= 0 )
        return Allowed;
    return 1;
}

static int If_ManParImproveApplyExpansion( If_ParThread_t * pThread, int iLeaf )
{
    If_Man_t * p = pThread->pRuntime->pMan;
    If_Obj_t * pLeaf = If_ManObj( p, pThread->pReqTouched[iLeaf] );
    If_Obj_t * pFanin0 = If_ObjFanin0(pLeaf);
    If_Obj_t * pFanin1 = If_ObjFanin1(pLeaf);
    If_ManParImproveFrontierRemove( pThread, iLeaf );
    if ( !If_ManParImproveFrontierAdd( pThread, pFanin0 ) )
        return 0;
    if ( !If_ManParImproveFrontierAdd( pThread, pFanin1 ) )
        return 0;
    return 1;
}

static int If_ObjImproveComputePar( If_ParThread_t * pThread, If_Obj_t * pObj )
{
    If_ParRuntime_t * pRuntime = pThread->pRuntime;
    If_Man_t * p = pRuntime->pMan;
    If_Cut_t * pOld, * pNew;
    If_Obj_t * pLeaf;
    float Delay;
    int i, fChanged, fExpanded, Rule;
    assert( pRuntime->Mode == 7 );
    assert( If_ObjIsAnd(pObj) );
    If_ManParThreadAddNodeSummary( pThread, pObj );
    if ( p->pParMeta->pIsChoiceProvider[pObj->Id] || pObj->nRefs == 0 )
        return 1;
    pThread->nImproveRoots++;
    if ( pThread->nMode1EvalTouched != 0 )
        { If_ManParThreadSetError( pThread, "improve_eval_leak", pObj ); return 0; }
    if ( pThread->nReqTouched != 0 )
        If_ManParImproveFrontierClear( pThread );
    pOld = If_ManParImproveOldCut( pRuntime, pObj->Id );
    pNew = If_ManParImproveNewCut( pRuntime, pObj->Id );
    If_CutCopy( p, pOld, If_ObjCutBest(pObj) );
    If_CutCopy( p, pNew, pOld );
    If_ManParImproveEvalClear( pThread );
    if ( !If_ManParMode1DerefEvalCut( pThread, pOld ) )
        goto fail;
    If_CutForEachLeaf( p, pOld, pLeaf, i )
        if ( !If_ManParImproveFrontierAdd( pThread, pLeaf ) )
            goto fail;
    fChanged = 0;
    do {
        fExpanded = 0;
        for ( i = 0; i < pThread->nReqTouched; i++ )
        {
            pLeaf = If_ManObj( p, pThread->pReqTouched[i] );
            if ( !If_ObjIsAnd(pLeaf) )
                continue;
            pThread->nCandidateCount++;
            pThread->nImproveCandidates++;
            Rule = If_ManParImproveLeafCanExpand( pThread, pLeaf, p->pPars->nLutSize );
            if ( Rule < 0 )
                goto fail;
            if ( Rule == 0 )
            {
                pThread->nImproveRejectRule++;
                continue;
            }
            if ( !If_ManParImproveApplyExpansion( pThread, i ) )
                goto fail;
            fChanged = 1;
            fExpanded = 1;
            break;
        }
    } while ( fExpanded );
    if ( !fChanged )
    {
        If_ManParImproveFrontierClear( pThread );
        If_ManParImproveEvalClear( pThread );
        return 1;
    }
    pNew->nLeaves = pThread->nReqTouched;
    for ( i = 0; i < pThread->nReqTouched; i++ )
        pNew->pLeaves[i] = pThread->pReqTouched[i];
    If_CutOrder( pNew );
    pNew->uSign = If_ObjCutSignCompute( pNew );
    Delay = If_CutDelay( p, pObj, pNew );
    if ( Delay == -1 )
        { If_ManParThreadSetError( pThread, "improve_delay_eval", pObj ); goto fail; }
    pNew->Delay = Delay;
    if ( pNew->Delay > pObj->Required + p->fEpsilon )
    {
        pThread->nImproveRejectDelay++;
        If_ManParImproveFrontierClear( pThread );
        If_ManParImproveEvalClear( pThread );
        return 1;
    }
    if ( If_ManParCutLeavesEqual( pOld, pNew ) )
    {
        If_ManParImproveFrontierClear( pThread );
        If_ManParImproveEvalClear( pThread );
        return 1;
    }
    pRuntime->pImproveAccepted[pObj->Id] = 1;
    pThread->nImproveAccepted++;
    If_ManParImproveFrontierClear( pThread );
    If_ManParImproveEvalClear( pThread );
    return 1;
fail:
    If_ManParImproveFrontierClear( pThread );
    If_ManParImproveEvalClear( pThread );
    return 0;
}

static void If_ManParRequiredThreadClear( If_ParThread_t * pThread )
{
    int i, ObjId;
    for ( i = 0; i < pThread->nReqTouched; i++ )
    {
        ObjId = pThread->pReqTouched[i];
        pThread->pReqMin[ObjId] = 0;
        pThread->pReqMark[ObjId] = 0;
    }
    pThread->nReqTouched = 0;
}

static int If_ManParRequiredAdd( If_ParThread_t * pThread, If_Obj_t * pLeaf, float Required )
{
    If_ParRuntime_t * pRuntime = pThread->pRuntime;
    int ObjId = pLeaf->Id;
    if ( ObjId < 0 || ObjId >= pRuntime->pMan->pParMeta->nObjs )
        { If_ManParThreadSetError( pThread, "required_leaf_range", pLeaf ); return 0; }
    if ( !pThread->pReqMark[ObjId] )
    {
        pThread->pReqMark[ObjId] = 1;
        pThread->pReqTouched[pThread->nReqTouched++] = ObjId;
        pThread->pReqMin[ObjId] = Required;
        if ( pThread->nReqTouchedPeak < pThread->nReqTouched )
            pThread->nReqTouchedPeak = pThread->nReqTouched;
    }
    else
        pThread->pReqMin[ObjId] = IF_MIN( pThread->pReqMin[ObjId], Required );
    pThread->nCandidateCount++;
    return 1;
}

static int If_ManParRequiredFloatValid( float Value )
{
    return isfinite( (double)Value ) && !isnan( (double)Value );
}

static uint32_t If_ManParRequiredEncodeRaw( float Value )
{
    uint32_t Bits;
    memcpy( &Bits, &Value, sizeof(Bits) );
    return (Bits & 0x80000000u) ? ~Bits : (Bits ^ 0x80000000u);
}

static float If_ManParRequiredDecodeRaw( uint32_t Key )
{
    uint32_t Bits = (Key & 0x80000000u) ? (Key ^ 0x80000000u) : ~Key;
    float Value;
    memcpy( &Value, &Bits, sizeof(Value) );
    return Value;
}

static int If_ManParRequiredEncode( float Value, uint32_t * pKey )
{
    if ( sizeof(float) != 4 || !If_ManParRequiredFloatValid(Value) )
        return 0;
    *pKey = If_ManParRequiredEncodeRaw( Value );
    return 1;
}

static int If_ManParRequiredAtomicMin( If_ParThread_t * pThread, If_Obj_t * pLeaf, float Required )
{
    If_ParRuntime_t * pRuntime = pThread->pRuntime;
    uint32_t Key, Old, Expected;
    int ObjId = pLeaf->Id;
    if ( ObjId < 0 || ObjId >= pRuntime->pMan->pParMeta->nObjs )
        { If_ManParThreadSetError( pThread, "required_leaf_range", pLeaf ); return 0; }
    if ( !If_ManParRequiredEncode( Required, &Key ) )
        { If_ManParThreadSetError( pThread, "required_float", pLeaf ); return 0; }
    Old = If_ParAtomicU32Load( pRuntime->pReqKeys + ObjId );
    while ( Key < Old )
    {
        Expected = Old;
        if ( If_ParAtomicU32CompareExchangeWeak( pRuntime->pReqKeys + ObjId, &Expected, Key ) )
            break;
        Old = Expected;
    }
    pThread->nCandidateCount++;
    return 1;
}

static int If_ManParRequiredReadKey( If_ParRuntime_t * pRuntime, If_Obj_t * pObj, float * pRequired )
{
    uint32_t Key;
    if ( pObj->Id < 0 || pObj->Id >= pRuntime->pMan->pParMeta->nObjs )
        return 0;
    Key = If_ParAtomicU32Load( pRuntime->pReqKeys + pObj->Id );
    *pRequired = If_ManParRequiredDecodeRaw( Key );
    return If_ManParRequiredFloatValid( *pRequired );
}

static int If_ObjPropagateRequiredStage9Par( If_ParThread_t * pThread, If_Obj_t * pObj )
{
    If_Man_t * p = pThread->pRuntime->pMan;
    If_Cut_t * pCut = If_ObjCutBest(pObj);
    If_Obj_t * pLeaf;
    float ObjRequired, Required;
    int i, ObjId;
    assert( If_ObjIsAnd(pObj) );
    if ( pObj->nRefs == 0 )
        return 1;
    if ( pCut == NULL )
        { If_ManParThreadSetError( pThread, "required_missing_cut", pObj ); return 0; }
    if ( pCut->fAndCut )
        { If_ManParThreadSetError( pThread, "required_and_cut", pObj ); return 0; }
    if ( pCut->fUser )
        { If_ManParThreadSetError( pThread, "required_user_cut", pObj ); return 0; }
    if ( !If_ManParRequiredReadKey( pThread->pRuntime, pObj, &ObjRequired ) )
        { If_ManParThreadSetError( pThread, "required_root_float", pObj ); return 0; }
    Required = ObjRequired - (float)1.0;
    if ( !If_ManParRequiredFloatValid( Required ) )
        { If_ManParThreadSetError( pThread, "required_candidate_float", pObj ); return 0; }
    for ( i = 0; i < (int)pCut->nLeaves; i++ )
    {
        ObjId = pCut->pLeaves[i];
        if ( ObjId < 0 || ObjId >= p->pParMeta->nObjs )
            { If_ManParThreadSetError( pThread, "required_leaf_range", pObj ); return 0; }
        pLeaf = If_ManObj( p, ObjId );
        if ( pLeaf->Id == pObj->Id )
            { If_ManParThreadSetError( pThread, "required_self_leaf", pObj ); return 0; }
        if ( If_ObjIsAnd(pLeaf) && If_ObjLevel(pLeaf) >= If_ObjLevel(pObj) )
            { If_ManParThreadSetError( pThread, "required_leaf_level", pLeaf ); return 0; }
        if ( !If_ManParRequiredAtomicMin( pThread, pLeaf, Required ) )
            return 0;
    }
    return 1;
}

static int If_ObjPropagateRequiredPar( If_ParThread_t * pThread, If_Obj_t * pObj )
{
    If_Man_t * p = pThread->pRuntime->pMan;
    If_Cut_t * pCut = If_ObjCutBest(pObj);
    If_Obj_t * pLeaf;
    int i;
    assert( If_ObjIsAnd(pObj) );
    if ( pObj->nRefs == 0 )
        return 1;
    if ( pCut == NULL )
        { If_ManParThreadSetError( pThread, "required_missing_cut", pObj ); return 0; }
    if ( pCut->fAndCut )
        { If_ManParThreadSetError( pThread, "required_and_cut", pObj ); return 0; }
    if ( pCut->fUser )
        { If_ManParThreadSetError( pThread, "required_user_cut", pObj ); return 0; }
    If_CutForEachLeaf( p, pCut, pLeaf, i )
    {
        if ( pLeaf->Id == pObj->Id )
            { If_ManParThreadSetError( pThread, "required_self_leaf", pObj ); return 0; }
        if ( If_ObjIsAnd(pLeaf) && If_ObjLevel(pLeaf) >= If_ObjLevel(pObj) )
            { If_ManParThreadSetError( pThread, "required_leaf_level", pLeaf ); return 0; }
        if ( !If_ManParRequiredAdd( pThread, pLeaf, pObj->Required - (float)1.0 ) )
            return 0;
    }
    return 1;
}

static int If_ManParMode1DerefEvalCut( If_ParThread_t * pThread, If_Cut_t * pCut )
{
    If_Man_t * p = pThread->pRuntime->pMan;
    If_Obj_t * pLeaf;
    int i, Refs;
    If_CutForEachLeaf( p, pCut, pLeaf, i )
    {
        Refs = If_ManParMode1EvalRefs( pThread, pLeaf );
        if ( Refs <= 0 )
            { If_ManParThreadSetError( pThread, "mode1_eval_negative_ref", pLeaf ); return 0; }
        If_ManParMode1EvalAdd( pThread, pLeaf, -1 );
        if ( Refs - 1 > 0 || !If_ObjIsAnd(pLeaf) )
            continue;
        if ( !If_ManParMode1DerefEvalCut( pThread, If_ObjCutBest(pLeaf) ) )
            return 0;
    }
    return 1;
}

static float If_CutAreaFlowMode1Par( If_ParThread_t * pThread, If_Cut_t * pCut )
{
    If_Man_t * p = pThread->pRuntime->pMan;
    If_Obj_t * pLeaf;
    float Flow, AddOn;
    int i, Refs;
    Flow = If_CutLutArea(p, pCut);
    If_CutForEachLeaf( p, pCut, pLeaf, i )
    {
        Refs = If_ManParMode1EvalRefs( pThread, pLeaf );
        if ( Refs < 0 )
        {
            If_ManParThreadSetError( pThread, "mode1_eval_negative_ref", pLeaf );
            return (float)1e32;
        }
        if ( Refs == 0 || If_ObjIsConst1(pLeaf) )
            AddOn = If_ObjCutBest(pLeaf)->Area;
        else
        {
            if ( pLeaf->EstRefs <= p->fEpsilon )
            {
                If_ManParThreadSetError( pThread, "mode1_est_refs", pLeaf );
                return (float)1e32;
            }
            AddOn = If_ObjCutBest(pLeaf)->Area / pLeaf->EstRefs;
        }
        if ( Flow >= (float)1e32 || AddOn >= (float)1e32 )
            Flow = (float)1e32;
        else
        {
            Flow += AddOn;
            if ( Flow > (float)1e32 )
                Flow = (float)1e32;
        }
    }
    return Flow;
}

static float If_CutEdgeFlowMode1Par( If_ParThread_t * pThread, If_Cut_t * pCut )
{
    If_Man_t * p = pThread->pRuntime->pMan;
    If_Obj_t * pLeaf;
    float Flow, AddOn;
    int i, Refs;
    Flow = pCut->nLeaves;
    If_CutForEachLeaf( p, pCut, pLeaf, i )
    {
        Refs = If_ManParMode1EvalRefs( pThread, pLeaf );
        if ( Refs < 0 )
        {
            If_ManParThreadSetError( pThread, "mode1_eval_negative_ref", pLeaf );
            return (float)1e32;
        }
        if ( Refs == 0 || If_ObjIsConst1(pLeaf) )
            AddOn = If_ObjCutBest(pLeaf)->Edge;
        else
        {
            if ( pLeaf->EstRefs <= p->fEpsilon )
            {
                If_ManParThreadSetError( pThread, "mode1_est_refs", pLeaf );
                return (float)1e32;
            }
            AddOn = If_ObjCutBest(pLeaf)->Edge / pLeaf->EstRefs;
        }
        if ( Flow >= (float)1e32 || AddOn >= (float)1e32 )
            Flow = (float)1e32;
        else
        {
            Flow += AddOn;
            if ( Flow > (float)1e32 )
                Flow = (float)1e32;
        }
    }
    return Flow;
}

static void If_ManParMode2MetricInit( If_ParMode2Metric_t * pMetric )
{
    pMetric->Area = 0.0;
    pMetric->Edge = 0.0;
}

static void If_ManParMode2MetricAddCut( If_Man_t * p, If_ParMode2Metric_t * pMetric, If_Cut_t * pCut )
{
    pMetric->Area += If_CutLutArea( p, pCut );
    pMetric->Edge += (float)pCut->nLeaves;
}

static int If_ManParMode2EvalRefCutMetricPair( If_ParThread_t * pThread, If_Cut_t * pCut, If_ParMode2Metric_t * pMetric )
{
    If_Man_t * p = pThread->pRuntime->pMan;
    If_Obj_t * pLeaf;
    int i, Refs;
    If_ManParMode2MetricAddCut( p, pMetric, pCut );
    If_CutForEachLeaf( p, pCut, pLeaf, i )
    {
        Refs = If_ManParMode1EvalRefs( pThread, pLeaf );
        if ( Refs < 0 )
            { If_ManParThreadSetError( pThread, "mode2_eval_negative_ref", pLeaf ); return 0; }
        If_ManParMode1EvalAdd( pThread, pLeaf, 1 );
        if ( Refs > 0 || !If_ObjIsAnd(pLeaf) )
            continue;
        if ( !If_ManParMode2EvalRefCutMetricPair( pThread, If_ObjCutBest(pLeaf), pMetric ) )
            return 0;
    }
    return 1;
}

static int If_ManParMode2EvalDerefCutMetricPair( If_ParThread_t * pThread, If_Cut_t * pCut, If_ParMode2Metric_t * pMetric )
{
    If_Man_t * p = pThread->pRuntime->pMan;
    If_Obj_t * pLeaf;
    int i, Refs;
    If_ManParMode2MetricAddCut( p, pMetric, pCut );
    If_CutForEachLeaf( p, pCut, pLeaf, i )
    {
        Refs = If_ManParMode1EvalRefs( pThread, pLeaf );
        if ( Refs <= 0 )
            { If_ManParThreadSetError( pThread, "mode2_eval_negative_ref", pLeaf ); return 0; }
        If_ManParMode1EvalAdd( pThread, pLeaf, -1 );
        if ( Refs - 1 > 0 || !If_ObjIsAnd(pLeaf) )
            continue;
        if ( !If_ManParMode2EvalDerefCutMetricPair( pThread, If_ObjCutBest(pLeaf), pMetric ) )
            return 0;
    }
    return 1;
}

static int If_ManParMode2ExactMetricPair( If_ParThread_t * pThread, If_Obj_t * pRoot, If_Cut_t * pCut, If_Cut_t * pOld, If_ParMode2Metric_t * pMetric )
{
    If_ParMode2Metric_t RefMetric, DerefMetric;
    if ( pThread->nMode1EvalTouched != 0 )
        { If_ManParThreadSetError( pThread, "mode2_eval_leak", pRoot ); return 0; }
    If_ManParMode2MetricInit( &RefMetric );
    If_ManParMode2MetricInit( &DerefMetric );
    if ( pCut->nLeaves < 2 )
    {
        RefMetric.Area = 0.0;
        RefMetric.Edge = (float)pCut->nLeaves;
    }
    else if ( !If_ManParMode2EvalRefCutMetricPair( pThread, pCut, &RefMetric ) )
        goto fail;
    if ( pRoot->nRefs > 0 && !If_ManParMode2EvalDerefCutMetricPair( pThread, pOld, &DerefMetric ) )
        goto fail;
    pMetric->Area = RefMetric.Area - DerefMetric.Area;
    pMetric->Edge = RefMetric.Edge - DerefMetric.Edge;
    If_ManParMode1EvalClear( pThread );
    return 1;
fail:
    If_ManParMode1EvalClear( pThread );
    return 0;
}

static int If_CutExactMetricMode2Par( If_ParThread_t * pThread, If_Obj_t * pObj, If_Cut_t * pCut, If_Cut_t * pOld )
{
    If_ParMode2Metric_t Metric;
    if ( !If_ManParMode2ExactMetricPair( pThread, pObj, pCut, pOld, &Metric ) )
        return 0;
    pCut->Area = Metric.Area;
    pCut->Edge = Metric.Edge;
    return 1;
}

static int If_ObjPerformMappingAndMode0Par( If_ParThread_t * pThread, If_Obj_t * pObj )
{
    If_ParRuntime_t * pRuntime = pThread->pRuntime;
    If_Man_t * p = pRuntime->pMan;
    If_Set_t * pCutSet = pObj->pCutSet;
    If_Cut_t * pCut0, * pCut1, * pCut;
    int i, k;
    assert( pRuntime->Mode == 0 );
    assert( If_ObjIsAnd(pObj) );
    if ( pCutSet == NULL )
        { If_ManParThreadSetError( pThread, "mode0_live_cutset", pObj ); return 0; }
    if ( If_ObjIsAnd(pObj->pFanin0) && (pObj->pFanin0->pCutSet == NULL || pObj->pFanin0->pCutSet->nCuts == 0) )
        { If_ManParThreadSetError( pThread, "mode0_missing_fanin_cutset", pObj ); return 0; }
    if ( If_ObjIsAnd(pObj->pFanin1) && (pObj->pFanin1->pCutSet == NULL || pObj->pFanin1->pCutSet->nCuts == 0) )
        { If_ManParThreadSetError( pThread, "mode0_missing_fanin_cutset", pObj ); return 0; }

    pObj->EstRefs = (float)pObj->nRefs;
    if ( !pRuntime->fFirst )
    {
        pCut = If_ObjCutBest(pObj);
        pCut->Delay = If_CutDelay( p, pObj, pCut );
        if ( pCut->Delay == -1 )
            { If_ManParThreadSetError( pThread, "mode0_nonstandard_flag", pObj ); return 0; }
        if ( pCut->Delay > pObj->Required + 2*p->fEpsilon )
            Abc_Print( 1, "If_ObjPerformMappingAndMode0Par(): Warning! Node with ID %d has delay (%f) exceeding the required times (%f).\n",
                pObj->Id, pCut->Delay, pObj->Required + p->fEpsilon );
        pCut->Area = If_CutAreaFlow( p, pCut );
        if ( p->pPars->fEdge )
            pCut->Edge = If_CutEdgeFlow( p, pCut );
        if ( !pRuntime->fPreprocess || pCut->nLeaves <= 1 )
        {
            assert( pCutSet->nCuts <= pCutSet->nCutsMax );
            If_CutCopy( p, pCutSet->ppCuts[pCutSet->nCuts++], pCut );
        }
    }

    If_ObjForEachCut( pObj->pFanin0, pCut0, i )
    If_ObjForEachCut( pObj->pFanin1, pCut1, k )
    {
        assert( pCutSet->nCuts <= pCutSet->nCutsMax );
        pCut = pCutSet->ppCuts[pCutSet->nCuts];
        if ( If_ParWordCountOnes(pCut0->uSign | pCut1->uSign) > p->pPars->nLutSize )
            continue;
        if ( !If_CutMergeOrdered( p, pCut0, pCut1, pCut ) )
            continue;
        if ( pObj->fSpec && pCut->nLeaves == (unsigned)p->pPars->nLutSize )
            continue;
        pThread->nCutsMerged++;
        pThread->nCutsTotal++;
        pThread->nCandidateCount++;
        if ( If_CutFilter( pCutSet, pCut, 0 ) )
            continue;
        pCut->fAndCut = 0;
        pCut->iCutFunc = -1;
        pCut->fCompl = 0;
        pCut->fUser = 0;
        pCut->fUseless = 0;
        pCut->Cost = 0;
        pCut->Delay = If_CutDelay( p, pObj, pCut );
        if ( pCut->Delay == -1 )
            continue;
        pCut->Area = If_CutAreaFlow( p, pCut );
        if ( p->pPars->fEdge )
            pCut->Edge = If_CutEdgeFlow( p, pCut );
        If_CutSort( p, pCutSet, pCut );
    }
    if ( pCutSet->nCuts == 0 )
        { If_ManParThreadSetError( pThread, "mode0_live_cutset", pObj ); return 0; }
    if ( !pRuntime->fPreprocess || pCutSet->ppCuts[0]->Delay <= pObj->Required + p->fEpsilon )
        If_CutCopy( p, If_ObjCutBest(pObj), pCutSet->ppCuts[0] );
    if ( !pObj->fSkipCut && If_ObjCutBest(pObj)->nLeaves > 1 )
    {
        assert( pCutSet->nCuts <= pCutSet->nCutsMax );
        If_ManSetupCutTriv( p, pCutSet->ppCuts[pCutSet->nCuts++], pObj->Id );
        assert( pCutSet->nCuts <= pCutSet->nCutsMax+1 );
    }
    if ( If_ObjCutBest(pObj)->fUseless )
        Abc_Print( 1, "The best cut is useless.  Please increase the number of cuts used by the mapper, for example: \"&if -C 32\"\n" );
    return 1;
}

static int If_ObjPerformMappingChoiceMode0Par( If_ParThread_t * pThread, If_Obj_t * pObj )
{
    If_ParRuntime_t * pRuntime = pThread->pRuntime;
    If_Man_t * p = pRuntime->pMan;
    If_Set_t * pCutSet = pObj->pCutSet;
    If_Obj_t * pTemp;
    If_Cut_t * pCutTemp, * pCut;
    int i, Limit;
    assert( pRuntime->Mode == 0 );
    assert( pObj->fRepr && pObj->pEquiv != NULL );
    if ( pCutSet == NULL || pCutSet->nCuts == 0 )
        { If_ManParThreadSetError( pThread, "mode0_choice_provider_cutset", pObj ); return 0; }
    /* Mode 0 only: provider cutsets stay read-only; skip provider trivial cuts by limit. */
    if ( pCutSet->nCuts > 1 )
        pCutSet->nCuts--;
    for ( pTemp = pObj->pEquiv; pTemp; pTemp = pTemp->pEquiv )
    {
        if ( pTemp->pCutSet == NULL )
            { If_ManParThreadSetError( pThread, "mode0_choice_provider_cutset", pTemp ); return 0; }
        Limit = pTemp->pCutSet->nCuts - 1;
        if ( Limit <= 0 )
            continue;
        for ( i = 0; i < Limit && (pCutTemp = pTemp->pCutSet->ppCuts[i]); i++ )
        {
            if ( pCutTemp->fUseless )
                continue;
            assert( pCutSet->nCuts <= pCutSet->nCutsMax );
            pCut = pCutSet->ppCuts[pCutSet->nCuts];
            If_CutCopy( p, pCut, pCutTemp );
            pThread->nCandidateCount++;
            if ( If_CutFilter( pCutSet, pCut, 0 ) )
                continue;
            pCut->fCompl = pObj->fPhase ^ pTemp->fPhase;
            pCut->Area = If_CutAreaFlow( p, pCut );
            if ( p->pPars->fEdge )
                pCut->Edge = If_CutEdgeFlow( p, pCut );
            If_CutSort( p, pCutSet, pCut );
        }
    }
    if ( pCutSet->nCuts == 0 )
        { If_ManParThreadSetError( pThread, "mode0_choice_provider_cutset", pObj ); return 0; }
    if ( !pRuntime->fPreprocess || pCutSet->ppCuts[0]->Delay <= pObj->Required + p->fEpsilon )
        If_CutCopy( p, If_ObjCutBest(pObj), pCutSet->ppCuts[0] );
    if ( !pObj->fSkipCut && If_ObjCutBest(pObj)->nLeaves > 1 )
    {
        assert( pCutSet->nCuts <= pCutSet->nCutsMax );
        If_ManSetupCutTriv( p, pCutSet->ppCuts[pCutSet->nCuts++], pObj->Id );
        assert( pCutSet->nCuts <= pCutSet->nCutsMax+1 );
    }
    pThread->nChoices++;
    return 1;
}

static void If_ManParMode1RecordNewCut( If_ParThread_t * pThread, If_Obj_t * pObj )
{
    If_ParRuntime_t * pRuntime = pThread->pRuntime;
    If_Man_t * p = pRuntime->pMan;
    If_Cut_t * pOld = If_ManParMode1OldCut( pRuntime, pObj->Id );
    If_Cut_t * pNew = If_ManParMode1NewCut( pRuntime, pObj->Id );
    If_CutCopy( p, pNew, If_ObjCutBest(pObj) );
    pRuntime->pMode1AllowFinalize[pObj->Id] = !p->pParMeta->pIsChoiceProvider[pObj->Id];
    pRuntime->pMode1Changed[pObj->Id] = (char)(pObj->nRefs > 0 && pRuntime->pMode1AllowFinalize[pObj->Id] && !If_ManParCutLeavesEqual( pOld, pNew ));
}

static int If_ObjPerformMappingAndMode1Par( If_ParThread_t * pThread, If_Obj_t * pObj )
{
    If_ParRuntime_t * pRuntime = pThread->pRuntime;
    If_Man_t * p = pRuntime->pMan;
    If_Set_t * pCutSet = pObj->pCutSet;
    If_Cut_t * pCut0, * pCut1, * pCut;
    If_Cut_t * pOld = If_ManParMode1OldCut( pRuntime, pObj->Id );
    int i, k;
    assert( pRuntime->Mode == 1 );
    assert( If_ObjIsAnd(pObj) );
    if ( pCutSet == NULL )
        { If_ManParThreadSetError( pThread, "mode1_live_cutset", pObj ); return 0; }
    if ( If_ObjIsAnd(pObj->pFanin0) && (pObj->pFanin0->pCutSet == NULL || pObj->pFanin0->pCutSet->nCuts == 0) )
        { If_ManParThreadSetError( pThread, "mode1_missing_fanin_cutset", pObj ); return 0; }
    if ( If_ObjIsAnd(pObj->pFanin1) && (pObj->pFanin1->pCutSet == NULL || pObj->pFanin1->pCutSet->nCuts == 0) )
        { If_ManParThreadSetError( pThread, "mode1_missing_fanin_cutset", pObj ); return 0; }

    If_CutCopy( p, pOld, If_ObjCutBest(pObj) );
    pObj->EstRefs = (float)((2.0 * pObj->EstRefs + pObj->nRefs) / 3.0);
    If_ManParMode1EvalClear( pThread );
    if ( pObj->nRefs > 0 && !If_ManParMode1DerefEvalCut( pThread, pOld ) )
        return 0;
    if ( !pRuntime->fFirst )
    {
        pCut = If_ObjCutBest(pObj);
        pCut->Delay = If_CutDelay( p, pObj, pCut );
        if ( pCut->Delay == -1 )
            { If_ManParThreadSetError( pThread, "mode1_nonstandard_flag", pObj ); return 0; }
        pCut->Area = If_CutAreaFlowMode1Par( pThread, pCut );
        if ( pThread->pError )
            return 0;
        if ( p->pPars->fEdge )
        {
            pCut->Edge = If_CutEdgeFlowMode1Par( pThread, pCut );
            if ( pThread->pError )
                return 0;
        }
        if ( !pRuntime->fPreprocess || pCut->nLeaves <= 1 )
        {
            assert( pCutSet->nCuts <= pCutSet->nCutsMax );
            If_CutCopy( p, pCutSet->ppCuts[pCutSet->nCuts++], pCut );
        }
    }

    If_ObjForEachCut( pObj->pFanin0, pCut0, i )
    If_ObjForEachCut( pObj->pFanin1, pCut1, k )
    {
        assert( pCutSet->nCuts <= pCutSet->nCutsMax );
        pCut = pCutSet->ppCuts[pCutSet->nCuts];
        if ( If_ParWordCountOnes(pCut0->uSign | pCut1->uSign) > p->pPars->nLutSize )
            continue;
        if ( !If_CutMergeOrdered( p, pCut0, pCut1, pCut ) )
            continue;
        if ( pObj->fSpec && pCut->nLeaves == (unsigned)p->pPars->nLutSize )
            continue;
        pThread->nCutsMerged++;
        pThread->nCutsTotal++;
        pThread->nCandidateCount++;
        if ( If_CutFilter( pCutSet, pCut, 0 ) )
            continue;
        pCut->fAndCut = 0;
        pCut->iCutFunc = -1;
        pCut->fCompl = 0;
        pCut->fUser = 0;
        pCut->fUseless = 0;
        pCut->Cost = 0;
        pCut->Delay = If_CutDelay( p, pObj, pCut );
        if ( pCut->Delay == -1 )
            continue;
        if ( pCut->Delay > pObj->Required + p->fEpsilon && pCutSet->nCuts > 0 )
            continue;
        pCut->Area = If_CutAreaFlowMode1Par( pThread, pCut );
        if ( pThread->pError )
            return 0;
        if ( p->pPars->fEdge )
        {
            pCut->Edge = If_CutEdgeFlowMode1Par( pThread, pCut );
            if ( pThread->pError )
                return 0;
        }
        If_CutSort( p, pCutSet, pCut );
    }
    if ( pCutSet->nCuts == 0 )
        { If_ManParThreadSetError( pThread, "mode1_live_cutset", pObj ); return 0; }
    if ( !pRuntime->fPreprocess || pCutSet->ppCuts[0]->Delay <= pObj->Required + p->fEpsilon )
        If_CutCopy( p, If_ObjCutBest(pObj), pCutSet->ppCuts[0] );
    if ( !pObj->fSkipCut && If_ObjCutBest(pObj)->nLeaves > 1 )
    {
        assert( pCutSet->nCuts <= pCutSet->nCutsMax );
        If_ManSetupCutTriv( p, pCutSet->ppCuts[pCutSet->nCuts++], pObj->Id );
        assert( pCutSet->nCuts <= pCutSet->nCutsMax+1 );
    }
    if ( If_ObjCutBest(pObj)->fUseless )
        { If_ManParThreadSetError( pThread, "mode1_useless_best_cut", pObj ); return 0; }
    If_ManParMode1RecordNewCut( pThread, pObj );
    If_ManParMode1EvalClear( pThread );
    return 1;
}

static int If_ObjPerformMappingChoiceMode1Par( If_ParThread_t * pThread, If_Obj_t * pObj )
{
    If_ParRuntime_t * pRuntime = pThread->pRuntime;
    If_Man_t * p = pRuntime->pMan;
    If_Set_t * pCutSet = pObj->pCutSet;
    If_Obj_t * pTemp;
    If_Cut_t * pCutTemp, * pCut;
    If_Cut_t * pOld = If_ManParMode1OldCut( pRuntime, pObj->Id );
    int i, Limit;
    assert( pRuntime->Mode == 1 );
    assert( pObj->fRepr && pObj->pEquiv != NULL );
    if ( pCutSet == NULL || pCutSet->nCuts == 0 )
        { If_ManParThreadSetError( pThread, "mode1_choice_provider_cutset", pObj ); return 0; }
    If_ManParMode1EvalClear( pThread );
    if ( pObj->nRefs > 0 && !If_ManParMode1DerefEvalCut( pThread, pOld ) )
        return 0;
    if ( pCutSet->nCuts > 1 )
        pCutSet->nCuts--;
    for ( pTemp = pObj->pEquiv; pTemp; pTemp = pTemp->pEquiv )
    {
        if ( pTemp->pCutSet == NULL )
            { If_ManParThreadSetError( pThread, "mode1_choice_provider_cutset", pTemp ); return 0; }
        Limit = pTemp->pCutSet->nCuts - 1;
        if ( Limit <= 0 )
            continue;
        for ( i = 0; i < Limit && (pCutTemp = pTemp->pCutSet->ppCuts[i]); i++ )
        {
            if ( pCutTemp->fUseless )
                continue;
            assert( pCutSet->nCuts <= pCutSet->nCutsMax );
            pCut = pCutSet->ppCuts[pCutSet->nCuts];
            If_CutCopy( p, pCut, pCutTemp );
            pThread->nCandidateCount++;
            pCut->fCompl = pObj->fPhase ^ pTemp->fPhase;
            if ( If_CutFilter( pCutSet, pCut, 0 ) )
                continue;
            if ( pCut->Delay > pObj->Required + p->fEpsilon && pCutSet->nCuts > 0 )
                continue;
            pCut->Area = If_CutAreaFlowMode1Par( pThread, pCut );
            if ( pThread->pError )
                return 0;
            if ( p->pPars->fEdge )
            {
                pCut->Edge = If_CutEdgeFlowMode1Par( pThread, pCut );
                if ( pThread->pError )
                    return 0;
            }
            If_CutSort( p, pCutSet, pCut );
        }
    }
    if ( pCutSet->nCuts == 0 )
        { If_ManParThreadSetError( pThread, "mode1_choice_provider_cutset", pObj ); return 0; }
    if ( !pRuntime->fPreprocess || pCutSet->ppCuts[0]->Delay <= pObj->Required + p->fEpsilon )
        If_CutCopy( p, If_ObjCutBest(pObj), pCutSet->ppCuts[0] );
    if ( !pObj->fSkipCut && If_ObjCutBest(pObj)->nLeaves > 1 )
    {
        assert( pCutSet->nCuts <= pCutSet->nCutsMax );
        If_ManSetupCutTriv( p, pCutSet->ppCuts[pCutSet->nCuts++], pObj->Id );
        assert( pCutSet->nCuts <= pCutSet->nCutsMax+1 );
    }
    If_ManParMode1RecordNewCut( pThread, pObj );
    If_ManParMode1EvalClear( pThread );
    pThread->nChoices++;
    return 1;
}

static int If_ObjPerformMappingAndMode2Par( If_ParThread_t * pThread, If_Obj_t * pObj )
{
    If_ParRuntime_t * pRuntime = pThread->pRuntime;
    If_Man_t * p = pRuntime->pMan;
    If_Set_t * pCutSet = pObj->pCutSet;
    If_Cut_t * pCut0, * pCut1, * pCut;
    If_Cut_t * pOld = If_ManParMode1OldCut( pRuntime, pObj->Id );
    int i, k;
    assert( pRuntime->Mode == 2 );
    assert( If_ObjIsAnd(pObj) );
    if ( pCutSet == NULL )
        { If_ManParThreadSetError( pThread, "mode2_live_cutset", pObj ); return 0; }
    if ( If_ObjIsAnd(pObj->pFanin0) && (pObj->pFanin0->pCutSet == NULL || pObj->pFanin0->pCutSet->nCuts == 0) )
        { If_ManParThreadSetError( pThread, "mode2_missing_fanin_cutset", pObj ); return 0; }
    if ( If_ObjIsAnd(pObj->pFanin1) && (pObj->pFanin1->pCutSet == NULL || pObj->pFanin1->pCutSet->nCuts == 0) )
        { If_ManParThreadSetError( pThread, "mode2_missing_fanin_cutset", pObj ); return 0; }

    If_CutCopy( p, pOld, If_ObjCutBest(pObj) );
    If_ManParMode1EvalClear( pThread );
    if ( !pRuntime->fFirst )
    {
        pCut = If_ObjCutBest(pObj);
        pCut->Delay = If_CutDelay( p, pObj, pCut );
        if ( pCut->Delay == -1 )
            { If_ManParThreadSetError( pThread, "mode2_nonstandard_flag", pObj ); return 0; }
        if ( pCut->Delay > pObj->Required + 2*p->fEpsilon )
            Abc_Print( 1, "If_ObjPerformMappingAndMode2Par(): Warning! Node with ID %d has delay (%f) exceeding the required times (%f).\n",
                pObj->Id, pCut->Delay, pObj->Required + p->fEpsilon );
        if ( !If_CutExactMetricMode2Par( pThread, pObj, pCut, pOld ) )
            return 0;
        if ( !pRuntime->fPreprocess || pCut->nLeaves <= 1 )
        {
            assert( pCutSet->nCuts <= pCutSet->nCutsMax );
            If_CutCopy( p, pCutSet->ppCuts[pCutSet->nCuts++], pCut );
        }
    }

    If_ObjForEachCut( pObj->pFanin0, pCut0, i )
    If_ObjForEachCut( pObj->pFanin1, pCut1, k )
    {
        assert( pCutSet->nCuts <= pCutSet->nCutsMax );
        pCut = pCutSet->ppCuts[pCutSet->nCuts];
        if ( If_ParWordCountOnes(pCut0->uSign | pCut1->uSign) > p->pPars->nLutSize )
            continue;
        if ( !If_CutMergeOrdered( p, pCut0, pCut1, pCut ) )
            continue;
        if ( pObj->fSpec && pCut->nLeaves == (unsigned)p->pPars->nLutSize )
            continue;
        pThread->nCutsMerged++;
        pThread->nCutsTotal++;
        pThread->nCandidateCount++;
        if ( If_CutFilter( pCutSet, pCut, 0 ) )
            continue;
        pCut->fAndCut = 0;
        pCut->iCutFunc = -1;
        pCut->fCompl = 0;
        pCut->fUser = 0;
        pCut->fUseless = 0;
        pCut->Cost = 0;
        pCut->Delay = If_CutDelay( p, pObj, pCut );
        if ( pCut->Delay == -1 )
            continue;
        if ( pCut->Delay > pObj->Required + p->fEpsilon && pCutSet->nCuts > 0 )
            continue;
        if ( !If_CutExactMetricMode2Par( pThread, pObj, pCut, pOld ) )
            return 0;
        If_CutSort( p, pCutSet, pCut );
    }
    if ( pCutSet->nCuts == 0 )
        { If_ManParThreadSetError( pThread, "mode2_live_cutset", pObj ); return 0; }
    if ( !pRuntime->fPreprocess || pCutSet->ppCuts[0]->Delay <= pObj->Required + p->fEpsilon )
        If_CutCopy( p, If_ObjCutBest(pObj), pCutSet->ppCuts[0] );
    if ( !pObj->fSkipCut && If_ObjCutBest(pObj)->nLeaves > 1 )
    {
        assert( pCutSet->nCuts <= pCutSet->nCutsMax );
        If_ManSetupCutTriv( p, pCutSet->ppCuts[pCutSet->nCuts++], pObj->Id );
        assert( pCutSet->nCuts <= pCutSet->nCutsMax+1 );
    }
    if ( If_ObjCutBest(pObj)->fUseless )
        { If_ManParThreadSetError( pThread, "mode2_useless_best_cut", pObj ); return 0; }
    If_ManParMode1RecordNewCut( pThread, pObj );
    If_ManParMode1EvalClear( pThread );
    return 1;
}

static int If_ObjPerformMappingChoiceMode2Par( If_ParThread_t * pThread, If_Obj_t * pObj )
{
    If_ParRuntime_t * pRuntime = pThread->pRuntime;
    If_Man_t * p = pRuntime->pMan;
    If_Set_t * pCutSet = pObj->pCutSet;
    If_Obj_t * pTemp;
    If_Cut_t * pCutTemp, * pCut;
    If_Cut_t * pOld = If_ManParMode1OldCut( pRuntime, pObj->Id );
    int i, Limit;
    assert( pRuntime->Mode == 2 );
    assert( pObj->fRepr && pObj->pEquiv != NULL );
    if ( pCutSet == NULL || pCutSet->nCuts == 0 )
        { If_ManParThreadSetError( pThread, "mode2_choice_provider_cutset", pObj ); return 0; }
    If_ManParMode1EvalClear( pThread );
    if ( pCutSet->nCuts > 1 )
        pCutSet->nCuts--;
    for ( pTemp = pObj->pEquiv; pTemp; pTemp = pTemp->pEquiv )
    {
        if ( pTemp->pCutSet == NULL )
            { If_ManParThreadSetError( pThread, "mode2_choice_provider_cutset", pTemp ); return 0; }
        Limit = pTemp->pCutSet->nCuts - 1;
        if ( Limit <= 0 )
            continue;
        for ( i = 0; i < Limit && (pCutTemp = pTemp->pCutSet->ppCuts[i]); i++ )
        {
            if ( pCutTemp->fUseless )
                continue;
            assert( pCutSet->nCuts <= pCutSet->nCutsMax );
            pCut = pCutSet->ppCuts[pCutSet->nCuts];
            If_CutCopy( p, pCut, pCutTemp );
            pThread->nCandidateCount++;
            pCut->fCompl = pObj->fPhase ^ pTemp->fPhase;
            if ( If_CutFilter( pCutSet, pCut, 0 ) )
                continue;
            if ( pCut->Delay > pObj->Required + p->fEpsilon && pCutSet->nCuts > 0 )
                continue;
            if ( !If_CutExactMetricMode2Par( pThread, pObj, pCut, pOld ) )
                return 0;
            If_CutSort( p, pCutSet, pCut );
        }
    }
    if ( pCutSet->nCuts == 0 )
        { If_ManParThreadSetError( pThread, "mode2_choice_provider_cutset", pObj ); return 0; }
    if ( !pRuntime->fPreprocess || pCutSet->ppCuts[0]->Delay <= pObj->Required + p->fEpsilon )
        If_CutCopy( p, If_ObjCutBest(pObj), pCutSet->ppCuts[0] );
    if ( !pObj->fSkipCut && If_ObjCutBest(pObj)->nLeaves > 1 )
    {
        assert( pCutSet->nCuts <= pCutSet->nCutsMax );
        If_ManSetupCutTriv( p, pCutSet->ppCuts[pCutSet->nCuts++], pObj->Id );
        assert( pCutSet->nCuts <= pCutSet->nCutsMax+1 );
    }
    If_ManParMode1RecordNewCut( pThread, pObj );
    If_ManParMode1EvalClear( pThread );
    pThread->nChoices++;
    return 1;
}

static char * If_ManParJobName( If_ParJobKind_t JobKind )
{
    switch ( JobKind )
    {
    case IF_PAR_JOB_SKELETON:     return "SKELETON";
    case IF_PAR_JOB_MODE0_AND:    return "MODE0_AND";
    case IF_PAR_JOB_MODE0_CHOICE: return "MODE0_CHOICE";
    case IF_PAR_JOB_MODE1_AND:    return "MODE1_AND";
    case IF_PAR_JOB_MODE1_CHOICE: return "MODE1_CHOICE";
    case IF_PAR_JOB_MODE2_AND:    return "MODE2_AND";
    case IF_PAR_JOB_MODE2_CHOICE: return "MODE2_CHOICE";
    case IF_PAR_JOB_CUTSET_ALLOC: return "CUTSET_ALLOC";
    case IF_PAR_JOB_CUTSET_RELEASE: return "CUTSET_RELEASE";
    case IF_PAR_JOB_REQUIRED_AND: return "REQUIRED_AND";
    case IF_PAR_JOB_REQUIRED_STAGE9_AND: return "REQUIRED_STAGE9_AND";
    case IF_PAR_JOB_REQUIRED_FLAT_SCAN: return "REQUIRED_FLAT_SCAN";
    case IF_PAR_JOB_REQUIRED_CI_TERMINAL: return "REQUIRED_CI_TERMINAL";
    case IF_PAR_JOB_REQUIRED_WRITEBACK: return "REQUIRED_WRITEBACK";
    case IF_PAR_JOB_IMPROVE_AND:  return "IMPROVE_AND";
    default:                      return "NONE";
    }
}

static int If_ManParJobMode( If_ParJobKind_t JobKind )
{
    switch ( JobKind )
    {
    case IF_PAR_JOB_MODE0_AND:
    case IF_PAR_JOB_MODE0_CHOICE:
        return 0;
    case IF_PAR_JOB_MODE1_AND:
    case IF_PAR_JOB_MODE1_CHOICE:
        return 1;
    case IF_PAR_JOB_MODE2_AND:
    case IF_PAR_JOB_MODE2_CHOICE:
        return 2;
    default:
        return -1;
    }
}

static int If_ManParJobIsChoice( If_ParJobKind_t JobKind )
{
    return JobKind == IF_PAR_JOB_MODE0_CHOICE || JobKind == IF_PAR_JOB_MODE1_CHOICE || JobKind == IF_PAR_JOB_MODE2_CHOICE;
}

static int If_ManParJobUsesAndTasks( If_ParJobKind_t JobKind )
{
    return JobKind == IF_PAR_JOB_SKELETON || JobKind == IF_PAR_JOB_MODE0_AND ||
           JobKind == IF_PAR_JOB_MODE1_AND || JobKind == IF_PAR_JOB_MODE2_AND ||
           JobKind == IF_PAR_JOB_REQUIRED_STAGE9_AND || JobKind == IF_PAR_JOB_IMPROVE_AND;
}

static int If_ManParJobIsStaticRange( If_ParJobKind_t JobKind )
{
    return JobKind == IF_PAR_JOB_CUTSET_ALLOC || JobKind == IF_PAR_JOB_CUTSET_RELEASE ||
           JobKind == IF_PAR_JOB_REQUIRED_STAGE9_AND || JobKind == IF_PAR_JOB_REQUIRED_FLAT_SCAN ||
           JobKind == IF_PAR_JOB_REQUIRED_CI_TERMINAL || JobKind == IF_PAR_JOB_REQUIRED_WRITEBACK;
}

static int If_ManParEffectiveChunkSize( int nTasks, int nThreads, int nNominalChunk )
{
    if ( nTasks <= 0 )
        return 0;
    if ( nThreads < 1 )
        nThreads = 1;
    if ( nTasks <= nThreads )
        return 1;
    if ( nTasks < nNominalChunk * nThreads )
        return Abc_MaxInt( 1, nTasks / nThreads );
    return nNominalChunk;
}

static int If_ManParSelectLevelTasks( If_ParRuntime_t * pRuntime, int Level, If_ParJobKind_t JobKind, int ** ppObjs )
{
    If_ParMeta_t * pMeta = pRuntime->pMan->pParMeta;
    int Start, Stop;
    if ( If_ManParJobIsChoice( JobKind ) )
    {
        Start = pMeta->pChoiceLevelStarts[Level];
        Stop  = pMeta->pChoiceLevelStarts[Level+1];
        *ppObjs = pMeta->pChoiceLevelObjs + Start;
        return Stop - Start;
    }
    Start = pMeta->pLevelStarts[Level];
    Stop  = pMeta->pLevelStarts[Level+1];
    *ppObjs = pMeta->pLevelObjs + Start;
    return Stop - Start;
}

static int If_ManParChoiceTaskCount( If_ParRuntime_t * pRuntime, int Level )
{
    If_ParMeta_t * pMeta = pRuntime->pMan->pParMeta;
    return pMeta->pChoiceLevelStarts[Level+1] - pMeta->pChoiceLevelStarts[Level];
}

static int If_ManParSelectStaticRangeTasks( If_ParRuntime_t * pRuntime, int Level, If_ParJobKind_t JobKind, int ** ppObjs )
{
    If_ParMeta_t * pMeta = pRuntime->pMan->pParMeta;
    int Start, Stop;
    if ( JobKind == IF_PAR_JOB_CUTSET_RELEASE )
    {
        Start = pMeta->pReleaseStarts[Level];
        Stop  = pMeta->pReleaseStarts[Level+1];
        *ppObjs = pMeta->pReleaseObjs + Start;
        return Stop - Start;
    }
    assert( JobKind == IF_PAR_JOB_CUTSET_ALLOC );
    Start = pMeta->pLevelStarts[Level];
    Stop  = pMeta->pLevelStarts[Level+1];
    *ppObjs = pMeta->pLevelObjs + Start;
    return Stop - Start;
}

static int If_ManParClaimChunk( If_ParThread_t * pThread, int * piStart, int * piStop )
{
    If_ParRuntime_t * pRuntime = pThread->pRuntime;
    abctime clk = Abc_Clock();
#ifdef ABC_USE_PTHREADS
    if ( pRuntime->nThreads > 1 )
        pthread_mutex_lock( &pRuntime->Mutex );
#endif
    if ( pRuntime->fJobStop || pRuntime->iJobNext >= pRuntime->nJobTasks )
    {
        pThread->nEmptyClaims++;
        pRuntime->nJobEmptyClaims++;
#ifdef ABC_USE_PTHREADS
        if ( pRuntime->nThreads > 1 )
            pthread_mutex_unlock( &pRuntime->Mutex );
#endif
        pThread->ClaimTime += Abc_Clock() - clk;
        return 0;
    }
    *piStart = pRuntime->iJobNext;
    *piStop  = Abc_MinInt( pRuntime->nJobTasks, pRuntime->iJobNext + pRuntime->nJobEffectiveChunk );
    pRuntime->iJobNext = *piStop;
    pRuntime->nJobClaimedChunks++;
    pThread->nClaimedChunks++;
#ifdef ABC_USE_PTHREADS
    if ( pRuntime->nThreads > 1 )
        pthread_mutex_unlock( &pRuntime->Mutex );
#endif
    pThread->ClaimTime += Abc_Clock() - clk;
    return 1;
}

static int If_ManParThreadRunObj( If_ParThread_t * pThread, If_Obj_t * pObj )
{
    If_ParRuntime_t * pRuntime = pThread->pRuntime;
    assert( If_ObjIsAnd(pObj) );
    if ( pRuntime->JobKind == IF_PAR_JOB_REQUIRED_AND )
    {
        if ( !If_ObjPropagateRequiredPar( pThread, pObj ) )
            return 0;
        pThread->nProcessedRoots++;
        return 1;
    }
    if ( pRuntime->JobKind == IF_PAR_JOB_REQUIRED_STAGE9_AND )
    {
        if ( !If_ObjPropagateRequiredStage9Par( pThread, pObj ) )
            return 0;
        pThread->nProcessedRoots++;
        return 1;
    }
    if ( pRuntime->JobKind == IF_PAR_JOB_IMPROVE_AND )
    {
        if ( !If_ObjImproveComputePar( pThread, pObj ) )
            return 0;
        pThread->nProcessedRoots++;
        return 1;
    }
    if ( If_ManParJobIsChoice( pRuntime->JobKind ) )
    {
        if ( !pObj->fRepr || pObj->pEquiv == NULL )
            { If_ManParThreadSetError( pThread, "choice_task_repr", pObj ); return 0; }
        if ( pRuntime->JobKind == IF_PAR_JOB_MODE0_CHOICE && !If_ObjPerformMappingChoiceMode0Par( pThread, pObj ) )
            return 0;
        if ( pRuntime->JobKind == IF_PAR_JOB_MODE1_CHOICE && !If_ObjPerformMappingChoiceMode1Par( pThread, pObj ) )
            return 0;
        if ( pRuntime->JobKind == IF_PAR_JOB_MODE2_CHOICE && !If_ObjPerformMappingChoiceMode2Par( pThread, pObj ) )
            return 0;
        pThread->nProcessedReprs++;
        return 1;
    }
    if ( pRuntime->JobKind == IF_PAR_JOB_MODE0_AND && !If_ObjPerformMappingAndMode0Par( pThread, pObj ) )
        return 0;
    if ( pRuntime->JobKind == IF_PAR_JOB_MODE1_AND && !If_ObjPerformMappingAndMode1Par( pThread, pObj ) )
        return 0;
    if ( pRuntime->JobKind == IF_PAR_JOB_MODE2_AND && !If_ObjPerformMappingAndMode2Par( pThread, pObj ) )
        return 0;
    if ( !If_ManParJobUsesAndTasks( pRuntime->JobKind ) )
        { If_ManParThreadSetError( pThread, "unknown_job_kind", pObj ); return 0; }
    If_ManParThreadAddNodeSummary( pThread, pObj );
    pThread->nProcessedRoots++;
    if ( pRuntime->JobKind == IF_PAR_JOB_SKELETON && pObj->fRepr && pObj->pEquiv )
        pThread->nChoices++;
    return 1;
}

static int If_ManParMode0AllocObjSlot( If_ParRuntime_t * pRuntime, int ObjId, char ** ppError );
static int If_ManParMode0ReleaseObjSlot( If_ParRuntime_t * pRuntime, int ObjId, char ** ppError );
static int If_ManParMode0AllocLevelSerial( If_ParRuntime_t * pRuntime, int Level );
static int If_ManParMode0ReleaseLevelSerial( If_ParRuntime_t * pRuntime, int Level );
static int If_ManParRequiredFlatScanObj( If_ParThread_t * pThread, int ObjId );
static int If_ManParRequiredCiTerminalObj( If_ParThread_t * pThread, int CiIndex );
static int If_ManParRequiredWriteBackObj( If_ParThread_t * pThread, int ObjId );

static int If_ManParThreadRunStaticTask( If_ParThread_t * pThread, int k, char ** ppError, If_Obj_t ** ppObj )
{
    If_ParRuntime_t * pRuntime = pThread->pRuntime;
    If_Man_t * p = pRuntime->pMan;
    If_Obj_t * pObj = NULL;
    int ObjId;
    if ( pRuntime->JobKind == IF_PAR_JOB_REQUIRED_STAGE9_AND )
    {
        ObjId = pRuntime->pJobObjs[k];
        pObj = ObjId >= 0 && ObjId < p->pParMeta->nObjs ? If_ManObj( p, ObjId ) : NULL;
        *ppObj = pObj;
        if ( pObj == NULL )
            { *ppError = "required_stage9_obj_range"; return 0; }
        return If_ManParThreadRunObj( pThread, pObj );
    }
    if ( pRuntime->JobKind == IF_PAR_JOB_REQUIRED_FLAT_SCAN )
    {
        *ppObj = k >= 0 && k < p->pParMeta->nObjs ? If_ManObj( p, k ) : NULL;
        return If_ManParRequiredFlatScanObj( pThread, k );
    }
    if ( pRuntime->JobKind == IF_PAR_JOB_REQUIRED_CI_TERMINAL )
    {
        *ppObj = k >= 0 && k < Vec_PtrSize(p->vCis) ? (If_Obj_t *)Vec_PtrEntry( p->vCis, k ) : NULL;
        return If_ManParRequiredCiTerminalObj( pThread, k );
    }
    if ( pRuntime->JobKind == IF_PAR_JOB_REQUIRED_WRITEBACK )
    {
        *ppObj = k >= 0 && k < p->pParMeta->nObjs ? If_ManObj( p, k ) : NULL;
        return If_ManParRequiredWriteBackObj( pThread, k );
    }
    ObjId = pRuntime->pJobObjs[k];
    pObj = ObjId >= 0 && ObjId < p->pParMeta->nObjs ? If_ManObj( p, ObjId ) : NULL;
    *ppObj = pObj;
    if ( pRuntime->JobKind == IF_PAR_JOB_CUTSET_ALLOC )
        return If_ManParMode0AllocObjSlot( pRuntime, ObjId, ppError );
    if ( pRuntime->JobKind == IF_PAR_JOB_CUTSET_RELEASE )
        return If_ManParMode0ReleaseObjSlot( pRuntime, ObjId, ppError );
    *ppError = "unknown_static_job";
    return 0;
}

static void If_ManParThreadWorkStaticRangeSpan( If_ParThread_t * pThread, int Start, int Stop )
{
    If_Obj_t * pObj = NULL;
    char * pError = NULL;
    int k;
    abctime clk = Abc_Clock();
    pThread->iStart = Start;
    pThread->iStop  = Stop;
    if ( Start < Stop )
        pThread->nClaimedChunks++;
    else
        pThread->nEmptyClaims++;
    for ( k = Start; k < Stop; k++ )
    {
        if ( If_ManParThreadRunStaticTask( pThread, k, &pError, &pObj ) )
            continue;
        If_ManParThreadSetError( pThread, pError ? pError : (char *)"static_job", pObj );
        break;
    }
    pThread->ActiveTime += Abc_Clock() - clk;
}

static void If_ManParThreadWorkStaticRange( If_ParThread_t * pThread )
{
    If_ParRuntime_t * pRuntime = pThread->pRuntime;
    long long nTasks = (long long)pRuntime->nJobTasks;
    long long nThreads = (long long)pRuntime->nThreads;
    int Start = (int)((nTasks * pThread->iThread) / nThreads);
    int Stop  = (int)((nTasks * (pThread->iThread + 1)) / nThreads);
    If_ManParThreadWorkStaticRangeSpan( pThread, Start, Stop );
}

static void If_ManParThreadWork( If_ParThread_t * pThread )
{
    If_ParRuntime_t * pRuntime = pThread->pRuntime;
    If_Man_t * p = pRuntime->pMan;
    If_Obj_t * pObj;
    int k, ObjId, Start, Stop;
    if ( If_ManParJobIsStaticRange( pRuntime->JobKind ) )
    {
        If_ManParThreadWorkStaticRange( pThread );
        return;
    }
    while ( If_ManParClaimChunk( pThread, &Start, &Stop ) )
    {
        abctime clk = Abc_Clock();
        pThread->iStart = Start;
        pThread->iStop  = Stop;
        for ( k = Start; k < Stop; k++ )
        {
            ObjId = pRuntime->pJobObjs[k];
            pObj = If_ManObj( p, ObjId );
            if ( !If_ManParThreadRunObj( pThread, pObj ) )
                break;
        }
        pThread->ActiveTime += Abc_Clock() - clk;
        if ( pThread->pError )
            return;
    }
}

static void If_ManParProfileInit( If_ParRuntime_t * pRuntime )
{
    char * pProfile = getenv( "IF_PAR_PROFILE" );
    char * pProfileOut = getenv( "IF_PAR_PROFILE_OUT" );
    if ( pProfile == NULL || pProfile[0] == 0 || !strcmp( pProfile, "0" ) )
        return;
    pRuntime->fProfileRequested = 1;
    if ( pProfileOut == NULL || pProfileOut[0] == 0 )
    {
        Abc_Print( 1, "IF_PAR_PROFILE_DISABLED reason=no_output_path\n" );
        return;
    }
    pRuntime->pProfileFile = fopen( pProfileOut, "w" );
    if ( pRuntime->pProfileFile == NULL )
    {
        Abc_Print( 1, "IF_PAR_PROFILE_DISABLED reason=open_failed path=%s\n", pProfileOut );
        return;
    }
    pRuntime->fProfileEnabled = 1;
    fprintf( pRuntime->pProfileFile,
        "row_type,mode,job,level,thread_id,threads,task_count,nominal_chunk_size,effective_chunk_size,chunk_count,claimed_chunks,empty_claims,processed_roots,processed_reprs,candidate_count,local_delta_touched_peak,active_time,wait_time,claim_time,job_wall_time\n" );
}

static void If_ManParProfileClose( If_ParRuntime_t * pRuntime )
{
    if ( pRuntime->pProfileFile == NULL )
        return;
    fclose( pRuntime->pProfileFile );
    pRuntime->pProfileFile = NULL;
    pRuntime->fProfileEnabled = 0;
}

static int If_ManParThreadTouchedPeak( If_ParThread_t * pThread, If_ParJobKind_t JobKind )
{
    if ( JobKind == IF_PAR_JOB_REQUIRED_AND )
        return pThread->nReqTouchedPeak;
    if ( JobKind == IF_PAR_JOB_REQUIRED_STAGE9_AND )
        return 0;
    if ( JobKind == IF_PAR_JOB_IMPROVE_AND )
        return Abc_MaxInt( pThread->nReqTouchedPeak, pThread->nMode1EvalTouchedPeak );
    return pThread->nMode1EvalTouchedPeak;
}

static void If_ManParProfileRecordJob( If_ParRuntime_t * pRuntime, If_ParJobKind_t JobKind, int Level,
    int nProcessedRoots, int nProcessedReprs, int nCandidateCount, int nTouchedPeak,
    abctime ActiveTime, abctime WaitTime, abctime ClaimTime, abctime JobTime )
{
    FILE * pFile = pRuntime->pProfileFile;
    int Mode = If_ManParJobMode( JobKind );
    int i;
    if ( pFile == NULL )
        return;
    if ( Mode < 0 )
        Mode = pRuntime->Mode;
    fprintf( pFile, "job,%d,%s,%d,-1,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%.6f,%.6f,%.6f,%.6f\n",
        Mode, If_ManParJobName(JobKind), Level, pRuntime->nThreads, pRuntime->nJobTasks,
        pRuntime->nJobNominalChunk, pRuntime->nJobEffectiveChunk, pRuntime->nJobChunks,
        pRuntime->nJobClaimedChunks, pRuntime->nJobEmptyClaims, nProcessedRoots, nProcessedReprs,
        nCandidateCount, nTouchedPeak, If_ManStage0TimeSec(ActiveTime),
        If_ManStage0TimeSec(WaitTime), If_ManStage0TimeSec(ClaimTime),
        If_ManStage0TimeSec(JobTime) );
    for ( i = 0; i < pRuntime->nThreads; i++ )
    {
        If_ParThread_t * pThread = pRuntime->pThreads + i;
        fprintf( pFile, "worker,%d,%s,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%.6f,%.6f,%.6f,%.6f\n",
            Mode, If_ManParJobName(JobKind), Level, pThread->iThread, pRuntime->nThreads,
            pRuntime->nJobTasks, pRuntime->nJobNominalChunk, pRuntime->nJobEffectiveChunk,
            pRuntime->nJobChunks, pThread->nClaimedChunks, pThread->nEmptyClaims,
            pThread->nProcessedRoots, pThread->nProcessedReprs, pThread->nCandidateCount,
            If_ManParThreadTouchedPeak( pThread, JobKind ), If_ManStage0TimeSec(pThread->ActiveTime),
            If_ManStage0TimeSec(pThread->WaitTime), If_ManStage0TimeSec(pThread->ClaimTime),
            If_ManStage0TimeSec(JobTime) );
    }
}

#ifdef ABC_USE_PTHREADS
static void * If_ManParWorkerThread( void * pArg )
{
    If_ParThread_t * pThread = (If_ParThread_t *)pArg;
    If_ParRuntime_t * pRuntime = pThread->pRuntime;
    pthread_mutex_lock( &pRuntime->Mutex );
    while ( 1 )
    {
        while ( pThread->iJob == pRuntime->iJob && !pRuntime->fStop )
            pthread_cond_wait( &pRuntime->CondStart, &pRuntime->Mutex );
        if ( pRuntime->fStop )
        {
            pthread_mutex_unlock( &pRuntime->Mutex );
            return NULL;
        }
        pThread->iJob = pRuntime->iJob;
        pthread_mutex_unlock( &pRuntime->Mutex );
        If_ManParThreadWork( pThread );
        pthread_mutex_lock( &pRuntime->Mutex );
        assert( pRuntime->nWorking > 0 );
        if ( --pRuntime->nWorking == 0 )
            pthread_cond_signal( &pRuntime->CondDone );
    }
}
#endif

static void If_ManParRuntimeStop( If_Man_t * p )
{
    If_ParRuntime_t * pRuntime = p ? p->pParRuntime : NULL;
    int i;
    if ( pRuntime == NULL )
        return;
#ifdef ABC_USE_PTHREADS
    if ( pRuntime->fStarted )
    {
        pthread_mutex_lock( &pRuntime->Mutex );
        pRuntime->fStop = 1;
        pRuntime->iJob++;
        pthread_cond_broadcast( &pRuntime->CondStart );
        pthread_mutex_unlock( &pRuntime->Mutex );
        for ( i = 0; i < pRuntime->nThreadsStarted; i++ )
            pthread_join( pRuntime->pThreads[i].Thread, NULL );
        pthread_cond_destroy( &pRuntime->CondDone );
        pthread_cond_destroy( &pRuntime->CondStart );
        pthread_mutex_destroy( &pRuntime->Mutex );
    }
#else
    (void)i;
#endif
    If_ManParProfileClose( pRuntime );
    ABC_FREE( pRuntime->pMode0CutSets );
    ABC_FREE( pRuntime->pMode1OldCuts );
    ABC_FREE( pRuntime->pMode1NewCuts );
    ABC_FREE( pRuntime->pMode1Changed );
    ABC_FREE( pRuntime->pMode1AllowFinalize );
    ABC_FREE( pRuntime->pMode1Delta );
    ABC_FREE( pRuntime->pMode1Touched );
    ABC_FREE( pRuntime->pMode1Wave );
    ABC_FREE( pRuntime->pMode1WaveDelta );
    ABC_FREE( pRuntime->pMode1DeltaMark );
    ABC_FREE( pRuntime->pMode1RefCheck );
    ABC_FREE( pRuntime->pReqLevelTouched );
    ABC_FREE( pRuntime->pReqLevelMark );
    ABC_FREE( pRuntime->pReqKeys );
    ABC_FREE( pRuntime->pImproveAccepted );
    if ( pRuntime->pThreads )
        for ( i = 0; i < pRuntime->nThreads; i++ )
        {
            ABC_FREE( pRuntime->pThreads[i].pMode1EvalDelta );
            ABC_FREE( pRuntime->pThreads[i].pMode1EvalTouched );
            ABC_FREE( pRuntime->pThreads[i].pMode1EvalMark );
            ABC_FREE( pRuntime->pThreads[i].pReqMin );
            ABC_FREE( pRuntime->pThreads[i].pReqTouched );
            ABC_FREE( pRuntime->pThreads[i].pReqMark );
        }
    ABC_FREE( pRuntime->pThreads );
    ABC_FREE( pRuntime );
    p->pParRuntime = NULL;
}

static int If_ManParRuntimeStart( If_Man_t * p, char ** ppReason )
{
    If_ParRuntime_t * pRuntime;
    int i;
    assert( p->pParMeta != NULL );
    if ( p->pPars->nParThreads < 1 || p->pPars->nParThreads > IF_PAR_THREAD_MAX )
        { If_ManParSetReason( ppReason, "runtime_thread_count" ); return 0; }
#ifndef ABC_USE_PTHREADS
    if ( p->pPars->nParThreads > 1 )
        { If_ManParSetReason( ppReason, "pthreads_disabled" ); return 0; }
#endif
    If_ManParRuntimeStop( p );
    pRuntime = ABC_CALLOC( If_ParRuntime_t, 1 );
    if ( pRuntime == NULL )
        { If_ManParSetReason( ppReason, "runtime_alloc" ); return 0; }
    pRuntime->pMan = p;
    pRuntime->nThreads = p->pPars->nParThreads;
    p->pParRuntime = pRuntime;
    If_ManParProfileInit( pRuntime );
    pRuntime->pThreads = ABC_CALLOC( If_ParThread_t, pRuntime->nThreads );
    if ( pRuntime->pThreads == NULL )
        { If_ManParSetReason( ppReason, "runtime_alloc" ); goto fail; }
    pRuntime->nMode0CutSets = p->pParMeta->nCutSetPeak;
    pRuntime->pMode0CutSets = (If_Set_t *)ABC_ALLOC( char, (size_t)pRuntime->nMode0CutSets * p->nSetBytes );
    if ( pRuntime->pMode0CutSets == NULL )
        { If_ManParSetReason( ppReason, "mode0_pool_alloc" ); goto fail; }
    pRuntime->pMode1OldCuts = (If_Cut_t *)ABC_ALLOC( char, (size_t)p->pParMeta->nObjs * p->nCutBytes );
    pRuntime->pMode1NewCuts = (If_Cut_t *)ABC_ALLOC( char, (size_t)p->pParMeta->nObjs * p->nCutBytes );
    pRuntime->pMode1Changed = ABC_CALLOC( char, p->pParMeta->nObjs );
    pRuntime->pMode1AllowFinalize = ABC_CALLOC( char, p->pParMeta->nObjs );
    pRuntime->pMode1Delta = ABC_CALLOC( int, p->pParMeta->nObjs );
    pRuntime->pMode1Touched = ABC_ALLOC( int, Abc_MaxInt(p->pParMeta->nObjs, 1) );
    pRuntime->pMode1Wave = ABC_ALLOC( int, Abc_MaxInt(p->pParMeta->nObjs, 1) );
    pRuntime->pMode1WaveDelta = ABC_ALLOC( int, Abc_MaxInt(p->pParMeta->nObjs, 1) );
    pRuntime->pMode1DeltaMark = ABC_CALLOC( char, p->pParMeta->nObjs );
    pRuntime->pMode1RefCheck = ABC_CALLOC( int, p->pParMeta->nObjs );
    pRuntime->pReqLevelTouched = ABC_ALLOC( int, Abc_MaxInt(p->pParMeta->nObjs, 1) );
    pRuntime->pReqLevelMark = ABC_CALLOC( char, p->pParMeta->nObjs );
    pRuntime->pReqKeys = ABC_ALLOC( uint32_t, Abc_MaxInt(p->pParMeta->nObjs, 1) );
    pRuntime->pImproveAccepted = ABC_CALLOC( char, p->pParMeta->nObjs );
    if ( pRuntime->pMode1OldCuts == NULL || pRuntime->pMode1NewCuts == NULL || pRuntime->pMode1Changed == NULL ||
         pRuntime->pMode1AllowFinalize == NULL || pRuntime->pMode1Delta == NULL || pRuntime->pMode1Touched == NULL ||
         pRuntime->pMode1Wave == NULL || pRuntime->pMode1WaveDelta == NULL || pRuntime->pMode1DeltaMark == NULL ||
         pRuntime->pMode1RefCheck == NULL || pRuntime->pReqLevelTouched == NULL || pRuntime->pReqLevelMark == NULL ||
         pRuntime->pReqKeys == NULL || pRuntime->pImproveAccepted == NULL )
        { If_ManParSetReason( ppReason, "runtime_buffer_alloc" ); goto fail; }
    If_ManParMode0PoolSetup( pRuntime );
    for ( i = 0; i < pRuntime->nThreads; i++ )
    {
        pRuntime->pThreads[i].pRuntime = pRuntime;
        pRuntime->pThreads[i].iThread = i;
        pRuntime->pThreads[i].pMode1EvalDelta = ABC_CALLOC( int, p->pParMeta->nObjs );
        pRuntime->pThreads[i].pMode1EvalTouched = ABC_ALLOC( int, Abc_MaxInt(p->pParMeta->nObjs, 1) );
        pRuntime->pThreads[i].pMode1EvalMark = ABC_CALLOC( char, p->pParMeta->nObjs );
        pRuntime->pThreads[i].pReqMin = ABC_ALLOC( float, Abc_MaxInt(p->pParMeta->nObjs, 1) );
        pRuntime->pThreads[i].pReqTouched = ABC_ALLOC( int, Abc_MaxInt(p->pParMeta->nObjs, 1) );
        pRuntime->pThreads[i].pReqMark = ABC_CALLOC( char, p->pParMeta->nObjs );
        if ( pRuntime->pThreads[i].pMode1EvalDelta == NULL || pRuntime->pThreads[i].pMode1EvalTouched == NULL || pRuntime->pThreads[i].pMode1EvalMark == NULL ||
             pRuntime->pThreads[i].pReqMin == NULL || pRuntime->pThreads[i].pReqTouched == NULL || pRuntime->pThreads[i].pReqMark == NULL )
            { If_ManParSetReason( ppReason, "runtime_thread_alloc" ); goto fail; }
    }
#ifdef ABC_USE_PTHREADS
    if ( pRuntime->nThreads > 1 )
    {
        if ( pthread_mutex_init( &pRuntime->Mutex, NULL ) )
            { If_ManParSetReason( ppReason, "pthread_mutex" ); goto fail; }
        if ( pthread_cond_init( &pRuntime->CondStart, NULL ) )
            { pthread_mutex_destroy( &pRuntime->Mutex ); If_ManParSetReason( ppReason, "pthread_cond" ); goto fail; }
        if ( pthread_cond_init( &pRuntime->CondDone, NULL ) )
            { pthread_cond_destroy( &pRuntime->CondStart ); pthread_mutex_destroy( &pRuntime->Mutex ); If_ManParSetReason( ppReason, "pthread_cond" ); goto fail; }
        pRuntime->fStarted = 1;
        for ( i = 0; i < pRuntime->nThreads; i++ )
        {
            if ( pthread_create( &pRuntime->pThreads[i].Thread, NULL, If_ManParWorkerThread, pRuntime->pThreads + i ) )
                { If_ManParSetReason( ppReason, "pthread_create" ); goto fail; }
            pRuntime->nThreadsStarted++;
        }
    }
#endif
    return 1;
fail:
    If_ManParRuntimeStop( p );
    return 0;
}

void If_ManParFree( If_Man_t * p )
{
    if ( p == NULL )
        return;
    If_ManParRuntimeStop( p );
    if ( p->pParMeta )
    {
        If_ManParFreeMeta( p->pParMeta );
        p->pParMeta = NULL;
    }
}

static void If_ManParUpdateFreeLevel( If_ParMeta_t * pMeta, If_Obj_t * pObj, int Level )
{
    if ( !If_ObjIsAnd(pObj) )
        return;
    if ( pMeta->pFreeLevel[pObj->Id] < Level )
        pMeta->pFreeLevel[pObj->Id] = Level;
    if ( pMeta->nMaxFreeLevel < pMeta->pFreeLevel[pObj->Id] )
        pMeta->nMaxFreeLevel = pMeta->pFreeLevel[pObj->Id];
}

static int If_ManParMetaBuildRelease( If_Man_t * p, If_ParMeta_t * pMeta, char ** ppReason )
{
    If_Obj_t * pObj;
    int * pReleaseCursor = NULL;
    int * pLiveDelta = NULL;
    int i, Level, FreeLevel, Entry, Live;
    pMeta->pReleaseStarts = ABC_CALLOC( int, pMeta->nLevels + 1 );
    pMeta->pReleaseObjs   = ABC_ALLOC( int, Abc_MaxInt(pMeta->nAnds, 1) );
    pLiveDelta            = ABC_CALLOC( int, pMeta->nLevels + 1 );
    if ( pMeta->pReleaseStarts == NULL || pMeta->pReleaseObjs == NULL || pLiveDelta == NULL )
        { If_ManParSetReason( ppReason, "metadata_alloc" ); goto fail; }
    If_ManForEachNode( p, pObj, i )
    {
        Level = If_ObjLevel(pObj);
        FreeLevel = pMeta->pFreeLevel[pObj->Id];
        if ( Level < 0 || Level >= pMeta->nLevels || FreeLevel < Level || FreeLevel >= pMeta->nLevels )
        {
            If_ManParSetReason( ppReason, "release_level" );
            goto fail;
        }
        pMeta->pReleaseStarts[FreeLevel+1]++;
        pLiveDelta[Level]++;
        pLiveDelta[FreeLevel+1]--;
    }
    for ( i = 0; i < pMeta->nLevels; i++ )
        pMeta->pReleaseStarts[i+1] += pMeta->pReleaseStarts[i];
    pReleaseCursor = ABC_ALLOC( int, pMeta->nLevels );
    if ( pReleaseCursor == NULL )
        { If_ManParSetReason( ppReason, "metadata_alloc" ); goto fail; }
    memcpy( pReleaseCursor, pMeta->pReleaseStarts, sizeof(int) * pMeta->nLevels );
    If_ManForEachNode( p, pObj, i )
    {
        FreeLevel = pMeta->pFreeLevel[pObj->Id];
        Entry = pReleaseCursor[FreeLevel]++;
        pMeta->pReleaseObjs[Entry] = pObj->Id;
    }
    Live = 0;
    for ( i = 0; i < pMeta->nLevels; i++ )
    {
        Live += pLiveDelta[i];
        if ( pMeta->nCutSetPeak < Live )
            pMeta->nCutSetPeak = Live;
    }
    if ( pMeta->nCutSetPeak < 1 )
        pMeta->nCutSetPeak = 1;
    ABC_FREE( pReleaseCursor );
    ABC_FREE( pLiveDelta );
    return 1;
fail:
    ABC_FREE( pReleaseCursor );
    ABC_FREE( pLiveDelta );
    return 0;
}

static int If_ManParMetaBuildCutSetSlots( If_Man_t * p, If_ParMeta_t * pMeta, char ** ppReason )
{
    int * pFreeSlots = NULL;
    int i, Level, k, ObjId, Slot, nFree;
    abctime clk = Abc_Clock();
    (void)p;
    assert( pMeta->pReleaseStarts != NULL );
    assert( pMeta->pReleaseObjs != NULL );
    assert( pMeta->nCutSetPeak > 0 );
    pMeta->pCutSetSlot = ABC_ALLOC( int, Abc_MaxInt(pMeta->nObjs, 1) );
    pFreeSlots = ABC_ALLOC( int, pMeta->nCutSetPeak );
    if ( pMeta->pCutSetSlot == NULL || pFreeSlots == NULL )
        { If_ManParSetReason( ppReason, "metadata_alloc" ); goto fail; }
    for ( i = 0; i < pMeta->nObjs; i++ )
        pMeta->pCutSetSlot[i] = -1;
    for ( i = 0; i < pMeta->nCutSetPeak; i++ )
        pFreeSlots[i] = pMeta->nCutSetPeak - 1 - i;
    nFree = pMeta->nCutSetPeak;
    for ( Level = 0; Level < pMeta->nLevels; Level++ )
    {
        for ( k = pMeta->pLevelStarts[Level]; k < pMeta->pLevelStarts[Level+1]; k++ )
        {
            ObjId = pMeta->pLevelObjs[k];
            if ( ObjId < 0 || ObjId >= pMeta->nObjs )
                { If_ManParSetReason( ppReason, "static_slot_range" ); goto fail; }
            if ( pMeta->pCutSetSlot[ObjId] != -1 )
                { If_ManParSetReason( ppReason, "static_slot_overlap" ); goto fail; }
            if ( nFree == 0 )
                { If_ManParSetReason( ppReason, "static_slot_underflow" ); goto fail; }
            pMeta->pCutSetSlot[ObjId] = pFreeSlots[--nFree];
        }
        for ( k = pMeta->pReleaseStarts[Level]; k < pMeta->pReleaseStarts[Level+1]; k++ )
        {
            ObjId = pMeta->pReleaseObjs[k];
            if ( ObjId < 0 || ObjId >= pMeta->nObjs )
                { If_ManParSetReason( ppReason, "static_slot_range" ); goto fail; }
            Slot = pMeta->pCutSetSlot[ObjId];
            if ( Slot < 0 )
                { If_ManParSetReason( ppReason, "static_slot_unassigned_release" ); goto fail; }
            if ( Slot >= pMeta->nCutSetPeak || nFree >= pMeta->nCutSetPeak )
                { If_ManParSetReason( ppReason, "static_slot_range" ); goto fail; }
            pFreeSlots[nFree++] = Slot;
        }
    }
    if ( nFree != pMeta->nCutSetPeak )
        { If_ManParSetReason( ppReason, "static_slot_unreleased" ); goto fail; }
    pMeta->SlotAssignTime = Abc_Clock() - clk;
    ABC_FREE( pFreeSlots );
    return 1;
fail:
    pMeta->SlotAssignTime = Abc_Clock() - clk;
    ABC_FREE( pFreeSlots );
    return 0;
}

static int If_ManParMetaValidateCutSetSlots( If_Man_t * p, If_ParMeta_t * pMeta, char ** ppReason )
{
    char * pSlotLive = NULL;
    char * pObjLive = NULL;
    If_Obj_t * pObj;
    int i, Level, k, ObjId, Slot, nLive = 0;
    abctime clk = Abc_Clock();
    pSlotLive = ABC_CALLOC( char, pMeta->nCutSetPeak );
    pObjLive  = ABC_CALLOC( char, Abc_MaxInt(pMeta->nObjs, 1) );
    if ( pSlotLive == NULL || pObjLive == NULL )
        { If_ManParSetReason( ppReason, "metadata_alloc" ); goto fail; }
    If_ManForEachObj( p, pObj, i )
        if ( !If_ObjIsAnd(pObj) && pMeta->pCutSetSlot[pObj->Id] != -1 )
            { If_ManParSetReason( ppReason, "static_slot_range" ); goto fail; }
    for ( Level = 0; Level < pMeta->nLevels; Level++ )
    {
        for ( k = pMeta->pLevelStarts[Level]; k < pMeta->pLevelStarts[Level+1]; k++ )
        {
            ObjId = pMeta->pLevelObjs[k];
            if ( ObjId < 0 || ObjId >= pMeta->nObjs )
                { If_ManParSetReason( ppReason, "static_slot_range" ); goto fail; }
            Slot = pMeta->pCutSetSlot[ObjId];
            if ( Slot < 0 || Slot >= pMeta->nCutSetPeak )
                { If_ManParSetReason( ppReason, "static_slot_range" ); goto fail; }
            if ( pObjLive[ObjId] || pSlotLive[Slot] )
                { If_ManParSetReason( ppReason, "static_slot_overlap" ); goto fail; }
            pObjLive[ObjId] = 1;
            pSlotLive[Slot] = 1;
            nLive++;
        }
        for ( k = pMeta->pReleaseStarts[Level]; k < pMeta->pReleaseStarts[Level+1]; k++ )
        {
            ObjId = pMeta->pReleaseObjs[k];
            if ( ObjId < 0 || ObjId >= pMeta->nObjs )
                { If_ManParSetReason( ppReason, "static_slot_range" ); goto fail; }
            Slot = pMeta->pCutSetSlot[ObjId];
            if ( Slot < 0 || Slot >= pMeta->nCutSetPeak )
                { If_ManParSetReason( ppReason, "static_slot_range" ); goto fail; }
            if ( !pObjLive[ObjId] || !pSlotLive[Slot] )
                { If_ManParSetReason( ppReason, "static_slot_double_release" ); goto fail; }
            pObjLive[ObjId] = 0;
            pSlotLive[Slot] = 0;
            nLive--;
        }
    }
    if ( nLive != 0 )
        { If_ManParSetReason( ppReason, "static_slot_unreleased" ); goto fail; }
    pMeta->SlotValidateTime = Abc_Clock() - clk;
    ABC_FREE( pSlotLive );
    ABC_FREE( pObjLive );
    return 1;
fail:
    pMeta->SlotValidateTime = Abc_Clock() - clk;
    ABC_FREE( pSlotLive );
    ABC_FREE( pObjLive );
    return 0;
}

static If_ParMeta_t * If_ManParMetaAlloc( If_Man_t * p, char ** ppReason )
{
    If_ParMeta_t * pMeta = NULL;
    If_Obj_t * pObj, * pFanin, * pTemp;
    int * pLevelCursor = NULL;
    int i, Level, Entry, iChoice, iProvider, nProviders;
    pMeta = ABC_CALLOC( If_ParMeta_t, 1 );
    if ( pMeta == NULL )
        { If_ManParSetReason( ppReason, "metadata_alloc" ); return NULL; }
    pMeta->nLevels = p->nLevelMax + 1;
    pMeta->nObjs   = If_ManObjNum(p);
    pMeta->nAnds   = If_ManAndNum(p);
    pMeta->pLevelCounts = ABC_CALLOC( int, pMeta->nLevels );
    pMeta->pLevelStarts = ABC_CALLOC( int, pMeta->nLevels + 1 );
    pMeta->pLevelObjs   = ABC_ALLOC( int, Abc_MaxInt(pMeta->nAnds, 1) );
    pMeta->pFreeLevel   = ABC_ALLOC( int, Abc_MaxInt(pMeta->nObjs, 1) );
    pMeta->pChoiceLevelCounts = ABC_CALLOC( int, pMeta->nLevels );
    pMeta->pIsChoiceProvider = ABC_CALLOC( char, Abc_MaxInt(pMeta->nObjs, 1) );
    if ( pMeta->pLevelCounts == NULL || pMeta->pLevelStarts == NULL || pMeta->pLevelObjs == NULL ||
         pMeta->pFreeLevel == NULL || pMeta->pChoiceLevelCounts == NULL || pMeta->pIsChoiceProvider == NULL )
        { If_ManParSetReason( ppReason, "metadata_alloc" ); goto fail; }
    for ( i = 0; i < pMeta->nObjs; i++ )
        pMeta->pFreeLevel[i] = -1;
    If_ManForEachNode( p, pObj, i )
    {
        Level = If_ObjLevel(pObj);
        if ( Level < 0 || Level >= pMeta->nLevels )
        {
            If_ManParSetReason( ppReason, "level_range" );
            goto fail;
        }
        pMeta->pLevelCounts[Level]++;
        pMeta->pFreeLevel[pObj->Id] = Level;
        if ( pObj->fRepr && pObj->pEquiv )
        {
            pMeta->nChoices++;
            pMeta->pChoiceLevelCounts[Level]++;
            for ( pTemp = pObj->pEquiv; pTemp; pTemp = pTemp->pEquiv )
                pMeta->nProviders++;
        }
    }
    for ( i = 0; i < pMeta->nLevels; i++ )
        pMeta->pLevelStarts[i+1] = pMeta->pLevelStarts[i] + pMeta->pLevelCounts[i];
    pLevelCursor = ABC_ALLOC( int, pMeta->nLevels );
    if ( pLevelCursor == NULL )
        { If_ManParSetReason( ppReason, "metadata_alloc" ); goto fail; }
    memcpy( pLevelCursor, pMeta->pLevelStarts, sizeof(int) * pMeta->nLevels );
    If_ManForEachNode( p, pObj, i )
    {
        Level = If_ObjLevel(pObj);
        Entry = pLevelCursor[Level]++;
        pMeta->pLevelObjs[Entry] = pObj->Id;
        If_ManParUpdateFreeLevel( pMeta, pObj, Level );
    }
    ABC_FREE( pLevelCursor );
    pLevelCursor = NULL;
    If_ManForEachNode( p, pObj, i )
    {
        pFanin = If_ObjFanin0(pObj);
        If_ManParUpdateFreeLevel( pMeta, pFanin, If_ObjLevel(pObj) );
        pFanin = If_ObjFanin1(pObj);
        If_ManParUpdateFreeLevel( pMeta, pFanin, If_ObjLevel(pObj) );
    }
    pMeta->pChoiceLevelStarts = ABC_CALLOC( int, pMeta->nLevels + 1 );
    pMeta->pChoiceLevelObjs   = ABC_ALLOC( int, Abc_MaxInt(pMeta->nChoices, 1) );
    pMeta->pChoiceReprs       = ABC_ALLOC( int, Abc_MaxInt(pMeta->nChoices, 1) );
    pMeta->pProviderStarts    = ABC_ALLOC( int, Abc_MaxInt(pMeta->nChoices + 1, 1) );
    pMeta->pProviders         = ABC_ALLOC( int, Abc_MaxInt(pMeta->nProviders, 1) );
    if ( pMeta->pChoiceLevelStarts == NULL || pMeta->pChoiceLevelObjs == NULL ||
         pMeta->pChoiceReprs == NULL || pMeta->pProviderStarts == NULL || pMeta->pProviders == NULL )
        { If_ManParSetReason( ppReason, "metadata_alloc" ); goto fail; }
    for ( i = 0; i < pMeta->nLevels; i++ )
        pMeta->pChoiceLevelStarts[i+1] = pMeta->pChoiceLevelStarts[i] + pMeta->pChoiceLevelCounts[i];
    pLevelCursor = ABC_ALLOC( int, pMeta->nLevels );
    if ( pLevelCursor == NULL )
        { If_ManParSetReason( ppReason, "metadata_alloc" ); goto fail; }
    memcpy( pLevelCursor, pMeta->pChoiceLevelStarts, sizeof(int) * pMeta->nLevels );
    iChoice = iProvider = 0;
    If_ManForEachNode( p, pObj, i )
    {
        if ( !pObj->fRepr || pObj->pEquiv == NULL )
            continue;
        Level = If_ObjLevel(pObj);
        Entry = pLevelCursor[Level]++;
        pMeta->pChoiceLevelObjs[Entry] = pObj->Id;
        pMeta->pChoiceReprs[iChoice] = pObj->Id;
        pMeta->pProviderStarts[iChoice] = iProvider;
        nProviders = 0;
        for ( pTemp = pObj->pEquiv; pTemp; pTemp = pTemp->pEquiv )
        {
            pMeta->pProviders[iProvider++] = pTemp->Id;
            pMeta->pIsChoiceProvider[pTemp->Id] = 1;
            nProviders++;
            If_ManParUpdateFreeLevel( pMeta, pTemp, If_ObjLevel(pObj) );
        }
        if ( nProviders == 0 )
        {
            If_ManParSetReason( ppReason, "choice_provider_empty" );
            goto fail;
        }
        iChoice++;
    }
    ABC_FREE( pLevelCursor );
    pLevelCursor = NULL;
    pMeta->pProviderStarts[iChoice] = iProvider;
    if ( iChoice != pMeta->nChoices )
        { If_ManParSetReason( ppReason, "choice_count_mismatch" ); goto fail; }
    if ( iProvider != pMeta->nProviders )
        { If_ManParSetReason( ppReason, "provider_count_mismatch" ); goto fail; }
    if ( !If_ManParMetaBuildRelease( p, pMeta, ppReason ) )
        goto fail;
    if ( !If_ManParMetaBuildCutSetSlots( p, pMeta, ppReason ) )
        goto fail;
    if ( !If_ManParMetaValidateCutSetSlots( p, pMeta, ppReason ) )
        goto fail;
    return pMeta;
fail:
    ABC_FREE( pLevelCursor );
    If_ManParFreeMeta( pMeta );
    return NULL;
}

static int If_ManParMetaCheckChoiceLevels( If_Man_t * p, char ** ppReason )
{
    If_ParMeta_t * pMeta = p->pParMeta;
    If_Obj_t * pObj;
    char * pSeen = NULL;
    int i, k, Start, Stop, ObjId, PrevId, Count = 0;
    if ( pMeta->pChoiceLevelStarts[0] != 0 || pMeta->pChoiceLevelStarts[pMeta->nLevels] != pMeta->nChoices )
        { If_ManParSetReason( ppReason, "choice_level_count" ); return 0; }
    pSeen = ABC_CALLOC( char, Abc_MaxInt(pMeta->nObjs, 1) );
    if ( pSeen == NULL )
        { If_ManParSetReason( ppReason, "choice_level_check_alloc" ); return 0; }
    for ( i = 0; i < pMeta->nLevels; i++ )
    {
        Start = pMeta->pChoiceLevelStarts[i];
        Stop  = pMeta->pChoiceLevelStarts[i+1];
        if ( Stop < Start || Stop - Start != pMeta->pChoiceLevelCounts[i] )
            { If_ManParSetReason( ppReason, "choice_level_count" ); goto fail; }
        PrevId = -1;
        for ( k = Start; k < Stop; k++ )
        {
            ObjId = pMeta->pChoiceLevelObjs[k];
            if ( ObjId <= PrevId || ObjId < 0 || ObjId >= pMeta->nObjs || pSeen[ObjId] )
                { If_ManParSetReason( ppReason, "choice_level_order" ); goto fail; }
            pObj = If_ManObj( p, ObjId );
            if ( !If_ObjIsAnd(pObj) || !pObj->fRepr || pObj->pEquiv == NULL || If_ObjLevel(pObj) != i )
                { If_ManParSetReason( ppReason, "choice_level_repr" ); goto fail; }
            pSeen[ObjId] = 1;
            PrevId = ObjId;
            Count++;
        }
    }
    if ( Count != pMeta->nChoices )
        { If_ManParSetReason( ppReason, "choice_level_count" ); goto fail; }
    for ( i = 0; i < pMeta->nChoices; i++ )
    {
        ObjId = pMeta->pChoiceReprs[i];
        if ( ObjId < 0 || ObjId >= pMeta->nObjs || !pSeen[ObjId] )
            { If_ManParSetReason( ppReason, "choice_level_repr" ); goto fail; }
    }
    ABC_FREE( pSeen );
    return 1;
fail:
    ABC_FREE( pSeen );
    return 0;
}

static int If_ManParMetaCheck( If_Man_t * p, char ** ppReason )
{
    If_ParMeta_t * pMeta = p->pParMeta;
    If_Obj_t * pObj, * pFanin, * pTemp;
    int i, k, Start, Stop, ObjId, PrevId;
    If_ManForEachNode( p, pObj, i )
    {
        pFanin = If_ObjFanin0(pObj);
        if ( If_ObjIsAnd(pFanin) && If_ObjLevel(pFanin) >= If_ObjLevel(pObj) )
            { If_ManParSetReason( ppReason, "fanin0_level" ); return 0; }
        pFanin = If_ObjFanin1(pObj);
        if ( If_ObjIsAnd(pFanin) && If_ObjLevel(pFanin) >= If_ObjLevel(pObj) )
            { If_ManParSetReason( ppReason, "fanin1_level" ); return 0; }
    }
    for ( i = 0; i < pMeta->nLevels; i++ )
    {
        Start = pMeta->pLevelStarts[i];
        Stop  = pMeta->pLevelStarts[i+1];
        PrevId = -1;
        for ( k = Start; k < Stop; k++ )
        {
            ObjId = pMeta->pLevelObjs[k];
            if ( ObjId <= PrevId )
                { If_ManParSetReason( ppReason, "level_order" ); return 0; }
            PrevId = ObjId;
        }
    }
    if ( !If_ManParMetaCheckChoiceLevels( p, ppReason ) )
        return 0;
    If_ManForEachNode( p, pObj, i )
    {
        if ( !pObj->fRepr )
            continue;
        for ( pTemp = pObj->pEquiv; pTemp; pTemp = pTemp->pEquiv )
        {
            if ( pTemp->fRepr )
                { If_ManParSetReason( ppReason, "multiple_choice_representatives" ); return 0; }
            if ( If_ObjLevel(pTemp) > If_ObjLevel(pObj) )
                { If_ManParSetReason( ppReason, "choice_provider_level" ); return 0; }
            if ( pMeta->pFreeLevel[pTemp->Id] < If_ObjLevel(pObj) )
                { If_ManParSetReason( ppReason, "choice_provider_lifetime" ); return 0; }
        }
    }
    If_ManParSetReason( ppReason, "ok" );
    return 1;
}

static void If_ManParPrintMeta( If_Man_t * p )
{
    If_ParMeta_t * pMeta = p->pParMeta;
    if ( !p->pPars->fVerbose || pMeta == NULL )
        return;
    Abc_Print( 1, "IF_PAR_SUPPORTED levels=%d ands=%d choices=%d providers=%d max_free_level=%d cutsets=%d threads=%d\n",
        pMeta->nLevels, pMeta->nAnds, pMeta->nChoices, pMeta->nProviders, pMeta->nMaxFreeLevel, pMeta->nCutSetPeak, p->pPars->nParThreads );
    Abc_Print( 1, "IF_PAR_SLOT_SUMMARY static_slot=1 slot_count=%d slot_assign_time=%.6f slot_validate_time=%.6f\n",
        pMeta->nCutSetPeak, If_ManStage0TimeSec(pMeta->SlotAssignTime), If_ManStage0TimeSec(pMeta->SlotValidateTime) );
}

int If_ManParPrecheck( If_Man_t * p )
{
    char * pReason = NULL;
    assert( p->pPars->fParMap );
    If_ManParFree( p );
    if ( !If_ManParIsSupported( p, &pReason ) )
    {
        Abc_Print( 1, "IF_PAR_UNSUPPORTED reason=%s\n", pReason ? pReason : "unknown" );
        return 1;
    }
    p->pParMeta = If_ManParMetaAlloc( p, &pReason );
    if ( p->pParMeta == NULL )
    {
        Abc_Print( -1, "IF_PAR_INVARIANT reason=%s\n", pReason ? pReason : "metadata_alloc" );
        return 0;
    }
    if ( !If_ManParMetaCheck( p, &pReason ) )
    {
        Abc_Print( -1, "IF_PAR_INVARIANT reason=%s\n", pReason ? pReason : "metadata_check" );
        return 0;
    }
    if ( !If_ManParRuntimeStart( p, &pReason ) )
    {
        Abc_Print( -1, "IF_PAR_INVARIANT reason=%s\n", pReason ? pReason : "runtime_start" );
        If_ManParFree( p );
        return 0;
    }
    If_ManParPrintMeta( p );
    return 1;
}

int If_ManParPostcheck( If_Man_t * p )
{
    If_Obj_t * pObj, * pTemp;
    int i;
    if ( p->pParMeta == NULL )
        return 1;
    If_ManForEachNode( p, pObj, i )
    {
        if ( !pObj->fRepr )
            continue;
        for ( pTemp = pObj->pEquiv; pTemp; pTemp = pTemp->pEquiv )
        {
            if ( pTemp->nRefs != 0 )
            {
                Abc_Print( -1, "IF_PAR_INVARIANT reason=choice_provider_mapping_ref repr=%d provider=%d refs=%d\n", pObj->Id, pTemp->Id, pTemp->nRefs );
                return 0;
            }
        }
    }
    if ( p->pPars->fVerbose )
        Abc_Print( 1, "IF_PAR_POSTCHECK choices=%d providers=%d status=ok\n", p->pParMeta->nChoices, p->pParMeta->nProviders );
    return 1;
}

static int If_ManParRunLevelJob( If_ParRuntime_t * pRuntime, int Level, If_ParJobKind_t JobKind )
{
    If_Man_t * p = pRuntime->pMan;
    int * pJobObjs = NULL;
    int nTasks = If_ManParSelectLevelTasks( pRuntime, Level, JobKind, &pJobObjs );
    int nProcessedRoots = 0, nProcessedReprs = 0, nCandidateCount = 0, nTouchedPeak = 0;
    abctime clkJob, JobTime, ActiveTime = 0, ClaimTime = 0, WaitTime = 0;
    int i, Mode;
    if ( nTasks == 0 )
        return 1;
    pRuntime->iLevel = Level;
    pRuntime->JobKind = JobKind;
    pRuntime->pJobObjs = pJobObjs;
    pRuntime->nJobTasks = nTasks;
    pRuntime->iJobNext = 0;
    pRuntime->nJobNominalChunk = IF_PAR_STAGE55_NOMINAL_CHUNK;
    pRuntime->nJobEffectiveChunk = If_ManParEffectiveChunkSize( nTasks, pRuntime->nThreads, pRuntime->nJobNominalChunk );
    pRuntime->nJobChunks = (nTasks + pRuntime->nJobEffectiveChunk - 1) / pRuntime->nJobEffectiveChunk;
    pRuntime->nJobClaimedChunks = 0;
    pRuntime->nJobEmptyClaims = 0;
    pRuntime->fJobStop = 0;
    for ( i = 0; i < pRuntime->nThreads; i++ )
    {
        If_ParThread_t * pThread = pRuntime->pThreads + i;
        pThread->iStart = 0;
        pThread->iStop  = 0;
        If_ManParThreadReset( pThread );
    }
    clkJob = Abc_Clock();
#ifdef ABC_USE_PTHREADS
    if ( pRuntime->nThreads > 1 )
    {
        pthread_mutex_lock( &pRuntime->Mutex );
        pRuntime->nWorking = pRuntime->nThreads;
        pRuntime->iJob++;
        pthread_cond_broadcast( &pRuntime->CondStart );
        while ( pRuntime->nWorking > 0 )
            pthread_cond_wait( &pRuntime->CondDone, &pRuntime->Mutex );
        pthread_mutex_unlock( &pRuntime->Mutex );
    }
    else
#endif
    {
        for ( i = 0; i < pRuntime->nThreads; i++ )
            If_ManParThreadWork( pRuntime->pThreads + i );
    }
    JobTime = Abc_Clock() - clkJob;
    for ( i = 0; i < pRuntime->nThreads; i++ )
    {
        If_ParThread_t * pThread = pRuntime->pThreads + i;
        if ( pThread->pError )
        {
            Abc_Print( -1, "IF_PAR_INVARIANT reason=%s obj=%d\n", pThread->pError, pThread->ErrorObj );
            return 0;
        }
    }
    for ( i = 0; i < pRuntime->nThreads; i++ )
    {
        If_ParThread_t * pThread = pRuntime->pThreads + i;
        abctime BusyTime = pThread->ActiveTime + pThread->ClaimTime;
        pThread->WaitTime = JobTime > BusyTime ? JobTime - BusyTime : 0;
        nProcessedRoots += pThread->nProcessedRoots;
        nProcessedReprs += pThread->nProcessedReprs;
        nCandidateCount += pThread->nCandidateCount;
        nTouchedPeak = Abc_MaxInt( nTouchedPeak, If_ManParThreadTouchedPeak( pThread, JobKind ) );
        ActiveTime += pThread->ActiveTime;
        ClaimTime += pThread->ClaimTime;
        WaitTime += pThread->WaitTime;
    }
    pRuntime->RoundActiveTime += ActiveTime;
    pRuntime->RoundClaimTime += ClaimTime;
    pRuntime->RoundWaitTime += WaitTime;
    pRuntime->nRoundClaimedChunks += pRuntime->nJobClaimedChunks;
    pRuntime->nRoundEmptyClaims += pRuntime->nJobEmptyClaims;
    if ( If_ManParJobUsesAndTasks( JobKind ) )
    {
        pRuntime->nRoundLevels++;
        for ( i = 0; i < pRuntime->nThreads; i++ )
        {
            If_ParThread_t * pThread = pRuntime->pThreads + i;
            pRuntime->nRoundObjs += pThread->nObjs;
            pRuntime->RoundChecksum += pThread->Checksum;
            if ( JobKind == IF_PAR_JOB_SKELETON )
                pRuntime->nRoundChoices += pThread->nChoices;
            else if ( JobKind == IF_PAR_JOB_IMPROVE_AND )
            {
                pRuntime->nImproveRoots += pThread->nImproveRoots;
                pRuntime->nImproveCandidates += pThread->nImproveCandidates;
                pRuntime->nImproveAccepted += pThread->nImproveAccepted;
                pRuntime->nImproveRejectRule += pThread->nImproveRejectRule;
                pRuntime->nImproveRejectDelay += pThread->nImproveRejectDelay;
            }
            else
            {
                p->nCutsMerged += pThread->nCutsMerged;
                p->nCutsTotal  += pThread->nCutsTotal;
            }
        }
        pRuntime->nRoundAndTasks += nTasks;
        pRuntime->nRoundAndChunks += pRuntime->nJobChunks;
    }
    else if ( If_ManParJobIsChoice( JobKind ) )
    {
        for ( i = 0; i < pRuntime->nThreads; i++ )
            pRuntime->nRoundChoices += pRuntime->pThreads[i].nChoices;
        pRuntime->nRoundChoiceTasks += nTasks;
        pRuntime->nRoundChoiceChunks += pRuntime->nJobChunks;
    }
    if ( p->pPars->fVerbose )
    {
        Mode = If_ManParJobMode( JobKind );
        if ( Mode < 0 )
            Mode = pRuntime->Mode;
        Abc_Print( 1, "IF_PAR_SCHED_SUMMARY mode=%d job=%s level=%d threads=%d task_count=%d nominal_chunk_size=%d effective_chunk_size=%d chunk_count=%d claimed_chunks=%d empty_claims=%d processed_roots=%d processed_reprs=%d candidate_count=%d local_delta_touched_peak=%d active_time=%.6f wait_time=%.6f claim_time=%.6f job_wall_time=%.6f\n",
            Mode, If_ManParJobName(JobKind), Level, pRuntime->nThreads, pRuntime->nJobTasks,
            pRuntime->nJobNominalChunk, pRuntime->nJobEffectiveChunk, pRuntime->nJobChunks,
            pRuntime->nJobClaimedChunks, pRuntime->nJobEmptyClaims, nProcessedRoots, nProcessedReprs,
            nCandidateCount, nTouchedPeak, If_ManStage0TimeSec(ActiveTime), If_ManStage0TimeSec(WaitTime),
            If_ManStage0TimeSec(ClaimTime), If_ManStage0TimeSec(JobTime) );
    }
    If_ManParProfileRecordJob( pRuntime, JobKind, Level, nProcessedRoots, nProcessedReprs,
        nCandidateCount, nTouchedPeak, ActiveTime, WaitTime, ClaimTime, JobTime );
    return 1;
}

static int If_ManParRunLevelStaticJob( If_ParRuntime_t * pRuntime, int Level, If_ParJobKind_t JobKind )
{
    int * pJobObjs = NULL;
    int nTasks = If_ManParSelectStaticRangeTasks( pRuntime, Level, JobKind, &pJobObjs );
    int i;
    assert( JobKind == IF_PAR_JOB_CUTSET_ALLOC || JobKind == IF_PAR_JOB_CUTSET_RELEASE );
    if ( nTasks == 0 )
        return 1;
    if ( pRuntime->nThreads <= 1 || nTasks < pRuntime->nThreads * IF_PAR_STATIC_MIN_TASKS_PER_THREAD )
    {
        if ( JobKind == IF_PAR_JOB_CUTSET_ALLOC )
            return If_ManParMode0AllocLevelSerial( pRuntime, Level );
        return If_ManParMode0ReleaseLevelSerial( pRuntime, Level );
    }
    pRuntime->iLevel = Level;
    pRuntime->JobKind = JobKind;
    pRuntime->pJobObjs = pJobObjs;
    pRuntime->nJobTasks = nTasks;
    pRuntime->iJobNext = 0;
    pRuntime->nJobNominalChunk = 0;
    pRuntime->nJobEffectiveChunk = 0;
    pRuntime->nJobChunks = pRuntime->nThreads;
    pRuntime->nJobClaimedChunks = 0;
    pRuntime->nJobEmptyClaims = 0;
    pRuntime->fJobStop = 0;
    for ( i = 0; i < pRuntime->nThreads; i++ )
    {
        If_ParThread_t * pThread = pRuntime->pThreads + i;
        pThread->iStart = 0;
        pThread->iStop  = 0;
        If_ManParThreadReset( pThread );
    }
#ifdef ABC_USE_PTHREADS
    if ( pRuntime->nThreads > 1 )
    {
        pthread_mutex_lock( &pRuntime->Mutex );
        pRuntime->nWorking = pRuntime->nThreads;
        pRuntime->iJob++;
        pthread_cond_broadcast( &pRuntime->CondStart );
        while ( pRuntime->nWorking > 0 )
            pthread_cond_wait( &pRuntime->CondDone, &pRuntime->Mutex );
        pthread_mutex_unlock( &pRuntime->Mutex );
    }
    else
#endif
    {
        for ( i = 0; i < pRuntime->nThreads; i++ )
            If_ManParThreadWork( pRuntime->pThreads + i );
    }
    for ( i = 0; i < pRuntime->nThreads; i++ )
    {
        If_ParThread_t * pThread = pRuntime->pThreads + i;
        if ( pThread->pError )
        {
            Abc_Print( -1, "IF_PAR_INVARIANT reason=%s obj=%d\n", pThread->pError, pThread->ErrorObj );
            return 0;
        }
    }
    return 1;
}

static int If_ManParJobIsRequiredStatic( If_ParJobKind_t JobKind )
{
    return JobKind == IF_PAR_JOB_REQUIRED_STAGE9_AND || JobKind == IF_PAR_JOB_REQUIRED_FLAT_SCAN ||
           JobKind == IF_PAR_JOB_REQUIRED_CI_TERMINAL || JobKind == IF_PAR_JOB_REQUIRED_WRITEBACK;
}

static int If_ManParRunRequiredStaticRangeJob( If_ParRuntime_t * pRuntime, int Level, If_ParJobKind_t JobKind, int * pJobObjs, int nTasks, int MinTasksPerThread )
{
    If_Man_t * p = pRuntime->pMan;
    int nProcessedRoots = 0, nProcessedReprs = 0, nCandidateCount = 0, nTouchedPeak = 0;
    abctime clkJob, JobTime, ActiveTime = 0, ClaimTime = 0, WaitTime = 0;
    int fParallel, i;
    assert( If_ManParJobIsRequiredStatic(JobKind) );
    if ( nTasks == 0 )
        return 1;
    fParallel = pRuntime->nThreads > 1 && nTasks >= pRuntime->nThreads * MinTasksPerThread;
    pRuntime->iLevel = Level;
    pRuntime->JobKind = JobKind;
    pRuntime->pJobObjs = pJobObjs;
    pRuntime->nJobTasks = nTasks;
    pRuntime->iJobNext = 0;
    pRuntime->nJobNominalChunk = 0;
    pRuntime->nJobEffectiveChunk = 0;
    pRuntime->nJobChunks = fParallel ? pRuntime->nThreads : 1;
    pRuntime->nJobClaimedChunks = 0;
    pRuntime->nJobEmptyClaims = 0;
    pRuntime->fJobStop = 0;
    for ( i = 0; i < pRuntime->nThreads; i++ )
    {
        If_ParThread_t * pThread = pRuntime->pThreads + i;
        pThread->iStart = 0;
        pThread->iStop  = 0;
        If_ManParThreadReset( pThread );
    }
    clkJob = Abc_Clock();
#ifdef ABC_USE_PTHREADS
    if ( fParallel )
    {
        pthread_mutex_lock( &pRuntime->Mutex );
        pRuntime->nWorking = pRuntime->nThreads;
        pRuntime->iJob++;
        pthread_cond_broadcast( &pRuntime->CondStart );
        while ( pRuntime->nWorking > 0 )
            pthread_cond_wait( &pRuntime->CondDone, &pRuntime->Mutex );
        pthread_mutex_unlock( &pRuntime->Mutex );
    }
    else
#endif
        If_ManParThreadWorkStaticRangeSpan( pRuntime->pThreads, 0, nTasks );
    JobTime = Abc_Clock() - clkJob;
    for ( i = 0; i < pRuntime->nThreads; i++ )
    {
        If_ParThread_t * pThread = pRuntime->pThreads + i;
        if ( pThread->pError )
        {
            Abc_Print( -1, "IF_PAR_INVARIANT reason=%s obj=%d\n", pThread->pError, pThread->ErrorObj );
            return 0;
        }
    }
    for ( i = 0; i < pRuntime->nThreads; i++ )
    {
        If_ParThread_t * pThread = pRuntime->pThreads + i;
        abctime BusyTime = pThread->ActiveTime + pThread->ClaimTime;
        pThread->WaitTime = fParallel && JobTime > BusyTime ? JobTime - BusyTime : 0;
        nProcessedRoots += pThread->nProcessedRoots;
        nProcessedReprs += pThread->nProcessedReprs;
        nCandidateCount += pThread->nCandidateCount;
        nTouchedPeak = Abc_MaxInt( nTouchedPeak, If_ManParThreadTouchedPeak( pThread, JobKind ) );
        ActiveTime += pThread->ActiveTime;
        ClaimTime += pThread->ClaimTime;
        WaitTime += pThread->WaitTime;
        pRuntime->nJobClaimedChunks += pThread->nClaimedChunks;
        pRuntime->nJobEmptyClaims += pThread->nEmptyClaims;
    }
    if ( p->pPars->fVerbose )
    {
        if ( JobKind == IF_PAR_JOB_REQUIRED_STAGE9_AND )
            Abc_Print( 1, "IF_PAR_REQUIRED_LEVEL mode=%d level=%d threads=%d task_count=%d nominal_chunk_size=%d effective_chunk_size=%d chunk_count=%d claimed_chunks=%d empty_claims=%d processed_roots=%d candidate_count=%d local_req_touched_peak=%d active_time=%.6f wait_time=%.6f claim_time=%.6f job_wall_time=%.6f\n",
                pRuntime->Mode, Level, pRuntime->nThreads, pRuntime->nJobTasks, pRuntime->nJobNominalChunk,
                pRuntime->nJobEffectiveChunk, pRuntime->nJobChunks, pRuntime->nJobClaimedChunks,
                pRuntime->nJobEmptyClaims, nProcessedRoots, nCandidateCount, nTouchedPeak,
                If_ManStage0TimeSec(ActiveTime), If_ManStage0TimeSec(WaitTime),
                If_ManStage0TimeSec(ClaimTime), If_ManStage0TimeSec(JobTime) );
        else
            Abc_Print( 1, "IF_PAR_SCHED_SUMMARY mode=%d job=%s level=%d threads=%d task_count=%d nominal_chunk_size=%d effective_chunk_size=%d chunk_count=%d claimed_chunks=%d empty_claims=%d processed_roots=%d processed_reprs=%d candidate_count=%d local_delta_touched_peak=%d active_time=%.6f wait_time=%.6f claim_time=%.6f job_wall_time=%.6f\n",
                pRuntime->Mode, If_ManParJobName(JobKind), Level, pRuntime->nThreads, pRuntime->nJobTasks,
                pRuntime->nJobNominalChunk, pRuntime->nJobEffectiveChunk, pRuntime->nJobChunks,
                pRuntime->nJobClaimedChunks, pRuntime->nJobEmptyClaims, nProcessedRoots, nProcessedReprs,
                nCandidateCount, nTouchedPeak, If_ManStage0TimeSec(ActiveTime), If_ManStage0TimeSec(WaitTime),
                If_ManStage0TimeSec(ClaimTime), If_ManStage0TimeSec(JobTime) );
    }
    If_ManParProfileRecordJob( pRuntime, JobKind, Level, nProcessedRoots, nProcessedReprs,
        nCandidateCount, nTouchedPeak, ActiveTime, WaitTime, ClaimTime, JobTime );
    return 1;
}

static void If_ManParRoundResetStats( If_ParRuntime_t * pRuntime )
{
    pRuntime->nRoundLevels = 0;
    pRuntime->nRoundObjs = 0;
    pRuntime->nRoundChoices = 0;
    pRuntime->RoundTime = 0;
    pRuntime->RoundFinalizeTime = 0;
    pRuntime->RoundAllocTime = 0;
    pRuntime->RoundReleaseTime = 0;
    pRuntime->RoundRefCheckTime = 0;
    pRuntime->RoundActiveTime = 0;
    pRuntime->RoundClaimTime = 0;
    pRuntime->RoundWaitTime = 0;
    pRuntime->nRoundAndTasks = 0;
    pRuntime->nRoundChoiceTasks = 0;
    pRuntime->nRoundAndChunks = 0;
    pRuntime->nRoundChoiceChunks = 0;
    pRuntime->nRoundClaimedChunks = 0;
    pRuntime->nRoundEmptyClaims = 0;
    pRuntime->nRoundAllocCount = 0;
    pRuntime->nRoundReleaseCount = 0;
    pRuntime->nRoundChoiceSkippedLevels = 0;
}

static int If_ManParRunRoundSkeleton( If_Man_t * p, int Mode, int fPreprocess, int fFirst, char * pLabel )
{
    If_ParRuntime_t * pRuntime = p->pParRuntime;
    int Level;
    abctime clk;
    if ( pRuntime == NULL || p->pParMeta == NULL )
        return 1;
    assert( p->pPars->fParMap );
    pRuntime->Mode = Mode;
    pRuntime->fPreprocess = fPreprocess;
    pRuntime->fFirst = fFirst;
    If_ManParRoundResetStats( pRuntime );
    pRuntime->nRoundLevels = 0;
    pRuntime->nRoundObjs = 0;
    pRuntime->nRoundChoices = 0;
    pRuntime->RoundChecksum = If_ManParHash( 2166136261u, (unsigned)Mode );
    pRuntime->RoundChecksum = If_ManParHash( pRuntime->RoundChecksum, (unsigned)fPreprocess );
    pRuntime->RoundChecksum = If_ManParHash( pRuntime->RoundChecksum, (unsigned)fFirst );
    clk = Abc_Clock();
    for ( Level = 0; Level < p->pParMeta->nLevels; Level++ )
        if ( !If_ManParRunLevelJob( pRuntime, Level, IF_PAR_JOB_SKELETON ) )
            return 0;
    pRuntime->RoundTime = Abc_Clock() - clk;
    if ( p->pPars->fVerbose )
        Abc_Print( 1, "IF_PAR_ROUND label=%s mode=%d preprocess=%d first=%d threads=%d levels=%d tasks=%d choices=%d checksum=%u skeleton_time=%.6f finalize_time=0.000000 release_time=0.000000 active_time=%.6f wait_time=%.6f claim_time=%.6f and_tasks=%d choice_tasks=%d and_chunks=%d choice_chunks=%d claimed_chunks=%d empty_claims=%d alloc_time=0.000000 alloc_count=0 release_count=0 refcheck_time=0.000000 choice_skipped_levels=0 round_validate=0\n",
            pLabel ? pLabel : "none", Mode, fPreprocess, fFirst, pRuntime->nThreads, pRuntime->nRoundLevels,
            pRuntime->nRoundObjs, pRuntime->nRoundChoices, pRuntime->RoundChecksum, If_ManStage0TimeSec(pRuntime->RoundTime),
            If_ManStage0TimeSec(pRuntime->RoundActiveTime), If_ManStage0TimeSec(pRuntime->RoundWaitTime),
            If_ManStage0TimeSec(pRuntime->RoundClaimTime), pRuntime->nRoundAndTasks, pRuntime->nRoundChoiceTasks,
            pRuntime->nRoundAndChunks, pRuntime->nRoundChoiceChunks, pRuntime->nRoundClaimedChunks, pRuntime->nRoundEmptyClaims );
    return 1;
}

static int If_ManParMode0CheckNoLiveCutsets( If_Man_t * p )
{
    If_Obj_t * pObj;
    int i;
    If_ManForEachNode( p, pObj, i )
        if ( pObj->pCutSet != NULL )
        {
            Abc_Print( -1, "IF_PAR_INVARIANT reason=mode0_live_cutset obj=%d\n", pObj->Id );
            return 0;
        }
    return 1;
}

static int If_ManParMode0AllocObjSlot( If_ParRuntime_t * pRuntime, int ObjId, char ** ppError )
{
    If_Man_t * p = pRuntime->pMan;
    If_ParMeta_t * pMeta = p->pParMeta;
    If_Obj_t * pObj;
    If_Set_t * pSet;
    int Slot;
    if ( ObjId < 0 || ObjId >= pMeta->nObjs )
        { If_ManParSetReason( ppError, "static_slot_range" ); return 0; }
    pObj = If_ManObj( p, ObjId );
    if ( !If_ObjIsAnd(pObj) )
        { If_ManParSetReason( ppError, "cutset_alloc_obj" ); return 0; }
    if ( pObj->pCutSet != NULL )
        { If_ManParSetReason( ppError, "mode0_live_cutset" ); return 0; }
    Slot = pMeta->pCutSetSlot[ObjId];
    if ( Slot < 0 || Slot >= pRuntime->nMode0CutSets )
        { If_ManParSetReason( ppError, "static_slot_range" ); return 0; }
    pSet = If_ManParMode0CutSet( pRuntime, Slot );
    if ( pSet == NULL || pSet->ppCuts == NULL )
        { If_ManParSetReason( ppError, "mode0_pool_setup" ); return 0; }
    pSet->nCuts = 0;
    pSet->nCutsMax = p->pPars->nCutsMax;
    pObj->pCutSet = pSet;
    return 1;
}

static int If_ManParMode0ReleaseObjSlot( If_ParRuntime_t * pRuntime, int ObjId, char ** ppError )
{
    If_Man_t * p = pRuntime->pMan;
    If_ParMeta_t * pMeta = p->pParMeta;
    If_Obj_t * pObj;
    If_Set_t * pSet;
    int Slot;
    if ( ObjId < 0 || ObjId >= pMeta->nObjs )
        { If_ManParSetReason( ppError, "static_slot_range" ); return 0; }
    pObj = If_ManObj( p, ObjId );
    if ( !If_ObjIsAnd(pObj) )
        { If_ManParSetReason( ppError, "cutset_release_obj" ); return 0; }
    if ( pObj->pCutSet == NULL )
        { If_ManParSetReason( ppError, "mode0_late_release" ); return 0; }
    Slot = pMeta->pCutSetSlot[ObjId];
    if ( Slot < 0 || Slot >= pRuntime->nMode0CutSets )
        { If_ManParSetReason( ppError, "static_slot_range" ); return 0; }
    pSet = If_ManParMode0CutSet( pRuntime, Slot );
    if ( pObj->pCutSet != pSet )
        { If_ManParSetReason( ppError, "static_slot_mismatch" ); return 0; }
    pSet->nCuts = 0;
    pObj->pCutSet = NULL;
    pObj->nVisits = 0;
    return 1;
}

static int If_ManParMode0CheckRoundCounts( If_ParRuntime_t * pRuntime, int Mode )
{
    int nExpected = pRuntime->pMan->pParMeta->nAnds;
    if ( pRuntime->nRoundAllocCount == nExpected && pRuntime->nRoundReleaseCount == nExpected )
        return 1;
    Abc_Print( -1, "IF_PAR_INVARIANT reason=mode%d_cutset_count alloc=%d release=%d expected=%d\n",
        Mode, pRuntime->nRoundAllocCount, pRuntime->nRoundReleaseCount, nExpected );
    return 0;
}

static int If_ManParRoundValidationEnabled()
{
    char * pEnv = getenv( "IF_PAR_ROUND_VALIDATE" );
    return pEnv != NULL && pEnv[0] != 0 && strcmp( pEnv, "0" );
}

static int If_ManParMode0AllocLevelSerial( If_ParRuntime_t * pRuntime, int Level )
{
    If_Man_t * p = pRuntime->pMan;
    If_ParMeta_t * pMeta = p->pParMeta;
    char * pError = NULL;
    int k, ObjId;
    for ( k = pMeta->pLevelStarts[Level]; k < pMeta->pLevelStarts[Level+1]; k++ )
    {
        ObjId = pMeta->pLevelObjs[k];
        if ( !If_ManParMode0AllocObjSlot( pRuntime, ObjId, &pError ) )
        {
            Abc_Print( -1, "IF_PAR_INVARIANT reason=%s obj=%d\n", pError ? pError : "cutset_alloc", ObjId );
            return 0;
        }
    }
    return 1;
}

static int If_ManParMode0ReleaseLevelSerial( If_ParRuntime_t * pRuntime, int Level )
{
    If_Man_t * p = pRuntime->pMan;
    If_ParMeta_t * pMeta = p->pParMeta;
    char * pError = NULL;
    int k, ObjId;
    for ( k = pMeta->pReleaseStarts[Level]; k < pMeta->pReleaseStarts[Level+1]; k++ )
    {
        ObjId = pMeta->pReleaseObjs[k];
        if ( !If_ManParMode0ReleaseObjSlot( pRuntime, ObjId, &pError ) )
        {
            Abc_Print( -1, "IF_PAR_INVARIANT reason=%s obj=%d\n", pError ? pError : "cutset_release", ObjId );
            return 0;
        }
    }
    return 1;
}

static int If_ManParCompareInt( const void * p0, const void * p1 )
{
    int i0 = *(int *)p0;
    int i1 = *(int *)p1;
    return (i0 > i1) - (i0 < i1);
}

static int If_ManParMode1FinalizeAddDelta( If_ParRuntime_t * pRuntime, int ObjId, int Delta )
{
    assert( ObjId >= 0 && ObjId < pRuntime->pMan->pParMeta->nObjs );
    if ( Delta == 0 )
        return 1;
    if ( !pRuntime->pMode1DeltaMark[ObjId] )
    {
        pRuntime->pMode1DeltaMark[ObjId] = 1;
        pRuntime->pMode1Touched[pRuntime->nMode1Touched++] = ObjId;
    }
    pRuntime->pMode1Delta[ObjId] += Delta;
    return 1;
}

static int If_ManParMode1FinalizeCutDelta( If_ParRuntime_t * pRuntime, If_Cut_t * pCut, int Delta )
{
    int i;
    for ( i = 0; i < (int)pCut->nLeaves; i++ )
        if ( !If_ManParMode1FinalizeAddDelta( pRuntime, pCut->pLeaves[i], Delta ) )
            return 0;
    return 1;
}

static void If_ManParMode1FinalizeClear( If_ParRuntime_t * pRuntime )
{
    int i, ObjId;
    for ( i = 0; i < pRuntime->nMode1Touched; i++ )
    {
        ObjId = pRuntime->pMode1Touched[i];
        pRuntime->pMode1Delta[ObjId] = 0;
        pRuntime->pMode1DeltaMark[ObjId] = 0;
    }
    pRuntime->nMode1Touched = 0;
}

static int If_ManParFinalizeMode1Level( If_ParRuntime_t * pRuntime, int Level )
{
    If_Man_t * p = pRuntime->pMan;
    If_ParMeta_t * pMeta = p->pParMeta;
    If_Obj_t * pObj;
    If_Cut_t * pOld, * pNew;
    int k, i, ObjId, Delta, OldRefs, NewRefs, nThis;
    assert( pRuntime->nMode1Touched == 0 );
    for ( k = pMeta->pLevelStarts[Level]; k < pMeta->pLevelStarts[Level+1]; k++ )
    {
        ObjId = pMeta->pLevelObjs[k];
        pObj = If_ManObj( p, ObjId );
        if ( !pRuntime->pMode1Changed[ObjId] )
            continue;
        if ( !pRuntime->pMode1AllowFinalize[ObjId] || pMeta->pIsChoiceProvider[ObjId] )
        {
            Abc_Print( -1, "IF_PAR_INVARIANT reason=mode1_finalize_provider_root obj=%d\n", ObjId );
            If_ManParMode1FinalizeClear( pRuntime );
            return 0;
        }
        if ( pObj->nRefs <= 0 )
            continue;
        pOld = If_ManParMode1OldCut( pRuntime, ObjId );
        pNew = If_ManParMode1NewCut( pRuntime, ObjId );
        if ( !If_ManParMode1FinalizeCutDelta( pRuntime, pOld, -1 ) || !If_ManParMode1FinalizeCutDelta( pRuntime, pNew, 1 ) )
            { If_ManParMode1FinalizeClear( pRuntime ); return 0; }
    }
    while ( pRuntime->nMode1Touched > 0 )
    {
        qsort( pRuntime->pMode1Touched, (size_t)pRuntime->nMode1Touched, sizeof(int), If_ManParCompareInt );
        nThis = pRuntime->nMode1Touched;
        memcpy( pRuntime->pMode1Wave, pRuntime->pMode1Touched, sizeof(int) * nThis );
        for ( i = 0; i < nThis; i++ )
        {
            ObjId = pRuntime->pMode1Wave[i];
            pRuntime->pMode1WaveDelta[i] = pRuntime->pMode1Delta[ObjId];
            pRuntime->pMode1Delta[ObjId] = 0;
            pRuntime->pMode1DeltaMark[ObjId] = 0;
        }
        pRuntime->nMode1Touched = 0;
        for ( i = 0; i < nThis; i++ )
        {
            ObjId = pRuntime->pMode1Wave[i];
            Delta = pRuntime->pMode1WaveDelta[i];
            pRuntime->pMode1WaveDelta[i] = 0;
            if ( Delta == 0 )
                continue;
            pObj = If_ManObj( p, ObjId );
            OldRefs = pObj->nRefs;
            NewRefs = OldRefs + Delta;
            if ( NewRefs < 0 )
            {
                Abc_Print( -1, "IF_PAR_INVARIANT reason=mode1_finalize_negative_ref obj=%d old=%d delta=%d\n", ObjId, OldRefs, Delta );
                If_ManParMode1FinalizeClear( pRuntime );
                return 0;
            }
            pObj->nRefs = NewRefs;
            if ( !If_ObjIsAnd(pObj) )
                continue;
            if ( OldRefs > 0 && NewRefs == 0 )
            {
                if ( !If_ManParMode1FinalizeCutDelta( pRuntime, If_ObjCutBest(pObj), -1 ) )
                    { If_ManParMode1FinalizeClear( pRuntime ); return 0; }
            }
            else if ( OldRefs == 0 && NewRefs > 0 )
            {
                if ( !If_ManParMode1FinalizeCutDelta( pRuntime, If_ObjCutBest(pObj), 1 ) )
                    { If_ManParMode1FinalizeClear( pRuntime ); return 0; }
            }
        }
    }
    return 1;
}

static int If_ManParCutLeavesSortedUnique( If_Cut_t * pCut )
{
    int i;
    for ( i = 1; i < (int)pCut->nLeaves; i++ )
        if ( pCut->pLeaves[i-1] >= pCut->pLeaves[i] )
            return 0;
    return 1;
}

static int If_ManParFinalizeCutDiffDelta( If_ParRuntime_t * pRuntime, If_Cut_t * pOld, If_Cut_t * pNew )
{
    int iOld = 0, iNew = 0, OldLeaf, NewLeaf;
    while ( iOld < (int)pOld->nLeaves || iNew < (int)pNew->nLeaves )
    {
        if ( iOld == (int)pOld->nLeaves )
        {
            if ( !If_ManParMode1FinalizeAddDelta( pRuntime, pNew->pLeaves[iNew++], 1 ) )
                return 0;
            continue;
        }
        if ( iNew == (int)pNew->nLeaves )
        {
            if ( !If_ManParMode1FinalizeAddDelta( pRuntime, pOld->pLeaves[iOld++], -1 ) )
                return 0;
            continue;
        }
        OldLeaf = pOld->pLeaves[iOld];
        NewLeaf = pNew->pLeaves[iNew];
        if ( OldLeaf == NewLeaf )
        {
            iOld++;
            iNew++;
        }
        else if ( OldLeaf < NewLeaf )
        {
            if ( !If_ManParMode1FinalizeAddDelta( pRuntime, OldLeaf, -1 ) )
                return 0;
            iOld++;
        }
        else
        {
            if ( !If_ManParMode1FinalizeAddDelta( pRuntime, NewLeaf, 1 ) )
                return 0;
            iNew++;
        }
    }
    return 1;
}

static int If_ManParFinalizeImproveLevel( If_ParRuntime_t * pRuntime, int Level )
{
    If_Man_t * p = pRuntime->pMan;
    If_ParMeta_t * pMeta = p->pParMeta;
    If_Obj_t * pObj;
    If_Cut_t * pOld, * pNew;
    int k, i, ObjId, Delta, OldRefs, NewRefs, nThis, nLevelAreaDelta = 0;
    assert( pRuntime->nMode1Touched == 0 );
    for ( k = pMeta->pLevelStarts[Level]; k < pMeta->pLevelStarts[Level+1]; k++ )
    {
        ObjId = pMeta->pLevelObjs[k];
        if ( !pRuntime->pImproveAccepted[ObjId] )
            continue;
        pObj = If_ManObj( p, ObjId );
        if ( pMeta->pIsChoiceProvider[ObjId] )
        {
            Abc_Print( -1, "IF_PAR_INVARIANT reason=improve_finalize_provider_root obj=%d\n", ObjId );
            If_ManParMode1FinalizeClear( pRuntime );
            return 0;
        }
        if ( pObj->nRefs <= 0 )
        {
            Abc_Print( -1, "IF_PAR_INVARIANT reason=improve_root_not_live obj=%d refs=%d\n", ObjId, pObj->nRefs );
            If_ManParMode1FinalizeClear( pRuntime );
            return 0;
        }
        pOld = If_ManParImproveOldCut( pRuntime, ObjId );
        pNew = If_ManParImproveNewCut( pRuntime, ObjId );
        if ( !If_ManParCutLeavesSortedUnique(pOld) || !If_ManParCutLeavesSortedUnique(pNew) )
        {
            Abc_Print( -1, "IF_PAR_INVARIANT reason=improve_cut_order obj=%d\n", ObjId );
            If_ManParMode1FinalizeClear( pRuntime );
            return 0;
        }
        if ( !If_ManParFinalizeCutDiffDelta( pRuntime, pOld, pNew ) )
            { If_ManParMode1FinalizeClear( pRuntime ); return 0; }
    }
    while ( pRuntime->nMode1Touched > 0 )
    {
        qsort( pRuntime->pMode1Touched, (size_t)pRuntime->nMode1Touched, sizeof(int), If_ManParCompareInt );
        nThis = pRuntime->nMode1Touched;
        memcpy( pRuntime->pMode1Wave, pRuntime->pMode1Touched, sizeof(int) * nThis );
        for ( i = 0; i < nThis; i++ )
        {
            ObjId = pRuntime->pMode1Wave[i];
            pRuntime->pMode1WaveDelta[i] = pRuntime->pMode1Delta[ObjId];
            pRuntime->pMode1Delta[ObjId] = 0;
            pRuntime->pMode1DeltaMark[ObjId] = 0;
        }
        pRuntime->nMode1Touched = 0;
        for ( i = 0; i < nThis; i++ )
        {
            ObjId = pRuntime->pMode1Wave[i];
            Delta = pRuntime->pMode1WaveDelta[i];
            pRuntime->pMode1WaveDelta[i] = 0;
            if ( Delta == 0 )
                continue;
            pObj = If_ManObj( p, ObjId );
            OldRefs = pObj->nRefs;
            NewRefs = OldRefs + Delta;
            if ( NewRefs < 0 )
            {
                Abc_Print( -1, "IF_PAR_INVARIANT reason=improve_finalize_negative_ref obj=%d old=%d delta=%d\n", ObjId, OldRefs, Delta );
                If_ManParMode1FinalizeClear( pRuntime );
                return 0;
            }
            pObj->nRefs = NewRefs;
            if ( !If_ObjIsAnd(pObj) )
                continue;
            if ( OldRefs > 0 && NewRefs == 0 )
            {
                nLevelAreaDelta--;
                if ( !If_ManParMode1FinalizeCutDelta( pRuntime, If_ObjCutBest(pObj), -1 ) )
                    { If_ManParMode1FinalizeClear( pRuntime ); return 0; }
            }
            else if ( OldRefs == 0 && NewRefs > 0 )
            {
                nLevelAreaDelta++;
                if ( !If_ManParMode1FinalizeCutDelta( pRuntime, If_ObjCutBest(pObj), 1 ) )
                    { If_ManParMode1FinalizeClear( pRuntime ); return 0; }
            }
        }
    }
    if ( nLevelAreaDelta > 0 )
    {
        Abc_Print( -1, "IF_PAR_INVARIANT reason=improve_area_increase level=%d delta=%d\n", Level, nLevelAreaDelta );
        return 0;
    }
    for ( k = pMeta->pLevelStarts[Level]; k < pMeta->pLevelStarts[Level+1]; k++ )
    {
        ObjId = pMeta->pLevelObjs[k];
        if ( !pRuntime->pImproveAccepted[ObjId] )
            continue;
        pObj = If_ManObj( p, ObjId );
        pNew = If_ManParImproveNewCut( pRuntime, ObjId );
        If_CutCopy( p, If_ObjCutBest(pObj), pNew );
        pRuntime->pImproveAccepted[ObjId] = 0;
    }
    pRuntime->nImproveAreaDelta += nLevelAreaDelta;
    return 1;
}

static void If_ManParMode1RefCheck_rec( If_Man_t * p, If_Obj_t * pObj, int * pRefs )
{
    If_Obj_t * pLeaf;
    If_Cut_t * pCutBest;
    int i;
    if ( pRefs[pObj->Id]++ || If_ObjIsCi(pObj) || If_ObjIsConst1(pObj) )
        return;
    assert( If_ObjIsAnd(pObj) );
    pCutBest = If_ObjCutBest(pObj);
    If_CutForEachLeaf( p, pCutBest, pLeaf, i )
        If_ManParMode1RefCheck_rec( p, pLeaf, pRefs );
}

static int If_ManParMode1CheckRefs( If_Man_t * p )
{
    If_ParRuntime_t * pRuntime = p->pParRuntime;
    If_Obj_t * pObj;
    int i;
    memset( pRuntime->pMode1RefCheck, 0, sizeof(int) * p->pParMeta->nObjs );
    If_ManForEachCo( p, pObj, i )
        If_ManParMode1RefCheck_rec( p, If_ObjFanin0(pObj), pRuntime->pMode1RefCheck );
    If_ManForEachObj( p, pObj, i )
    {
        if ( pObj->nRefs != pRuntime->pMode1RefCheck[pObj->Id] )
        {
            Abc_Print( -1, "IF_PAR_INVARIANT reason=mode1_ref_check obj=%d refs=%d expected=%d\n",
                pObj->Id, pObj->nRefs, pRuntime->pMode1RefCheck[pObj->Id] );
            return 0;
        }
    }
    return 1;
}

static int If_ManParCheckMode1Standard( If_Man_t * p, char ** ppReason )
{
    If_Obj_t * pObj, * pTemp;
    int i;
    if ( !If_ManParCheckMode0Standard( p, ppReason ) )
        return 0;
    if ( p->pPars->fArea || p->pPars->fFancy )
        { If_ManParSetReason( ppReason, "mode1_nonstandard_flag" ); return 0; }
    If_ManForEachNode( p, pObj, i )
    {
        if ( !pObj->fRepr )
            continue;
        for ( pTemp = pObj->pEquiv; pTemp; pTemp = pTemp->pEquiv )
            if ( pTemp->nRefs != 0 )
                { If_ManParSetReason( ppReason, "mode1_choice_provider_ref" ); return 0; }
    }
    If_ManParSetReason( ppReason, "ok" );
    return 1;
}

static int If_ManParCheckMode2Standard( If_Man_t * p, char ** ppReason )
{
    If_Obj_t * pObj, * pTemp;
    int i;
    if ( !If_ManParCheckMode0Standard( p, ppReason ) )
        return 0;
    if ( p->pPars->fArea || p->pPars->fFancy )
        { If_ManParSetReason( ppReason, "mode2_nonstandard_flag" ); return 0; }
    If_ManForEachNode( p, pObj, i )
    {
        if ( !pObj->fRepr )
            continue;
        for ( pTemp = pObj->pEquiv; pTemp; pTemp = pTemp->pEquiv )
            if ( pTemp->nRefs != 0 )
                { If_ManParSetReason( ppReason, "mode2_choice_provider_ref" ); return 0; }
    }
    If_ManParSetReason( ppReason, "ok" );
    return 1;
}

static int If_ManParRunMode0Round( If_Man_t * p, int fPreprocess, int fFirst, char * pLabel )
{
    If_ParRuntime_t * pRuntime = p->pParRuntime;
    char * pReason = NULL;
    int Level;
    int fRoundValidate;
    int fHasChoices;
    abctime clk, clkStep;
    if ( pRuntime == NULL || p->pParMeta == NULL )
        return 1;
    if ( !If_ManParCheckMode0Standard( p, &pReason ) )
    {
        Abc_Print( -1, "IF_PAR_INVARIANT reason=%s\n", pReason ? pReason : "mode0_nonstandard_flag" );
        return 0;
    }
    fRoundValidate = If_ManParRoundValidationEnabled();
    fHasChoices = p->pParMeta->nChoices > 0;
    if ( fRoundValidate && !If_ManParMode0CheckNoLiveCutsets( p ) )
        return 0;
    pRuntime->Mode = 0;
    pRuntime->fPreprocess = fPreprocess;
    pRuntime->fFirst = fFirst;
    If_ManParRoundResetStats( pRuntime );
    pRuntime->nRoundLevels = 0;
    pRuntime->nRoundObjs = 0;
    pRuntime->nRoundChoices = 0;
    pRuntime->RoundChecksum = If_ManParHash( 2166136261u, 0 );
    pRuntime->RoundChecksum = If_ManParHash( pRuntime->RoundChecksum, (unsigned)fPreprocess );
    pRuntime->RoundChecksum = If_ManParHash( pRuntime->RoundChecksum, (unsigned)fFirst );
    clk = Abc_Clock();
    for ( Level = 0; Level < p->pParMeta->nLevels; Level++ )
    {
        clkStep = Abc_Clock();
        if ( !If_ManParRunLevelStaticJob( pRuntime, Level, IF_PAR_JOB_CUTSET_ALLOC ) )
            return 0;
        pRuntime->RoundAllocTime += Abc_Clock() - clkStep;
        pRuntime->nRoundAllocCount += p->pParMeta->pLevelStarts[Level+1] - p->pParMeta->pLevelStarts[Level];
        if ( !If_ManParRunLevelJob( pRuntime, Level, IF_PAR_JOB_MODE0_AND ) )
            return 0;
        if ( fHasChoices && If_ManParChoiceTaskCount( pRuntime, Level ) > 0 )
        {
            if ( !If_ManParRunLevelJob( pRuntime, Level, IF_PAR_JOB_MODE0_CHOICE ) )
                return 0;
        }
        else if ( fHasChoices )
            pRuntime->nRoundChoiceSkippedLevels++;
        clkStep = Abc_Clock();
        if ( !If_ManParRunLevelStaticJob( pRuntime, Level, IF_PAR_JOB_CUTSET_RELEASE ) )
            return 0;
        pRuntime->RoundReleaseTime += Abc_Clock() - clkStep;
        pRuntime->nRoundReleaseCount += p->pParMeta->pReleaseStarts[Level+1] - p->pParMeta->pReleaseStarts[Level];
    }
    pRuntime->RoundTime = Abc_Clock() - clk;
    if ( !If_ManParMode0CheckRoundCounts( pRuntime, 0 ) )
        return 0;
    if ( p->pPars->fVerbose )
        Abc_Print( 1, "IF_PAR_ROUND label=%s mode=0 preprocess=%d first=%d threads=%d levels=%d tasks=%d choices=%d checksum=%u skeleton_time=0.000000 mode0_compute_time=%.6f finalize_time=0.000000 release_time=%.6f active_time=%.6f wait_time=%.6f claim_time=%.6f and_tasks=%d choice_tasks=%d and_chunks=%d choice_chunks=%d claimed_chunks=%d empty_claims=%d alloc_time=%.6f alloc_count=%d release_count=%d refcheck_time=%.6f choice_skipped_levels=%d round_validate=%d\n",
            pLabel ? pLabel : "none", fPreprocess, fFirst, pRuntime->nThreads, pRuntime->nRoundLevels,
            pRuntime->nRoundObjs, pRuntime->nRoundChoices, pRuntime->RoundChecksum, If_ManStage0TimeSec(pRuntime->RoundTime),
            If_ManStage0TimeSec(pRuntime->RoundReleaseTime), If_ManStage0TimeSec(pRuntime->RoundActiveTime),
            If_ManStage0TimeSec(pRuntime->RoundWaitTime), If_ManStage0TimeSec(pRuntime->RoundClaimTime),
            pRuntime->nRoundAndTasks, pRuntime->nRoundChoiceTasks, pRuntime->nRoundAndChunks, pRuntime->nRoundChoiceChunks,
            pRuntime->nRoundClaimedChunks, pRuntime->nRoundEmptyClaims, If_ManStage0TimeSec(pRuntime->RoundAllocTime),
            pRuntime->nRoundAllocCount, pRuntime->nRoundReleaseCount, If_ManStage0TimeSec(pRuntime->RoundRefCheckTime),
            pRuntime->nRoundChoiceSkippedLevels, fRoundValidate );
    return 1;
}

static int If_ManParRunMode1Round( If_Man_t * p, int fPreprocess, int fFirst, char * pLabel )
{
    If_ParRuntime_t * pRuntime = p->pParRuntime;
    char * pReason = NULL;
    int Level;
    int fRoundValidate;
    int fHasChoices;
    abctime clk, clkStep;
    if ( pRuntime == NULL || p->pParMeta == NULL )
        return 1;
    if ( !If_ManParCheckMode1Standard( p, &pReason ) )
    {
        Abc_Print( -1, "IF_PAR_INVARIANT reason=%s\n", pReason ? pReason : "mode1_nonstandard_flag" );
        return 0;
    }
    fRoundValidate = If_ManParRoundValidationEnabled();
    fHasChoices = p->pParMeta->nChoices > 0;
    if ( fRoundValidate && !If_ManParMode0CheckNoLiveCutsets( p ) )
        return 0;
    If_ManParMode1FinalizeClear( pRuntime );
    memset( pRuntime->pMode1Changed, 0, sizeof(char) * p->pParMeta->nObjs );
    memset( pRuntime->pMode1AllowFinalize, 0, sizeof(char) * p->pParMeta->nObjs );
    pRuntime->Mode = 1;
    pRuntime->fPreprocess = fPreprocess;
    pRuntime->fFirst = fFirst;
    If_ManParRoundResetStats( pRuntime );
    pRuntime->nRoundLevels = 0;
    pRuntime->nRoundObjs = 0;
    pRuntime->nRoundChoices = 0;
    pRuntime->RoundChecksum = If_ManParHash( 2166136261u, 1 );
    pRuntime->RoundChecksum = If_ManParHash( pRuntime->RoundChecksum, (unsigned)fPreprocess );
    pRuntime->RoundChecksum = If_ManParHash( pRuntime->RoundChecksum, (unsigned)fFirst );
    clk = Abc_Clock();
    for ( Level = 0; Level < p->pParMeta->nLevels; Level++ )
    {
        clkStep = Abc_Clock();
        if ( !If_ManParRunLevelStaticJob( pRuntime, Level, IF_PAR_JOB_CUTSET_ALLOC ) )
            return 0;
        pRuntime->RoundAllocTime += Abc_Clock() - clkStep;
        pRuntime->nRoundAllocCount += p->pParMeta->pLevelStarts[Level+1] - p->pParMeta->pLevelStarts[Level];
        if ( !If_ManParRunLevelJob( pRuntime, Level, IF_PAR_JOB_MODE1_AND ) )
            return 0;
        if ( fHasChoices && If_ManParChoiceTaskCount( pRuntime, Level ) > 0 )
        {
            if ( !If_ManParRunLevelJob( pRuntime, Level, IF_PAR_JOB_MODE1_CHOICE ) )
                return 0;
        }
        else if ( fHasChoices )
            pRuntime->nRoundChoiceSkippedLevels++;
        clkStep = Abc_Clock();
        if ( !If_ManParFinalizeMode1Level( pRuntime, Level ) )
            return 0;
        pRuntime->RoundFinalizeTime += Abc_Clock() - clkStep;
        clkStep = Abc_Clock();
        if ( !If_ManParRunLevelStaticJob( pRuntime, Level, IF_PAR_JOB_CUTSET_RELEASE ) )
            return 0;
        pRuntime->RoundReleaseTime += Abc_Clock() - clkStep;
        pRuntime->nRoundReleaseCount += p->pParMeta->pReleaseStarts[Level+1] - p->pParMeta->pReleaseStarts[Level];
    }
    pRuntime->RoundTime = Abc_Clock() - clk;
    if ( !If_ManParMode0CheckRoundCounts( pRuntime, 1 ) )
        return 0;
    if ( fRoundValidate )
    {
        clkStep = Abc_Clock();
        if ( !If_ManParMode1CheckRefs( p ) )
        {
            pRuntime->RoundRefCheckTime += Abc_Clock() - clkStep;
            return 0;
        }
        pRuntime->RoundRefCheckTime += Abc_Clock() - clkStep;
    }
    if ( p->pPars->fVerbose )
        Abc_Print( 1, "IF_PAR_ROUND label=%s mode=1 preprocess=%d first=%d threads=%d levels=%d tasks=%d choices=%d checksum=%u skeleton_time=0.000000 mode1_compute_time=%.6f finalize_time=%.6f release_time=%.6f active_time=%.6f wait_time=%.6f claim_time=%.6f and_tasks=%d choice_tasks=%d and_chunks=%d choice_chunks=%d claimed_chunks=%d empty_claims=%d alloc_time=%.6f alloc_count=%d release_count=%d refcheck_time=%.6f choice_skipped_levels=%d round_validate=%d\n",
            pLabel ? pLabel : "none", fPreprocess, fFirst, pRuntime->nThreads, pRuntime->nRoundLevels,
            pRuntime->nRoundObjs, pRuntime->nRoundChoices, pRuntime->RoundChecksum, If_ManStage0TimeSec(pRuntime->RoundTime),
            If_ManStage0TimeSec(pRuntime->RoundFinalizeTime), If_ManStage0TimeSec(pRuntime->RoundReleaseTime),
            If_ManStage0TimeSec(pRuntime->RoundActiveTime), If_ManStage0TimeSec(pRuntime->RoundWaitTime),
            If_ManStage0TimeSec(pRuntime->RoundClaimTime), pRuntime->nRoundAndTasks, pRuntime->nRoundChoiceTasks,
            pRuntime->nRoundAndChunks, pRuntime->nRoundChoiceChunks, pRuntime->nRoundClaimedChunks, pRuntime->nRoundEmptyClaims,
            If_ManStage0TimeSec(pRuntime->RoundAllocTime), pRuntime->nRoundAllocCount, pRuntime->nRoundReleaseCount,
            If_ManStage0TimeSec(pRuntime->RoundRefCheckTime), pRuntime->nRoundChoiceSkippedLevels, fRoundValidate );
    return 1;
}

static int If_ManParRunMode2Round( If_Man_t * p, int fPreprocess, int fFirst, char * pLabel )
{
    If_ParRuntime_t * pRuntime = p->pParRuntime;
    char * pReason = NULL;
    int Level;
    int fRoundValidate;
    int fHasChoices;
    abctime clk, clkStep;
    if ( pRuntime == NULL || p->pParMeta == NULL )
        return 1;
    if ( !If_ManParCheckMode2Standard( p, &pReason ) )
    {
        Abc_Print( -1, "IF_PAR_INVARIANT reason=%s\n", pReason ? pReason : "mode2_nonstandard_flag" );
        return 0;
    }
    fRoundValidate = If_ManParRoundValidationEnabled();
    fHasChoices = p->pParMeta->nChoices > 0;
    if ( fRoundValidate && !If_ManParMode0CheckNoLiveCutsets( p ) )
        return 0;
    If_ManParMode1FinalizeClear( pRuntime );
    memset( pRuntime->pMode1Changed, 0, sizeof(char) * p->pParMeta->nObjs );
    memset( pRuntime->pMode1AllowFinalize, 0, sizeof(char) * p->pParMeta->nObjs );
    pRuntime->Mode = 2;
    pRuntime->fPreprocess = fPreprocess;
    pRuntime->fFirst = fFirst;
    If_ManParRoundResetStats( pRuntime );
    pRuntime->nRoundLevels = 0;
    pRuntime->nRoundObjs = 0;
    pRuntime->nRoundChoices = 0;
    pRuntime->RoundChecksum = If_ManParHash( 2166136261u, 2 );
    pRuntime->RoundChecksum = If_ManParHash( pRuntime->RoundChecksum, (unsigned)fPreprocess );
    pRuntime->RoundChecksum = If_ManParHash( pRuntime->RoundChecksum, (unsigned)fFirst );
    clk = Abc_Clock();
    for ( Level = 0; Level < p->pParMeta->nLevels; Level++ )
    {
        clkStep = Abc_Clock();
        if ( !If_ManParRunLevelStaticJob( pRuntime, Level, IF_PAR_JOB_CUTSET_ALLOC ) )
            return 0;
        pRuntime->RoundAllocTime += Abc_Clock() - clkStep;
        pRuntime->nRoundAllocCount += p->pParMeta->pLevelStarts[Level+1] - p->pParMeta->pLevelStarts[Level];
        if ( !If_ManParRunLevelJob( pRuntime, Level, IF_PAR_JOB_MODE2_AND ) )
            return 0;
        if ( fHasChoices && If_ManParChoiceTaskCount( pRuntime, Level ) > 0 )
        {
            if ( !If_ManParRunLevelJob( pRuntime, Level, IF_PAR_JOB_MODE2_CHOICE ) )
                return 0;
        }
        else if ( fHasChoices )
            pRuntime->nRoundChoiceSkippedLevels++;
        clkStep = Abc_Clock();
        if ( !If_ManParFinalizeMode1Level( pRuntime, Level ) )
            return 0;
        pRuntime->RoundFinalizeTime += Abc_Clock() - clkStep;
        clkStep = Abc_Clock();
        if ( !If_ManParRunLevelStaticJob( pRuntime, Level, IF_PAR_JOB_CUTSET_RELEASE ) )
            return 0;
        pRuntime->RoundReleaseTime += Abc_Clock() - clkStep;
        pRuntime->nRoundReleaseCount += p->pParMeta->pReleaseStarts[Level+1] - p->pParMeta->pReleaseStarts[Level];
    }
    pRuntime->RoundTime = Abc_Clock() - clk;
    if ( !If_ManParMode0CheckRoundCounts( pRuntime, 2 ) )
        return 0;
    if ( fRoundValidate )
    {
        clkStep = Abc_Clock();
        if ( !If_ManParMode1CheckRefs( p ) )
        {
            pRuntime->RoundRefCheckTime += Abc_Clock() - clkStep;
            return 0;
        }
        pRuntime->RoundRefCheckTime += Abc_Clock() - clkStep;
    }
    if ( p->pPars->fVerbose )
        Abc_Print( 1, "IF_PAR_ROUND label=%s mode=2 preprocess=%d first=%d threads=%d levels=%d tasks=%d choices=%d checksum=%u skeleton_time=0.000000 mode2_compute_time=%.6f finalize_time=%.6f release_time=%.6f active_time=%.6f wait_time=%.6f claim_time=%.6f and_tasks=%d choice_tasks=%d and_chunks=%d choice_chunks=%d claimed_chunks=%d empty_claims=%d alloc_time=%.6f alloc_count=%d release_count=%d refcheck_time=%.6f choice_skipped_levels=%d round_validate=%d\n",
            pLabel ? pLabel : "none", fPreprocess, fFirst, pRuntime->nThreads, pRuntime->nRoundLevels,
            pRuntime->nRoundObjs, pRuntime->nRoundChoices, pRuntime->RoundChecksum, If_ManStage0TimeSec(pRuntime->RoundTime),
            If_ManStage0TimeSec(pRuntime->RoundFinalizeTime), If_ManStage0TimeSec(pRuntime->RoundReleaseTime),
            If_ManStage0TimeSec(pRuntime->RoundActiveTime), If_ManStage0TimeSec(pRuntime->RoundWaitTime),
            If_ManStage0TimeSec(pRuntime->RoundClaimTime), pRuntime->nRoundAndTasks, pRuntime->nRoundChoiceTasks,
            pRuntime->nRoundAndChunks, pRuntime->nRoundChoiceChunks, pRuntime->nRoundClaimedChunks, pRuntime->nRoundEmptyClaims,
            If_ManStage0TimeSec(pRuntime->RoundAllocTime), pRuntime->nRoundAllocCount, pRuntime->nRoundReleaseCount,
            If_ManStage0TimeSec(pRuntime->RoundRefCheckTime), pRuntime->nRoundChoiceSkippedLevels, fRoundValidate );
    return 1;
}

static void If_ManParImproveResetStats( If_ParRuntime_t * pRuntime )
{
    pRuntime->nImproveRoots = 0;
    pRuntime->nImproveCandidates = 0;
    pRuntime->nImproveAccepted = 0;
    pRuntime->nImproveRejectRule = 0;
    pRuntime->nImproveRejectDelay = 0;
    pRuntime->nImproveAreaDelta = 0;
}

static int If_ManParImproveValidationEnabled()
{
    char * pEnv = getenv( "IF_PAR_IMPROVE_VALIDATE" );
    return pEnv != NULL && pEnv[0] != 0 && strcmp( pEnv, "0" );
}

static int If_ManParCheckImproveStandard( If_Man_t * p, char ** ppReason )
{
    if ( p->pParRuntime == NULL || p->pParMeta == NULL )
        { If_ManParSetReason( ppReason, "improve_runtime_missing" ); return 0; }
    if ( !If_ManParCheckMode1Standard( p, ppReason ) )
        return 0;
    If_ManParSetReason( ppReason, "ok" );
    return 1;
}

static int If_ManParRunImproveMapping( If_Man_t * p )
{
    If_ParRuntime_t * pRuntime = p->pParRuntime;
    char * pReason = NULL;
    int Level;
    int fValidate;
    abctime clk, clkStep, ComputeTime, ValidationTime = 0;
    if ( !If_ManParCheckImproveStandard( p, &pReason ) )
    {
        Abc_Print( -1, "IF_PAR_INVARIANT reason=%s\n", pReason ? pReason : "improve_nonstandard_flag" );
        return 0;
    }
    if ( !If_ManParMode0CheckNoLiveCutsets( p ) )
        return 0;
    If_ManParMode1FinalizeClear( pRuntime );
    memset( pRuntime->pImproveAccepted, 0, sizeof(char) * p->pParMeta->nObjs );
    pRuntime->Mode = 7;
    pRuntime->fPreprocess = 0;
    pRuntime->fFirst = 0;
    If_ManParRoundResetStats( pRuntime );
    If_ManParImproveResetStats( pRuntime );
    pRuntime->RoundChecksum = If_ManParHash( 2166136261u, 7 );
    fValidate = If_ManParImproveValidationEnabled();
    clk = Abc_Clock();
    for ( Level = 0; Level < p->pParMeta->nLevels; Level++ )
    {
        if ( !If_ManParRunLevelJob( pRuntime, Level, IF_PAR_JOB_IMPROVE_AND ) )
            return 0;
        clkStep = Abc_Clock();
        if ( !If_ManParFinalizeImproveLevel( pRuntime, Level ) )
            return 0;
        pRuntime->RoundFinalizeTime += Abc_Clock() - clkStep;
        if ( fValidate )
        {
            clkStep = Abc_Clock();
            if ( !If_ManParMode1CheckRefs( p ) )
                return 0;
            ValidationTime += Abc_Clock() - clkStep;
        }
    }
    pRuntime->RoundTime = Abc_Clock() - clk;
    ComputeTime = pRuntime->RoundTime > pRuntime->RoundFinalizeTime + ValidationTime ? pRuntime->RoundTime - pRuntime->RoundFinalizeTime - ValidationTime : 0;
    if ( p->pPars->fVerbose )
        Abc_Print( 1, "IF_PAR_IMPROVE_SUMMARY threads=%d levels=%d tasks=%d live_roots=%d leaf_candidates=%d accepted_roots=%d leaf_rule_rejects=%d delay_reject_roots=%d area_delta=%d compute_time=%.6f finalize_time=%.6f validation_time=%.6f total_time=%.6f active_time=%.6f wait_time=%.6f claim_time=%.6f and_chunks=%d claimed_chunks=%d empty_claims=%d checksum=%u validation=%d\n",
            pRuntime->nThreads, pRuntime->nRoundLevels, pRuntime->nRoundObjs, pRuntime->nImproveRoots,
            pRuntime->nImproveCandidates, pRuntime->nImproveAccepted, pRuntime->nImproveRejectRule,
            pRuntime->nImproveRejectDelay, pRuntime->nImproveAreaDelta, If_ManStage0TimeSec(ComputeTime),
            If_ManStage0TimeSec(pRuntime->RoundFinalizeTime), If_ManStage0TimeSec(ValidationTime),
            If_ManStage0TimeSec(pRuntime->RoundTime), If_ManStage0TimeSec(pRuntime->RoundActiveTime),
            If_ManStage0TimeSec(pRuntime->RoundWaitTime), If_ManStage0TimeSec(pRuntime->RoundClaimTime),
            pRuntime->nRoundAndChunks, pRuntime->nRoundClaimedChunks, pRuntime->nRoundEmptyClaims,
            pRuntime->RoundChecksum, fValidate );
    return 1;
}

int If_ManParImproveMapping( If_Man_t * p, int * pfHandled )
{
    char * pReason = NULL;
    if ( pfHandled )
        *pfHandled = 0;
    if ( !p->pPars->fParMap )
        return 1;
    if ( !If_ManParIsSupported( p, &pReason ) )
        return 1;
    if ( pfHandled )
        *pfHandled = 1;
    if ( p->pParRuntime == NULL || p->pParMeta == NULL )
    {
        Abc_Print( -1, "IF_PAR_INVARIANT reason=improve_runtime_missing\n" );
        return 0;
    }
    return If_ManParRunImproveMapping( p );
}

static int If_ManParCheckRequiredStandard( If_Man_t * p, char ** ppReason )
{
    If_Par_t * pPars = p->pPars;
    if ( p->pParRuntime == NULL || p->pParMeta == NULL )
        { If_ManParSetReason( ppReason, "required_runtime_missing" ); return 0; }
    if ( !If_ManParCheckDefaultTiming( p, ppReason ) )
        return 0;
    if ( !If_ManParCheckMode0Standard( p, ppReason ) )
        return 0;
    if ( pPars->fAreaOnly )
        { If_ManParSetReason( ppReason, "area_only" ); return 0; }
    if ( pPars->fDoAverage )
        { If_ManParSetReason( ppReason, "average_required" ); return 0; }
    if ( pPars->fLatchPaths || pPars->nLatchesCi || pPars->nLatchesCo || pPars->nLatchesCiBox || pPars->nLatchesCoBox )
        { If_ManParSetReason( ppReason, "latch_path" ); return 0; }
    If_ManParSetReason( ppReason, "ok" );
    return 1;
}

static const char * If_ManParRequiredSourceName( If_ParRequiredSource_t Source )
{
    return Source == IF_PAR_REQUIRED_SOURCE_IMPROVE ? "improve" : "round";
}

static int If_ManParRequiredAtomicMinObj( If_ParRuntime_t * pRuntime, If_Obj_t * pObj, float Required )
{
    uint32_t Key, Old, Expected;
    if ( pObj->Id < 0 || pObj->Id >= pRuntime->pMan->pParMeta->nObjs )
        return 0;
    if ( !If_ManParRequiredEncode( Required, &Key ) )
        return 0;
    Old = If_ParAtomicU32Load( pRuntime->pReqKeys + pObj->Id );
    while ( Key < Old )
    {
        Expected = Old;
        if ( If_ParAtomicU32CompareExchangeWeak( pRuntime->pReqKeys + pObj->Id, &Expected, Key ) )
            break;
        Old = Expected;
    }
    return 1;
}

static int If_ManParRequiredCiTerminalObj( If_ParThread_t * pThread, int CiIndex )
{
    If_ParRuntime_t * pRuntime = pThread->pRuntime;
    If_Man_t * p = pRuntime->pMan;
    If_Obj_t * pObj;
    If_Cut_t * pCut;
    float ObjRequired, Required;
    if ( CiIndex < 0 || CiIndex >= Vec_PtrSize(p->vCis) )
        { If_ManParThreadSetError( pThread, "required_ci_index_range", NULL ); return 0; }
    pObj = (If_Obj_t *)Vec_PtrEntry( p->vCis, CiIndex );
    if ( pObj == NULL || !If_ObjIsCi(pObj) )
        { If_ManParThreadSetError( pThread, "required_ci_obj", pObj ); return 0; }
    if ( pObj->nRefs == 0 )
        return 1;
    pCut = If_ObjCutBest(pObj);
    if ( pCut == NULL )
        { If_ManParThreadSetError( pThread, "required_ci_missing_cut", pObj ); return 0; }
    if ( pCut->nLeaves != 1 || pCut->pLeaves[0] != pObj->Id )
        { If_ManParThreadSetError( pThread, "required_ci_nonself_cut", pObj ); return 0; }
    if ( !If_ManParRequiredReadKey( pRuntime, pObj, &ObjRequired ) )
        { If_ManParThreadSetError( pThread, "required_ci_root_float", pObj ); return 0; }
    Required = ObjRequired - (float)1.0;
    if ( !If_ManParRequiredFloatValid( Required ) )
        { If_ManParThreadSetError( pThread, "required_ci_candidate_float", pObj ); return 0; }
    if ( !If_ManParRequiredAtomicMinObj( pRuntime, pObj, Required ) )
        { If_ManParThreadSetError( pThread, "required_ci_update", pObj ); return 0; }
    pThread->nProcessedRoots++;
    pThread->nCandidateCount++;
    return 1;
}

static int If_ManParRequiredWriteBackObj( If_ParThread_t * pThread, int ObjId )
{
    If_ParRuntime_t * pRuntime = pThread->pRuntime;
    If_Man_t * p = pRuntime->pMan;
    If_Obj_t * pObj;
    If_Cut_t * pCut;
    float Required;
    if ( ObjId < 0 || ObjId >= p->pParMeta->nObjs )
        { If_ManParThreadSetError( pThread, "required_writeback_obj_range", NULL ); return 0; }
    pObj = If_ManObj( p, ObjId );
    if ( pObj == NULL )
        { If_ManParThreadSetError( pThread, "required_writeback_obj_null", NULL ); return 0; }
    if ( pObj->nRefs > 0 && !If_ObjIsAnd(pObj) && !If_ObjIsCi(pObj) )
    {
        pCut = If_ObjCutBest(pObj);
        if ( !If_ObjIsConst1(pObj) || pCut == NULL || pCut->nLeaves != 0 )
            { If_ManParThreadSetError( pThread, "required_live_nonand_terminal", pObj ); return 0; }
    }
    if ( !If_ManParRequiredReadKey( pRuntime, pObj, &Required ) )
        { If_ManParThreadSetError( pThread, "required_writeback_float", pObj ); return 0; }
    pObj->Required = Required;
    pThread->nProcessedRoots++;
    return 1;
}

static int If_ManParRequiredKeysInit( If_Man_t * p )
{
    If_ParRuntime_t * pRuntime = p->pParRuntime;
    uint32_t Key;
    int i;
    if ( pRuntime == NULL || pRuntime->pReqKeys == NULL )
        return 0;
    if ( !If_ManParRequiredEncode( IF_FLOAT_LARGE, &Key ) )
        return 0;
    for ( i = 0; i < p->pParMeta->nObjs; i++ )
        If_ParAtomicU32Store( pRuntime->pReqKeys + i, Key );
    return 1;
}

static int If_ManParRequiredCheckCutStandard( If_Man_t * p, If_Obj_t * pObj, If_Cut_t * pCut, char ** ppReason )
{
    int i, ObjId;
    if ( pCut == NULL )
        { If_ManParSetReason( ppReason, "required_missing_cut" ); return 0; }
    if ( pCut->fAndCut )
        { If_ManParSetReason( ppReason, "required_and_cut" ); return 0; }
    if ( pCut->fUser )
        { If_ManParSetReason( ppReason, "required_user_cut" ); return 0; }
    for ( i = 0; i < (int)pCut->nLeaves; i++ )
    {
        ObjId = pCut->pLeaves[i];
        if ( ObjId < 0 || ObjId >= p->pParMeta->nObjs )
            { If_ManParSetReason( ppReason, "required_leaf_range" ); return 0; }
        if ( ObjId == pObj->Id )
            { If_ManParSetReason( ppReason, "required_self_leaf" ); return 0; }
        if ( If_ObjIsAnd(If_ManObj(p, ObjId)) && If_ObjLevel(If_ManObj(p, ObjId)) >= If_ObjLevel(pObj) )
            { If_ManParSetReason( ppReason, "required_leaf_level" ); return 0; }
    }
    return 1;
}

static int If_ManParRequiredFlatScanObj( If_ParThread_t * pThread, int ObjId )
{
    If_ParRuntime_t * pRuntime = pThread->pRuntime;
    If_Man_t * p = pRuntime->pMan;
    If_Obj_t * pObj, * pLeaf;
    If_Cut_t * pCut;
    float * pSwitching = p->vSwitching ? (float*)p->vSwitching->pArray : NULL;
    char * pReason = NULL;
    int k, LeafId;
    if ( ObjId < 0 || ObjId >= p->pParMeta->nObjs )
        { If_ManParThreadSetError( pThread, "required_flat_obj_range", NULL ); return 0; }
    pObj = If_ManObj( p, ObjId );
    if ( pObj == NULL )
        { If_ManParThreadSetError( pThread, "required_flat_obj_null", NULL ); return 0; }
    pObj->nVisits = pObj->nVisitsCopy;
    if ( !If_ObjIsAnd(pObj) || pObj->nRefs == 0 )
        return 1;
    pCut = If_ObjCutBest(pObj);
    if ( !If_ManParRequiredCheckCutStandard( p, pObj, pCut, &pReason ) )
        { If_ManParThreadSetError( pThread, pReason ? pReason : (char *)"required_cut", pObj ); return 0; }
    pThread->RequiredAreaGlo += If_CutLutArea( p, pCut );
    pThread->RequiredNets += pCut->nLeaves;
    if ( pSwitching )
        for ( k = 0; k < (int)pCut->nLeaves; k++ )
        {
            LeafId = pCut->pLeaves[k];
            pLeaf = If_ManObj( p, LeafId );
            pThread->RequiredPower += pSwitching[pLeaf->Id];
        }
    pThread->nProcessedRoots++;
    return 1;
}

static int If_ManParRequiredFlatSanitizeStats( If_Man_t * p )
{
    If_ParRuntime_t * pRuntime = p->pParRuntime;
    int i;
    p->AreaGlo = 0.0;
    p->nNets = 0;
    p->dPower = 0.0;
    if ( !If_ManParRunRequiredStaticRangeJob( pRuntime, -1, IF_PAR_JOB_REQUIRED_FLAT_SCAN, NULL, p->pParMeta->nObjs, IF_PAR_REQUIRED_MIN_TASKS_PER_THREAD ) )
        return 0;
    for ( i = 0; i < pRuntime->nThreads; i++ )
    {
        p->AreaGlo += pRuntime->pThreads[i].RequiredAreaGlo;
        p->nNets += pRuntime->pThreads[i].RequiredNets;
        p->dPower += pRuntime->pThreads[i].RequiredPower;
    }
    return 1;
}

static int If_ManParRequiredInitDefaultStage9( If_Man_t * p )
{
    If_Obj_t * pObj;
    int i;
    p->RequiredGlo = If_ManDelayMax( p, 0 );
    if ( !If_ManParRequiredFloatValid( p->RequiredGlo ) )
    {
        Abc_Print( -1, "IF_PAR_INVARIANT reason=required_global_float\n" );
        return 0;
    }
    if ( !If_ManParRequiredKeysInit( p ) )
    {
        Abc_Print( -1, "IF_PAR_INVARIANT reason=required_key_init\n" );
        return 0;
    }
    If_ManForEachCo( p, pObj, i )
        if ( !If_ManParRequiredAtomicMinObj( p->pParRuntime, If_ObjFanin0(pObj), p->RequiredGlo ) )
        {
            Abc_Print( -1, "IF_PAR_INVARIANT reason=required_co_init obj=%d\n", If_ObjFanin0(pObj)->Id );
            return 0;
        }
    return 1;
}

static int If_ManParRequiredPropagateTerminalsStage9( If_Man_t * p )
{
    return If_ManParRunRequiredStaticRangeJob( p->pParRuntime, -1, IF_PAR_JOB_REQUIRED_CI_TERMINAL, NULL, Vec_PtrSize(p->vCis), IF_PAR_REQUIRED_MIN_TASKS_PER_THREAD );
}

static int If_ManParRequiredWriteBackStage9( If_Man_t * p )
{
    return If_ManParRunRequiredStaticRangeJob( p->pParRuntime, -1, IF_PAR_JOB_REQUIRED_WRITEBACK, NULL, p->pParMeta->nObjs, IF_PAR_REQUIRED_MIN_TASKS_PER_THREAD );
}

static int If_ManParRunRequiredStage9LevelJob( If_ParRuntime_t * pRuntime, int Level )
{
    int * pJobObjs = NULL;
    int nTasks = If_ManParSelectLevelTasks( pRuntime, Level, IF_PAR_JOB_REQUIRED_STAGE9_AND, &pJobObjs );
    return If_ManParRunRequiredStaticRangeJob( pRuntime, Level, IF_PAR_JOB_REQUIRED_STAGE9_AND, pJobObjs, nTasks, IF_PAR_REQUIRED_MIN_TASKS_PER_THREAD );
}

static int If_ManParComputeRequiredStage9Fast( If_Man_t * p, int Mode, int fMode0Rebuild )
{
    int Level;
    abctime clk, Elapsed;
    p->pParRuntime->Mode = Mode;
    clk = Abc_Clock();
    if ( fMode0Rebuild )
        If_ManMarkMapping( p );
    else if ( !If_ManParRequiredFlatSanitizeStats( p ) )
        return 0;
    Elapsed = Abc_Clock() - clk;
    p->Stage0Time.required_rebuild_time += Elapsed;
    if ( fMode0Rebuild )
        p->Stage0Time.required_rebuild_mode0_time += Elapsed;
    else
        p->Stage0Time.required_rebuild_flat_time += Elapsed;

    clk = Abc_Clock();
    if ( !If_ManParRequiredInitDefaultStage9( p ) )
        return 0;
    p->Stage0Time.required_init_time += Abc_Clock() - clk;

    clk = Abc_Clock();
    for ( Level = p->pParMeta->nLevels - 1; Level >= 0; Level-- )
        if ( !If_ManParRunRequiredStage9LevelJob( p->pParRuntime, Level ) )
            return 0;
    Elapsed = Abc_Clock() - clk;
    p->Stage0Time.required_prop_time += Elapsed;
    p->Stage0Time.required_prop_and_time += Elapsed;
    clk = Abc_Clock();
    if ( !If_ManParRequiredPropagateTerminalsStage9( p ) )
        return 0;
    Elapsed = Abc_Clock() - clk;
    p->Stage0Time.required_prop_time += Elapsed;
    p->Stage0Time.required_prop_terminal_time += Elapsed;

    clk = Abc_Clock();
    if ( !If_ManParRequiredWriteBackStage9( p ) )
        return 0;
    p->Stage0Time.required_terminal_time += Abc_Clock() - clk;
    return 1;
}

static void If_ManParRequiredLevelClear( If_ParRuntime_t * pRuntime )
{
    int i, ObjId;
    for ( i = 0; i < pRuntime->nReqLevelTouched; i++ )
    {
        ObjId = pRuntime->pReqLevelTouched[i];
        pRuntime->pReqLevelMark[ObjId] = 0;
    }
    pRuntime->nReqLevelTouched = 0;
}

static void If_ManParRequiredReduceLevel( If_ParRuntime_t * pRuntime, abctime * pReduceTime )
{
    If_Man_t * p = pRuntime->pMan;
    If_ParThread_t * pThread;
    If_Obj_t * pLeaf;
    int i, k, ObjId;
    abctime clk = Abc_Clock();
    assert( pRuntime->nReqLevelTouched == 0 );
    for ( i = 0; i < pRuntime->nThreads; i++ )
    {
        pThread = pRuntime->pThreads + i;
        for ( k = 0; k < pThread->nReqTouched; k++ )
        {
            ObjId = pThread->pReqTouched[k];
            if ( !pRuntime->pReqLevelMark[ObjId] )
            {
                pRuntime->pReqLevelMark[ObjId] = 1;
                pRuntime->pReqLevelTouched[pRuntime->nReqLevelTouched++] = ObjId;
            }
        }
    }
    qsort( pRuntime->pReqLevelTouched, (size_t)pRuntime->nReqLevelTouched, sizeof(int), If_ManParCompareInt );
    for ( k = 0; k < pRuntime->nReqLevelTouched; k++ )
    {
        ObjId = pRuntime->pReqLevelTouched[k];
        pLeaf = If_ManObj( p, ObjId );
        for ( i = 0; i < pRuntime->nThreads; i++ )
        {
            pThread = pRuntime->pThreads + i;
            if ( pThread->pReqMark[ObjId] )
                pLeaf->Required = IF_MIN( pLeaf->Required, pThread->pReqMin[ObjId] );
        }
    }
    If_ManParRequiredLevelClear( pRuntime );
    for ( i = 0; i < pRuntime->nThreads; i++ )
        If_ManParRequiredThreadClear( pRuntime->pThreads + i );
    if ( pReduceTime )
        *pReduceTime += Abc_Clock() - clk;
}

static int If_ManParRunRequiredLevelJob( If_ParRuntime_t * pRuntime, int Level, abctime * pPropTime, abctime * pReduceTime )
{
    If_Man_t * p = pRuntime->pMan;
    int * pJobObjs = NULL;
    int nTasks = If_ManParSelectLevelTasks( pRuntime, Level, IF_PAR_JOB_REQUIRED_AND, &pJobObjs );
    int nProcessedRoots = 0, nProcessedReprs = 0, nCandidateCount = 0, nTouchedPeak = 0;
    abctime clkJob, JobTime, ActiveTime = 0, ClaimTime = 0, WaitTime = 0;
    int i;
    if ( nTasks == 0 )
        return 1;
    pRuntime->iLevel = Level;
    pRuntime->JobKind = IF_PAR_JOB_REQUIRED_AND;
    pRuntime->pJobObjs = pJobObjs;
    pRuntime->nJobTasks = nTasks;
    pRuntime->iJobNext = 0;
    pRuntime->nJobNominalChunk = IF_PAR_STAGE55_NOMINAL_CHUNK;
    pRuntime->nJobEffectiveChunk = If_ManParEffectiveChunkSize( nTasks, pRuntime->nThreads, pRuntime->nJobNominalChunk );
    pRuntime->nJobChunks = (nTasks + pRuntime->nJobEffectiveChunk - 1) / pRuntime->nJobEffectiveChunk;
    pRuntime->nJobClaimedChunks = 0;
    pRuntime->nJobEmptyClaims = 0;
    pRuntime->fJobStop = 0;
    If_ManParRequiredLevelClear( pRuntime );
    for ( i = 0; i < pRuntime->nThreads; i++ )
    {
        If_ParThread_t * pThread = pRuntime->pThreads + i;
        pThread->iStart = 0;
        pThread->iStop  = 0;
        If_ManParRequiredThreadClear( pThread );
        If_ManParThreadReset( pThread );
    }
    clkJob = Abc_Clock();
#ifdef ABC_USE_PTHREADS
    if ( pRuntime->nThreads > 1 )
    {
        pthread_mutex_lock( &pRuntime->Mutex );
        pRuntime->nWorking = pRuntime->nThreads;
        pRuntime->iJob++;
        pthread_cond_broadcast( &pRuntime->CondStart );
        while ( pRuntime->nWorking > 0 )
            pthread_cond_wait( &pRuntime->CondDone, &pRuntime->Mutex );
        pthread_mutex_unlock( &pRuntime->Mutex );
    }
    else
#endif
    {
        for ( i = 0; i < pRuntime->nThreads; i++ )
            If_ManParThreadWork( pRuntime->pThreads + i );
    }
    JobTime = Abc_Clock() - clkJob;
    if ( pPropTime )
        *pPropTime += JobTime;
    for ( i = 0; i < pRuntime->nThreads; i++ )
    {
        If_ParThread_t * pThread = pRuntime->pThreads + i;
        if ( pThread->pError )
        {
            Abc_Print( -1, "IF_PAR_INVARIANT reason=%s obj=%d\n", pThread->pError, pThread->ErrorObj );
            return 0;
        }
    }
    for ( i = 0; i < pRuntime->nThreads; i++ )
    {
        If_ParThread_t * pThread = pRuntime->pThreads + i;
        abctime BusyTime = pThread->ActiveTime + pThread->ClaimTime;
        pThread->WaitTime = JobTime > BusyTime ? JobTime - BusyTime : 0;
        nProcessedRoots += pThread->nProcessedRoots;
        nProcessedReprs += pThread->nProcessedReprs;
        nCandidateCount += pThread->nCandidateCount;
        nTouchedPeak = Abc_MaxInt( nTouchedPeak, If_ManParThreadTouchedPeak( pThread, IF_PAR_JOB_REQUIRED_AND ) );
        ActiveTime += pThread->ActiveTime;
        ClaimTime += pThread->ClaimTime;
        WaitTime += pThread->WaitTime;
    }
    if ( p->pPars->fVerbose )
        Abc_Print( 1, "IF_PAR_REQUIRED_LEVEL mode=%d level=%d threads=%d task_count=%d nominal_chunk_size=%d effective_chunk_size=%d chunk_count=%d claimed_chunks=%d empty_claims=%d processed_roots=%d candidate_count=%d local_req_touched_peak=%d active_time=%.6f wait_time=%.6f claim_time=%.6f job_wall_time=%.6f\n",
            pRuntime->Mode, Level, pRuntime->nThreads, pRuntime->nJobTasks, pRuntime->nJobNominalChunk,
            pRuntime->nJobEffectiveChunk, pRuntime->nJobChunks, pRuntime->nJobClaimedChunks,
            pRuntime->nJobEmptyClaims, nProcessedRoots, nCandidateCount, nTouchedPeak,
            If_ManStage0TimeSec(ActiveTime), If_ManStage0TimeSec(WaitTime),
            If_ManStage0TimeSec(ClaimTime), If_ManStage0TimeSec(JobTime) );
    If_ManParProfileRecordJob( pRuntime, IF_PAR_JOB_REQUIRED_AND, Level, nProcessedRoots, nProcessedReprs,
        nCandidateCount, nTouchedPeak, ActiveTime, WaitTime, ClaimTime, JobTime );
    If_ManParRequiredReduceLevel( pRuntime, pReduceTime );
    return 1;
}

static int If_ManParComputeRequiredLegacyLocalReduce( If_Man_t * p, abctime * pPropTime, abctime * pReduceTime )
{
    If_ParRuntime_t * pRuntime = p->pParRuntime;
    char * pReason = NULL;
    int Level;
    if ( pPropTime )
        *pPropTime = 0;
    if ( pReduceTime )
        *pReduceTime = 0;
    if ( pRuntime == NULL || p->pParMeta == NULL )
        return 1;
    if ( !If_ManParCheckRequiredStandard( p, &pReason ) )
    {
        Abc_Print( -1, "IF_PAR_INVARIANT reason=required_%s\n", pReason ? pReason : "unsupported" );
        return 0;
    }
    for ( Level = p->pParMeta->nLevels - 1; Level >= 0; Level-- )
        if ( !If_ManParRunRequiredLevelJob( pRuntime, Level, pPropTime, pReduceTime ) )
            return 0;
    return 1;
}

int If_ManParComputeRequiredStage9( If_Man_t * p, If_ParRequiredSource_t Source, int Mode, int * pfHandled, int * pfSkipped )
{
    If_ParRuntime_t * pRuntime = p ? p->pParRuntime : NULL;
    char * pReason = NULL;
    int fMode0Rebuild = 0;
    if ( pfHandled )
        *pfHandled = 0;
    if ( pfSkipped )
        *pfSkipped = 0;
    if ( p == NULL || pRuntime == NULL || p->pParMeta == NULL )
    {
        Abc_Print( -1, "IF_PAR_INVARIANT reason=required_runtime_missing source=%s\n",
            If_ManParRequiredSourceName(Source) );
        return 0;
    }
    if ( !If_ManParCheckRequiredStandard( p, &pReason ) )
    {
        Abc_Print( -1, "IF_PAR_INVARIANT reason=required_unsupported_after_parallel source=%s detail=%s\n",
            If_ManParRequiredSourceName(Source), pReason ? pReason : "unsupported" );
        return 0;
    }
    if ( Source == IF_PAR_REQUIRED_SOURCE_IMPROVE )
    {
        if ( pRuntime->nImproveAccepted == 0 )
        {
            if ( pfHandled )
                *pfHandled = 1;
            if ( pfSkipped )
                *pfSkipped = 1;
            p->Stage0Time.required_skip_count++;
            p->Stage0Time.required_skip_noop_improve_count++;
            if ( p->pPars->fVerbose )
                Abc_Print( 1, "IF_PAR_REQUIRED_SKIP source=improve reason=accepted_roots_zero accepted_roots=0\n" );
            return 1;
        }
        Mode = 7;
    }
    else if ( Source == IF_PAR_REQUIRED_SOURCE_ROUND )
    {
        if ( Mode == 0 )
            fMode0Rebuild = 1;
        else if ( Mode != 1 && Mode != 2 )
        {
            Abc_Print( -1, "IF_PAR_INVARIANT reason=required_invalid_round_mode mode=%d\n", Mode );
            return 0;
        }
    }
    else
    {
        Abc_Print( -1, "IF_PAR_INVARIANT reason=required_invalid_source source=%d\n", (int)Source );
        return 0;
    }
    if ( pfHandled )
        *pfHandled = 1;
    return If_ManParComputeRequiredStage9Fast( p, Mode, fMode0Rebuild );
}

int If_ManParRunMappingRound( If_Man_t * p, int Mode, int fPreprocess, int fFirst, char * pLabel, int * pfHandled )
{
    if ( pfHandled )
        *pfHandled = 0;
    if ( p->pParRuntime == NULL || p->pParMeta == NULL )
        return 1;
    assert( p->pPars->fParMap );
    if ( Mode == 0 )
    {
        if ( pfHandled )
            *pfHandled = 1;
        return If_ManParRunMode0Round( p, fPreprocess, fFirst, pLabel );
    }
    if ( Mode == 1 )
    {
        if ( pfHandled )
            *pfHandled = 1;
        return If_ManParRunMode1Round( p, fPreprocess, fFirst, pLabel );
    }
    if ( Mode == 2 )
    {
        if ( pfHandled )
            *pfHandled = 1;
        return If_ManParRunMode2Round( p, fPreprocess, fFirst, pLabel );
    }
    return If_ManParRunRoundSkeleton( p, Mode, fPreprocess, fFirst, pLabel );
}

////////////////////////////////////////////////////////////////////////
///                       END OF FILE                                ///
////////////////////////////////////////////////////////////////////////

ABC_NAMESPACE_IMPL_END
