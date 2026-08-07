/**CFile****************************************************************

  FileName    [ifCore.c]

  SystemName  [ABC: Logic synthesis and verification system.]

  PackageName [FPGA mapping based on priority cuts.]

  Synopsis    [The central part of the mapper.]

  Author      [Alan Mishchenko]
  
  Affiliation [UC Berkeley]

  Date        [Ver. 1.0. Started - November 21, 2006.]

  Revision    [$Id: ifCore.c,v 1.00 2006/11/21 00:00:00 alanmi Exp $]

***********************************************************************/

#include "if.h"

ABC_NAMESPACE_IMPL_START


////////////////////////////////////////////////////////////////////////
///                        DECLARATIONS                              ///
////////////////////////////////////////////////////////////////////////

extern abctime s_MappingTime;

////////////////////////////////////////////////////////////////////////
///                     FUNCTION DEFINITIONS                         ///
////////////////////////////////////////////////////////////////////////

/**Function*************************************************************

  Synopsis    []

  Description []
               
  SideEffects []

  SeeAlso     []

***********************************************************************/
void If_ManSetDefaultPars( If_Par_t * pPars )
{
    memset( pPars, 0, sizeof(If_Par_t) ); //把参数结构体的所有字段初始化为 0；如果后续没有特别设置的话，这些字段的默认值就是 0 或 NULL。
    pPars->nLutSize    = -1;
    pPars->nCutsMax    =  8;
    pPars->nFlowIters  =  1;
    pPars->nAreaIters  =  2;
    pPars->DelayTarget = -1;
    pPars->Epsilon     =  (float)0.005;
    pPars->fPreprocess =  1;
    pPars->fArea       =  0;
    pPars->fFancy      =  0;
    pPars->fExpRed     =  1;
    pPars->fLatchPaths =  0;
    pPars->fEdge       =  1;
    pPars->fPower      =  0;
    pPars->fCutMin     =  0;
    pPars->fBidec      =  0;
    pPars->fUserLutDec =  0;
    pPars->fUserLut2D  =  0;
    pPars->nParThreads =  1;
    pPars->fVerbose    =  0;
}


/**Function*************************************************************

  Synopsis    []

  Description []
               
  SideEffects []

  SeeAlso     []

***********************************************************************/
int If_ManPerformMapping( If_Man_t * p )
{
    //fArea 是当前轮次的临时开关，fAreaOnly 是全流程的初始策略快照。
    p->pPars->fAreaOnly = p->pPars->fArea; // temporary，保存用户最初是否选择 area-only 模式”，避免后面流程里临时改写 fArea 影响全局语义
    // 一次完整 mapping 的一次性准备：
    // 1) 给 CI 建 trivial cut
    // 2) 按 cross-cut 规模分配 cutset 池
    // 3) 预建反向拓扑序，供 required-time 传播使用
    // create the CI cutsets
    If_ManSetupCiCutSets( p );
    // allocate memory for other cutsets
    If_ManSetupSetAll( p, If_ManCrossCut(p) ); //先估算一次映射过程中的峰值活跃 cutset 数量，再按这个峰值一次性分配 cutset 内存池
    // derive reverse top order
    p->vObjsRev = If_ManReverseOrder( p );
    return If_ManPerformMappingComb( p );
}


/**Function*************************************************************

  Synopsis    []

  Description []
               
  SideEffects []

  SeeAlso     []

***********************************************************************/
int If_ManPerformMappingComb( If_Man_t * p )
{
    If_Obj_t * pObj;
    abctime clkTotal = Abc_Clock();
    int i;

    If_ManStage0TimeReset( p );

    //p->vVisited2 = Vec_IntAlloc( 100 );
    //p->vMarks = Vec_StrStart( If_ManObjNum(p) );

    // set arrival times and fanout estimates
    If_ManForEachCi( p, pObj, i )
    {
        If_ObjSetArrTime( pObj, p->pPars->pTimesArr ? p->pPars->pTimesArr[i] : (float)0.0 );
        pObj->EstRefs = (float)1.0; //给每个 CI 设置 EstRefs 初值为 1.0（表示每个 CI 未来至少会被用到一次），后续在建图过程中根据 fanout 数量动态调整这个值，供 cut 选择时参考。
    }

    // 先跑 delay/preprocess 轮，建立一个可行的初始 mapping。
    // delay oriented mapping
    if ( p->pPars->fPreprocess && !p->pPars->fArea )
    {
        // map for delay
        if ( !If_ManPerformMappingRound( p, p->pPars->nCutsMax, 0, 1, 1, "Delay" ) )
            return 0;

        // map for delay second option
        p->pPars->fFancy = 1;
        If_ManResetOriginalRefs( p ); //按照aig结构重置引用计数nrefs，为下一轮做准备
        if ( !If_ManPerformMappingRound( p, p->pPars->nCutsMax, 0, 1, 0, "Delay-2" ) )
        {
            p->pPars->fFancy = 0;
            return 0;
        }
        p->pPars->fFancy = 0;
        // map for area
        p->pPars->fArea = 1;
        If_ManResetOriginalRefs( p );
        if ( !If_ManPerformMappingRound( p, p->pPars->nCutsMax, 0, 1, 0, "Area" ) )
        {
            p->pPars->fArea = 0;
            return 0;
        }
        p->pPars->fArea = 0;
    }
    else
        if ( !If_ManPerformMappingRound( p, p->pPars->nCutsMax, 0, 0, 1, "Delay" ) )
            return 0;

    // 在已有 mapping 上做一次 expand/reduce，尝试先把面积往下压。
    // try to improve area by expanding and reducing the cuts
    if ( p->pPars->fExpRed )
        if ( !If_ManImproveMapping( p ) )
            return 0;

    // 接着跑 area-flow 轮；这些轮次会反复用新的 required time 重新选 cut。
    // area flow oriented mapping
    for ( i = 0; i < p->pPars->nFlowIters; i++ )
    {
        if ( !If_ManPerformMappingRound( p, p->pPars->nCutsMax, 1, 0, 0, "Flow" ) )
            return 0;
        if ( p->pPars->fExpRed )
            if ( !If_ManImproveMapping( p ) )
                return 0;
    }

    // 最后跑 exact-area 轮，把 CutBest 尽量收敛到面积更小的方案。
    // area oriented mapping
    for ( i = 0; i < p->pPars->nAreaIters; i++ )
    {
        if ( p->pPars->fDumpFile && p->pPars->nLutSize <= 6 && i == p->pPars->nAreaIters-1 ) {
            //为导出 cut 数据做一次性缓冲区准备，只在最后一轮 area 迭代触发，fdumpfile默认为0，即不导出。
            p->vCuts = Vec_IntAlloc( 1 << 20 );
            p->vCutCosts = Vec_IntAlloc( 1 << 16 );
        }
        if ( !If_ManPerformMappingRound( p, p->pPars->nCutsMax, 2, 0, 0, "Area" ) )
            return 0;
        if ( p->pPars->fExpRed )
            if ( !If_ManImproveMapping( p ) )
                return 0;
    }

    p->Stage0Time.total_mapping_time = Abc_Clock() - clkTotal;
    p->Stage0Time.abc_reported_total_time = p->Stage0Time.total_mapping_time;
    if ( p->pPars->fVerbose )
    {
//        Abc_Print( 1, "Total memory = %7.2f MB. Peak cut memory = %7.2f MB.  ", 
//            1.0 * (p->nObjBytes + 2*sizeof(void *)) * If_ManObjNum(p) / (1<<20), 
//            1.0 * p->nSetBytes * Mem_FixedReadMaxEntriesUsed(p->pMemSet) / (1<<20) );
        Abc_PrintTime( 1, "Total time", p->Stage0Time.abc_reported_total_time );
        If_ManStage0TimePrintSummary( p );
    }
//    Abc_Print( 1, "Cross cut memory = %d.\n", Mem_FixedReadMaxEntriesUsed(p->pMemSet) );
    s_MappingTime = p->Stage0Time.total_mapping_time;
//    Abc_Print( 1, "Special POs = %d.\n", If_ManCountSpecialPos(p) );

/*
    {
        static char * pLastName = NULL;
        FILE * pTable = fopen( "fpga/ucsb/stats.txt", "a+" );
        if ( pLastName == NULL || strcmp(pLastName, p->pName) )
        {
            fprintf( pTable, "\n" );
            fprintf( pTable, "%s ", p->pName );

            fprintf( pTable, "%d ", If_ManCiNum(p) );
            fprintf( pTable, "%d ", If_ManCoNum(p) );
            fprintf( pTable, "%d ", If_ManAndNum(p) );

            ABC_FREE( pLastName );
            pLastName = Abc_UtilStrsav( p->pName );
        }

        fprintf( pTable, "%d ", (int)p->AreaGlo );
        fprintf( pTable, "%d ", (int)p->RequiredGlo );
        fclose( pTable );
    }
*/
    p->pPars->FinalDelay = p->RequiredGlo;
    p->pPars->FinalArea  = p->AreaGlo;
    return 1;
}

////////////////////////////////////////////////////////////////////////
///                       END OF FILE                                ///
////////////////////////////////////////////////////////////////////////


ABC_NAMESPACE_IMPL_END
