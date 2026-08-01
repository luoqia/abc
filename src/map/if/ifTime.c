/**CFile****************************************************************

  FileName    [ifTime.c]

  SystemName  [ABC: Logic synthesis and verification system.]

  PackageName [FPGA mapping based on priority cuts.]

  Synopsis    [Computation of delay paramters depending on the library.]

  Author      [Alan Mishchenko]
  
  Affiliation [UC Berkeley]

  Date        [Ver. 1.0. Started - November 21, 2006.]

  Revision    [$Id: ifTime.c,v 1.00 2006/11/21 00:00:00 alanmi Exp $]

***********************************************************************/

#include "if.h"

ABC_NAMESPACE_IMPL_START

////////////////////////////////////////////////////////////////////////
///                        DECLARATIONS                              ///
////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////
///                     FUNCTION DEFINITIONS                         ///
////////////////////////////////////////////////////////////////////////

/**Function*************************************************************

  Synopsis    [Sorts the pins in the decreasing order of delays.]

  Description []
               
  SideEffects []

  SeeAlso     []

***********************************************************************/
void If_CutSortInputPins( If_Man_t * p, If_Cut_t * pCut, int * pPinPerm, float * pPinDelays )
{
    If_Obj_t * pLeaf;
    int i, j, best_i, temp;
    // start the trivial permutation and collect pin delays
    If_CutForEachLeaf( p, pCut, pLeaf, i )
    {
        pPinPerm[i] = i;
        pPinDelays[i] = If_ObjCutBest(pLeaf)->Delay;
    }
    // selection sort the pins in the decreasible order of delays
    // this order will match the increasing order of LUT input pins
    for ( i = 0; i < (int)pCut->nLeaves-1; i++ )
    {
        best_i = i;
        for ( j = i+1; j < (int)pCut->nLeaves; j++ )
            if ( pPinDelays[pPinPerm[j]] > pPinDelays[pPinPerm[best_i]] )
                best_i = j;
        if ( best_i == i )
            continue;
        temp = pPinPerm[i]; 
        pPinPerm[i] = pPinPerm[best_i]; 
        pPinPerm[best_i] = temp;
    }
/*
    // verify
    assert( pPinPerm[0] < (int)pCut->nLeaves );
    for ( i = 1; i < (int)pCut->nLeaves; i++ )
    {
        assert( pPinPerm[i] < (int)pCut->nLeaves );
        assert( pPinDelays[pPinPerm[i-1]] >= pPinDelays[pPinPerm[i]] );
    }
*/
}


/**Function*************************************************************

  Synopsis    [Computes delay.]

  Description []
               
  SideEffects []

  SeeAlso     []

***********************************************************************/
float If_CutDelay( If_Man_t * p, If_Obj_t * pObj, If_Cut_t * pCut )
{
    static int pPinPerm[IF_MAX_LUTSIZE];
    static float pPinDelays[IF_MAX_LUTSIZE];
    char * pPerm = If_CutPerm( pCut );
    If_Obj_t * pLeaf;
    float Delay, DelayCur;
    float * pLutDelays;
    int i, Shift, Pin2PinDelay;//, iLeaf;
    Delay = -IF_FLOAT_LARGE;
    if ( pCut->fAndCut )
    {
        If_CutForEachLeaf( p, pCut, pLeaf, i )
        {
            DelayCur = If_ObjCutBest(pLeaf)->Delay + p->pPars->nAndDelay;
            Delay = IF_MAX( Delay, DelayCur );
        }
    }
    else if ( p->pPars->pLutLib )
    {
        assert( !p->pPars->fLiftLeaves );
        pLutDelays = p->pPars->pLutLib->pLutDelays[pCut->nLeaves];
        if ( p->pPars->pLutLib->fVarPinDelays )
        {
            // compute the delay using sorted pins
            If_CutSortInputPins( p, pCut, pPinPerm, pPinDelays );
            for ( i = 0; i < (int)pCut->nLeaves; i++ )
            {
                DelayCur = pPinDelays[pPinPerm[i]] + pLutDelays[i];
                Delay = IF_MAX( Delay, DelayCur );
            }
        }
        else
        {
            If_CutForEachLeaf( p, pCut, pLeaf, i )
            {
                DelayCur = If_ObjCutBest(pLeaf)->Delay + pLutDelays[0];
                Delay = IF_MAX( Delay, DelayCur );
            }
        }
    }
    else
    {
        if ( p->pPars->fEnableCheck07 && p->pPars->fDelayOptCell && p->pPars->pCellLib )
        {
            int Intrinsic[IF_MAX_LUTSIZE];
            if ( pCut->nLeaves == 0 )
                return 0.0;
            assert( pCut->nLeaves == 1 || pCut->Config != 0 );
            If_CutComputeIntrinsicJ( p, pCut->Config, pCut->nLeaves, Intrinsic );
            If_CutForEachLeaf( p, pCut, pLeaf, i )
            {
                DelayCur = If_ObjCutBest(pLeaf)->Delay + (float)Intrinsic[i];
                Delay = IF_MAX( Delay, DelayCur );
            }
            return Delay;
        }
        if ( pCut->fUser )
        {
            assert( !p->pPars->fLiftLeaves );
            If_CutForEachLeaf( p, pCut, pLeaf, i )
            {
                Pin2PinDelay = pPerm ? (pPerm[i] == IF_BIG_CHAR ? -IF_BIG_CHAR : pPerm[i]) : 1;
                DelayCur = If_ObjCutBest(pLeaf)->Delay + (float)Pin2PinDelay;
                Delay = IF_MAX( Delay, DelayCur );
            }
        }
        else
        {
            if ( p->pPars->fLiftLeaves )
            {
                If_CutForEachLeafSeq( p, pCut, pLeaf, Shift, i )
                {
                    DelayCur = If_ObjCutBest(pLeaf)->Delay - Shift * p->Period;
                    Delay = IF_MAX( Delay, DelayCur + 1.0 );
                }
            }
            else
            {
                If_CutForEachLeaf( p, pCut, pLeaf, i )
                {
                    DelayCur = If_ObjCutBest(pLeaf)->Delay + 1.0;
                    Delay = IF_MAX( Delay, DelayCur );
                }
            }
        }
    }
    return Delay;
}

/**Function*************************************************************

  Synopsis    []

  Description []
               
  SideEffects []

  SeeAlso     []

***********************************************************************/
void If_CutPropagateRequired( If_Man_t * p, If_Obj_t * pObj, If_Cut_t * pCut, float ObjRequired )
{
    static int pPinPerm[IF_MAX_LUTSIZE];
    static float pPinDelays[IF_MAX_LUTSIZE];
    If_Obj_t * pLeaf;
    float * pLutDelays;
    float Required;
    int i, Pin2PinDelay;//, iLeaf;
    assert( !p->pPars->fLiftLeaves );
    // compute the pins
    if ( pCut->fAndCut )
    {
        If_CutForEachLeaf( p, pCut, pLeaf, i )
            pLeaf->Required = IF_MIN( pLeaf->Required, ObjRequired - p->pPars->nAndDelay );
    }
    else if ( p->pPars->pLutLib )
    {
        pLutDelays = p->pPars->pLutLib->pLutDelays[pCut->nLeaves];
        if ( p->pPars->pLutLib->fVarPinDelays )
        {
            // compute the delay using sorted pins
            If_CutSortInputPins( p, pCut, pPinPerm, pPinDelays );
            for ( i = 0; i < (int)pCut->nLeaves; i++ )
            {
                Required = ObjRequired - pLutDelays[i];
                pLeaf = If_ManObj( p, pCut->pLeaves[pPinPerm[i]] );
                pLeaf->Required = IF_MIN( pLeaf->Required, Required );
            }
        }
        else
        {
            Required = ObjRequired;
            If_CutForEachLeaf( p, pCut, pLeaf, i )
                pLeaf->Required = IF_MIN( pLeaf->Required, Required - pLutDelays[0] );
        }
    }
    else if ( p->pPars->fUserLutDec || p->pPars->fUserLut2D )
    {
        Required = ObjRequired;
        If_CutForEachLeaf( p, pCut, pLeaf, i )
            pLeaf->Required = IF_MIN( pLeaf->Required, Required - If_LutDecPinRequired( p, pCut, i, ObjRequired ) );
    }
    else
    {
        if ( p->pPars->fEnableCheck07 && p->pPars->fDelayOptCell && p->pPars->pCellLib )
        {
            int Intrinsic[IF_MAX_LUTSIZE];
            if ( pCut->nLeaves == 0 )
                return;
            assert( (pObj && pObj->Id == 0) || pCut->nLeaves == 1 || pCut->Config != 0 );
            If_CutComputeIntrinsicJ( p, pCut->Config, pCut->nLeaves, Intrinsic );
            If_CutForEachLeaf( p, pCut, pLeaf, i )
                pLeaf->Required = IF_MIN( pLeaf->Required, ObjRequired - (float)Intrinsic[i] );
            return;
        }
        if ( pCut->fUser )
        {
            char Perm[IF_MAX_FUNC_LUTSIZE], * pPerm = Perm;
            if ( p->pPars->fDelayOpt )
            {
                int Delay = If_CutSopBalancePinDelays( p, pCut, pPerm );
                assert( -Delay > IF_INFINITY/2 || Delay > IF_INFINITY/2 || Delay == (int)pCut->Delay );
            }
            else if ( p->pPars->fDelayOptLut )
            {
                int Delay = If_CutLutBalancePinDelays( p, pCut, pPerm );
                assert( -Delay > IF_INFINITY/2 || Delay > IF_INFINITY/2 || Delay == (int)pCut->Delay );
            }
            else if ( p->pPars->fDsdBalance )
            {
                int Delay = If_CutDsdBalancePinDelays( p, pCut, pPerm );
                assert( -Delay > IF_INFINITY/2 || Delay > IF_INFINITY/2 || Delay == (int)pCut->Delay );
            }
            else
                pPerm = If_CutPerm(pCut);
            If_CutForEachLeaf( p, pCut, pLeaf, i )
            {
                Pin2PinDelay = pPerm ? (pPerm[i] == IF_BIG_CHAR ? -IF_BIG_CHAR : pPerm[i]) : 1;
                Required = ObjRequired - (float)Pin2PinDelay;
                pLeaf->Required = IF_MIN( pLeaf->Required, Required );
            }
        }
        else
        {
            Required = ObjRequired;
            If_CutForEachLeaf( p, pCut, pLeaf, i )
                pLeaf->Required = IF_MIN( pLeaf->Required, Required - (float)1.0 );
        }
    }
}

/**Function*************************************************************

  Synopsis    [Returns the max delay of the POs.]

  Description []
               
  SideEffects []

  SeeAlso     []

***********************************************************************/
float If_ManDelayMax( If_Man_t * p, int fSeq )
{
    If_Obj_t * pObj;
    float DelayBest;
    int i;
    if ( p->pPars->fLatchPaths && (p->pPars->nLatchesCi == 0 || p->pPars->nLatchesCo == 0) )
    {
        Abc_Print( 0, "Delay optimization of latch path is not performed because there is no latches.\n" );
        p->pPars->fLatchPaths = 0;
    }
    DelayBest = -IF_FLOAT_LARGE;
    if ( fSeq )
    {
        assert( p->pPars->nLatchesCi > 0 );
        If_ManForEachPo( p, pObj, i )
            if ( DelayBest < If_ObjArrTime(If_ObjFanin0(pObj)) )
                 DelayBest = If_ObjArrTime(If_ObjFanin0(pObj));
    }
    else if ( p->pPars->fLatchPaths )
    {
        If_ManForEachLatchInput( p, pObj, i )
            if ( DelayBest < If_ObjArrTime(If_ObjFanin0(pObj)) )
                 DelayBest = If_ObjArrTime(If_ObjFanin0(pObj));
    }
    else 
    {
        If_ManForEachCo( p, pObj, i )
            if ( DelayBest < If_ObjArrTime(If_ObjFanin0(pObj)) )
                 DelayBest = If_ObjArrTime(If_ObjFanin0(pObj));
    }
    return DelayBest;
}

/**Function*************************************************************

  Synopsis    [Computes the required times of all nodes.]

  Description []
               
  SideEffects []

  SeeAlso     []

***********************************************************************/
static int If_ManComputeRequiredInitDefault( If_Man_t * p )
{
    If_Obj_t * pObj;
    int i, Counter;

    // get the global required times
    p->RequiredGlo = If_ManDelayMax( p, 0 );

    // consider the case when the required times are given
    if ( p->pPars->pTimesReq && !p->pPars->fAreaOnly )
    {
        // make sure that the required time hold
        //用户给了 pTimesReq
        Counter = 0;
        If_ManForEachCo( p, pObj, i )
        {
            //对每个 CO，如果 arrival > 用户要求的 required，则用 arrival 兜底（不能倒退时间）；否则用用户给的值。
            if ( If_ObjArrTime(If_ObjFanin0(pObj)) > p->pPars->pTimesReq[i] + p->fEpsilon )
            {
                If_ObjFanin0(pObj)->Required = If_ObjArrTime(If_ObjFanin0(pObj));
                Counter++;
    //                Abc_Print( 0, "Required times are violated for output %d (arr = %d; req = %d).\n", 
    //                    i, (int)If_ObjArrTime(If_ObjFanin0(pObj)), (int)p->pPars->pTimesReq[i] );
            }
            else
                If_ObjFanin0(pObj)->Required = p->pPars->pTimesReq[i];
        }
        if ( Counter && !p->fReqTimeWarn )
        {
            Abc_Print( 0, "Required times are exceeded at %d output%s. The earliest arrival times are used.\n", Counter, Counter > 1 ? "s":"" );
            p->fReqTimeWarn = 1;
        }
        return 1;
    }

    // find new delay target，找到一个新的target required time
    if ( p->pPars->nRelaxRatio && p->pPars->DelayTargetNew == 0 )
        p->pPars->DelayTargetNew = p->RequiredGlo * (100.0 + p->pPars->nRelaxRatio) / 100.0;

    // update the required times according to the target，根据target更新required times
    // 如果用户设了 DelayTarget
    if ( p->pPars->DelayTarget != -1 )
    {
        if ( p->RequiredGlo > p->pPars->DelayTarget + p->fEpsilon )
        {   //如果设了 DelayTarget 但电路做不到 → 警告但继续
            if ( p->fNextRound == 0 )
            {
                p->fNextRound = 1;
                Abc_Print( 0, "Cannot meet the target required times (%4.2f). Mapping continues anyway.\n", p->pPars->DelayTarget );
            }
        }
        else if ( p->RequiredGlo < p->pPars->DelayTarget - p->fEpsilon )
        {   //如果电路比目标更快 → RequiredGlo 抬高到目标值（给 area 优化留松弛空间）
            if ( p->fNextRound == 0 )
            {
                p->fNextRound = 1;
//                        Abc_Print( 0, "Relaxing the required times from (%4.2f) to the target (%4.2f).\n", p->RequiredGlo, p->pPars->DelayTarget );
            }
            p->RequiredGlo = p->pPars->DelayTarget;
        }
    }
    else if ( p->pPars->DelayTargetNew > 0 ) // relax the required times
        p->RequiredGlo = p->pPars->DelayTargetNew;

    // do not propagate required times if area minimization is requested
    if ( p->pPars->fAreaOnly )
        return 0;

    // set the required times for the POs
    if ( p->pPars->fDoAverage ) //特殊模式 fDoAverage：每个 CO 用自己的 arrival time 作 required（各输出独立松弛）。
    {
        if ( p->pPars->nRelaxRatio )
        {
            If_ManForEachCo( p, pObj, i )
                If_ObjFanin0(pObj)->Required = If_ObjArrTime(If_ObjFanin0(pObj)) * (100.0 + p->pPars->nRelaxRatio) / 100.0;
        }
        else
        {
            If_ManForEachCo( p, pObj, i )
                If_ObjFanin0(pObj)->Required = If_ObjArrTime(If_ObjFanin0(pObj));
        }
    }
    else if ( p->pPars->fLatchPaths )   //特殊模式 fLatchPaths：只松弛 latch input，PO 设 IF_FLOAT_LARGE
    {
        If_ManForEachLatchInput( p, pObj, i )
            If_ObjFanin0(pObj)->Required = p->RequiredGlo;
    }
    else
    {
        // 标准模式：所有 CO 的 required = RequiredGlo
        If_ManForEachCo( p, pObj, i )
            If_ObjFanin0(pObj)->Required = p->RequiredGlo;
    }
    return 1;
}

static void If_ManComputeRequiredPropagateDefault( If_Man_t * p )
{
    If_Obj_t * pObj;
    int i;
    // go through the nodes in the reverse topological order，反向拓扑传播
//    Vec_PtrForEachEntry( If_Obj_t *, p->vMapped, pObj, i )
//        If_CutPropagateRequired( p, pObj, If_ObjCutBest(pObj), pObj->Required );
    If_ManForEachObjReverse( p, pObj, i )
    {
        if ( pObj->nRefs == 0 )
            continue; //只传播被映射用到的节点
        If_CutPropagateRequired( p, pObj, If_ObjCutBest(pObj), pObj->Required );
    }
}

static void If_ManComputeRequiredPropagateTerminals( If_Man_t * p )
{
    If_Obj_t * pObj;
    int i;
    If_ManForEachObjReverse( p, pObj, i )
    {
        if ( pObj->nRefs == 0 || If_ObjIsAnd(pObj) )
            continue;
        If_CutPropagateRequired( p, pObj, If_ObjCutBest(pObj), pObj->Required );
    }
}

static int If_ManComputeRequiredInitTimed( If_Man_t * p )
{
    If_Obj_t * pObj;
    int i, Counter;
    float reqTime;

    // get the global required times
    p->RequiredGlo = If_ManDelayMax( p, 0 );

    // find new delay target
    if ( p->pPars->nRelaxRatio && p->pPars->DelayTargetNew == 0 )
        p->pPars->DelayTargetNew = p->RequiredGlo * (100.0 + p->pPars->nRelaxRatio) / 100.0;

    // update the required times according to the target
    if ( p->pPars->DelayTarget != -1 )
    {
        if ( p->RequiredGlo > p->pPars->DelayTarget + p->fEpsilon )
        {
            if ( p->fNextRound == 0 )
            {
                p->fNextRound = 1;
                Abc_Print( 0, "Cannot meet the target required times (%4.2f). Mapping continues anyway.\n", p->pPars->DelayTarget );
            }
        }
        else if ( p->RequiredGlo < p->pPars->DelayTarget - p->fEpsilon )
        {
            if ( p->fNextRound == 0 )
            {
                p->fNextRound = 1;
//                    Abc_Print( 0, "Relaxing the required times from (%4.2f) to the target (%4.2f).\n", p->RequiredGlo, p->pPars->DelayTarget );
            }
            p->RequiredGlo = p->pPars->DelayTarget;
        }
    }
    else if ( p->pPars->DelayTargetNew > 0 ) // relax the required times
        p->RequiredGlo = p->pPars->DelayTargetNew;

    // do not propagate required times if area minimization is requested
    if ( p->pPars->fAreaOnly )
        return 0;
    // set the required times for the POs
    Tim_ManIncrementTravId( p->pManTim );
    if ( p->pPars->pTimesReq )
    {
        Counter = 0;
        If_ManForEachCo( p, pObj, i )
        {
            reqTime = p->pPars->pTimesReq[i];
            if ( If_ObjArrTime(If_ObjFanin0(pObj)) > reqTime + p->fEpsilon )
            {
                reqTime = If_ObjArrTime(If_ObjFanin0(pObj));
                Counter++;
            }
            Tim_ManSetCoRequired( p->pManTim, i, reqTime );
        }
        if ( Counter && !p->fReqTimeWarn )
        {
            Abc_Print( 0, "Required times are exceeded at %d output%s. The earliest arrival times are used.\n", Counter, Counter > 1 ? "s":"" );
            p->fReqTimeWarn = 1;
        }
    }
    else if ( p->vCoAttrs )
    {
        assert( If_ManCoNum(p) == Vec_IntSize(p->vCoAttrs) );
        If_ManForEachCo( p, pObj, i )
        {
            if ( Vec_IntEntry(p->vCoAttrs, i) == -1 )       // -1=internal
                continue;
            if ( Vec_IntEntry(p->vCoAttrs, i) == 0 )        //  0=optimize
                Tim_ManSetCoRequired( p->pManTim, i, p->RequiredGlo );
            else if ( Vec_IntEntry(p->vCoAttrs, i) == 1 )   //  1=keep
                Tim_ManSetCoRequired( p->pManTim, i, If_ObjArrTime(If_ObjFanin0(pObj)) );
            else if ( Vec_IntEntry(p->vCoAttrs, i) == 2 )   //  2=relax
                Tim_ManSetCoRequired( p->pManTim, i, IF_FLOAT_LARGE );
            else assert( 0 );
        }
    }
    else if ( p->pPars->fDoAverage )
    {
        if ( p->pPars->nRelaxRatio )
        {
            If_ManForEachCo( p, pObj, i )
                Tim_ManSetCoRequired( p->pManTim, i, If_ObjArrTime(If_ObjFanin0(pObj)) * (100.0 + p->pPars->nRelaxRatio) / 100.0 );
        }
        else
        {
            If_ManForEachCo( p, pObj, i )
                Tim_ManSetCoRequired( p->pManTim, i, If_ObjArrTime(If_ObjFanin0(pObj)) );
        }
    }
    else if ( p->pPars->fLatchPaths )
    {
        If_ManForEachPo( p, pObj, i )
            Tim_ManSetCoRequired( p->pManTim, i, IF_FLOAT_LARGE );
        If_ManForEachLatchInput( p, pObj, i )
            Tim_ManSetCoRequired( p->pManTim, i, p->RequiredGlo );
    }
    else
    {
        Tim_ManInitPoRequiredAll( p->pManTim, p->RequiredGlo );
//            If_ManForEachCo( p, pObj, i )
//                Tim_ManSetCoRequired( p->pManTim, pObj->IdPio, p->RequiredGlo );
    }
    return 1;
}

static void If_ManComputeRequiredPropagateTimed( If_Man_t * p )
{
    If_Obj_t * pObj;
    float reqTime;
    int i;
    // go through the nodes in the reverse topological order
    If_ManForEachObjReverse( p, pObj, i )
    {
        if ( If_ObjIsAnd(pObj) )
        {
            if ( pObj->nRefs == 0 )
                continue;
            If_CutPropagateRequired( p, pObj, If_ObjCutBest(pObj), pObj->Required );
        }
        else if ( If_ObjIsCi(pObj) )
        {
            reqTime = pObj->Required;
            Tim_ManSetCiRequired( p->pManTim, pObj->IdPio, reqTime );
        }
        else if ( If_ObjIsCo(pObj) )
        {
            reqTime = Tim_ManGetCoRequired( p->pManTim, pObj->IdPio );
            If_ObjFanin0(pObj)->Required = IF_MIN( reqTime, If_ObjFanin0(pObj)->Required );
        }
        else if ( If_ObjIsConst1(pObj) )
        {
        }
        else // add the node to the mapper
            assert( 0 );
    }
}

int If_ManComputeRequired( If_Man_t * p )
{
    abctime clk;
    int fPropagate;

    // 这一层做三件事：
    // 1) 先根据当前 CutBest 标记实际 mapping，更新nrefs，并统计全局面积/边数/功耗
    // 2) 计算当前轮的全局 required time
    // 3) 再按反向拓扑序把 required 传播回每个已使用节点
    // compute area, clean required times, collect nodes used in the mapping
//    p->AreaGlo = If_ManScanMapping( p );
    clk = Abc_Clock();
    If_ManMarkMapping( p );
    p->Stage0Time.required_rebuild_time += Abc_Clock() - clk;

    if ( p->pManTim == NULL )
    {
        clk = Abc_Clock();
        fPropagate = If_ManComputeRequiredInitDefault( p );
        p->Stage0Time.required_init_time += Abc_Clock() - clk;
        if ( !fPropagate )
            return 1;
        clk = Abc_Clock();
        If_ManComputeRequiredPropagateDefault( p );
        p->Stage0Time.required_prop_time += Abc_Clock() - clk;
        return 1;
    }

    clk = Abc_Clock();
    fPropagate = If_ManComputeRequiredInitTimed( p );
    p->Stage0Time.required_init_time += Abc_Clock() - clk;
    if ( !fPropagate )
        return 1;
    clk = Abc_Clock();
    If_ManComputeRequiredPropagateTimed( p );
    p->Stage0Time.required_prop_time += Abc_Clock() - clk;
    return 1;
}

////////////////////////////////////////////////////////////////////////
///                       END OF FILE                                ///
////////////////////////////////////////////////////////////////////////


ABC_NAMESPACE_IMPL_END
