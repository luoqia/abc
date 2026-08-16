/**CFile****************************************************************

  FileName    [dch2Man.c]

  SystemName  [ABC: Logic synthesis and verification system.]

  PackageName [Task 42 Stage 6: independent deterministic DCH2 engine.]

  Synopsis    [Deterministic windowed choice construction.

  Pipeline: stable AIG/class/cone analysis -> deterministic overlapping
  windows with bounded halos -> worker-private local simulation/SAT state
  -> immutable candidate equivalence records -> global-context SAT
  verification -> stable conflict resolution (union-find, level-acyclic)
  -> deterministic choice construction (pEquivs).

  Workers never write shared managers, traversal IDs, random state, SAT
  maps, or the network. Fixed seed, stable window ids, stable merge order;
  deterministic for a fixed worker/window configuration.]

***********************************************************************/

#include "new/dch2/dch2.h"
#include "sat/bsat/satSolver.h"
#include "aig/gia/gia.h"
#include "aig/gia/giaAig.h"
#include "misc/util/utilTruth.h"
#include <thread>
#include <vector>
#include <map>
#include <algorithm>
#include <functional>

ABC_NAMESPACE_IMPL_START

////////////////////////////////////////////////////////////////////////
///                     SIMULATION (fixed seed)                       ///
////////////////////////////////////////////////////////////////////////

// 4x64-bit deterministic simulation patterns. xoshiro256** (256-bit
// state): the pattern set must span the input space, so a 64-bit-state
// generator is not enough -- its output columns are GF(2)-linear in the
// seed and all patterns would live in a <=64-dim subspace, structurally
// missing the difference set of nearly-equal functions. pData holds word
// 0; pSig1/pSig2/pSig3 hold the rest.
static inline uint64_t Dch2_Rotl( uint64_t x, int k ) { return (x << k) | (x >> (64 - k)); }
static uint64_t Dch2_Rand( uint64_t s[4] )
{
	uint64_t Result = Dch2_Rotl( s[1] * 5, 7 ) * 9;
	uint64_t t = s[1] << 17;
	s[2] ^= s[0];
	s[3] ^= s[1];
	s[1] ^= s[2];
	s[0] ^= s[3];
	s[2] ^= t;
	s[3] = Dch2_Rotl( s[3], 45 );
	return Result;
}
static void Dch2_Seed( uint64_t s[4], uint64_t seed )
{
	for ( int i = 0; i < 4; i++ )
	{
		seed += 0x9E3779B97F4A7C15ULL;
		uint64_t z = seed;
		z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL;
		z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL;
		s[i] = z ^ (z >> 31);
	}
}
static void Dch2_ManSim( Aig_Man_t * p, const std::vector<Aig_Obj_t *> & vOrder,
		uint64_t * pSig0, uint64_t * pSig1, uint64_t * pSig2, uint64_t * pSig3 )
{
	Aig_Obj_t * pObj;
	int i;
	uint64_t s[4];
	Dch2_Seed( s, 1 );
	// const1 evaluates to all-ones on every pattern (it never appears as
	// a node fanin in a strashed AIG, but keep the base exact)
	pSig0[0] = ~0ULL; pSig1[0] = ~0ULL; pSig2[0] = ~0ULL; pSig3[0] = ~0ULL;
	i = 0;
	Aig_ManForEachCi( p, pObj, i )
	{
		uint64_t w[4];
		for ( int k = 0; k < 4; k++ )
			w[k] = Dch2_Rand( s );
		pSig0[pObj->Id] = w[0];
		pSig1[pObj->Id] = w[1];
		pSig2[pObj->Id] = w[2];
		pSig3[pObj->Id] = w[3];
	}
	for ( auto * pObj : vOrder )
	{
		Aig_Obj_t * pF0 = Aig_ObjFanin0(pObj), * pF1 = Aig_ObjFanin1(pObj);
		uint64_t a[4], b[4], o[4];
		a[0] = pSig0[pF0->Id]; a[1] = pSig1[pF0->Id];
		a[2] = pSig2[pF0->Id]; a[3] = pSig3[pF0->Id];
		b[0] = pSig0[pF1->Id]; b[1] = pSig1[pF1->Id];
		b[2] = pSig2[pF1->Id]; b[3] = pSig3[pF1->Id];
		for ( int k = 0; k < 4; k++ )
		{
			uint64_t x0 = Aig_ObjFaninC0(pObj) ? ~a[k] : a[k];
			uint64_t x1 = Aig_ObjFaninC1(pObj) ? ~b[k] : b[k];
			o[k] = x0 & x1;
		}
		pSig0[pObj->Id] = o[0];
		pSig1[pObj->Id] = o[1];
		pSig2[pObj->Id] = o[2];
		pSig3[pObj->Id] = o[3];
	}
}

static inline uint64_t Dch2_Sig0( uint64_t * pSig0, Aig_Obj_t * pObj ) { return pSig0[pObj->Id]; }
static inline uint64_t Dch2_Sig1( uint64_t * pSig1, Aig_Obj_t * pObj ) { return pSig1[pObj->Id]; }

////////////////////////////////////////////////////////////////////////
///                     SAT VERIFICATION                             ///
////////////////////////////////////////////////////////////////////////

typedef struct Dch2_WinRec_t_ Dch2_WinRec_t;
struct Dch2_WinRec_t_
{
	int iWin;
	int n1; // smaller id
	int n2; // larger id
	int fCompl; // 1 when the pair is complementary (n1 == ~n2)
};

// Batch verification: all candidate pairs of one window share one miter
// GIA and one circuit-SAT solver state. The copy array and the miter
// manager are per-thread and reused across windows.
typedef struct Dch2_VerifyCtx_t_ Dch2_VerifyCtx_t;
struct Dch2_VerifyCtx_t_
{
	Aig_Man_t * pMit;
	Aig_Obj_t ** pCopy;
	int nAlloc;
};

static void Dch2_VerifyCtxInit( Dch2_VerifyCtx_t * pCtx, Aig_Man_t * p, int nMax )
{
	pCtx->pMit = Aig_ManStart( 0 );
	pCtx->pCopy = ABC_ALLOC( Aig_Obj_t *, nMax );
	memset( pCtx->pCopy, 0, sizeof(Aig_Obj_t *) * nMax );
	pCtx->nAlloc = nMax;
	(void)p;
}

static void Dch2_DupConeRec2( Aig_Man_t * pNew, Aig_Man_t * p, Aig_Obj_t * pObj, Aig_Obj_t ** pCopy )
{
	if ( Aig_ObjIsNode(pObj) && pCopy[pObj->Id] == NULL )
	{
		Aig_Obj_t * pF0 = Aig_ObjFanin0(pObj);
		Aig_Obj_t * pF1 = Aig_ObjFanin1(pObj);
		Dch2_DupConeRec2( pNew, p, pF0, pCopy );
		Dch2_DupConeRec2( pNew, p, pF1, pCopy );
		pCopy[pObj->Id] = Aig_And( pNew,
				Aig_NotCond( pCopy[Aig_ObjId(pF0)], Aig_ObjFaninC0(pObj) ),
				Aig_NotCond( pCopy[Aig_ObjId(pF1)], Aig_ObjFaninC1(pObj) ) );
	}
}

// Verifies the candidate pairs of one window in a single batch: one GIA,
// one circuit-SAT call, per-output status. Returns the verified records.
static void Dch2_ManVerifyBatch( Aig_Man_t * p, Dch2_VerifyCtx_t * pCtx,
		const std::vector<Dch2_WinRec_t> & vCand, int nConfMax,
		std::vector<Dch2_WinRec_t> & vRecs, int iWin, int fVerbose )
{
	int nPairs = (int)vCand.size();
	if ( nPairs == 0 )
		return;
	Aig_Man_t * pMit = pCtx->pMit;
	Aig_Obj_t ** pCopy = pCtx->pCopy;
	// reset the miter manager and the copy mapping (per-window)
	Aig_ManStop( pMit );
	pMit = pCtx->pMit = Aig_ManStart( 0 );
	memset( pCopy, 0, sizeof(Aig_Obj_t *) * pCtx->nAlloc );
	pCopy[0] = Aig_ManConst1(pMit);
	Aig_Obj_t * pObj;
	int i;
	Aig_ManForEachCi( p, pObj, i )
		pCopy[pObj->Id] = Aig_ObjCreateCi( pMit );
	// dup all cones of all candidates (shared subgraphs are shared)
	for ( auto & Rec : vCand )
	{
		Aig_Obj_t * pN1 = Aig_ManObj( p, Rec.n1 );
		Aig_Obj_t * pN2 = Aig_ManObj( p, Rec.n2 );
		Dch2_DupConeRec2( pMit, p, Aig_Regular(pN1), pCopy );
		Dch2_DupConeRec2( pMit, p, Aig_Regular(pN2), pCopy );
	}
	// one XOR/XNOR miter output per pair
	for ( auto & Rec : vCand )
	{
		Aig_Obj_t * pN1 = Aig_ManObj( p, Rec.n1 );
		Aig_Obj_t * pN2 = Aig_ManObj( p, Rec.n2 );
		Aig_Obj_t * pL1 = Aig_NotCond( pCopy[Aig_ObjId(Aig_Regular(pN1))], Aig_IsComplement(pN1) );
		Aig_Obj_t * pL2 = Aig_NotCond( pCopy[Aig_ObjId(Aig_Regular(pN2))], Aig_IsComplement(pN2) );
		Aig_Obj_t * pX = Rec.fCompl ?
			Aig_Or( pMit, Aig_And( pMit, pL1, pL2 ), Aig_And( pMit, Aig_Not(pL1), Aig_Not(pL2) ) ) :
			Aig_Or( pMit, Aig_And( pMit, pL1, Aig_Not(pL2) ), Aig_And( pMit, Aig_Not(pL1), pL2 ) );
		Aig_ObjCreateCo( pMit, pX );
	}
	Aig_ManSetRegNum( pMit, 0 );
	Gia_Man_t * pGia = Gia_ManFromAig( pMit );
	Vec_Str_t * pStatus = NULL;
	Cbs_ManSolveMiter( pGia, nConfMax > 0 ? nConfMax : 10000000, &pStatus, 0 );
	for ( i = 0; i < nPairs; i++ )
		if ( Vec_StrSize(pStatus) > i && Vec_StrEntry(pStatus, i) == 1 )
		{
			Dch2_WinRec_t Rec = vCand[i];
			Rec.iWin = iWin;
			vRecs.push_back( Rec );
		}
	if ( fVerbose )
	{
		int nProved = 0, nRejected = 0, nUndecided = 0;
		for ( i = 0; i < nPairs && Vec_StrSize(pStatus) > i; i++ )
		{
			if ( Vec_StrEntry(pStatus, i) == 1 ) nProved++;
			else if ( Vec_StrEntry(pStatus, i) == 0 ) nRejected++;
			else nUndecided++;
		}
		printf( "DCH2-BATCH win=%d pairs=%d proved=%d rejected=%d undecided=%d\n",
				iWin, nPairs, nProved, nRejected, nUndecided );
		fflush(stdout);
	}
	Vec_StrFree( pStatus );
	Gia_ManStop( pGia );
}

////////////////////////////////////////////////////////////////////////
///                     CHOICE CONSTRUCTION                          ///
////////////////////////////////////////////////////////////////////////

// pReprs on the original manager: pReprs[node] = class root (smaller id).
static void Dch2_ManSetReprs( Aig_Man_t * p, const std::vector<std::pair<int,int>> & vMerges,
		const std::vector<std::pair<int,int>> & vSubsts )
{
	Aig_Obj_t * pObj;
	int i;
	p->pReprs = ABC_CALLOC( Aig_Obj_t *, Aig_ManObjNumMax(p) );
	Aig_ManForEachNode( p, pObj, i )
		p->pReprs[pObj->Id] = pObj;
	for ( auto & pr : vMerges )
		p->pReprs[pr.first] = Aig_ManObj( p, pr.second );
	// complementary classes: the complemented rep pointer encodes the phase
	for ( auto & pr : vSubsts )
		p->pReprs[pr.first] = Aig_Not( Aig_ManObj( p, pr.second ) );
	// collapse the chains deterministically, accumulating the phase
	Aig_ManForEachNode( p, pObj, i )
	{
		Aig_Obj_t * pR = p->pReprs[pObj->Id];
		if ( pR == NULL )
			continue;
		int fC = Aig_IsComplement(pR);
		Aig_Obj_t * pRReg = Aig_Regular(pR);
		while ( p->pReprs[pRReg->Id] && p->pReprs[pRReg->Id] != pRReg )
		{
			pR = p->pReprs[pRReg->Id];
			fC ^= Aig_IsComplement(pR);
			pRReg = Aig_Regular(pR);
		}
		p->pReprs[pObj->Id] = Aig_NotCond( pRReg, fC );
	}
}

// Derives the choice AIG (pEquivs) from the representative classes.
// Iterates nodes in (level, id) order: the fanout redirect may point at a
// class representative whose id is larger than the member's (the AIG
// creation order is DFS-based, not level-monotone), so manager order
// would reference not-yet-derived copies.
static Aig_Man_t * Dch2_ManDeriveChoices( Aig_Man_t * pAig,
		const std::vector<Aig_Obj_t *> & vOrder )
{
	Aig_Man_t * pChoices;
	Aig_Obj_t * pObj;
	int i;
	pChoices = Aig_ManStart( Aig_ManObjNumMax(pAig) );
	pChoices->pEquivs = ABC_CALLOC( Aig_Obj_t *, Aig_ManObjNumMax(pAig) );
	pChoices->pReprs  = ABC_CALLOC( Aig_Obj_t *, Aig_ManObjNumMax(pAig) );
	// map constants and CIs
	Aig_ManCleanData( pAig );
	Aig_ManConst1(pAig)->pData = Aig_ManConst1(pChoices);
	Aig_ManForEachCi( pAig, pObj, i )
		pObj->pData = Aig_ObjCreateCi( pChoices );
	// construct the nodes with choices
	for ( auto * pObj : vOrder )
	{
		Aig_Obj_t * pRepr = Aig_ObjRepr( pAig, pObj );
		Aig_Obj_t * pNew;
		Aig_Obj_t * pC0, * pC1;
		if ( pRepr && Aig_IsComplement(pRepr) )
		{
			// complementary class member: direct substitution with the
			// phase-flipped representative structure
			Aig_Obj_t * pReprReg = Aig_Regular(pRepr);
			pObj->pData = Aig_NotCond( (Aig_Obj_t *)pReprReg->pData, 1 );
			continue;
		}
		if ( pRepr && (Aig_ObjIsConst1(pRepr) || Aig_ObjIsCi(pRepr)) )
		{
			pObj->pData = Aig_NotCond( (Aig_Obj_t *)pRepr->pData, pObj->fPhase ^ pRepr->fPhase );
			continue;
		}
		// same-phase class members: redirect the usage to the class
		// representative so the alternative node stays fanout-free
		// (required by the Ntk choice check); substituted fanins already
		// carry the correct alias in their pData
		Aig_Obj_t * pF0 = Aig_ObjFanin0(pObj);
		Aig_Obj_t * pF1 = Aig_ObjFanin1(pObj);
		// redirect usage to the class representative ordered by
		// (level, id): the AIG creation order is DFS-based, so the id is
		// NOT level-monotone and an id comparison would reject shallow
		// representatives with larger ids, leaving the deep structure in
		// place (measured on XS1c: levels did not decrease at all)
		if ( Aig_ObjIsNode(pF0) )
		{
			Aig_Obj_t * pR0 = Aig_ObjRepr( pAig, pF0 );
			// only regular rep pointers are class reps here; complemented
			// reps are substituted aliases carried in pData (never deref
			// a complemented pointer: it is a literal, not an object)
			if ( pR0 && !Aig_IsComplement(pR0) && Aig_ObjIsNode(pR0) &&
					(Aig_ObjLevel(pR0) < Aig_ObjLevel(pF0) ||
					 (Aig_ObjLevel(pR0) == Aig_ObjLevel(pF0) && pR0->Id < pF0->Id)) )
				pF0 = pR0;
		}
		if ( Aig_ObjIsNode(pF1) )
		{
			Aig_Obj_t * pR1 = Aig_ObjRepr( pAig, pF1 );
			if ( pR1 && !Aig_IsComplement(pR1) && Aig_ObjIsNode(pR1) &&
					(Aig_ObjLevel(pR1) < Aig_ObjLevel(pF1) ||
					 (Aig_ObjLevel(pR1) == Aig_ObjLevel(pF1) && pR1->Id < pF1->Id)) )
				pF1 = pR1;
		}
		pC0 = Aig_NotCond( (Aig_Obj_t *)Aig_Regular(pF0)->pData,
				Aig_ObjFaninC0(pObj) ^ Aig_IsComplement(pF0) );
		pC1 = Aig_NotCond( (Aig_Obj_t *)Aig_Regular(pF1)->pData,
				Aig_ObjFaninC1(pObj) ^ Aig_IsComplement(pF1) );
		// Aig_And takes literals and applies the complement semantics
		pNew = Aig_And( pChoices, pC0, pC1 );
		pObj->pData = pNew;
		// link the equivalence class (rep chain through pEquivs)
		if ( pRepr && Aig_ObjIsNode(pRepr) && pRepr->Id < pObj->Id )
		{
			Aig_Obj_t * pReprNew = (Aig_Obj_t *)pRepr->pData;
			Aig_Obj_t * pObjNew  = Aig_Regular((Aig_Obj_t *)pObj->pData);
			Aig_Obj_t * pTail = pReprNew;
			while ( pChoices->pEquivs[pTail->Id] != NULL )
				pTail = pChoices->pEquivs[pTail->Id];
			if ( pTail->Id < pObjNew->Id )
				pChoices->pEquivs[pTail->Id] = pObjNew;
		}
	}
	// connect the COs
	Aig_ManForEachCo( pAig, pObj, i )
		Aig_ObjCreateCo( pChoices, Aig_ObjChild0Copy(pObj) );
	Aig_ManSetRegNum( pChoices, Aig_ManRegNum(pAig) );
	return pChoices;
}

////////////////////////////////////////////////////////////////////////
///                     WINDOW ENGINE                                ///
////////////////////////////////////////////////////////////////////////

static void Dch2_ManProcessWindows( Aig_Man_t * p, uint64_t * pSig0, uint64_t * pSig1, uint64_t * pSig2, uint64_t * pSig3,
		Dch2_Pars_t * pPars,
		const std::vector<Aig_Obj_t *> & vOrder,
		std::vector<Dch2_WinRec_t> & vRecs, int iThread, int nThreads,
		long long * pMsVerify )
{
	int nNodes = (int)vOrder.size();
	int nWin = (nNodes + pPars->nWinSize - 1) / pPars->nWinSize;
	Dch2_VerifyCtx_t Ctx;
	Dch2_VerifyCtxInit( &Ctx, p, Aig_ManObjNumMax(p) );
	// candidate pool: pairs generated by window i, i+1 (halo overlap); each
	// window is processed by exactly one thread, so every boundary pair is
	// generated exactly once regardless of the thread count
	std::map<uint64_t, std::vector<std::pair<Aig_Obj_t *, int>>> bySigNext;
	int nHalo = pPars->nHalo > 0 ? pPars->nHalo : 0;
	for ( int iWin = iThread; iWin < nWin; iWin += nThreads )
	{
		int iBeg = iWin * pPars->nWinSize;
		int iEnd = std::min( iBeg + pPars->nWinSize, nNodes );
		// group the primary-block nodes by phase-canonical simulation
		// signature (a node and its complement share the canonical key)
		std::map<uint64_t, std::vector<std::pair<Aig_Obj_t *, int>>> bySig;
		std::map<uint64_t, std::vector<std::pair<Aig_Obj_t *, int>>>::iterator it;
		for ( int i = iBeg; i < iEnd; i++ )
		{
			Aig_Obj_t * pObj = vOrder[i];
			uint64_t s0 = Dch2_Sig0(pSig0, pObj);
			uint64_t s0N = ~s0;
			uint64_t key = s0 < s0N ? s0 : s0N;
			int fCompl = (s0 > s0N) ? 1 : 0;
			bySig[key].push_back( { pObj, fCompl } );
		}
		// halo tail of the ADJACENT window (cross-window candidates).
		// The candidate set must not depend on the thread count, so the
		// halo always pairs window iWin with window iWin+1; each window is
		// processed by exactly one thread, so every boundary pair is
		// generated exactly once.
		if ( nHalo > 0 && iWin + 1 < nWin )
		{
			int nBeg = (iWin + 1) * pPars->nWinSize;
			int nEnd = std::min( nBeg + nHalo, nNodes );
			bySigNext.clear();
			for ( int i = nBeg; i < nEnd; i++ )
			{
				Aig_Obj_t * pObj = vOrder[i];
				uint64_t s0 = Dch2_Sig0(pSig0, pObj);
				uint64_t s0N = ~s0;
				uint64_t key = s0 < s0N ? s0 : s0N;
				int fCompl = (s0 > s0N) ? 1 : 0;
				bySigNext[key].push_back( { pObj, fCompl } );
			}
		}
		// candidate collection with full four-word agreement
		std::vector<Dch2_WinRec_t> vCand;
		auto Collect = [&]( Aig_Obj_t * pN1, Aig_Obj_t * pN2, int fCompl )
		{
			int fMatch = 1;
			for ( int w = 0; w < 4 && fMatch; w++ )
			{
				uint64_t wA = (w == 0) ? Dch2_Sig0(pSig0, pN1) :
					(w == 1) ? pSig1[pN1->Id] : (w == 2) ? pSig2[pN1->Id] : pSig3[pN1->Id];
				uint64_t wB = (w == 0) ? Dch2_Sig0(pSig0, pN2) :
					(w == 1) ? pSig1[pN2->Id] : (w == 2) ? pSig2[pN2->Id] : pSig3[pN2->Id];
				if ( fCompl )
					wB = ~wB;
				if ( wA != wB )
					fMatch = 0;
			}
			if ( !fMatch )
				return;
			Dch2_WinRec_t Rec;
			Rec.iWin = iWin; Rec.n1 = Aig_ObjId(pN1); Rec.n2 = Aig_ObjId(pN2);
			Rec.fCompl = fCompl;
			vCand.push_back( Rec );
		};
		// adjacent pairs per signature group (bounded candidate set)
		for ( it = bySig.begin(); it != bySig.end(); ++it )
		{
			auto & vGroup = it->second;
			std::sort( vGroup.begin(), vGroup.end(),
					[]( const std::pair<Aig_Obj_t *, int> & a, const std::pair<Aig_Obj_t *, int> & b )
					{ return Aig_ObjId(a.first) < Aig_ObjId(b.first); } );
			for ( size_t k = 0; k + 1 < vGroup.size(); k++ )
				for ( size_t m = k + 1; m < vGroup.size() && m <= k + 4; m++ )
					Collect( vGroup[k].first, vGroup[m].first, vGroup[k].second ^ vGroup[m].second );
		}
		// cross-window pairs: this window's halo tail vs the next window's head
		if ( nHalo > 0 && !bySigNext.empty() )
		{
			int hBeg = std::max( iBeg, iEnd - nHalo );
			for ( int i = hBeg; i < iEnd; i++ )
			{
				Aig_Obj_t * pObj = vOrder[i];
				uint64_t s0 = Dch2_Sig0(pSig0, pObj);
				uint64_t key = std::min( s0, ~s0 );
				auto itN = bySigNext.find( key );
				if ( itN == bySigNext.end() )
					continue;
				int fComplA = (s0 > ~s0) ? 1 : 0;
				for ( auto & prN : itN->second )
					Collect( pObj, prN.first, fComplA ^ prN.second );
			}
		}
		// test-only fault hook (default off): DCH2_TEST_INJECT=n1,n2 pushes
		// one deliberately chosen pair into window 0's verification batch;
		// the real SAT verifier must reject a false pair and the network
		// must not change
		if ( iWin == 0 && getenv("DCH2_TEST_INJECT") )
		{
			int t1 = -1, t2 = -1;
			if ( sscanf( getenv("DCH2_TEST_INJECT"), "%d,%d", &t1, &t2 ) == 2 &&
					t1 >= 0 && t1 < Aig_ManObjNumMax(p) && t2 > 0 && t2 < Aig_ManObjNumMax(p) )
			{
				Dch2_WinRec_t Rec;
				Rec.iWin = iWin; Rec.n1 = t1; Rec.n2 = t2; Rec.fCompl = 0;
				vCand.push_back( Rec );
				if ( pPars->fVerbose )
				{
					printf( "DCH2-TEST-INJECT n1=%d n2=%d\n", t1, t2 );
					fflush(stdout);
				}
			}
		}
		// batch verification (shared miter GIA, one SAT session)
		{
			struct timespec t0, t1;
			clock_gettime( CLOCK_MONOTONIC, &t0 );
			Dch2_ManVerifyBatch( p, &Ctx, vCand, pPars->nConfMax, vRecs, iWin, pPars->fVerbose );
			clock_gettime( CLOCK_MONOTONIC, &t1 );
			*pMsVerify += (long long)(t1.tv_sec - t0.tv_sec) * 1000 + (t1.tv_nsec - t0.tv_nsec) / 1000000;
		}
	}
	Aig_ManStop( Ctx.pMit );
	ABC_FREE( Ctx.pCopy );
}

////////////////////////////////////////////////////////////////////////
///                     TOP-LEVEL                                    ///
////////////////////////////////////////////////////////////////////////

Aig_Man_t * Dch2_ManComputeChoices( Aig_Man_t * pAig, Dch2_Pars_t * pPars )
{
	Aig_Obj_t * pObj;
	int i;
	// stable analysis: levels + deterministic node order by (level, id)
	if ( Aig_ManLevelNum(pAig) == 0 )
		Aig_ManLevels( pAig );
	std::vector<Aig_Obj_t *> vOrder;
	Aig_ManForEachNode( pAig, pObj, i )
		vOrder.push_back( pObj );
	std::stable_sort( vOrder.begin(), vOrder.end(),
			[]( Aig_Obj_t * a, Aig_Obj_t * b )
			{ return Aig_ObjLevel(a) != Aig_ObjLevel(b) ? Aig_ObjLevel(a) < Aig_ObjLevel(b) : Aig_ObjId(a) < Aig_ObjId(b); } );

	// fixed-seed simulation (4x64-bit patterns, deterministic)
	uint64_t * pSig0 = ABC_ALLOC( uint64_t, Aig_ManObjNumMax(pAig) );
	uint64_t * pSig1 = ABC_ALLOC( uint64_t, Aig_ManObjNumMax(pAig) );
	uint64_t * pSig2 = ABC_ALLOC( uint64_t, Aig_ManObjNumMax(pAig) );
	uint64_t * pSig3 = ABC_ALLOC( uint64_t, Aig_ManObjNumMax(pAig) );
	memset( pSig0, 0, sizeof(uint64_t) * Aig_ManObjNumMax(pAig) );
	memset( pSig1, 0, sizeof(uint64_t) * Aig_ManObjNumMax(pAig) );
	memset( pSig2, 0, sizeof(uint64_t) * Aig_ManObjNumMax(pAig) );
	memset( pSig3, 0, sizeof(uint64_t) * Aig_ManObjNumMax(pAig) );
	Dch2_ManSim( pAig, vOrder, pSig0, pSig1, pSig2, pSig3 );

	// window processing (j1 serial or j4 threaded, worker-private SAT state)
	std::vector<Dch2_WinRec_t> vRecs;
	long long msVerify = 0;
	if ( pPars->nThreads <= 1 )
		Dch2_ManProcessWindows( pAig, pSig0, pSig1, pSig2, pSig3, pPars, vOrder, vRecs, 0, 1, &msVerify );
	else
	{
		int nThreads = std::min( pPars->nThreads, 4 );
		std::vector<std::vector<Dch2_WinRec_t>> vPerThread( nThreads );
		std::vector<long long> vMsVerify( nThreads, 0 );
		std::vector<std::thread> vThreads;
		for ( int t = 0; t < nThreads; t++ )
			vThreads.push_back( std::thread( Dch2_ManProcessWindows, pAig, pSig0, pSig1, pSig2, pSig3,
					pPars, std::cref(vOrder), std::ref(vPerThread[t]), t, nThreads, &vMsVerify[t] ) );
		for ( auto & th : vThreads )
			th.join();
		for ( auto & v : vPerThread )
			vRecs.insert( vRecs.end(), v.begin(), v.end() );
		for ( auto ms : vMsVerify )
			msVerify += ms;
	}
	struct timespec tM0, tM1;
	clock_gettime( CLOCK_MONOTONIC, &tM0 );

	// stable conflict resolution: union-find over the records in
	// (window, n2) order; only level-acyclic merges are accepted
	std::stable_sort( vRecs.begin(), vRecs.end(),
			[]( const Dch2_WinRec_t & a, const Dch2_WinRec_t & b )
			{
				if ( a.iWin != b.iWin ) return a.iWin < b.iWin;
				if ( a.n2 != b.n2 ) return a.n2 < b.n2;
				if ( a.n1 != b.n1 ) return a.n1 < b.n1;
				return a.fCompl < b.fCompl;
			} );
	std::vector<int> vParent( Aig_ManObjNumMax(pAig) );
	std::vector<int> vPhase( Aig_ManObjNumMax(pAig), 0 );
	for ( i = 0; i < Aig_ManObjNumMax(pAig); i++ )
		vParent[i] = i;
	// Find returns the class root and the accumulated phase of x relative
	// to that root (x == root XOR phase); vPhase[y] is y's phase relative
	// to its immediate parent vParent[y]
	std::function<std::pair<int,int>(int)> Find = [&]( int x ) -> std::pair<int,int>
	{
		int f = 0;
		while ( vParent[x] != x )
		{
			f ^= vPhase[x];
			x = vParent[x];
		}
		return { x, f };
	};
	std::vector<std::pair<int,int>> vMerges;
	std::vector<std::pair<int,int>> vSubsts; // (node, root) with relative phase
	int nConflict = 0, nTfiSkip = 0;
	for ( auto & Rec : vRecs )
	{
		auto fr1 = Find( Rec.n1 );
		auto fr2 = Find( Rec.n2 );
		int r1 = fr1.first;
		int r2 = fr2.first;
		if ( r1 == r2 )
		{
			nConflict++;
			continue;
		}
		// keep the shallower node as the root (tie-break smaller id):
		// substitutions/merges then always replace a deeper structure
		// with a shallower one and levels cannot accumulate
		Aig_Obj_t * pRoot1 = Aig_ManObj( pAig, r1 );
		Aig_Obj_t * pRoot2 = Aig_ManObj( pAig, r2 );
		int root = (Aig_ObjLevel(pRoot1) != Aig_ObjLevel(pRoot2)) ?
			(Aig_ObjLevel(pRoot1) < Aig_ObjLevel(pRoot2) ? r1 : r2) :
			std::min( r1, r2 );
		int node = (root == r1) ? r2 : r1;
		// relative phase of the merged node against the root:
		// n1 == r1^p1, n2 == r2^p2, n1 == n2^fCompl -> r2 == r1^(p1^p2^fCompl)
		int fPhase = Rec.fCompl ^ fr1.second ^ fr2.second;
		Aig_Obj_t * pRoot = Aig_ManObj( pAig, root );
		Aig_Obj_t * pNode = Aig_ManObj( pAig, node );
		// TFI check in BOTH directions: the root must not be in the
		// transitive fanin of the node (choice/substitution would loop),
		// and the node must not be in the root's TFI (the substitution
		// would make the root's structure reference its own copy).
		{
			int fSkip = 0;
			Aig_ManIncrementTravId( pAig );
			std::vector<Aig_Obj_t *> vStack;
			vStack.push_back( pNode );
			Aig_ObjSetTravIdCurrent( pAig, pNode );
			while ( !vStack.empty() && !fSkip )
			{
				Aig_Obj_t * pTop = vStack.back();
				vStack.pop_back();
				Aig_Obj_t * pFans[2];
				pFans[0] = Aig_ObjFanin0( pTop );
				pFans[1] = Aig_ObjFanin1( pTop );
				for ( int k = 0; k < 2; k++ )
				{
					Aig_Obj_t * pFan = pFans[k];
					if ( pFan == pRoot ) { fSkip = 1; break; }
					if ( Aig_ObjIsNode(pFan) && !Aig_ObjIsTravIdCurrent(pAig, pFan) )
					{
						Aig_ObjSetTravIdCurrent( pAig, pFan );
						vStack.push_back( pFan );
					}
				}
			}
			if ( !fSkip )
			{
				Aig_ManIncrementTravId( pAig );
				vStack.clear();
				vStack.push_back( pRoot );
				Aig_ObjSetTravIdCurrent( pAig, pRoot );
				while ( !vStack.empty() && !fSkip )
				{
					Aig_Obj_t * pTop = vStack.back();
					vStack.pop_back();
					Aig_Obj_t * pFans[2];
					pFans[0] = Aig_ObjFanin0( pTop );
					pFans[1] = Aig_ObjFanin1( pTop );
					for ( int k = 0; k < 2; k++ )
					{
						Aig_Obj_t * pFan = pFans[k];
						if ( pFan == pNode ) { fSkip = 1; break; }
						if ( Aig_ObjIsNode(pFan) && !Aig_ObjIsTravIdCurrent(pAig, pFan) )
						{
							Aig_ObjSetTravIdCurrent( pAig, pFan );
							vStack.push_back( pFan );
						}
					}
				}
			}
			if ( fSkip )
			{
				nTfiSkip++;
				continue;
			}
		}
		vParent[node] = root;
		vPhase[node] = fPhase;
		if ( fPhase )
			vSubsts.push_back( { node, root } );
		else
			vMerges.push_back( { node, root } );
	}
	ABC_FREE( pSig0 );
	ABC_FREE( pSig1 );
	ABC_FREE( pSig2 );
	ABC_FREE( pSig3 );
	// validate every pair against the FINAL classes: the derived structure
	// of a class member must not reference the class representative (the
	// fanout redirect would create a cycle through the pEquivs chain, and a
	// substituted member inside the rep's cone would make the rep's
	// structure reference its own copy)
	{
		auto WalkBad = [&]( int start, int root ) -> int
		{
			Aig_ManIncrementTravId( pAig );
			std::vector<Aig_Obj_t *> vStack;
			vStack.push_back( Aig_ManObj(pAig, start) );
			Aig_ObjSetTravIdCurrent( pAig, Aig_ManObj(pAig, start) );
			while ( !vStack.empty() )
			{
				Aig_Obj_t * pTop = vStack.back();
				vStack.pop_back();
				Aig_Obj_t * pFans[2];
				pFans[0] = Aig_ObjFanin0( pTop );
				pFans[1] = Aig_ObjFanin1( pTop );
				for ( int k = 0; k < 2; k++ )
				{
					Aig_Obj_t * pFan = pFans[k];
					if ( !Aig_ObjIsNode(pFan) )
						continue;
					if ( Find( pFan->Id ).first == root )
						return 1;
					if ( !Aig_ObjIsTravIdCurrent(pAig, pFan) )
					{
						Aig_ObjSetTravIdCurrent( pAig, pFan );
						vStack.push_back( pFan );
					}
				}
			}
			return 0;
		};
		std::vector<std::pair<int,int>> vMergesF, vSubstsF;
		for ( auto & pr : vMerges )
		{
			int node = pr.first, root = pr.second;
			if ( WalkBad( node, root ) || WalkBad( root, root ) )
			{
				if ( pPars->fVerbose )
					printf( "DBG-DROP merge node=%d root=%d\n", node, root );
				continue;
			}
			vMergesF.push_back( pr );
		}
		for ( auto & pr : vSubsts )
		{
			int node = pr.first, root = pr.second;
			if ( WalkBad( node, root ) || WalkBad( root, root ) )
			{
				if ( pPars->fVerbose )
					printf( "DBG-DROP subst node=%d root=%d\n", node, root );
				continue;
			}
			vSubstsF.push_back( pr );
		}
		vMerges = vMergesF;
		vSubsts = vSubstsF;
	}
	if ( pPars->fVerbose )
	{
		for ( auto & pr : vSubsts )
			printf( "DBG-SUBST node=%d root=%d levN=%d levR=%d\n", pr.first, pr.second,
					Aig_ObjLevel(Aig_ManObj(pAig, pr.first)), Aig_ObjLevel(Aig_ManObj(pAig, pr.second)) );
		fflush(stdout);
	}
	clock_gettime( CLOCK_MONOTONIC, &tM1 );
	long long msMerge = (long long)(tM1.tv_sec - tM0.tv_sec) * 1000 + (tM1.tv_nsec - tM0.tv_nsec) / 1000000;
	printf( "DCH2: %d windows, %d candidates, %d verified, %d merged, %d substituted, %d conflicts, %d tfi-skipped, verify_ms=%lld merge_ms=%lld\n",
			(int)((vOrder.size() + pPars->nWinSize - 1) / pPars->nWinSize),
			(int)vRecs.size(), (int)vRecs.size(), (int)vMerges.size(), (int)vSubsts.size(),
			nConflict, nTfiSkip, msVerify, msMerge );

	// no verified merges: return a plain dup with an (empty) pEquivs array
	// so the choice-aware conversion keeps its invariant
	if ( vMerges.empty() && vSubsts.empty() )
	{
		Aig_Man_t * pDup = Aig_ManDupDfs( pAig );
		pDup->pEquivs = ABC_CALLOC( Aig_Obj_t *, Aig_ManObjNumMax(pDup) );
		return pDup;
	}

	// choice construction
	Dch2_ManSetReprs( pAig, vMerges, vSubsts );
	Aig_Man_t * pChoices = Dch2_ManDeriveChoices( pAig, vOrder );
	ABC_FREE( pChoices->pReprs );
	// safety net: the combined (fanin + equiv) graph must be acyclic;
	// if not, fall back to no choices rather than hang the dup
	{
		int nObjsC = Aig_ManObjNumMax(pChoices);
		std::vector<int> vColor( nObjsC, 0 );
		std::function<int(Aig_Obj_t*)> Vis = [&]( Aig_Obj_t * pO ) -> int
		{
			if ( !pO || !Aig_ObjIsNode(pO) )
				return 0;
			if ( vColor[pO->Id] == 1 )
				return 1;
			if ( vColor[pO->Id] == 2 )
				return 0;
			vColor[pO->Id] = 1;
			if ( pChoices->pEquivs && pChoices->pEquivs[pO->Id] && Vis( pChoices->pEquivs[pO->Id] ) )
				return 1;
			if ( Vis( Aig_ObjFanin0(pO) ) || Vis( Aig_ObjFanin1(pO) ) )
				return 1;
			vColor[pO->Id] = 2;
			return 0;
		};
		int fCyc = 0;
		Aig_Obj_t * pObjC;
		int iC;
		Aig_ManForEachNode( pChoices, pObjC, iC )
			if ( Vis( pObjC ) )
			{
				fCyc = 1;
				break;
			}
		if ( fCyc )
		{
			if ( pPars->fVerbose )
				printf( "DCH2: combined graph cycle detected; falling back to no choices\n" );
			fflush(stdout);
			Aig_ManStop( pChoices );
			Aig_ManCleanData( pAig );
			Aig_Man_t * pDup = Aig_ManDupDfs( pAig );
			pDup->pEquivs = ABC_CALLOC( Aig_Obj_t *, Aig_ManObjNumMax(pDup) );
			ABC_FREE( pAig->pReprs );
			return pDup;
		}
	}
	Aig_Man_t * pTemp = pChoices;
	pChoices = Aig_ManDupDfs( pTemp );
	Aig_ManStop( pTemp );
	ABC_FREE( pAig->pReprs );
	return pChoices;
}

Abc_Ntk_t * Abc_NtkDch2( Abc_Ntk_t * pNtk, Dch2_Pars_t * pPars )
{
	extern Abc_Ntk_t * Abc_NtkFromDarChoices( Abc_Ntk_t * pNtkOld, Aig_Man_t * pMan );
	extern Aig_Man_t * Abc_NtkToDar( Abc_Ntk_t * pNtk, int fExors, int fRegisters );
	Aig_Man_t * pMan, * pTemp;
	Abc_Ntk_t * pNtkAig;
	if ( !Abc_NtkIsStrash(pNtk) )
		return NULL;
	pMan = Abc_NtkToDar( pNtk, 0, 0 );
	if ( pMan == NULL )
		return NULL;
	pMan = Dch2_ManComputeChoices( pTemp = pMan, pPars );
	if ( pMan == pTemp )
	{
		Aig_ManStop( pMan );
		return NULL;
	}
	pNtkAig = Abc_NtkFromDarChoices( pNtk, pMan );
	Aig_ManStop( pMan );
	Aig_ManStop( pTemp );
	return pNtkAig;
}

ABC_NAMESPACE_IMPL_END
