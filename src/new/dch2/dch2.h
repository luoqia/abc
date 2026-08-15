#ifndef __DCH2_H__
#define __DCH2_H__

#include "aig/aig/aig.h"
#include "base/abc/abc.h"

ABC_NAMESPACE_HEADER_START

typedef struct Dch2_Pars_t_ Dch2_Pars_t;
struct Dch2_Pars_t_
{
	int nThreads;  // 1 (j1) or 4 (j4)
	int nWinSize;  // nodes per window (primary block)
	int nHalo;     // overlap halo on each side
	int nSeed;     // fixed simulation seed
	int nConfMax;  // SAT conflict limit per verification
	int fVerbose;
};

extern Aig_Man_t * Dch2_ManComputeChoices( Aig_Man_t * pAig, Dch2_Pars_t * pPars );
extern Abc_Ntk_t * Abc_NtkDch2( Abc_Ntk_t * pNtk, Dch2_Pars_t * pPars );

ABC_NAMESPACE_HEADER_END

#endif
