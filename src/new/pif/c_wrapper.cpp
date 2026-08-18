#include "c_wrapper.h"
#include "base/io/ioAbc.h"

namespace pif
{
	Abc_Ntk_t *run(Abc_Ntk_t *pNtk, uint32_t nParts, uint32_t sCluster,
				   char *dirName, const char *optScript, const char *mapType,
				   const char *libPath, int nMaxConcurrent, const char *tmpDir,
				   bool fStrict)
	{
		shared_ptr<PartNtk> spPN = make_shared<PartNtk>(pNtk, nParts, sCluster,
														dirName, optScript, mapType, libPath,
														nMaxConcurrent, tmpDir, fStrict);
		spPN->runPipeline();
		return spPN->getResNtk();
	}

} // for namespace

extern "C" Abc_Ntk_t *pif_run_wrapper(Abc_Ntk_t *pNtk, uint32_t nParts,
									  uint32_t sCluster, char *dirName, char *optScript, char *mapType, char *libPath,
									  int nMaxConcurrent, char *tmpDir, bool fStrict)
{
	return pif::run(pNtk, nParts, sCluster, dirName, optScript, mapType, libPath,
					nMaxConcurrent, tmpDir, fStrict);
}
