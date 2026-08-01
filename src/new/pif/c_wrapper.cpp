#include "c_wrapper.h"
#include "base/io/ioAbc.h"

namespace ymc
{
	int hello()
	{
		printf("Now in ymc_hello()!\n");
		return 0;
	}

	int test_yaig()
	{
		printf("Now in test_yaig()!\n");
		int32_t node[100];

		MetisAig y;
		node[0] = y.addPi();
		node[1] = y.addPi();
		node[2] = y.addPi();
		node[3] = y.addAndNode(node[0], 0, node[1], 1);
		node[4] = y.addAndNode(node[1], 1, node[2], 0);
		node[5] = y.addAndNode(node[3], 0, node[4], 1);
		node[6] = y.addPo(node[5], 0);
		node[7] = y.addPi();
		node[8] = y.addAndNode(node[4], 0, node[7], 1);
		node[9] = y.addPo(node[8], 0);
		node[10] = y.addPi();
		node[11] = y.addPi();
		node[12] = y.addPi();
		node[13] = y.addAndNode(node[10], 0, node[11], 0);
		node[14] = y.addAndNode(node[13], 0, node[12], 0);
		node[15] = y.addPo(node[14], 0);
		y.computeAllLevel();
		y.parseAig();

		return 0;
	}

	Abc_Ntk_t *pif(Abc_Ntk_t *pNtk, uint32_t nParts, uint32_t sCluster,
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

extern "C" int ymc_hello_wrapper()
{
	return ymc::hello();
}

extern "C" int ymc_test_yaig_wrapper()
{
	return ymc::test_yaig();
}

extern "C" Abc_Ntk_t *ymc_pif_wrapper(Abc_Ntk_t *pNtk, uint32_t nParts,
									  uint32_t sCluster, char *dirName, char *optScript, char *mapType, char *libPath,
									  int nMaxConcurrent, char *tmpDir, bool fStrict)
{
	return ymc::pif(pNtk, nParts, sCluster, dirName, optScript, mapType, libPath,
					nMaxConcurrent, tmpDir, fStrict);
}