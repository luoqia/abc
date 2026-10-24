/**CFile****************************************************************

  FileName    [AgdmapCommand.cpp]

  SystemName  [ABC: Logic synthesis and verification system.]

  PackageName [Agdmap.]

  Synopsis    [External declarations.]

  Author      [Longfei Fan, Zhiyong Zhang, Chang Wu]

  Affiliation [Fudan University]

  Date        [August 1, 2023]

  Vision      [ 1.0 ]

***********************************************************************/

#include "base/main/main.h"
#include "agdmap.h"

#include <cctype>
#include <cerrno>
#include <climits>
#include <cstdlib>
#include <cstring>

ABC_NAMESPACE_IMPL_START

////////////////////////////////////////////////////////////////////////
///                      DECLARATIONS                                ///
////////////////////////////////////////////////////////////////////////
extern "C"
{
  int Agdmap(Abc_Frame_t *pAbc, int argc, char **argv);
}

// Strict base-10 parsing for retained command options: the whole argument must
// be an in-range integer. Leading whitespace and trailing junk are rejected.
static bool AgdmapParseInt(const char *option, const char *text, int &value)
{
  if (text == nullptr || *text == '\0' || std::isspace((unsigned char)*text))
  {
    Abc_Print(-1, "agdmap: %s requires a base-10 integer value.\n", option);
    return false;
  }
  errno = 0;
  char *end = nullptr;
  long parsed = std::strtol(text, &end, 10);
  if (end == text || *end != '\0' || errno == ERANGE || parsed < INT_MIN || parsed > INT_MAX)
  {
    Abc_Print(-1, "agdmap: invalid value '%s' for %s: expected a base-10 integer in range [%d, %d].\n",
              text, option, INT_MIN, INT_MAX);
    return false;
  }
  value = (int)parsed;
  return true;
}

////////////////////////////////////////////////////////////////////////
///                      FUNCTION DEFINITIONS                        ///
////////////////////////////////////////////////////////////////////////

/**Function*************************************************************

  Synopsis    [Main func for Agdmap - area command]

  Description []

  SideEffects []

  SeeAlso     []

***********************************************************************/
int Agdmap(Abc_Frame_t *pAbc, int argc, char **argv)
{
  using namespace std;
  Abc_Ntk_t *pNtk, *pNtkRes;
  pNtk = Abc_FrameReadNtk(pAbc);

  auto run_mapper = [](bool delay, int k, int w, bool verbose, bool recover,
                       int recover_sweeps, long long recover_max_evals,
                       Abc_Ntk_t *ntk)
  {
    Abc_Ntk_t *res = nullptr;
    AgdMap::Para para(k, w, !delay, verbose, recover, recover_sweeps,
                      recover_max_evals);

    std::cout << "\n"
              << ntk->pName << "\t is running ... \n";
    AgdMap::Mapper mapper(ntk, para);
    if (mapper.map() != 0)
      std::cout << "adgmap failed!\n";
    res = mapper.getNtk();
    return res;
  };

  int k = 6;
  int w = 8;
  bool delay_oriented = false;
  bool verbose = false;
  bool recover = false;
  int recover_sweeps = 8;
  long long recover_max_evals = 0;

  for (size_t i = 1; i < argc; ++i)
  {
    if (strcmp(argv[i], "-k") == 0)
    {
      if (i + 1 >= (size_t)argc || !AgdmapParseInt("-k", argv[i + 1], k))
        goto usage;
      ++i;
      if (k < 3)
        goto usage;
    }
    else if (strcmp(argv[i], "-w") == 0)
    {
      if (i + 1 >= (size_t)argc || !AgdmapParseInt("-w", argv[i + 1], w))
        goto usage;
      ++i;
      // Retained compatibility: widths below 3 disable gate decomposition.
      if (w < 3)
        w = 0;
    }
    else if (strcmp(argv[i], "-d") == 0)
      delay_oriented = true;
    else if (strcmp(argv[i], "-v") == 0)
      verbose = true;
    else if (strcmp(argv[i], "-recover") == 0)
      recover = true;
    else if (strcmp(argv[i], "-recover-sweeps") == 0)
    {
      if (i + 1 >= (size_t)argc || !AgdmapParseInt("-recover-sweeps", argv[i + 1], recover_sweeps))
        goto usage;
      ++i;
      if (recover_sweeps < 1)
        goto usage;
    }
    else if (strcmp(argv[i], "-recover-max-evals") == 0)
    {
      int cap = 0;
      if (i + 1 >= (size_t)argc || !AgdmapParseInt("-recover-max-evals", argv[i + 1], cap))
        goto usage;
      ++i;
      if (cap < 0)
        goto usage;
      recover_max_evals = cap; // 0 keeps the search unbounded
    }
    else if (strcmp(argv[i], "-h") == 0)
      goto usage;
    else
    {
      printf("unknown argument %s\n", argv[i]);
      goto usage;
    }
  }

  if (pNtk == NULL)
  {
    Abc_Print(-1, "Empty network.\n");
    return 0;
  }
  if (!Abc_NtkIsStrash(pNtk))
  {
    pNtk = Abc_NtkStrash(pNtk, 0, 0, 0);
    if (pNtk == NULL)
    {
      Abc_Print(-1, "Strashing before FPGA mapping has failed.\n");
      return 1;
    }
    pNtk = Abc_NtkBalance(pNtkRes = pNtk, 0, 0, 1);
    Abc_NtkDelete(pNtkRes);
    if (pNtk == NULL)
    {
      Abc_Print(-1, "Balancing before FPGA mapping has failed.\n");
      return 1;
    }
    if (!Abc_FrameReadFlag("silentmode"))
      Abc_Print(1, "The network was strashed and balanced before FPGA mapping.\n");
    // get the new network
    pNtkRes = run_mapper(delay_oriented, k, w, verbose, recover, recover_sweeps,
                         recover_max_evals, pNtk);
    if (pNtkRes == NULL)
    {
      Abc_NtkDelete(pNtk);
      Abc_Print(-1, "FPGA mapping has failed.\n");
      return 0;
    }
  }
  else
  {
    pNtkRes = run_mapper(delay_oriented, k, w, verbose, recover, recover_sweeps,
                         recover_max_evals, pNtk);
    if (pNtkRes == NULL)
    {
      Abc_Print(-1, "FPGA mapping has failed.\n");
      return 0;
    }
  }
  // replace the current network
  Abc_FrameReplaceCurrentNetwork(pAbc, pNtkRes);
  Abc_FrameClearVerifStatus(pAbc);

  return 0;

usage:
  Abc_Print(-2, "\nusage: agdmap\n");
  Abc_Print(-2, "\t         a patent pending FPGA technology mapping with adaptive gate decomposition algorithm \n");
  Abc_Print(-2, "\t-k  [int]   : LUT input size [default = 6]\n");
  Abc_Print(-2, "\t-w  [int]   : wide gate input size limit [default = 8]\n");
  Abc_Print(-2, "\t            : 0 means no gate decomposition\n");
  Abc_Print(-2, "\t-d          : delay-oriented technology mapping, default is area-oriented\n");
  Abc_Print(-2, "\t-v          : print verbose information\n");
  Abc_Print(-2, "\t-recover    : exact live-cover recovery on the frozen cut pool\n");
  Abc_Print(-2, "\t-recover-sweeps [int] : recovery sweeps [default = 8]\n");
  Abc_Print(-2, "\t-recover-max-evals [int] : bound recovery evaluations per mapped network; 0 = unbounded [default = 0]\n");
  Abc_Print(-2, "\t-h          : print the command usage\n");
  return 1;
}

////////////////////////////////////////////////////////////////////////
///                       END OF FILE                                ///
////////////////////////////////////////////////////////////////////////

ABC_NAMESPACE_IMPL_END
