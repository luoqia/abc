#pragma once
#include <iostream>
#include <cstdio>
#include <vector>
#include <memory>

#include "partNtk.h"
#include "omp.h"

using namespace std;

namespace ymc
{

    int hello();
    int test_yaig();
    Abc_Ntk_t *pif(Abc_Ntk_t *pNtk, uint32_t nParts, uint32_t sCluster, char *dirName, const char *optScript, const char *mapType);

} // for namespace