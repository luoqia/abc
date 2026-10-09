/**CFile****************************************************************

  FileName    [AgdmapCommand.h]

  SystemName  [ABC: Logic synthesis and verification system.]

  PackageName [Agdmap.]

  Synopsis    [External declarations.]

  Author      [Longfei Fan]

  Affiliation [Fudan University]

  Date        [August 1, 2023]

  Vision      [ 1.0 ]

***********************************************************************/

#pragma once

////////////////////////////////////////////////////////////////////////
///                          INCLUDES                                ///
////////////////////////////////////////////////////////////////////////

#include "base/main/main.h"

////////////////////////////////////////////////////////////////////////
///                         PARAMETERS                               ///
////////////////////////////////////////////////////////////////////////

ABC_NAMESPACE_HEADER_START

////////////////////////////////////////////////////////////////////////
///                         BASIC TYPES                              ///
////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////
///                      FUNCTION DEFINITIONS                        ///
////////////////////////////////////////////////////////////////////////

// Task113 M0 compatibility: ABC's integrated build compiles every source as
// C++, so the command entry keeps C linkage on both sides of the declaration.
// The guard leaves a standalone C compilation of AgdmapRegister.c unchanged.
#ifdef __cplusplus
extern "C"
#endif
int Agdmap(Abc_Frame_t * pAbc, int argc, char **argv);

ABC_NAMESPACE_HEADER_END
