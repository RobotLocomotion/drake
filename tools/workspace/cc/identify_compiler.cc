// Since Bazel by default does not use C++-specific compiler executables, such
// as clang++ and g++, this file is intended to be compilable by both C and C++
// compilers.

#include <stdio.h>

int main() {
// Order is significant since some compilers imitate Clang, GNU, and/or MSVC per
// https://gitlab.kitware.com/cmake/cmake/blob/v3.12.0/Modules/CMakeCompilerIdDetection.cmake
//
// Identifying predefined macro names taken from
// https://gitlab.kitware.com/cmake/cmake/blob/v3.12.0/Modules/Compiler/*-DetermineCompiler.cmake
//
// Compiler identifiers match those of CMake 3.12.0, modulo some old versions
// of the unsupported Borland/Embarcadero, OpenWatcom/Watcom, and
// VisualAge/XL/zOS compilers.

#if defined(__COMO__)
  const char compiler_id[] = "Comeau";

#elif defined(__ICC) || defined(__INTEL_COMPILER)
  const char compiler_id[] = "Intel";

#elif defined(__PATHCC__)
  const char compiler_id[] = "PathScale";

#elif defined(__BORLANDC__)
  const char compiler_id[] = "Embarcadero";

#elif defined(__WATCOMC__)
  const char compiler_id[] = "OpenWatcom";

#elif defined(__SUNPRO_C) || defined(__SUNPRO_CC)
  const char compiler_id[] = "SunPro";

#elif defined(__HP_aCC) || defined(__HP_cc)
  const char compiler_id[] = "HP";

#elif defined(__DECC) || defined(__DECCXX)
  const char compiler_id[] = "Compaq";

#elif defined(__ibmxl__) || defined(__IBMC__) || defined(__IBMCPP__)
  const char compiler_id[] = "XL";

#elif defined(__PGI)
  const char compiler_id[] = "PGI";

#elif defined(_CRAYC)
  const char compiler_id[] = "Cray";

#elif defined(__TI_COMPILER_VERSION__)
  const char compiler_id[] = "TI";

#elif defined(__fcc_version) || defined(__FCC_VERSION) || defined(__FUJITSU)
  const char compiler_id[] = "Fujitsu";

#elif defined(__SCO_VERSION__)
  const char compiler_id[] = "SCO";

#elif defined(__apple_build_version__) && defined(__clang__)
  const char compiler_id[] = "AppleClang";

#elif defined(__clang__)
  const char compiler_id[] = "Clang";

#elif defined(__GNUC__) || defined(__GNUG__)
  const char compiler_id[] = "GNU";

#elif defined(_MSC_VER)
  const char compiler_id[] = "MSVC";

// NOLINTNEXTLINE(whitespace/line_length)
#elif defined(__ADSP21000__) || defined(__ADSPBLACKFIN__) || defined(__ADSPTS__) || defined(__VISUALDSPVERSION__)
  const char compiler_id[] = "ADSP";

#elif defined(__IAR_SYSTEMS_ICC) || defined(__IAR_SYSTEMS_ICC__)
  const char compiler_id[] = "IAR";

#elif defined(__ARMCC_VERSION)
  const char compiler_id[] = "ARMCC";

#elif defined(_SGI_COMPILER_VERSION) || defined(_COMPILER_VERSION)
  const char compiler_id[] = "MIPSpro";

#else
  fprintf(stderr, "C/C++ compiler could NOT be identified.");
  return 1;
#endif

  printf("%s", compiler_id);
  return 0;
}
