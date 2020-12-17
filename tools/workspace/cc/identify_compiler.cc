// Since Bazel by default does not use C++-specific compiler executables, such
// as clang++ and g++, this file is intended to be compilable by both C and C++
// compilers.

#include <stdio.h>

int main() {
// Order is significant since some compilers imitate Clang, GNU, and/or MSVC per
// https://gitlab.kitware.com/cmake/cmake/blob/v3.12.1/Modules/CMakeCompilerIdDetection.cmake
//
// Identifying and version predefined macro names taken from
// https://gitlab.kitware.com/cmake/cmake/blob/v3.12.1/Modules/Compiler/*-DetermineCompiler.cmake
//
// Compiler identifiers match those of CMake 3.12.1, modulo some old versions
// of the unsupported Borland/Embarcadero, OpenWatcom/Watcom, and
// VisualAge/XL/zOS compilers.

#if defined(__COMO__)
  const char compiler_id[] = "Comeau";
  const int compiler_version_major = __COMO_VERSION__ / 100;
  const int compiler_version_minor = __COMO_VERSION__ % 100;

#elif defined(__ICC) || defined(__INTEL_COMPILER)
  const char compiler_id[] = "Intel";
  const int compiler_version_major = __INTEL_COMPILER / 100;
  const int compiler_version_minor = __INTEL_COMPILER / 10 % 10;

#elif defined(__PATHCC__)
  const char compiler_id[] = "PathScale";
  const int compiler_version_major = __PATHCC__;
  const int compiler_version_minor = __PATHCC_MINOR__;

#elif defined(__BORLANDC__)
  const char compiler_id[] = "Embarcadero";

#if defined(__CODEGEARC_VERSION__)
  const int compiler_version_major = __CODEGEARC_VERSION__ >> 24 & 0x00FF;
  const int compiler_version_minor = __CODEGEARC_VERSION__ >> 16 & 0x00FF;
#else
  const int compiler_version_major = 0;
  const int compiler_version_minor = 0;
#endif

#elif defined(__WATCOMC__)
  const char compiler_id[] = "OpenWatcom";
  const int compiler_version_major = (__WATCOMC__ - 1100) / 100;
  const int compiler_version_minor = __WATCOMC__ / 10 % 10;

#elif defined(__SUNPRO_C) || defined(__SUNPRO_CC)
  const char compiler_id[] = "SunPro";

#if defined(__SUNPRO_C) && __SUNPRO_C >= 0x5100
  const int compiler_version_major = __SUNPRO_C >> 12;
  const int compiler_version_minor = __SUNPRO_C >> 4 & 0xFF;
#elif defined(__SUNPRO_C)
  const int compiler_version_major = __SUNPRO_C >> 8;
  const int compiler_version_minor = __SUNPRO_C >> 4 & 0xF;
#elif __SUNPRO_CC >= 0x5100
  const int compiler_version_major = __SUNPRO_CC >> 12;
  const int compiler_version_minor = __SUNPRO_CC >> 4 & 0xFF;
#else
  const int compiler_version_major = __SUNPRO_CC >> 8;
  const int compiler_version_minor = __SUNPRO_CC >> 4 & 0xF;
#endif

#elif defined(__HP_aCC) || defined(__HP_cc)
  const char compiler_id[] = "HP";

#if defined(__HP_aCC)
  const int compiler_version_major = __HP_aCC / 10000;
  const int compiler_version_minor = __HP_aCC / 100 % 100;
#else
  const int compiler_version_major = __HP_cc / 10000;
  const int compiler_version_minor = __HP_cc / 100 % 100;
#endif

#elif defined(__DECC) || defined(__DECCXX)
  const char compiler_id[] = "Compaq";

#if defined(__DECC)
  const int compiler_version_major = __DECC_VER / 10000000;
  const int compiler_version_minor = __DECC_VER / 100000 % 100;
#else
  const int compiler_version_major = __DECCXX_VER / 10000000;
  const int compiler_version_minor = __DECCXX_VER / 100000 % 100;
#endif

#elif defined(__ibmxl__) || defined(__IBMC__) || defined(__IBMCPP__)
  const char compiler_id[] = "XL";

#if defined(__ibmxl__)
  const int compiler_version_major = __ibmxl_version__;
  const int compiler_version_minor = __ibmxl_release__;
#elif defined(__IBMC__)
  const int compiler_version_major = __IBMC__ / 100;
  const int compiler_version_minor = __IBMC__ / 10 % 10;
#else
  const int compiler_version_major = __IBMCPP__ / 100;
  const int compiler_version_minor = __IBMCPP__ / 10 % 10;
#endif

#elif defined(__PGI)
  const char compiler_id[] = "PGI";
  const int compiler_version_major = __PGIC__;
  const int compiler_version_minor = __PGIC_MINOR__;

#elif defined(_CRAYC)
  const char compiler_id[] = "Cray";
  const int compiler_version_major = _RELEASE_MAJOR;
  const int compiler_version_minor = _RELEASE_MINOR;

#elif defined(__TI_COMPILER_VERSION__)
  const char compiler_id[] = "TI";
  const int compiler_version_major = __TI_COMPILER_VERSION__ / 1000000;
  const int compiler_version_minor = __TI_COMPILER_VERSION__ / 1000 % 1000;

#elif defined(__fcc_version) || defined(__FCC_VERSION) || defined(__FUJITSU)
  const char compiler_id[] = "Fujitsu";
  const int compiler_version_major = 0;
  const int compiler_version_minor = 0;

#elif defined(__SCO_VERSION__)
  const char compiler_id[] = "SCO";
  const int compiler_version_major = __SCO_VERSION__ / 100000000;
  const int compiler_version_minor = __SCO_VERSION__ / 1000000 % 100;

#elif defined(__apple_build_version__) && defined(__clang__)
  const char compiler_id[] = "AppleClang";
  const int compiler_version_major = __clang_major__;
  const int compiler_version_minor = __clang_minor__;

#elif defined(__clang__)
  const char compiler_id[] = "Clang";
  const int compiler_version_major = __clang_major__;
  const int compiler_version_minor = __clang_minor__;

#elif defined(__GNUC__) || defined(__GNUG__)
  const char compiler_id[] = "GNU";

#if defined(__GNUC__)
  const int compiler_version_major = __GNUC__;
#else
  const int compiler_version_major = __GNUG__;
#endif

  const int compiler_version_minor = __GNUC_MINOR__;

#elif defined(_MSC_VER)
  const char compiler_id[] = "MSVC";
  const int compiler_version_major = _MSC_VER / 100;
  const int compiler_version_minor = _MSC_VER % 100;

// NOLINTNEXTLINE(whitespace/line_length)
#elif defined(__ADSP21000__) || defined(__ADSPBLACKFIN__) || defined(__ADSPTS__) || defined(__VISUALDSPVERSION__)
  const char compiler_id[] = "ADSP";

#if defined(__VISUALDSPVERSION__)
  const int compiler_version_major = __VISUALDSPVERSION__ >> 24;
  const int compiler_version_minor = __VISUALDSPVERSION__ >> 16 & 0xFF;
#else
  const int compiler_version_major = 0;
  const int compiler_version_minor = 0;
#endif

#elif defined(__IAR_SYSTEMS_ICC) || defined(__IAR_SYSTEMS_ICC__)
  const char compiler_id[] = "IAR";

#if defined(__VER__)
  const int compiler_version_major = __VER__ / 1000000;
  const int compiler_version_minor = __VER__ / 1000 % 1000;
#else
  const int compiler_version_major = 0;
  const int compiler_version_minor = 0;
#endif

#elif defined(__ARMCC_VERSION)
  const char compiler_id[] = "ARMCC";

#if __ARMCC_VERSION >= 1000000
  const int compiler_version_major = __ARMCC_VERSION / 1000000;
  const int compiler_version_minor = __ARMCC_VERSION / 10000 % 100;
#else
  const int compiler_version_major = __ARMCC_VERSION / 100000;
  const int compiler_version_minor = __ARMCC_VERSION / 10000 % 10;
#endif

#elif defined(_SGI_COMPILER_VERSION) || defined(_COMPILER_VERSION)
  const char compiler_id[] = "MIPSpro";

#if defined(_SGI_COMPILER_VERSION)
  const int compiler_version_major = _SGI_COMPILER_VERSION / 100;
  const int compiler_version_minor = _SGI_COMPILER_VERSION / 10 % 10;
#else
  const int compiler_version_major = _COMPILER_VERSION / 100;
  const int compiler_version_minor = _COMPILER_VERSION / 10 % 10;
#endif

#else
  fprintf(stderr, "C/C++ compiler could NOT be identified.");
  return 1;
#endif

  printf("%s %i %i\n", compiler_id, compiler_version_major,
      compiler_version_minor);
  return 0;
}
