      PROGRAM CMakeFortranCompilerId
#if 0
! Identify the compiler
#endif
#if defined(_MSC_VER)
       PRINT *, 'INFO:simulate[MSVC]'
# if _MSC_VER >= 1900
       PRINT *, 'INFO:simulate_version[019.00]'
# elif _MSC_VER >= 1800
       PRINT *, 'INFO:simulate_version[018.00]'
# elif _MSC_VER >= 1700
       PRINT *, 'INFO:simulate_version[017.00]'
# elif _MSC_VER >= 1600
       PRINT *, 'INFO:simulate_version[016.00]'
# elif _MSC_VER >= 1500
       PRINT *, 'INFO:simulate_version[015.00]'
# elif _MSC_VER >= 1400
       PRINT *, 'INFO:simulate_version[014.00]'
# elif _MSC_VER >= 1310
       PRINT *, 'INFO:simulate_version[013.01]'
# else
       PRINT *, 'INFO:simulate_version[013.00]'
# endif
#endif
#if defined(__INTEL_LLVM_COMPILER)
        PRINT *, 'INFO:compiler[IntelLLVM]'
! __INTEL_LLVM_COMPILER = VVVVRP prior to 2021.2.0, VVVVRRPP for 2021.2.0 and
! later.  Look for 6 digit vs. 8 digit version number to decide encoding.
! VVVV is no smaller than the current year when a version is released.
# if __INTEL_LLVM_COMPILER < 1000000
#  define COMPILER_VERSION_MAJOR DEC(__INTEL_LLVM_COMPILER/100)
#  define COMPILER_VERSION_MINOR DEC(__INTEL_LLVM_COMPILER/10 % 10)
#  define COMPILER_VERSION_PATCH DEC(__INTEL_LLVM_COMPILER    % 10)
# else
#  define COMPILER_VERSION_MAJOR DEC(__INTEL_LLVM_COMPILER/10000)
#  define COMPILER_VERSION_MINOR DEC(__INTEL_LLVM_COMPILER/100 % 100)
#  define COMPILER_VERSION_PATCH DEC(__INTEL_LLVM_COMPILER     % 100)
# endif
#elif defined(__INTEL_COMPILER) && __INTEL_COMPILER == 201900
        PRINT *, 'INFO:compiler[IntelLLVM]'
! ifx 2021.1 forgot to define __INTEL_LLVM_COMPILER.
! Instead it defines __INTEL_COMPILER == 201900.
# define COMPILER_VERSION_MAJOR DEC(2021)
# define COMPILER_VERSION_MINOR DEC(1)
# define COMPILER_VERSION_PATCH DEC(__INTEL_COMPILER_UPDATE)
# define COMPILER_VERSION_TWEAK DEC(__INTEL_COMPILER_BUILD_DATE)
#elif defined(__INTEL_COMPILER) || defined(__ICC)
        PRINT *, 'INFO:compiler[Intel]'
! __INTEL_COMPILER = VRP prior to 2021, and then VVVV for 2021 and later.
# if __INTEL_COMPILER < 2021
#  define COMPILER_VERSION_MAJOR DEC(__INTEL_COMPILER/100)
#  define COMPILER_VERSION_MINOR DEC(__INTEL_COMPILER/10 % 10)
#  if defined(__INTEL_COMPILER_UPDATE)
#   define COMPILER_VERSION_PATCH DEC(__INTEL_COMPILER_UPDATE)
#  else
#   define COMPILER_VERSION_PATCH DEC(__INTEL_COMPILER   % 10)
#  endif
# else
#  define COMPILER_VERSION_MAJOR DEC(__INTEL_COMPILER)
#  define COMPILER_VERSION_MINOR DEC(__INTEL_COMPILER_UPDATE)
!  The third version component from --version is an update index,
!  but no macro is provided for it.
#  define COMPILER_VERSION_PATCH DEC(0)
# endif
# if defined(__INTEL_COMPILER_BUILD_DATE)
#  define COMPILER_VERSION_TWEAK DEC(__INTEL_COMPILER_BUILD_DATE)
# endif
#elif defined(__SUNPRO_F95)
        PRINT *, 'INFO:compiler[SunPro]'
# define COMPILER_VERSION_MAJOR HEX(__SUNPRO_F95>>8)
# define COMPILER_VERSION_MINOR HEX(__SUNPRO_F95>>4 & 0xF)
# define COMPILER_VERSION_PATCH HEX(__SUNPRO_F95    & 0xF)
#elif defined(__SUNPRO_F90)
        PRINT *, 'INFO:compiler[SunPro]'
# define COMPILER_VERSION_MAJOR HEX(__SUNPRO_F90>>8)
# define COMPILER_VERSION_MINOR HEX(__SUNPRO_F90>>4 & 0xF)
# define COMPILER_VERSION_PATCH HEX(__SUNPRO_F90    & 0xF)
#elif defined(_CRAYFTN)
        PRINT *, 'INFO:compiler[Cray]'
# define COMPILER_VERSION_MAJOR DEC(_RELEASE_MAJOR)
# define COMPILER_VERSION_MINOR DEC(_RELEASE_MINOR)
# if defined(_RELEASE_PATCHLEVEL)
#  define COMPILER_VERSION_PATCH DEC(_RELEASE_PATCHLEVEL)
# endif
#elif defined(__G95__)
        PRINT *, 'INFO:compiler[G95]'
# define COMPILER_VERSION_MAJOR DEC(__G95__)
# define COMPILER_VERSION_MINOR DEC(__G95_MINOR__)
#elif defined(__PATHSCALE__)
        PRINT *, 'INFO:compiler[PathScale]'
# define COMPILER_VERSION_MAJOR DEC(__PATHCC__)
# define COMPILER_VERSION_MINOR DEC(__PATHCC_MINOR__)
# if defined(__PATHCC_PATCHLEVEL__)
#  define COMPILER_VERSION_PATCH DEC(__PATHCC_PATCHLEVEL__)
# endif
#elif defined(__ABSOFT__)
        PRINT *, 'INFO:compiler[Absoft]'
#elif defined(__GNUC__)
        PRINT *, 'INFO:compiler[GNU]'
# define COMPILER_VERSION_MAJOR DEC(__GNUC__)
# define COMPILER_VERSION_MINOR DEC(__GNUC_MINOR__)
# if defined(__GNUC_PATCHLEVEL__)
#  define COMPILER_VERSION_PATCH DEC(__GNUC_PATCHLEVEL__)
# endif
#elif defined(__IBMC__)
# if defined(__COMPILER_VER__)
        PRINT *, 'INFO:compiler[zOS]'
# elif __IBMC__ >= 800
        PRINT *, 'INFO:compiler[XL]'
#  define COMPILER_VERSION_MAJOR DEC(__IBMC__/100)
#  define COMPILER_VERSION_MINOR DEC(__IBMC__/10 % 10)
#  define COMPILER_VERSION_PATCH DEC(__IBMC__    % 10)
# else
        PRINT *, 'INFO:compiler[VisualAge]'
#  define COMPILER_VERSION_MAJOR DEC(__IBMC__/100)
#  define COMPILER_VERSION_MINOR DEC(__IBMC__/10 % 10)
#  define COMPILER_VERSION_PATCH DEC(__IBMC__    % 10)
# endif
#elif defined(__NVCOMPILER) || defined(__NVCOMPILER_LLVM__)
        PRINT *, 'INFO:compiler[NVHPC]'
# if defined(__NVCOMPILER_MAJOR__)
#  define COMPILER_VERSION_MAJOR DEC(__NVCOMPILER_MAJOR__)
# else
#  define COMPILER_VERSION_MAJOR DEC(__PGIC__)
# endif
# if defined(__NVCOMPILER_MINOR__)
#  define COMPILER_VERSION_MINOR DEC(__NVCOMPILER_MINOR__)
# else
#  define COMPILER_VERSION_MINOR DEC(__PGIC_MINOR__)
# endif
# if defined(__NVCOMPILER_PATCHLEVEL__)
#  define COMPILER_VERSION_PATCH DEC(__NVCOMPILER_PATCHLEVEL__)
# elif defined(__PGIC_PATCHLEVEL__)
#  define COMPILER_VERSION_PATCH DEC(__PGIC_PATCHLEVEL__)
# endif
#elif defined(__PGI)
        PRINT *, 'INFO:compiler[PGI]'
# define COMPILER_VERSION_MAJOR DEC(__PGIC__)
# define COMPILER_VERSION_MINOR DEC(__PGIC_MINOR__)
# if defined(__PGIC_PATCHLEVEL__)
#  define COMPILER_VERSION_PATCH DEC(__PGIC_PATCHLEVEL__)
# endif
#elif defined(__FLANG)
        PRINT *, 'INFO:compiler[Flang]'
# define COMPILER_VERSION_MAJOR DEC(__FLANG_MAJOR__)
# define COMPILER_VERSION_MINOR DEC(__FLANG_MINOR__)
# if defined(__FLANG_PATCHLEVEL__)
#  define COMPILER_VERSION_PATCH DEC(__FLANG_PATCHLEVEL__)
# endif
#elif defined(_AIX) || defined(__AIX) || defined(__AIX__) || defined(__aix) || defined(__aix__)
        PRINT *, 'INFO:compiler[VisualAge]'
#elif defined(__hpux) || defined(__hpux__)
        PRINT *, 'INFO:compiler[HP]'
#elif defined(NAGFOR)
        PRINT *, 'INFO:compiler[NAG]'
#define COMPILER_VERSION_MAJOR DEC(__NAG_COMPILER_RELEASE/10)
#define COMPILER_VERSION_MINOR DEC(__NAG_COMPILER_RELEASE % 10)
#define COMPILER_VERSION_PATCH DEC(__NAG_COMPILER_BUILD)
#elif defined(__FUJITSU)
        PRINT *, 'INFO:compiler[Fujitsu]'
# if defined(__FRT_major__)
#   define COMPILER_VERSION_MAJOR DEC(__FRT_major__)
#   define COMPILER_VERSION_MINOR DEC(__FRT_minor__)
#   define COMPILER_VERSION_PATCH DEC(__FRT_patchlevel__)
# elif defined(__FRT_version__)
        PRINT *, 'INFO:compiler_version['//__FRT_version__//']'
# endif
#else
        PRINT *, 'INFO:compiler[]'
#endif
#if defined(__CRAYXT_COMPUTE_LINUX_TARGET)
        PRINT *, 'INFO:compiler_wrapper[CrayPrgEnv]'
#endif

#if 0
! Identify the platform
#endif
#if defined(__linux) || defined(__linux__) || defined(linux)
        PRINT *, 'INFO:platform[Linux]'
#elif defined(__CYGWIN__)
        PRINT *, 'INFO:platform[Cygwin]'
#elif defined(__MINGW32__)
        PRINT *, 'INFO:platform[MinGW]'
#elif defined(__APPLE__)
        PRINT *, 'INFO:platform[Darwin]'
#elif defined(_WIN32) || defined(__WIN32__) || defined(WIN32)
        PRINT *, 'INFO:platform[Windows]'
#elif defined(__FreeBSD__) || defined(__FreeBSD)
        PRINT *, 'INFO:platform[FreeBSD]'
#elif defined(__NetBSD__) || defined(__NetBSD)
        PRINT *, 'INFO:platform[NetBSD]'
#elif defined(__OpenBSD__) || defined(__OPENBSD)
        PRINT *, 'INFO:platform[OpenBSD]'
#elif defined(__sun) || defined(sun)
        PRINT *, 'INFO:platform[SunOS]'
#elif defined(_AIX) || defined(__AIX) || defined(__AIX__) || defined(__aix) || defined(__aix__)
        PRINT *, 'INFO:platform[AIX]'
#elif defined(__hpux) || defined(__hpux__)
        PRINT *, 'INFO:platform[HP-UX]'
#elif defined(__HAIKU__)
        PRINT *, 'INFO:platform[Haiku]'
#elif defined(__BeOS) || defined(__BEOS__) || defined(_BEOS)
        PRINT *, 'INFO:platform[BeOS]'
#elif defined(__QNX__) || defined(__QNXNTO__)
        PRINT *, 'INFO:platform[QNX]'
#elif defined(__tru64) || defined(_tru64) || defined(__TRU64__)
        PRINT *, 'INFO:platform[Tru64]'
#elif defined(__riscos) || defined(__riscos__)
        PRINT *, 'INFO:platform[RISCos]'
#elif defined(__sinix) || defined(__sinix__) || defined(__SINIX__)
        PRINT *, 'INFO:platform[SINIX]'
#elif defined(__UNIX_SV__)
        PRINT *, 'INFO:platform[UNIX_SV]'
#elif defined(__bsdos__)
        PRINT *, 'INFO:platform[BSDOS]'
#elif defined(_MPRAS) || defined(MPRAS)
        PRINT *, 'INFO:platform[MP-RAS]'
#elif defined(__osf) || defined(__osf__)
        PRINT *, 'INFO:platform[OSF1]'
#elif defined(_SCO_SV) || defined(SCO_SV) || defined(sco_sv)
        PRINT *, 'INFO:platform[SCO_SV]'
#elif defined(__ultrix) || defined(__ultrix__) || defined(_ULTRIX)
        PRINT *, 'INFO:platform[ULTRIX]'
#elif defined(__XENIX__) || defined(_XENIX) || defined(XENIX)
        PRINT *, 'INFO:platform[Xenix]'
#else
        PRINT *, 'INFO:platform[]'
#endif
#if defined(_WIN32) && (defined(__INTEL_COMPILER) || defined(__ICC))
# if defined(_M_IA64)
        PRINT *, 'INFO:arch[IA64]'
# elif defined(_M_X64) || defined(_M_AMD64)
        PRINT *, 'INFO:arch[x64]'
# elif defined(_M_IX86)
        PRINT *, 'INFO:arch[X86]'
# endif
#endif

#if 0
! Encode compiler version digits
#endif
#define DEC_8(n) (((n) / 10000000) % 10)
#define DEC_7(n) (((n) / 1000000)  % 10)
#define DEC_6(n) (((n) / 100000)   % 10)
#define DEC_5(n) (((n) / 10000)    % 10)
#define DEC_4(n) (((n) / 1000)     % 10)
#define DEC_3(n) (((n) / 100)      % 10)
#define DEC_2(n) (((n) / 10)       % 10)
#define DEC_1(n) (((n)    )        % 10)
#define HEX_8(n) ((n)>>28 & 0xF)
#define HEX_7(n) ((n)>>24 & 0xF)
#define HEX_6(n) ((n)>>20 & 0xF)
#define HEX_5(n) ((n)>>16 & 0xF)
#define HEX_4(n) ((n)>>12 & 0xF)
#define HEX_3(n) ((n)>>8  & 0xF)
#define HEX_2(n) ((n)>>4  & 0xF)
#define HEX_1(n) ((n)     & 0xF)

#if defined(COMPILER_VERSION_MAJOR)
# undef DEC
# undef HEX
# define DEC(n) DEC_1(n)
# define HEX(n) HEX_1(n)
# if COMPILER_VERSION_MAJOR == 0
        PRINT *, 'INFO:compiler_version_MAJOR_digit_1[0]'
# elif COMPILER_VERSION_MAJOR == 1
        PRINT *, 'INFO:compiler_version_MAJOR_digit_1[1]'
# elif COMPILER_VERSION_MAJOR == 2
        PRINT *, 'INFO:compiler_version_MAJOR_digit_1[2]'
# elif COMPILER_VERSION_MAJOR == 3
        PRINT *, 'INFO:compiler_version_MAJOR_digit_1[3]'
# elif COMPILER_VERSION_MAJOR == 4
        PRINT *, 'INFO:compiler_version_MAJOR_digit_1[4]'
# elif COMPILER_VERSION_MAJOR == 5
        PRINT *, 'INFO:compiler_version_MAJOR_digit_1[5]'
# elif COMPILER_VERSION_MAJOR == 6
        PRINT *, 'INFO:compiler_version_MAJOR_digit_1[6]'
# elif COMPILER_VERSION_MAJOR == 7
        PRINT *, 'INFO:compiler_version_MAJOR_digit_1[7]'
# elif COMPILER_VERSION_MAJOR == 8
        PRINT *, 'INFO:compiler_version_MAJOR_digit_1[8]'
# elif COMPILER_VERSION_MAJOR == 9
        PRINT *, 'INFO:compiler_version_MAJOR_digit_1[9]'
# endif

# undef DEC
# undef HEX
# define DEC(n) DEC_2(n)
# define HEX(n) HEX_2(n)
# if COMPILER_VERSION_MAJOR == 0
        PRINT *, 'INFO:compiler_version_MAJOR_digit_2[0]'
# elif COMPILER_VERSION_MAJOR == 1
        PRINT *, 'INFO:compiler_version_MAJOR_digit_2[1]'
# elif COMPILER_VERSION_MAJOR == 2
        PRINT *, 'INFO:compiler_version_MAJOR_digit_2[2]'
# elif COMPILER_VERSION_MAJOR == 3
        PRINT *, 'INFO:compiler_version_MAJOR_digit_2[3]'
# elif COMPILER_VERSION_MAJOR == 4
        PRINT *, 'INFO:compiler_version_MAJOR_digit_2[4]'
# elif COMPILER_VERSION_MAJOR == 5
        PRINT *, 'INFO:compiler_version_MAJOR_digit_2[5]'
# elif COMPILER_VERSION_MAJOR == 6
        PRINT *, 'INFO:compiler_version_MAJOR_digit_2[6]'
# elif COMPILER_VERSION_MAJOR == 7
        PRINT *, 'INFO:compiler_version_MAJOR_digit_2[7]'
# elif COMPILER_VERSION_MAJOR == 8
        PRINT *, 'INFO:compiler_version_MAJOR_digit_2[8]'
# elif COMPILER_VERSION_MAJOR == 9
        PRINT *, 'INFO:compiler_version_MAJOR_digit_2[9]'
# endif

# undef DEC
# undef HEX
# define DEC(n) DEC_3(n)
# define HEX(n) HEX_3(n)
# if COMPILER_VERSION_MAJOR == 0
        PRINT *, 'INFO:compiler_version_MAJOR_digit_3[0]'
# elif COMPILER_VERSION_MAJOR == 1
        PRINT *, 'INFO:compiler_version_MAJOR_digit_3[1]'
# elif COMPILER_VERSION_MAJOR == 2
        PRINT *, 'INFO:compiler_version_MAJOR_digit_3[2]'
# elif COMPILER_VERSION_MAJOR == 3
        PRINT *, 'INFO:compiler_version_MAJOR_digit_3[3]'
# elif COMPILER_VERSION_MAJOR == 4
        PRINT *, 'INFO:compiler_version_MAJOR_digit_3[4]'
# elif COMPILER_VERSION_MAJOR == 5
        PRINT *, 'INFO:compiler_version_MAJOR_digit_3[5]'
# elif COMPILER_VERSION_MAJOR == 6
        PRINT *, 'INFO:compiler_version_MAJOR_digit_3[6]'
# elif COMPILER_VERSION_MAJOR == 7
        PRINT *, 'INFO:compiler_version_MAJOR_digit_3[7]'
# elif COMPILER_VERSION_MAJOR == 8
        PRINT *, 'INFO:compiler_version_MAJOR_digit_3[8]'
# elif COMPILER_VERSION_MAJOR == 9
        PRINT *, 'INFO:compiler_version_MAJOR_digit_3[9]'
# endif

# undef DEC
# undef HEX
# define DEC(n) DEC_4(n)
# define HEX(n) HEX_4(n)
# if COMPILER_VERSION_MAJOR == 0
        PRINT *, 'INFO:compiler_version_MAJOR_digit_4[0]'
# elif COMPILER_VERSION_MAJOR == 1
        PRINT *, 'INFO:compiler_version_MAJOR_digit_4[1]'
# elif COMPILER_VERSION_MAJOR == 2
        PRINT *, 'INFO:compiler_version_MAJOR_digit_4[2]'
# elif COMPILER_VERSION_MAJOR == 3
        PRINT *, 'INFO:compiler_version_MAJOR_digit_4[3]'
# elif COMPILER_VERSION_MAJOR == 4
        PRINT *, 'INFO:compiler_version_MAJOR_digit_4[4]'
# elif COMPILER_VERSION_MAJOR == 5
        PRINT *, 'INFO:compiler_version_MAJOR_digit_4[5]'
# elif COMPILER_VERSION_MAJOR == 6
        PRINT *, 'INFO:compiler_version_MAJOR_digit_4[6]'
# elif COMPILER_VERSION_MAJOR == 7
        PRINT *, 'INFO:compiler_version_MAJOR_digit_4[7]'
# elif COMPILER_VERSION_MAJOR == 8
        PRINT *, 'INFO:compiler_version_MAJOR_digit_4[8]'
# elif COMPILER_VERSION_MAJOR == 9
        PRINT *, 'INFO:compiler_version_MAJOR_digit_4[9]'
# endif

# undef DEC
# undef HEX
# define DEC(n) DEC_5(n)
# define HEX(n) HEX_5(n)
# if COMPILER_VERSION_MAJOR == 0
        PRINT *, 'INFO:compiler_version_MAJOR_digit_5[0]'
# elif COMPILER_VERSION_MAJOR == 1
        PRINT *, 'INFO:compiler_version_MAJOR_digit_5[1]'
# elif COMPILER_VERSION_MAJOR == 2
        PRINT *, 'INFO:compiler_version_MAJOR_digit_5[2]'
# elif COMPILER_VERSION_MAJOR == 3
        PRINT *, 'INFO:compiler_version_MAJOR_digit_5[3]'
# elif COMPILER_VERSION_MAJOR == 4
        PRINT *, 'INFO:compiler_version_MAJOR_digit_5[4]'
# elif COMPILER_VERSION_MAJOR == 5
        PRINT *, 'INFO:compiler_version_MAJOR_digit_5[5]'
# elif COMPILER_VERSION_MAJOR == 6
        PRINT *, 'INFO:compiler_version_MAJOR_digit_5[6]'
# elif COMPILER_VERSION_MAJOR == 7
        PRINT *, 'INFO:compiler_version_MAJOR_digit_5[7]'
# elif COMPILER_VERSION_MAJOR == 8
        PRINT *, 'INFO:compiler_version_MAJOR_digit_5[8]'
# elif COMPILER_VERSION_MAJOR == 9
        PRINT *, 'INFO:compiler_version_MAJOR_digit_5[9]'
# endif

# undef DEC
# undef HEX
# define DEC(n) DEC_6(n)
# define HEX(n) HEX_6(n)
# if COMPILER_VERSION_MAJOR == 0
        PRINT *, 'INFO:compiler_version_MAJOR_digit_6[0]'
# elif COMPILER_VERSION_MAJOR == 1
        PRINT *, 'INFO:compiler_version_MAJOR_digit_6[1]'
# elif COMPILER_VERSION_MAJOR == 2
        PRINT *, 'INFO:compiler_version_MAJOR_digit_6[2]'
# elif COMPILER_VERSION_MAJOR == 3
        PRINT *, 'INFO:compiler_version_MAJOR_digit_6[3]'
# elif COMPILER_VERSION_MAJOR == 4
        PRINT *, 'INFO:compiler_version_MAJOR_digit_6[4]'
# elif COMPILER_VERSION_MAJOR == 5
        PRINT *, 'INFO:compiler_version_MAJOR_digit_6[5]'
# elif COMPILER_VERSION_MAJOR == 6
        PRINT *, 'INFO:compiler_version_MAJOR_digit_6[6]'
# elif COMPILER_VERSION_MAJOR == 7
        PRINT *, 'INFO:compiler_version_MAJOR_digit_6[7]'
# elif COMPILER_VERSION_MAJOR == 8
        PRINT *, 'INFO:compiler_version_MAJOR_digit_6[8]'
# elif COMPILER_VERSION_MAJOR == 9
        PRINT *, 'INFO:compiler_version_MAJOR_digit_6[9]'
# endif

# undef DEC
# undef HEX
# define DEC(n) DEC_7(n)
# define HEX(n) HEX_7(n)
# if COMPILER_VERSION_MAJOR == 0
        PRINT *, 'INFO:compiler_version_MAJOR_digit_7[0]'
# elif COMPILER_VERSION_MAJOR == 1
        PRINT *, 'INFO:compiler_version_MAJOR_digit_7[1]'
# elif COMPILER_VERSION_MAJOR == 2
        PRINT *, 'INFO:compiler_version_MAJOR_digit_7[2]'
# elif COMPILER_VERSION_MAJOR == 3
        PRINT *, 'INFO:compiler_version_MAJOR_digit_7[3]'
# elif COMPILER_VERSION_MAJOR == 4
        PRINT *, 'INFO:compiler_version_MAJOR_digit_7[4]'
# elif COMPILER_VERSION_MAJOR == 5
        PRINT *, 'INFO:compiler_version_MAJOR_digit_7[5]'
# elif COMPILER_VERSION_MAJOR == 6
        PRINT *, 'INFO:compiler_version_MAJOR_digit_7[6]'
# elif COMPILER_VERSION_MAJOR == 7
        PRINT *, 'INFO:compiler_version_MAJOR_digit_7[7]'
# elif COMPILER_VERSION_MAJOR == 8
        PRINT *, 'INFO:compiler_version_MAJOR_digit_7[8]'
# elif COMPILER_VERSION_MAJOR == 9
        PRINT *, 'INFO:compiler_version_MAJOR_digit_7[9]'
# endif

# undef DEC
# undef HEX
# define DEC(n) DEC_8(n)
# define HEX(n) HEX_8(n)
# if COMPILER_VERSION_MAJOR == 0
        PRINT *, 'INFO:compiler_version_MAJOR_digit_8[0]'
# elif COMPILER_VERSION_MAJOR == 1
        PRINT *, 'INFO:compiler_version_MAJOR_digit_8[1]'
# elif COMPILER_VERSION_MAJOR == 2
        PRINT *, 'INFO:compiler_version_MAJOR_digit_8[2]'
# elif COMPILER_VERSION_MAJOR == 3
        PRINT *, 'INFO:compiler_version_MAJOR_digit_8[3]'
# elif COMPILER_VERSION_MAJOR == 4
        PRINT *, 'INFO:compiler_version_MAJOR_digit_8[4]'
# elif COMPILER_VERSION_MAJOR == 5
        PRINT *, 'INFO:compiler_version_MAJOR_digit_8[5]'
# elif COMPILER_VERSION_MAJOR == 6
        PRINT *, 'INFO:compiler_version_MAJOR_digit_8[6]'
# elif COMPILER_VERSION_MAJOR == 7
        PRINT *, 'INFO:compiler_version_MAJOR_digit_8[7]'
# elif COMPILER_VERSION_MAJOR == 8
        PRINT *, 'INFO:compiler_version_MAJOR_digit_8[8]'
# elif COMPILER_VERSION_MAJOR == 9
        PRINT *, 'INFO:compiler_version_MAJOR_digit_8[9]'
# endif

#endif
#if defined(COMPILER_VERSION_MINOR)
# undef DEC
# undef HEX
# define DEC(n) DEC_1(n)
# define HEX(n) HEX_1(n)
# if COMPILER_VERSION_MINOR == 0
        PRINT *, 'INFO:compiler_version_MINOR_digit_1[0]'
# elif COMPILER_VERSION_MINOR == 1
        PRINT *, 'INFO:compiler_version_MINOR_digit_1[1]'
# elif COMPILER_VERSION_MINOR == 2
        PRINT *, 'INFO:compiler_version_MINOR_digit_1[2]'
# elif COMPILER_VERSION_MINOR == 3
        PRINT *, 'INFO:compiler_version_MINOR_digit_1[3]'
# elif COMPILER_VERSION_MINOR == 4
        PRINT *, 'INFO:compiler_version_MINOR_digit_1[4]'
# elif COMPILER_VERSION_MINOR == 5
        PRINT *, 'INFO:compiler_version_MINOR_digit_1[5]'
# elif COMPILER_VERSION_MINOR == 6
        PRINT *, 'INFO:compiler_version_MINOR_digit_1[6]'
# elif COMPILER_VERSION_MINOR == 7
        PRINT *, 'INFO:compiler_version_MINOR_digit_1[7]'
# elif COMPILER_VERSION_MINOR == 8
        PRINT *, 'INFO:compiler_version_MINOR_digit_1[8]'
# elif COMPILER_VERSION_MINOR == 9
        PRINT *, 'INFO:compiler_version_MINOR_digit_1[9]'
# endif

# undef DEC
# undef HEX
# define DEC(n) DEC_2(n)
# define HEX(n) HEX_2(n)
# if COMPILER_VERSION_MINOR == 0
        PRINT *, 'INFO:compiler_version_MINOR_digit_2[0]'
# elif COMPILER_VERSION_MINOR == 1
        PRINT *, 'INFO:compiler_version_MINOR_digit_2[1]'
# elif COMPILER_VERSION_MINOR == 2
        PRINT *, 'INFO:compiler_version_MINOR_digit_2[2]'
# elif COMPILER_VERSION_MINOR == 3
        PRINT *, 'INFO:compiler_version_MINOR_digit_2[3]'
# elif COMPILER_VERSION_MINOR == 4
        PRINT *, 'INFO:compiler_version_MINOR_digit_2[4]'
# elif COMPILER_VERSION_MINOR == 5
        PRINT *, 'INFO:compiler_version_MINOR_digit_2[5]'
# elif COMPILER_VERSION_MINOR == 6
        PRINT *, 'INFO:compiler_version_MINOR_digit_2[6]'
# elif COMPILER_VERSION_MINOR == 7
        PRINT *, 'INFO:compiler_version_MINOR_digit_2[7]'
# elif COMPILER_VERSION_MINOR == 8
        PRINT *, 'INFO:compiler_version_MINOR_digit_2[8]'
# elif COMPILER_VERSION_MINOR == 9
        PRINT *, 'INFO:compiler_version_MINOR_digit_2[9]'
# endif

# undef DEC
# undef HEX
# define DEC(n) DEC_3(n)
# define HEX(n) HEX_3(n)
# if COMPILER_VERSION_MINOR == 0
        PRINT *, 'INFO:compiler_version_MINOR_digit_3[0]'
# elif COMPILER_VERSION_MINOR == 1
        PRINT *, 'INFO:compiler_version_MINOR_digit_3[1]'
# elif COMPILER_VERSION_MINOR == 2
        PRINT *, 'INFO:compiler_version_MINOR_digit_3[2]'
# elif COMPILER_VERSION_MINOR == 3
        PRINT *, 'INFO:compiler_version_MINOR_digit_3[3]'
# elif COMPILER_VERSION_MINOR == 4
        PRINT *, 'INFO:compiler_version_MINOR_digit_3[4]'
# elif COMPILER_VERSION_MINOR == 5
        PRINT *, 'INFO:compiler_version_MINOR_digit_3[5]'
# elif COMPILER_VERSION_MINOR == 6
        PRINT *, 'INFO:compiler_version_MINOR_digit_3[6]'
# elif COMPILER_VERSION_MINOR == 7
        PRINT *, 'INFO:compiler_version_MINOR_digit_3[7]'
# elif COMPILER_VERSION_MINOR == 8
        PRINT *, 'INFO:compiler_version_MINOR_digit_3[8]'
# elif COMPILER_VERSION_MINOR == 9
        PRINT *, 'INFO:compiler_version_MINOR_digit_3[9]'
# endif

# undef DEC
# undef HEX
# define DEC(n) DEC_4(n)
# define HEX(n) HEX_4(n)
# if COMPILER_VERSION_MINOR == 0
        PRINT *, 'INFO:compiler_version_MINOR_digit_4[0]'
# elif COMPILER_VERSION_MINOR == 1
        PRINT *, 'INFO:compiler_version_MINOR_digit_4[1]'
# elif COMPILER_VERSION_MINOR == 2
        PRINT *, 'INFO:compiler_version_MINOR_digit_4[2]'
# elif COMPILER_VERSION_MINOR == 3
        PRINT *, 'INFO:compiler_version_MINOR_digit_4[3]'
# elif COMPILER_VERSION_MINOR == 4
        PRINT *, 'INFO:compiler_version_MINOR_digit_4[4]'
# elif COMPILER_VERSION_MINOR == 5
        PRINT *, 'INFO:compiler_version_MINOR_digit_4[5]'
# elif COMPILER_VERSION_MINOR == 6
        PRINT *, 'INFO:compiler_version_MINOR_digit_4[6]'
# elif COMPILER_VERSION_MINOR == 7
        PRINT *, 'INFO:compiler_version_MINOR_digit_4[7]'
# elif COMPILER_VERSION_MINOR == 8
        PRINT *, 'INFO:compiler_version_MINOR_digit_4[8]'
# elif COMPILER_VERSION_MINOR == 9
        PRINT *, 'INFO:compiler_version_MINOR_digit_4[9]'
# endif

# undef DEC
# undef HEX
# define DEC(n) DEC_5(n)
# define HEX(n) HEX_5(n)
# if COMPILER_VERSION_MINOR == 0
        PRINT *, 'INFO:compiler_version_MINOR_digit_5[0]'
# elif COMPILER_VERSION_MINOR == 1
        PRINT *, 'INFO:compiler_version_MINOR_digit_5[1]'
# elif COMPILER_VERSION_MINOR == 2
        PRINT *, 'INFO:compiler_version_MINOR_digit_5[2]'
# elif COMPILER_VERSION_MINOR == 3
        PRINT *, 'INFO:compiler_version_MINOR_digit_5[3]'
# elif COMPILER_VERSION_MINOR == 4
        PRINT *, 'INFO:compiler_version_MINOR_digit_5[4]'
# elif COMPILER_VERSION_MINOR == 5
        PRINT *, 'INFO:compiler_version_MINOR_digit_5[5]'
# elif COMPILER_VERSION_MINOR == 6
        PRINT *, 'INFO:compiler_version_MINOR_digit_5[6]'
# elif COMPILER_VERSION_MINOR == 7
        PRINT *, 'INFO:compiler_version_MINOR_digit_5[7]'
# elif COMPILER_VERSION_MINOR == 8
        PRINT *, 'INFO:compiler_version_MINOR_digit_5[8]'
# elif COMPILER_VERSION_MINOR == 9
        PRINT *, 'INFO:compiler_version_MINOR_digit_5[9]'
# endif

# undef DEC
# undef HEX
# define DEC(n) DEC_6(n)
# define HEX(n) HEX_6(n)
# if COMPILER_VERSION_MINOR == 0
        PRINT *, 'INFO:compiler_version_MINOR_digit_6[0]'
# elif COMPILER_VERSION_MINOR == 1
        PRINT *, 'INFO:compiler_version_MINOR_digit_6[1]'
# elif COMPILER_VERSION_MINOR == 2
        PRINT *, 'INFO:compiler_version_MINOR_digit_6[2]'
# elif COMPILER_VERSION_MINOR == 3
        PRINT *, 'INFO:compiler_version_MINOR_digit_6[3]'
# elif COMPILER_VERSION_MINOR == 4
        PRINT *, 'INFO:compiler_version_MINOR_digit_6[4]'
# elif COMPILER_VERSION_MINOR == 5
        PRINT *, 'INFO:compiler_version_MINOR_digit_6[5]'
# elif COMPILER_VERSION_MINOR == 6
        PRINT *, 'INFO:compiler_version_MINOR_digit_6[6]'
# elif COMPILER_VERSION_MINOR == 7
        PRINT *, 'INFO:compiler_version_MINOR_digit_6[7]'
# elif COMPILER_VERSION_MINOR == 8
        PRINT *, 'INFO:compiler_version_MINOR_digit_6[8]'
# elif COMPILER_VERSION_MINOR == 9
        PRINT *, 'INFO:compiler_version_MINOR_digit_6[9]'
# endif

# undef DEC
# undef HEX
# define DEC(n) DEC_7(n)
# define HEX(n) HEX_7(n)
# if COMPILER_VERSION_MINOR == 0
        PRINT *, 'INFO:compiler_version_MINOR_digit_7[0]'
# elif COMPILER_VERSION_MINOR == 1
        PRINT *, 'INFO:compiler_version_MINOR_digit_7[1]'
# elif COMPILER_VERSION_MINOR == 2
        PRINT *, 'INFO:compiler_version_MINOR_digit_7[2]'
# elif COMPILER_VERSION_MINOR == 3
        PRINT *, 'INFO:compiler_version_MINOR_digit_7[3]'
# elif COMPILER_VERSION_MINOR == 4
        PRINT *, 'INFO:compiler_version_MINOR_digit_7[4]'
# elif COMPILER_VERSION_MINOR == 5
        PRINT *, 'INFO:compiler_version_MINOR_digit_7[5]'
# elif COMPILER_VERSION_MINOR == 6
        PRINT *, 'INFO:compiler_version_MINOR_digit_7[6]'
# elif COMPILER_VERSION_MINOR == 7
        PRINT *, 'INFO:compiler_version_MINOR_digit_7[7]'
# elif COMPILER_VERSION_MINOR == 8
        PRINT *, 'INFO:compiler_version_MINOR_digit_7[8]'
# elif COMPILER_VERSION_MINOR == 9
        PRINT *, 'INFO:compiler_version_MINOR_digit_7[9]'
# endif

# undef DEC
# undef HEX
# define DEC(n) DEC_8(n)
# define HEX(n) HEX_8(n)
# if COMPILER_VERSION_MINOR == 0
        PRINT *, 'INFO:compiler_version_MINOR_digit_8[0]'
# elif COMPILER_VERSION_MINOR == 1
        PRINT *, 'INFO:compiler_version_MINOR_digit_8[1]'
# elif COMPILER_VERSION_MINOR == 2
        PRINT *, 'INFO:compiler_version_MINOR_digit_8[2]'
# elif COMPILER_VERSION_MINOR == 3
        PRINT *, 'INFO:compiler_version_MINOR_digit_8[3]'
# elif COMPILER_VERSION_MINOR == 4
        PRINT *, 'INFO:compiler_version_MINOR_digit_8[4]'
# elif COMPILER_VERSION_MINOR == 5
        PRINT *, 'INFO:compiler_version_MINOR_digit_8[5]'
# elif COMPILER_VERSION_MINOR == 6
        PRINT *, 'INFO:compiler_version_MINOR_digit_8[6]'
# elif COMPILER_VERSION_MINOR == 7
        PRINT *, 'INFO:compiler_version_MINOR_digit_8[7]'
# elif COMPILER_VERSION_MINOR == 8
        PRINT *, 'INFO:compiler_version_MINOR_digit_8[8]'
# elif COMPILER_VERSION_MINOR == 9
        PRINT *, 'INFO:compiler_version_MINOR_digit_8[9]'
# endif

#endif
#if defined(COMPILER_VERSION_PATCH)
# undef DEC
# undef HEX
# define DEC(n) DEC_1(n)
# define HEX(n) HEX_1(n)
# if COMPILER_VERSION_PATCH == 0
        PRINT *, 'INFO:compiler_version_PATCH_digit_1[0]'
# elif COMPILER_VERSION_PATCH == 1
        PRINT *, 'INFO:compiler_version_PATCH_digit_1[1]'
# elif COMPILER_VERSION_PATCH == 2
        PRINT *, 'INFO:compiler_version_PATCH_digit_1[2]'
# elif COMPILER_VERSION_PATCH == 3
        PRINT *, 'INFO:compiler_version_PATCH_digit_1[3]'
# elif COMPILER_VERSION_PATCH == 4
        PRINT *, 'INFO:compiler_version_PATCH_digit_1[4]'
# elif COMPILER_VERSION_PATCH == 5
        PRINT *, 'INFO:compiler_version_PATCH_digit_1[5]'
# elif COMPILER_VERSION_PATCH == 6
        PRINT *, 'INFO:compiler_version_PATCH_digit_1[6]'
# elif COMPILER_VERSION_PATCH == 7
        PRINT *, 'INFO:compiler_version_PATCH_digit_1[7]'
# elif COMPILER_VERSION_PATCH == 8
        PRINT *, 'INFO:compiler_version_PATCH_digit_1[8]'
# elif COMPILER_VERSION_PATCH == 9
        PRINT *, 'INFO:compiler_version_PATCH_digit_1[9]'
# endif

# undef DEC
# undef HEX
# define DEC(n) DEC_2(n)
# define HEX(n) HEX_2(n)
# if COMPILER_VERSION_PATCH == 0
        PRINT *, 'INFO:compiler_version_PATCH_digit_2[0]'
# elif COMPILER_VERSION_PATCH == 1
        PRINT *, 'INFO:compiler_version_PATCH_digit_2[1]'
# elif COMPILER_VERSION_PATCH == 2
        PRINT *, 'INFO:compiler_version_PATCH_digit_2[2]'
# elif COMPILER_VERSION_PATCH == 3
        PRINT *, 'INFO:compiler_version_PATCH_digit_2[3]'
# elif COMPILER_VERSION_PATCH == 4
        PRINT *, 'INFO:compiler_version_PATCH_digit_2[4]'
# elif COMPILER_VERSION_PATCH == 5
        PRINT *, 'INFO:compiler_version_PATCH_digit_2[5]'
# elif COMPILER_VERSION_PATCH == 6
        PRINT *, 'INFO:compiler_version_PATCH_digit_2[6]'
# elif COMPILER_VERSION_PATCH == 7
        PRINT *, 'INFO:compiler_version_PATCH_digit_2[7]'
# elif COMPILER_VERSION_PATCH == 8
        PRINT *, 'INFO:compiler_version_PATCH_digit_2[8]'
# elif COMPILER_VERSION_PATCH == 9
        PRINT *, 'INFO:compiler_version_PATCH_digit_2[9]'
# endif

# undef DEC
# undef HEX
# define DEC(n) DEC_3(n)
# define HEX(n) HEX_3(n)
# if COMPILER_VERSION_PATCH == 0
        PRINT *, 'INFO:compiler_version_PATCH_digit_3[0]'
# elif COMPILER_VERSION_PATCH == 1
        PRINT *, 'INFO:compiler_version_PATCH_digit_3[1]'
# elif COMPILER_VERSION_PATCH == 2
        PRINT *, 'INFO:compiler_version_PATCH_digit_3[2]'
# elif COMPILER_VERSION_PATCH == 3
        PRINT *, 'INFO:compiler_version_PATCH_digit_3[3]'
# elif COMPILER_VERSION_PATCH == 4
        PRINT *, 'INFO:compiler_version_PATCH_digit_3[4]'
# elif COMPILER_VERSION_PATCH == 5
        PRINT *, 'INFO:compiler_version_PATCH_digit_3[5]'
# elif COMPILER_VERSION_PATCH == 6
        PRINT *, 'INFO:compiler_version_PATCH_digit_3[6]'
# elif COMPILER_VERSION_PATCH == 7
        PRINT *, 'INFO:compiler_version_PATCH_digit_3[7]'
# elif COMPILER_VERSION_PATCH == 8
        PRINT *, 'INFO:compiler_version_PATCH_digit_3[8]'
# elif COMPILER_VERSION_PATCH == 9
        PRINT *, 'INFO:compiler_version_PATCH_digit_3[9]'
# endif

# undef DEC
# undef HEX
# define DEC(n) DEC_4(n)
# define HEX(n) HEX_4(n)
# if COMPILER_VERSION_PATCH == 0
        PRINT *, 'INFO:compiler_version_PATCH_digit_4[0]'
# elif COMPILER_VERSION_PATCH == 1
        PRINT *, 'INFO:compiler_version_PATCH_digit_4[1]'
# elif COMPILER_VERSION_PATCH == 2
        PRINT *, 'INFO:compiler_version_PATCH_digit_4[2]'
# elif COMPILER_VERSION_PATCH == 3
        PRINT *, 'INFO:compiler_version_PATCH_digit_4[3]'
# elif COMPILER_VERSION_PATCH == 4
        PRINT *, 'INFO:compiler_version_PATCH_digit_4[4]'
# elif COMPILER_VERSION_PATCH == 5
        PRINT *, 'INFO:compiler_version_PATCH_digit_4[5]'
# elif COMPILER_VERSION_PATCH == 6
        PRINT *, 'INFO:compiler_version_PATCH_digit_4[6]'
# elif COMPILER_VERSION_PATCH == 7
        PRINT *, 'INFO:compiler_version_PATCH_digit_4[7]'
# elif COMPILER_VERSION_PATCH == 8
        PRINT *, 'INFO:compiler_version_PATCH_digit_4[8]'
# elif COMPILER_VERSION_PATCH == 9
        PRINT *, 'INFO:compiler_version_PATCH_digit_4[9]'
# endif

# undef DEC
# undef HEX
# define DEC(n) DEC_5(n)
# define HEX(n) HEX_5(n)
# if COMPILER_VERSION_PATCH == 0
        PRINT *, 'INFO:compiler_version_PATCH_digit_5[0]'
# elif COMPILER_VERSION_PATCH == 1
        PRINT *, 'INFO:compiler_version_PATCH_digit_5[1]'
# elif COMPILER_VERSION_PATCH == 2
        PRINT *, 'INFO:compiler_version_PATCH_digit_5[2]'
# elif COMPILER_VERSION_PATCH == 3
        PRINT *, 'INFO:compiler_version_PATCH_digit_5[3]'
# elif COMPILER_VERSION_PATCH == 4
        PRINT *, 'INFO:compiler_version_PATCH_digit_5[4]'
# elif COMPILER_VERSION_PATCH == 5
        PRINT *, 'INFO:compiler_version_PATCH_digit_5[5]'
# elif COMPILER_VERSION_PATCH == 6
        PRINT *, 'INFO:compiler_version_PATCH_digit_5[6]'
# elif COMPILER_VERSION_PATCH == 7
        PRINT *, 'INFO:compiler_version_PATCH_digit_5[7]'
# elif COMPILER_VERSION_PATCH == 8
        PRINT *, 'INFO:compiler_version_PATCH_digit_5[8]'
# elif COMPILER_VERSION_PATCH == 9
        PRINT *, 'INFO:compiler_version_PATCH_digit_5[9]'
# endif

# undef DEC
# undef HEX
# define DEC(n) DEC_6(n)
# define HEX(n) HEX_6(n)
# if COMPILER_VERSION_PATCH == 0
        PRINT *, 'INFO:compiler_version_PATCH_digit_6[0]'
# elif COMPILER_VERSION_PATCH == 1
        PRINT *, 'INFO:compiler_version_PATCH_digit_6[1]'
# elif COMPILER_VERSION_PATCH == 2
        PRINT *, 'INFO:compiler_version_PATCH_digit_6[2]'
# elif COMPILER_VERSION_PATCH == 3
        PRINT *, 'INFO:compiler_version_PATCH_digit_6[3]'
# elif COMPILER_VERSION_PATCH == 4
        PRINT *, 'INFO:compiler_version_PATCH_digit_6[4]'
# elif COMPILER_VERSION_PATCH == 5
        PRINT *, 'INFO:compiler_version_PATCH_digit_6[5]'
# elif COMPILER_VERSION_PATCH == 6
        PRINT *, 'INFO:compiler_version_PATCH_digit_6[6]'
# elif COMPILER_VERSION_PATCH == 7
        PRINT *, 'INFO:compiler_version_PATCH_digit_6[7]'
# elif COMPILER_VERSION_PATCH == 8
        PRINT *, 'INFO:compiler_version_PATCH_digit_6[8]'
# elif COMPILER_VERSION_PATCH == 9
        PRINT *, 'INFO:compiler_version_PATCH_digit_6[9]'
# endif

# undef DEC
# undef HEX
# define DEC(n) DEC_7(n)
# define HEX(n) HEX_7(n)
# if COMPILER_VERSION_PATCH == 0
        PRINT *, 'INFO:compiler_version_PATCH_digit_7[0]'
# elif COMPILER_VERSION_PATCH == 1
        PRINT *, 'INFO:compiler_version_PATCH_digit_7[1]'
# elif COMPILER_VERSION_PATCH == 2
        PRINT *, 'INFO:compiler_version_PATCH_digit_7[2]'
# elif COMPILER_VERSION_PATCH == 3
        PRINT *, 'INFO:compiler_version_PATCH_digit_7[3]'
# elif COMPILER_VERSION_PATCH == 4
        PRINT *, 'INFO:compiler_version_PATCH_digit_7[4]'
# elif COMPILER_VERSION_PATCH == 5
        PRINT *, 'INFO:compiler_version_PATCH_digit_7[5]'
# elif COMPILER_VERSION_PATCH == 6
        PRINT *, 'INFO:compiler_version_PATCH_digit_7[6]'
# elif COMPILER_VERSION_PATCH == 7
        PRINT *, 'INFO:compiler_version_PATCH_digit_7[7]'
# elif COMPILER_VERSION_PATCH == 8
        PRINT *, 'INFO:compiler_version_PATCH_digit_7[8]'
# elif COMPILER_VERSION_PATCH == 9
        PRINT *, 'INFO:compiler_version_PATCH_digit_7[9]'
# endif

# undef DEC
# undef HEX
# define DEC(n) DEC_8(n)
# define HEX(n) HEX_8(n)
# if COMPILER_VERSION_PATCH == 0
        PRINT *, 'INFO:compiler_version_PATCH_digit_8[0]'
# elif COMPILER_VERSION_PATCH == 1
        PRINT *, 'INFO:compiler_version_PATCH_digit_8[1]'
# elif COMPILER_VERSION_PATCH == 2
        PRINT *, 'INFO:compiler_version_PATCH_digit_8[2]'
# elif COMPILER_VERSION_PATCH == 3
        PRINT *, 'INFO:compiler_version_PATCH_digit_8[3]'
# elif COMPILER_VERSION_PATCH == 4
        PRINT *, 'INFO:compiler_version_PATCH_digit_8[4]'
# elif COMPILER_VERSION_PATCH == 5
        PRINT *, 'INFO:compiler_version_PATCH_digit_8[5]'
# elif COMPILER_VERSION_PATCH == 6
        PRINT *, 'INFO:compiler_version_PATCH_digit_8[6]'
# elif COMPILER_VERSION_PATCH == 7
        PRINT *, 'INFO:compiler_version_PATCH_digit_8[7]'
# elif COMPILER_VERSION_PATCH == 8
        PRINT *, 'INFO:compiler_version_PATCH_digit_8[8]'
# elif COMPILER_VERSION_PATCH == 9
        PRINT *, 'INFO:compiler_version_PATCH_digit_8[9]'
# endif

#endif
#if defined(COMPILER_VERSION_TWEAK)
# undef DEC
# undef HEX
# define DEC(n) DEC_1(n)
# define HEX(n) HEX_1(n)
# if COMPILER_VERSION_TWEAK == 0
        PRINT *, 'INFO:compiler_version_TWEAK_digit_1[0]'
# elif COMPILER_VERSION_TWEAK == 1
        PRINT *, 'INFO:compiler_version_TWEAK_digit_1[1]'
# elif COMPILER_VERSION_TWEAK == 2
        PRINT *, 'INFO:compiler_version_TWEAK_digit_1[2]'
# elif COMPILER_VERSION_TWEAK == 3
        PRINT *, 'INFO:compiler_version_TWEAK_digit_1[3]'
# elif COMPILER_VERSION_TWEAK == 4
        PRINT *, 'INFO:compiler_version_TWEAK_digit_1[4]'
# elif COMPILER_VERSION_TWEAK == 5
        PRINT *, 'INFO:compiler_version_TWEAK_digit_1[5]'
# elif COMPILER_VERSION_TWEAK == 6
        PRINT *, 'INFO:compiler_version_TWEAK_digit_1[6]'
# elif COMPILER_VERSION_TWEAK == 7
        PRINT *, 'INFO:compiler_version_TWEAK_digit_1[7]'
# elif COMPILER_VERSION_TWEAK == 8
        PRINT *, 'INFO:compiler_version_TWEAK_digit_1[8]'
# elif COMPILER_VERSION_TWEAK == 9
        PRINT *, 'INFO:compiler_version_TWEAK_digit_1[9]'
# endif

# undef DEC
# undef HEX
# define DEC(n) DEC_2(n)
# define HEX(n) HEX_2(n)
# if COMPILER_VERSION_TWEAK == 0
        PRINT *, 'INFO:compiler_version_TWEAK_digit_2[0]'
# elif COMPILER_VERSION_TWEAK == 1
        PRINT *, 'INFO:compiler_version_TWEAK_digit_2[1]'
# elif COMPILER_VERSION_TWEAK == 2
        PRINT *, 'INFO:compiler_version_TWEAK_digit_2[2]'
# elif COMPILER_VERSION_TWEAK == 3
        PRINT *, 'INFO:compiler_version_TWEAK_digit_2[3]'
# elif COMPILER_VERSION_TWEAK == 4
        PRINT *, 'INFO:compiler_version_TWEAK_digit_2[4]'
# elif COMPILER_VERSION_TWEAK == 5
        PRINT *, 'INFO:compiler_version_TWEAK_digit_2[5]'
# elif COMPILER_VERSION_TWEAK == 6
        PRINT *, 'INFO:compiler_version_TWEAK_digit_2[6]'
# elif COMPILER_VERSION_TWEAK == 7
        PRINT *, 'INFO:compiler_version_TWEAK_digit_2[7]'
# elif COMPILER_VERSION_TWEAK == 8
        PRINT *, 'INFO:compiler_version_TWEAK_digit_2[8]'
# elif COMPILER_VERSION_TWEAK == 9
        PRINT *, 'INFO:compiler_version_TWEAK_digit_2[9]'
# endif

# undef DEC
# undef HEX
# define DEC(n) DEC_3(n)
# define HEX(n) HEX_3(n)
# if COMPILER_VERSION_TWEAK == 0
        PRINT *, 'INFO:compiler_version_TWEAK_digit_3[0]'
# elif COMPILER_VERSION_TWEAK == 1
        PRINT *, 'INFO:compiler_version_TWEAK_digit_3[1]'
# elif COMPILER_VERSION_TWEAK == 2
        PRINT *, 'INFO:compiler_version_TWEAK_digit_3[2]'
# elif COMPILER_VERSION_TWEAK == 3
        PRINT *, 'INFO:compiler_version_TWEAK_digit_3[3]'
# elif COMPILER_VERSION_TWEAK == 4
        PRINT *, 'INFO:compiler_version_TWEAK_digit_3[4]'
# elif COMPILER_VERSION_TWEAK == 5
        PRINT *, 'INFO:compiler_version_TWEAK_digit_3[5]'
# elif COMPILER_VERSION_TWEAK == 6
        PRINT *, 'INFO:compiler_version_TWEAK_digit_3[6]'
# elif COMPILER_VERSION_TWEAK == 7
        PRINT *, 'INFO:compiler_version_TWEAK_digit_3[7]'
# elif COMPILER_VERSION_TWEAK == 8
        PRINT *, 'INFO:compiler_version_TWEAK_digit_3[8]'
# elif COMPILER_VERSION_TWEAK == 9
        PRINT *, 'INFO:compiler_version_TWEAK_digit_3[9]'
# endif

# undef DEC
# undef HEX
# define DEC(n) DEC_4(n)
# define HEX(n) HEX_4(n)
# if COMPILER_VERSION_TWEAK == 0
        PRINT *, 'INFO:compiler_version_TWEAK_digit_4[0]'
# elif COMPILER_VERSION_TWEAK == 1
        PRINT *, 'INFO:compiler_version_TWEAK_digit_4[1]'
# elif COMPILER_VERSION_TWEAK == 2
        PRINT *, 'INFO:compiler_version_TWEAK_digit_4[2]'
# elif COMPILER_VERSION_TWEAK == 3
        PRINT *, 'INFO:compiler_version_TWEAK_digit_4[3]'
# elif COMPILER_VERSION_TWEAK == 4
        PRINT *, 'INFO:compiler_version_TWEAK_digit_4[4]'
# elif COMPILER_VERSION_TWEAK == 5
        PRINT *, 'INFO:compiler_version_TWEAK_digit_4[5]'
# elif COMPILER_VERSION_TWEAK == 6
        PRINT *, 'INFO:compiler_version_TWEAK_digit_4[6]'
# elif COMPILER_VERSION_TWEAK == 7
        PRINT *, 'INFO:compiler_version_TWEAK_digit_4[7]'
# elif COMPILER_VERSION_TWEAK == 8
        PRINT *, 'INFO:compiler_version_TWEAK_digit_4[8]'
# elif COMPILER_VERSION_TWEAK == 9
        PRINT *, 'INFO:compiler_version_TWEAK_digit_4[9]'
# endif

# undef DEC
# undef HEX
# define DEC(n) DEC_5(n)
# define HEX(n) HEX_5(n)
# if COMPILER_VERSION_TWEAK == 0
        PRINT *, 'INFO:compiler_version_TWEAK_digit_5[0]'
# elif COMPILER_VERSION_TWEAK == 1
        PRINT *, 'INFO:compiler_version_TWEAK_digit_5[1]'
# elif COMPILER_VERSION_TWEAK == 2
        PRINT *, 'INFO:compiler_version_TWEAK_digit_5[2]'
# elif COMPILER_VERSION_TWEAK == 3
        PRINT *, 'INFO:compiler_version_TWEAK_digit_5[3]'
# elif COMPILER_VERSION_TWEAK == 4
        PRINT *, 'INFO:compiler_version_TWEAK_digit_5[4]'
# elif COMPILER_VERSION_TWEAK == 5
        PRINT *, 'INFO:compiler_version_TWEAK_digit_5[5]'
# elif COMPILER_VERSION_TWEAK == 6
        PRINT *, 'INFO:compiler_version_TWEAK_digit_5[6]'
# elif COMPILER_VERSION_TWEAK == 7
        PRINT *, 'INFO:compiler_version_TWEAK_digit_5[7]'
# elif COMPILER_VERSION_TWEAK == 8
        PRINT *, 'INFO:compiler_version_TWEAK_digit_5[8]'
# elif COMPILER_VERSION_TWEAK == 9
        PRINT *, 'INFO:compiler_version_TWEAK_digit_5[9]'
# endif

# undef DEC
# undef HEX
# define DEC(n) DEC_6(n)
# define HEX(n) HEX_6(n)
# if COMPILER_VERSION_TWEAK == 0
        PRINT *, 'INFO:compiler_version_TWEAK_digit_6[0]'
# elif COMPILER_VERSION_TWEAK == 1
        PRINT *, 'INFO:compiler_version_TWEAK_digit_6[1]'
# elif COMPILER_VERSION_TWEAK == 2
        PRINT *, 'INFO:compiler_version_TWEAK_digit_6[2]'
# elif COMPILER_VERSION_TWEAK == 3
        PRINT *, 'INFO:compiler_version_TWEAK_digit_6[3]'
# elif COMPILER_VERSION_TWEAK == 4
        PRINT *, 'INFO:compiler_version_TWEAK_digit_6[4]'
# elif COMPILER_VERSION_TWEAK == 5
        PRINT *, 'INFO:compiler_version_TWEAK_digit_6[5]'
# elif COMPILER_VERSION_TWEAK == 6
        PRINT *, 'INFO:compiler_version_TWEAK_digit_6[6]'
# elif COMPILER_VERSION_TWEAK == 7
        PRINT *, 'INFO:compiler_version_TWEAK_digit_6[7]'
# elif COMPILER_VERSION_TWEAK == 8
        PRINT *, 'INFO:compiler_version_TWEAK_digit_6[8]'
# elif COMPILER_VERSION_TWEAK == 9
        PRINT *, 'INFO:compiler_version_TWEAK_digit_6[9]'
# endif

# undef DEC
# undef HEX
# define DEC(n) DEC_7(n)
# define HEX(n) HEX_7(n)
# if COMPILER_VERSION_TWEAK == 0
        PRINT *, 'INFO:compiler_version_TWEAK_digit_7[0]'
# elif COMPILER_VERSION_TWEAK == 1
        PRINT *, 'INFO:compiler_version_TWEAK_digit_7[1]'
# elif COMPILER_VERSION_TWEAK == 2
        PRINT *, 'INFO:compiler_version_TWEAK_digit_7[2]'
# elif COMPILER_VERSION_TWEAK == 3
        PRINT *, 'INFO:compiler_version_TWEAK_digit_7[3]'
# elif COMPILER_VERSION_TWEAK == 4
        PRINT *, 'INFO:compiler_version_TWEAK_digit_7[4]'
# elif COMPILER_VERSION_TWEAK == 5
        PRINT *, 'INFO:compiler_version_TWEAK_digit_7[5]'
# elif COMPILER_VERSION_TWEAK == 6
        PRINT *, 'INFO:compiler_version_TWEAK_digit_7[6]'
# elif COMPILER_VERSION_TWEAK == 7
        PRINT *, 'INFO:compiler_version_TWEAK_digit_7[7]'
# elif COMPILER_VERSION_TWEAK == 8
        PRINT *, 'INFO:compiler_version_TWEAK_digit_7[8]'
# elif COMPILER_VERSION_TWEAK == 9
        PRINT *, 'INFO:compiler_version_TWEAK_digit_7[9]'
# endif

# undef DEC
# undef HEX
# define DEC(n) DEC_8(n)
# define HEX(n) HEX_8(n)
# if COMPILER_VERSION_TWEAK == 0
        PRINT *, 'INFO:compiler_version_TWEAK_digit_8[0]'
# elif COMPILER_VERSION_TWEAK == 1
        PRINT *, 'INFO:compiler_version_TWEAK_digit_8[1]'
# elif COMPILER_VERSION_TWEAK == 2
        PRINT *, 'INFO:compiler_version_TWEAK_digit_8[2]'
# elif COMPILER_VERSION_TWEAK == 3
        PRINT *, 'INFO:compiler_version_TWEAK_digit_8[3]'
# elif COMPILER_VERSION_TWEAK == 4
        PRINT *, 'INFO:compiler_version_TWEAK_digit_8[4]'
# elif COMPILER_VERSION_TWEAK == 5
        PRINT *, 'INFO:compiler_version_TWEAK_digit_8[5]'
# elif COMPILER_VERSION_TWEAK == 6
        PRINT *, 'INFO:compiler_version_TWEAK_digit_8[6]'
# elif COMPILER_VERSION_TWEAK == 7
        PRINT *, 'INFO:compiler_version_TWEAK_digit_8[7]'
# elif COMPILER_VERSION_TWEAK == 8
        PRINT *, 'INFO:compiler_version_TWEAK_digit_8[8]'
# elif COMPILER_VERSION_TWEAK == 9
        PRINT *, 'INFO:compiler_version_TWEAK_digit_8[9]'
# endif

#endif

      END
