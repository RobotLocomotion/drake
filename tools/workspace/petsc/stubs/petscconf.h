#pragma once

// TODO(jwnimmer-tri): This file is a lightly-edited version of what comes out
// of upstream's configure script that has been tweaked to work on all of our
// supported platforms, and to remove some unwanted bits. We need to document
// a more careful procedure for how to update and/or re-create this file.

#define MPI_Comm_create_errhandler(p_err_fun, p_errhandler) \
  MPI_Errhandler_create((p_err_fun), (p_errhandler))
#define MPI_Comm_set_errhandler(comm, p_errhandler) \
  MPI_Errhandler_set((comm), (p_errhandler))
#define MPI_Type_create_struct(count, lens, displs, types, newtype) \
  MPI_Type_struct((count), (lens), (displs), (types), (newtype))
#define PETSC_ARCH "NO_PETSC_ARCH"
#define PETSC_ATTRIBUTEALIGNED(size) __attribute((aligned(size)))
#define PETSC_Alignx(a, b)
#define PETSC_BLASLAPACK_UNDERSCORE 1
#define PETSC_CLANGUAGE_C 1
#define PETSC_CXX_INLINE inline
#define PETSC_CXX_RESTRICT __restrict
#define PETSC_C_INLINE inline
#define PETSC_C_RESTRICT __restrict
#define PETSC_DEPRECATED_ENUM(why) __attribute((deprecated))
#define PETSC_DEPRECATED_FUNCTION(why) __attribute((deprecated))
#define PETSC_DEPRECATED_MACRO(why) _Pragma(why)
#define PETSC_DEPRECATED_TYPEDEF(why) __attribute((deprecated))
#define PETSC_DIR "NO_PETSC_DIR"
#define PETSC_DIR_SEPARATOR '/'
#define PETSC_FUNCTION_NAME_C __func__
#define PETSC_FUNCTION_NAME_CXX __func__
#define PETSC_HAVE_ACCESS 1
#define PETSC_HAVE_ATOLL 1
#define PETSC_HAVE_ATTRIBUTEALIGNED 1
#define PETSC_HAVE_BUILTIN_EXPECT 1
#define PETSC_HAVE_BZERO 1
#define PETSC_HAVE_C99_COMPLEX 1
#define PETSC_HAVE_CLOCK 1
#define PETSC_HAVE_CXX_COMPLEX 1
#define PETSC_HAVE_CXX_COMPLEX_FIX 1
#define PETSC_HAVE_CXX_DIALECT_CXX11 1
#define PETSC_HAVE_CXX_DIALECT_CXX14 1
#define PETSC_HAVE_CXX_DIALECT_CXX17 1
#define PETSC_HAVE_DLADDR 1
#define PETSC_HAVE_DLCLOSE 1
#define PETSC_HAVE_DLERROR 1
#define PETSC_HAVE_DLFCN_H 1
#define PETSC_HAVE_DLOPEN 1
#define PETSC_HAVE_DLSYM 1
#define PETSC_HAVE_DOUBLE_ALIGN_MALLOC 1
#define PETSC_HAVE_DRAND48 1
#define PETSC_HAVE_DYNAMIC_LIBRARIES 1
#define PETSC_HAVE_ERF 1
#define PETSC_HAVE_FCNTL_H 1
#define PETSC_HAVE_FENV_H 1
#define PETSC_HAVE_FLOAT_H 1
#define PETSC_HAVE_FORK 1
#define PETSC_HAVE_GETCWD 1
#define PETSC_HAVE_GETDOMAINNAME 1
#define PETSC_HAVE_GETHOSTBYNAME 1
#define PETSC_HAVE_GETHOSTNAME 1
#define PETSC_HAVE_GETPAGESIZE 1
#define PETSC_HAVE_GETRUSAGE 1
#define PETSC_HAVE_IMMINTRIN_H 1
#define PETSC_HAVE_INTTYPES_H 1
#define PETSC_HAVE_ISINF 1
#define PETSC_HAVE_ISNAN 1
#define PETSC_HAVE_ISNORMAL 1
#define PETSC_HAVE_LGAMMA 1
#define PETSC_HAVE_LOG2 1
#define PETSC_HAVE_LSEEK 1
#define PETSC_HAVE_MEMMOVE 1
#define PETSC_HAVE_MMAP 1
#define PETSC_HAVE_MPIUNI 1
#define PETSC_HAVE_NANOSLEEP 1
#define PETSC_HAVE_NETDB_H 1
#define PETSC_HAVE_NETINET_IN_H 1
#define PETSC_HAVE_PACKAGES ":blaslapack:mathlib:mpi:regex:"
#define PETSC_HAVE_POPEN 1
#define PETSC_HAVE_PTHREAD_H 1
#define PETSC_HAVE_PWD_H 1
#define PETSC_HAVE_RAND 1
#define PETSC_HAVE_READLINK 1
#define PETSC_HAVE_REALPATH 1
#define PETSC_HAVE_REGEX 1
#define PETSC_HAVE_RTLD_GLOBAL 1
#define PETSC_HAVE_RTLD_LAZY 1
#define PETSC_HAVE_RTLD_LOCAL 1
#define PETSC_HAVE_RTLD_NOW 1
#define PETSC_HAVE_SETJMP_H 1
#define PETSC_HAVE_SLEEP 1
#define PETSC_HAVE_SNPRINTF 1
#define PETSC_HAVE_STDINT_H 1
#define PETSC_HAVE_STRCASECMP 1
#define PETSC_HAVE_STRINGS_H 1
#define PETSC_HAVE_STRUCT_SIGACTION 1
#define PETSC_HAVE_SYSINFO 1
#define PETSC_HAVE_SYS_PARAM_H 1
#define PETSC_HAVE_SYS_RESOURCE_H 1
#define PETSC_HAVE_SYS_SOCKET_H 1
#define PETSC_HAVE_SYS_TIMES_H 1
#define PETSC_HAVE_SYS_TIME_H 1
#define PETSC_HAVE_SYS_TYPES_H 1
#define PETSC_HAVE_SYS_UTSNAME_H 1
#define PETSC_HAVE_SYS_WAIT_H 1
#define PETSC_HAVE_TGAMMA 1
#define PETSC_HAVE_THREADSAFETY 1
#define PETSC_HAVE_TIME 1
#define PETSC_HAVE_TIME_H 1
#define PETSC_HAVE_UNAME 1
#define PETSC_HAVE_UNISTD_H 1
#define PETSC_HAVE_USLEEP 1
#define PETSC_HAVE_VA_COPY 1
#define PETSC_HAVE_VSNPRINTF 1
#ifdef __SSE__
# define PETSC_HAVE_XMMINTRIN_H 1
#endif
#define PETSC_IS_COLORING_MAX USHRT_MAX
#define PETSC_IS_COLORING_VALUE_TYPE short  // NOLINT
#define PETSC_IS_COLORING_VALUE_TYPE_F integer2
#define PETSC_LEVEL1_DCACHE_LINESIZE 64
#define PETSC_LIB_DIR "NO_PETSC_LIB_DIR"
#define PETSC_MAX_PATH_LEN 4096
#define PETSC_MEMALIGN 8
#define PETSC_MPICC_SHOW "Unavailable"
#define PETSC_MPIU_IS_COLORING_VALUE_TYPE MPI_UNSIGNED_SHORT
// For prefetching, we always use the GCC-ism (not XMM) for portability.
// https://github.com/petsc/petsc/blob/70719257/config/PETSc/Configure.py#L565-L585
#define PETSC_PREFETCH_HINT_NTA 0
#define PETSC_PREFETCH_HINT_T0 3
#define PETSC_PREFETCH_HINT_T1 2
#define PETSC_PREFETCH_HINT_T2 1
#define PETSC_PYTHON_EXE "NO_PETSC_PYTHON_EXE"
#define PETSC_Prefetch(a, b, c) __builtin_prefetch((a), (b), (c))
#define PETSC_REPLACE_DIR_SEPARATOR '\\'
#define PETSC_SIGNAL_CAST
#define PETSC_SIZEOF_ENUM 4
#define PETSC_SIZEOF_INT 4
#define PETSC_SIZEOF_LONG 8
#define PETSC_SIZEOF_LONG_LONG 8
#define PETSC_SIZEOF_SHORT 2
#define PETSC_SIZEOF_SIZE_T 8
#define PETSC_SIZEOF_VOID_P 8
#define PETSC_SLSUFFIX "so"
#define PETSC_UINTPTR_T uintptr_t
#define PETSC_UNUSED __attribute((unused))
#define PETSC_USE_BACKWARD_LOOP 1
#define PETSC_USE_CTABLE 1
#define PETSC_USE_INFO 1
#define PETSC_USE_ISATTY 1
#define PETSC_USE_MALLOC_COALESCED 1
#define PETSC_USE_PROC_FOR_SIZE 1
#define PETSC_USE_REAL_DOUBLE 1
#define PETSC_USE_SINGLE_LIBRARY 1
#define PETSC_USE_VISIBILITY_C 1
#define PETSC_USE_VISIBILITY_CXX 1
#define PETSC_USING_64BIT_PTR 1
#define PETSC_VERSION_BRANCH_GIT "NO_PETSC_VERSION_BRANCH_GIT"
#define PETSC_VERSION_DATE_GIT "NO_PETSC_VERSION_DATE_GIT"
#define PETSC_VERSION_GIT "NO_PETSC_VERSION_GIT"
#define PETSC__BSD_SOURCE 1
#define PETSC__DEFAULT_SOURCE 1
#define PETSC__GNU_SOURCE 1
