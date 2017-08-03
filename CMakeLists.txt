cmake_minimum_required(VERSION 3.5)
project(drake-superbuild VERSION 0.0.0)

include(CTest)

include(cmake/config.cmake)
include(cmake/externals.cmake)
include(cmake/options.cmake)

drake_setup_superbuild()
drake_setup_platform()
drake_setup_options()

###############################################################################
# BEGIN external projects

# External projects in order of dependencies.

# eigen
# N.B. See #5785; do your best not to have to bump this to a newer commit.
drake_add_external(eigen PUBLIC CMAKE
  URL https://bitbucket.org/eigen/eigen/get/3.3.3.tar.gz
  URL_HASH SHA256=94878cbfa27b0d0fbc64c00d4aafa137f678d5315ae62ba4aecddbd4269ae75f
  CMAKE_ARGS
    -DEIGEN_TEST_NO_OPENGL=ON
    -DEIGEN_TEST_NOQT=ON)

# fmt
drake_add_external(fmt PUBLIC CMAKE ALWAYS
  CMAKE_ARGS
    -DFMT_DOC=OFF
    -DFMT_TEST=OFF)

# gflags
drake_add_external(gflags PUBLIC CMAKE ALWAYS
  CMAKE_ARGS
    -DBUILD_SHARED_LIBS=ON
    -DGFLAGS_BUILD_gflags_nothreads_LIB=OFF)

# googletest
drake_add_external(googletest PUBLIC CMAKE
  CMAKE_ARGS
    -DBUILD_SHARED_LIBS=ON
    -DCMAKE_INSTALL_NAME_DIR=${CMAKE_INSTALL_PREFIX}/lib)

# google_styleguide
drake_add_external(google_styleguide PUBLIC PYTHON
  BUILD_COMMAND :)

# ipopt
if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # Work around horrific clang* = clang, */cl* = msvc bug in BuildTools/coin.m4
  get_filename_component(IPOPT_C_COMPILER "${CMAKE_C_COMPILER}" NAME)
  get_filename_component(IPOPT_CXX_COMPILER "${CMAKE_CXX_COMPILER}" NAME)
else()
  set(IPOPT_C_COMPILER "${CMAKE_C_COMPILER}")
  set(IPOPT_CXX_COMPILER "${CMAKE_CXX_COMPILER}")
endif()

drake_add_external(ipopt PUBLIC AUTOTOOLS FORTRAN
  AUTOTOOLS_ENV
    CC=${IPOPT_C_COMPILER}
    CXX=${IPOPT_CXX_COMPILER}
  AUTOTOOLS_CONFIGURE_ARGS
    --disable-shared
    --includedir=${CMAKE_INSTALL_PREFIX}/include/ipopt
    --with-blas=BUILD
    --with-lapack=BUILD
    --with-pic)

# nlopt
drake_add_external(nlopt PUBLIC CMAKE
  CMAKE_ARGS
    -DBUILD_SHARED_LIBS=ON
    -DBUILD_PYTHON=OFF
    -DBUILD_OCTAVE=OFF
    -DBUILD_MATLAB=OFF
    -DBUILD_GUILE=OFF
    -DINSTALL_INCLUDE_DIR=${CMAKE_INSTALL_PREFIX}/include/nlopt
    -DUSE_SWIG=OFF)

# protobuf
drake_add_external(protobuf PUBLIC CMAKE ALWAYS
  SOURCE_SUBDIR cmake
  CMAKE_ARGS
    -DBUILD_SHARED_LIBS=ON
    -Dprotobuf_BUILD_TESTS=OFF
    -Dprotobuf_MODULE_COMPATIBLE=ON)

# pybind11
drake_add_external(pybind11 PUBLIC CMAKE
  CMAKE_ARGS
    -DPYBIND11_PYTHON_VERSION=${PYTHON_VERSION_MAJOR}.${PYTHON_VERSION_MINOR}
    -DPYBIND11_TEST=OFF)

# tinyobjloader
drake_add_external(tinyobjloader PUBLIC CMAKE ALWAYS
  CMAKE_ARGS
    -DTINYOBJLOADER_COMPILATION_SHARED=ON
  )

drake_add_external(drake LOCAL PUBLIC CMAKE ALWAYS MATLAB PYTHON
  SOURCE_DIR ${PROJECT_SOURCE_DIR}/drake
  BINARY_DIR ${PROJECT_BINARY_DIR}/drake
  CMAKE_ARGS
    -DDISABLE_MATLAB:BOOL=${DISABLE_MATLAB}
    -DDISABLE_PYTHON:BOOL=${DISABLE_PYTHON}
    -DDISABLE_FORTRAN:BOOL=${DISABLE_FORTRAN}
    -DWITH_EIGEN3:BOOL=${WITH_EIGEN}  # TODO(jamiesnape): Fix naming inconsistency
    -DWITH_GFLAGS:BOOL=${WITH_GFLAGS}
    -DWITH_GOOGLE_STYLEGUIDE:BOOL=${WITH_GOOGLE_STYLEGUIDE}
    -DWITH_GTEST:BOOL=${WITH_GOOGLETEST}  # TODO(jamiesnape): Fix naming inconsistency
    -DWITH_IPOPT:BOOL=${WITH_IPOPT}
    -DWITH_NLOPT:BOOL=${WITH_NLOPT}
    -DWITH_PROTOBUF:BOOL=${WITH_PROTOBUF}
    -DWITH_PYBIND11:BOOL=${WITH_PYBIND11}
    -DWITH_TINYOBJLOADER:BOOL=${WITH_TINYOBJLOADER}
  DEPENDS
    eigen
    fmt
    gflags
    google_styleguide
    googletest
    ipopt
    nlopt
    protobuf
    pybind11
    tinyobjloader
)

# END external projects
###############################################################################
