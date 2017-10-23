cmake_minimum_required(VERSION 3.5.1)
project(drake-superbuild)

include(CTest)

include(cmake/config.cmake)
include(cmake/externals.cmake)

drake_setup_superbuild()
drake_setup_platform()

###############################################################################
# BEGIN external projects

# External projects in order of dependencies.

# eigen
# N.B. See #5785; do your best not to have to bump this to a newer commit.
drake_add_external(eigen
  URL https://bitbucket.org/eigen/eigen/get/3.3.3.tar.gz
  URL_HASH SHA256=94878cbfa27b0d0fbc64c00d4aafa137f678d5315ae62ba4aecddbd4269ae75f
  CMAKE_ARGS
    -DEIGEN_TEST_NO_OPENGL=ON
    -DEIGEN_TEST_NOQT=ON)

# fmt
drake_add_external(fmt
  URL https://github.com/fmtlib/fmt/archive/3.0.1.tar.gz
  URL_HASH SHA256=dce62ab75a161dd4353a98364feb166d35e7eea382169d59d9ce842c49c55bad
  CMAKE_ARGS
    -DFMT_DOC=OFF
    -DFMT_TEST=OFF)

# gflags
drake_add_external(gflags
  URL https://github.com/gflags/gflags/archive/95ffb27c9c7496ede1409e042571054c70cb9519.tar.gz
  URL_HASH SHA256=723c21f783c720c0403c9b44bf500d1961a08bd2635cbc117107af22d2e1643f
  CMAKE_ARGS
    -DBUILD_SHARED_LIBS=ON
    -DGFLAGS_BUILD_gflags_nothreads_LIB=OFF)

# nlopt
drake_add_external(nlopt
  URL https://github.com/stevengj/nlopt/archive/45553da97c890ef58f95e7ef73c5409d2169e824.tar.gz
  URL_HASH SHA256=931fd125c50acf7cd7e709887ab4923af42a8a07be139572bf8b76bccca76450
  CMAKE_ARGS
    -DBUILD_SHARED_LIBS=ON
    -DBUILD_PYTHON=OFF
    -DBUILD_OCTAVE=OFF
    -DBUILD_MATLAB=OFF
    -DBUILD_GUILE=OFF
    -DINSTALL_INCLUDE_DIR=${CMAKE_INSTALL_PREFIX}/include/nlopt
    -DUSE_SWIG=OFF)

# protobuf
drake_add_external(protobuf
  URL https://github.com/google/protobuf/archive/v3.1.0.tar.gz
  URL_HASH SHA256=fb2a314f4be897491bb2446697be693d489af645cb0e165a85e7e64e07eb134d
  SOURCE_SUBDIR cmake
  CMAKE_ARGS
    -DBUILD_SHARED_LIBS=ON
    -Dprotobuf_BUILD_TESTS=OFF
    -Dprotobuf_MODULE_COMPATIBLE=ON)

# pybind11
drake_add_external(pybind11
  URL https://github.com/RobotLocomotion/pybind11/archive/6d72785766558047ee2e2075198c07d8c25eb631.tar.gz
  URL_HASH SHA256=08b4813b3b17f607efc4e8ba8b73bf55759ba744cab125e9fc666b5161cb1d0a
  CMAKE_ARGS
    -DPYBIND11_PYTHON_VERSION=${PYTHON_VERSION_MAJOR}.${PYTHON_VERSION_MINOR}
    -DPYBIND11_TEST=OFF)

# tinyobjloader
drake_add_external(tinyobjloader
  URL https://github.com/syoyo/tinyobjloader/archive/v1.0.6.tar.gz
  URL_HASH SHA256=19ee82cd201761954dd833de551edb570e33b320d6027e0d91455faf7cd4c341
  CMAKE_ARGS
    -DTINYOBJLOADER_COMPILATION_SHARED=ON
  )

drake_add_external(drake LOCAL
  SOURCE_DIR ${PROJECT_SOURCE_DIR}/drake
  BINARY_DIR ${PROJECT_BINARY_DIR}/drake
  CMAKE_ARGS -DDISABLE_MATLAB:BOOL=${DISABLE_MATLAB}
  DEPENDS
    eigen
    fmt
    gflags
    nlopt
    protobuf
    pybind11
    tinyobjloader
)

# END external projects
###############################################################################
