cmake_minimum_required(VERSION 3.5)
project(drake-superbuild VERSION 0.0.0)

include(CTest)

include(cmake/config.cmake)
include(cmake/externals.cmake)
include(cmake/examples.cmake)
include(cmake/options.cmake)
include(cmake/package.cmake)
include(cmake/tools.cmake)
include(cmake/git/hooks.cmake)

drake_setup_superbuild()
drake_setup_platform()
drake_setup_tools_superbuild()
drake_setup_git_hooks()
drake_setup_options()

###############################################################################
# BEGIN external projects

# External projects in order of dependencies; 'trivial' ones first
drake_add_external(avl PUBLIC CMAKE FORTRAN)
drake_add_external(eigen PUBLIC CMAKE)
drake_add_external(meshconverters PUBLIC CMAKE)
drake_add_external(qt_property_browser CMAKE QT)
drake_add_external(sedumi PUBLIC CMAKE MATLAB)
drake_add_external(spdlog PUBLIC CMAKE)
drake_add_external(spotless PUBLIC CMAKE MATLAB)
drake_add_external(xfoil PUBLIC CMAKE FORTRAN)
drake_add_external(yalmip PUBLIC CMAKE MATLAB)

# bullet
drake_add_external(bullet PUBLIC CMAKE
  CMAKE_ARGS
    -DBUILD_SHARED_LIBS=ON
    -DBUILD_BULLET2_DEMOS=OFF
    -DBUILD_CPU_DEMOS=OFF
    -DBUILD_EXTRAS=OFF
    -DBUILD_OPENGL3_DEMOS=OFF
    -DBUILD_UNIT_TESTS=OFF
    -DCMAKE_DEBUG_POSTFIX=
    -DCMAKE_MINSIZEREL_POSTFIX=
    -DCMAKE_RELWITHDEBINFO_POSTFIX=
    -DINSTALL_LIBS=ON
    -DPKGCONFIG_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}/lib/pkgconfig
    -DUSE_DOUBLE_PRECISION=ON)

# ccd
drake_add_external(ccd PUBLIC CMAKE
  CMAKE_ARGS
    -DBUILD_SHARED_LIBS=ON
    -DBUILD_TESTING=OFF
    -DENABLE_DOUBLE_PRECISION=ON)

# gflags
drake_add_external(gflags PUBLIC CMAKE
  CMAKE_ARGS
    -DBUILD_SHARED_LIBS=ON
    -DCMAKE_EXPORT_NO_PACKAGE_REGISTRY=ON
    -DGFLAGS_BUILD_gflags_nothreads_LIB=OFF)

# googletest
drake_add_external(googletest PUBLIC CMAKE
  CMAKE_ARGS
    -DBUILD_SHARED_LIBS=ON
    -DCMAKE_INSTALL_NAME_DIR=${CMAKE_INSTALL_PREFIX}/lib)

# google_styleguide
drake_add_external(google_styleguide PUBLIC PYTHON
  BUILD_COMMAND :)

# gurobi
drake_add_external(gurobi CMAKE MATLAB
  CMAKE_ARGS
    -DBUILD_TESTING=OFF
    -DWITH_CPP=ON
    -DWITH_MATLAB=${Matlab_FOUND}
    -DWITH_PYTHON=OFF)

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

# mosek
drake_add_external(mosek PUBLIC CMAKE MATLAB
  CMAKE_ARGS
    -DBUILD_TESTING=OFF
    -DWITH_MATLAB=${Matlab_FOUND}
    -DWITH_PYTHON=OFF)

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

# pybind11
drake_add_external(pybind11 CMAKE
  CMAKE_ARGS
    -DPYBIND11_PYTHON_VERSION=${PYTHON_VERSION_MAJOR}.${PYTHON_VERSION_MINOR}
    -DPYBIND11_TEST=OFF)

# pythonqt
drake_add_external(pythonqt CMAKE PYTHON QT
  CMAKE_ARGS
    -DPythonQt_Wrap_Qtcore:BOOL=ON
    -DPythonQt_Wrap_Qtgui:BOOL=ON
    -DPythonQt_Wrap_Qtuitools:BOOL=ON
)

# snopt
drake_add_external(snopt CMAKE MATLAB
  CMAKE_ARGS
    -DBUILD_SNOPT_CPP=OFF
    -DBUILD_SHARED_LIBS=OFF  # For licensing reasons
    -DBUILD_TESTING=OFF
    -DBUILD_SNOPT_C_MEX=${Matlab_FOUND}
    -DCMAKE_POSITION_INDEPENDENT_CODE=ON)

# octomap
drake_add_external(octomap PUBLIC CMAKE
  CMAKE_ARGS
    -DBUILD_TESTING=OFF
    -DCMAKE_EXPORT_NO_PACKAGE_REGISTRY=ON
  SOURCE_SUBDIR octomap)

# vtk
drake_add_external(vtk PUBLIC CMAKE PYTHON QT
  CMAKE_ARGS
    -DBUILD_SHARED_LIBS=ON
    -DBUILD_TESTING=OFF
    -DBUILD_EXAMPLES=OFF
    "-DCMAKE_CXX_FLAGS=${CMAKE_CXX_FLAGS} -DGLX_GLXEXT_LEGACY=1"
    -DVTK_USE_GUISUPPORT=ON
    -DVTK_USE_QT=ON
    -DVTK_WRAP_PYTHON=ON
    -DVTK_WRAP_TCL=OFF
    -DVTK_USE_TK=OFF)

# fcl
drake_add_external(fcl PUBLIC CMAKE
  CMAKE_ARGS -DFCL_BUILD_TESTS=OFF
  DEPENDS ccd eigen octomap)

# textbook
drake_add_external(textbook PUBLIC CMAKE MATLAB PYTHON
  CMAKE_ARGS -DBUILD_TESTING=${Matlab_FOUND}
  INSTALL_COMMAND :
  SOURCE_DIR ${PROJECT_SOURCE_DIR}/drake/doc/textbook)

# yaml_cpp
drake_add_external(yaml_cpp PUBLIC CMAKE
 CMAKE_ARGS
   -DBUILD_SHARED_LIBS=ON
   -DCMAKE_EXPORT_NO_PACKAGE_REGISTRY=ON
   -DYAML_CPP_BUILD_CONTRIB=OFF
   -DYAML_CPP_BUILD_TOOLS=OFF)

# ctk_python_console
drake_add_external(ctk_python_console CMAKE PYTHON QT
  DEPENDS pythonqt)

# iris
drake_add_external(iris PUBLIC CMAKE MATLAB PYTHON
  CMAKE_ARGS
    -DBUILD_TESTING=OFF
    -DIRIS_WITH_CDD=ON
    -DIRIS_WITH_EIGEN=OFF
    -DIRIS_WITH_MATLAB=${Matlab_FOUND}
    -DIRIS_WITH_MOSEK=OFF
    -DIRIS_WITH_PYBIND11=OFF
    -DPYTHON_EXECUTABLE=${PYTHON_EXECUTABLE}
    -DPYTHON_INCLUDE_DIR=${PYTHON_INCLUDE_DIR}
    -DPYTHON_LIBRARY=${PYTHON_LIBRARY}
  DEPENDS eigen mosek pybind11)

# lcm
drake_add_external(lcm PUBLIC CMAKE PYTHON
  CMAKE_ARGS
    -DBUILD_SHARED_LIBS=ON
    -DLCM_ENABLE_EXAMPLES=OFF
    -DLCM_ENABLE_LUA=OFF
    -DLCM_ENABLE_TESTS=OFF
    -DLCM_INSTALL_M4MACROS=OFF)

# libbot
drake_add_external(libbot PUBLIC CMAKE PYTHON
  DEPENDS lcm)

# bot_core_lcmtypes
drake_add_external(bot_core_lcmtypes PUBLIC CMAKE PYTHON
  DEPENDS lcm libbot) # Conflicts with libbot; ensure this is built after

# robotlocomotion_lcmtypes
drake_add_external(robotlocomotion_lcmtypes PUBLIC CMAKE PYTHON
  DEPENDS bot_core_lcmtypes lcm)

# signalscope
drake_add_external(signalscope PUBLIC CMAKE PYTHON QT
  DEPENDS ctk_python_console lcm pythonqt)

# Option to skip building drake proper via the superbuild. This allows the
# superbuild to build everything BUT drake, which can still be built separately
# from its build directory. This is used by the dashboards to make separate
# submissions for drake proper and the superbuild without drake. Some users may
# also find it useful, especially to build drake with ninja using fewer than
# the default number of jobs.
option(SKIP_DRAKE_BUILD "Build external projects but not drake itself" OFF)

# drake: For drake, list both compilation AND RUNTIME dependencies. Runtime
# dependencies are needed because the drake project must configure only after
# any dependencies used by MATLAB have been installed.
# TODO: re-add TEST to drake flags when subprojects arrive
drake_add_external(drake LOCAL PUBLIC CMAKE ALWAYS MATLAB PYTHON VTK
  SOURCE_DIR ${PROJECT_SOURCE_DIR}/drake
  BINARY_DIR ${PROJECT_BINARY_DIR}/drake
  CMAKE_ARGS
    -DDISABLE_MATLAB:BOOL=${DISABLE_MATLAB}
    -DDISABLE_PYTHON:BOOL=${DISABLE_PYTHON}
    -DDISABLE_FORTRAN:BOOL=${DISABLE_FORTRAN}
    -DWITH_AVL:BOOL=${WITH_AVL}
    -DWITH_BOT_CORE_LCMTYPES:BOOL=${WITH_BOT_CORE_LCMTYPES}
    -DWITH_BULLET:BOOL=${WITH_BULLET}
    -DWITH_CCD:BOOL=${WITH_CCD}
    -DWITH_EIGEN3:BOOL=${WITH_EIGEN}  # TODO(jamiesnape): Fix naming inconsistency
    -DWITH_FCL:BOOL=${WITH_FCL}
    -DWITH_GFLAGS:BOOL=${WITH_GFLAGS}
    -DWITH_GOOGLE_STYLEGUIDE:BOOL=${WITH_GOOGLE_STYLEGUIDE}
    -DWITH_GTEST:BOOL=${WITH_GOOGLETEST}  # TODO(jamiesnape): Fix naming inconsistency
    -DWITH_GUROBI:BOOL=${WITH_GUROBI}
    -DWITH_IPOPT:BOOL=${WITH_IPOPT}
    -DWITH_IRIS:BOOL=${WITH_IRIS}
    -DWITH_LCM:BOOL=${WITH_LCM}
    -DWITH_LIBBOT:BOOL=${WITH_LIBBOT}
    -DWITH_MESHCONVERTERS:BOOL=${WITH_MESHCONVERTERS}
    -DWITH_MOSEK:BOOL=${WITH_MOSEK}
    -DWITH_NLOPT:BOOL=${WITH_NLOPT}
    -DWITH_OCTOMAP:BOOL=${WITH_OCTOMAP}
    -DWITH_PYBIND11:BOOL=${WITH_PYBIND11}
    -DWITH_PYTHON_3:BOOL=${WITH_PYTHON_3}
    -DWITH_SEDUMI:BOOL=${WITH_SEDUMI}
    -DWITH_SIGNALSCOPE:BOOL=${WITH_SIGNALSCOPE}
    -DWITH_SNOPT:BOOL=${WITH_SNOPT}
    -DWITH_SPDLOG:BOOL=${WITH_SPDLOG}
    -DWITH_SPOTLESS:BOOL=${WITH_SPOTLESS}
    -DWITH_TEXTBOOK:BOOL=${WITH_TEXTBOOK}
    -DWITH_XFOIL:BOOL=${WITH_XFOIL}
    -DWITH_YALMIP:BOOL=${WITH_YALMIP}
    -DWITH_YAML_CPP:BOOL=${WITH_YAML_CPP}
    -DUSE_CLANG_TIDY:BOOL=${USE_CLANG_TIDY}
    -DUSE_INCLUDE_WHAT_YOU_USE:BOOL=${USE_INCLUDE_WHAT_YOU_USE}
    -DUSE_LINK_WHAT_YOU_USE:BOOL=${USE_LINK_WHAT_YOU_USE}
    -DUSE_SANITIZER:STRING=${USE_SANITIZER}
    -DUSE_VALGRIND:STRING=${USE_VALGRIND}
  DEPENDS
    avl
    bot_core_lcmtypes
    bullet
    ccd
    eigen
    fcl
    gflags
    google_styleguide
    googletest
    gurobi
    ipopt
    iris
    lcm
    libbot
    meshconverters
    mosek
    nlopt
    octomap
    pybind11
    robotlocomotion_lcmtypes
    sedumi
    snopt
    spdlog
    spotless
    textbook
    vtk
    xfoil
    yalmip
    yaml_cpp
)

# director
if(APPLE)
  # TODO(patmarion): fix director tests on macOS and re-add TEST always
  set(TEST_DIRECTOR)
else()
  set(TEST_DIRECTOR TEST)
endif()
drake_add_external(director PUBLIC CMAKE ${TEST_DIRECTOR} PYTHON QT VTK
  CMAKE_ARGS
    -DUSE_DRAKE=ON
    -DUSE_LCM=${HAVE_LCM}
    -DUSE_LCMGL=${HAVE_LIBBOT}
    -DUSE_LIBBOT=OFF
    -DENABLE_TESTING=ON
  DEPENDS
    bot_core_lcmtypes
    ctk_python_console
    drake
    lcm
    libbot
    pythonqt
    qt_property_browser
    vtk
)

# END external projects
###############################################################################
# BEGIN examples

# Optional examples
drake_add_example(LittleDog OFF
  "planning and control for a small quadruped robot")

# END examples
###############################################################################
# BEGIN ros packages

# END ros packages
###############################################################################

drake_setup_cpack()
