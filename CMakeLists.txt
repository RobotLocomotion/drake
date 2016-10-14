cmake_minimum_required(VERSION 3.5)
project(drake-superbuild)

include(CTest)

include(cmake/config.cmake)
include(cmake/externals.cmake)
include(cmake/examples.cmake)
include(cmake/options.cmake)
include(cmake/git/hooks.cmake)

drake_setup_superbuild()
drake_setup_platform()
drake_setup_git_hooks()
drake_setup_options()

###############################################################################
# BEGIN external projects

# External projects in order of dependencies; 'trivial' ones first
drake_add_external(eigen PUBLIC CMAKE)
drake_add_external(gurobi)
drake_add_external(meshconverters PUBLIC CMAKE)
drake_add_external(mosek PUBLIC)
drake_add_external(spdlog PUBLIC CMAKE)
drake_add_external(swigmake PUBLIC CMAKE)
drake_add_external(yalmip PUBLIC CMAKE)

# avl
# The Ninja generator does not support Fortran.
drake_add_external(avl PUBLIC CMAKE GENERATOR "Unix Makefiles")

# bullet
if(WIN32)
  set(BULLET_EXTRA_CMAKE_ARGS -DUSE_MSVC_RUNTIME_LIBRARY_DLL=ON)
else()
  set(BULLET_EXTRA_CMAKE_ARGS -DBUILD_SHARED_LIBS=ON)
endif()

drake_add_external(bullet PUBLIC CMAKE
  CMAKE_ARGS
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
    -DUSE_DOUBLE_PRECISION=ON
    ${BULLET_EXTRA_CMAKE_ARGS})

# ccd
drake_add_external(ccd PUBLIC CMAKE
  CMAKE_ARGS
    -DBUILD_SHARED_LIBS=ON
    -DBUILD_TESTING=OFF
    -DENABLE_DOUBLE_PRECISION=ON)

# dreal
drake_add_external(dreal PUBLIC CMAKE
  CMAKE_ARGS -DUSE_NLOPT=OFF -DBUILD_TESTING=OFF
  SOURCE_SUBDIR src)

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
    -DGTEST_CREATE_SHARED_LIBRARY=1 # Needed for parameterized tests on Windows
    -DCMAKE_INSTALL_NAME_DIR=${CMAKE_INSTALL_PREFIX}/lib)

# google_styleguide
drake_add_external(google_styleguide PUBLIC
  BUILD_COMMAND :)

# ipopt
drake_add_external(ipopt PUBLIC
  # TODO(sam.creasey) add an alternate build command for WIN32 which
  # downloads the precopiled binary and installs that.
  CONFIGURE_COMMAND ./configure
    --with-blas=BUILD
    --with-lapack=BUILD
    --prefix=${CMAKE_INSTALL_PREFIX}
    --includedir=${CMAKE_INSTALL_PREFIX}/include/ipopt
    --disable-shared
    --with-pic
  INSTALL_COMMAND ${MAKE_COMMAND} install)

# nlopt
drake_add_external(nlopt PUBLIC CMAKE
  CMAKE_ARGS
    -DBUILD_SHARED_LIBS=ON
    -DBUILD_PYTHON=OFF
    -DBUILD_OCTAVE=OFF
    -DBUILD_MATLAB=OFF
    -DBUILD_GUILE=OFF
    -DUSE_SWIG=OFF)

# sedumi
drake_add_external(sedumi CMAKE
  CMAKE_ARGS -DMatlab_ROOT_DIR=${MATLAB_ROOT_DIR})

# snopt
if(MATLAB_EXECUTABLE)
  set(SNOPT_EXTRA_CMAKE_ARGS
    -DBUILD_SNOPT_C_MEX=ON
    -DMatlab_ROOT_DIR=${MATLAB_ROOT_DIR})
else()
  set(SNOPT_EXTRA_CMAKE_ARGS
    -DBUILD_SNOPT_C_MEX=OFF)
endif()

drake_add_external(snopt CMAKE
  CMAKE_ARGS
    -DBUILD_SNOPT_CPP=OFF
    -DBUILD_SHARED_LIBS=ON
    -DBUILD_TESTING=OFF
    ${SNOPT_EXTRA_CMAKE_ARGS})

# spotless
drake_add_external(spotless PUBLIC CMAKE
  CMAKE_ARGS -DMatlab_ROOT_DIR=${MATLAB_ROOT_DIR})

# octomap
drake_add_external(octomap PUBLIC CMAKE
  CMAKE_ARGS
    -DBUILD_TESTING=OFF
    -DCMAKE_EXPORT_NO_PACKAGE_REGISTRY=ON
  SOURCE_SUBDIR octomap)

# swig_matlab
drake_add_external(swig_matlab PUBLIC
  PATCH_COMMAND ./autogen.sh
  CONFIGURE_COMMAND ./configure
    --prefix=${CMAKE_INSTALL_PREFIX}
    --with-matlab
  INSTALL_COMMAND ${MAKE_COMMAND} install)

# textbook
if(MATLAB_EXECUTABLE)
  set(TEXTBOOK_CMAKE_ARGS
    -DBUILD_TESTING=ON
    -DMatlab_ROOT_DIR=${MATLAB_ROOT_DIR})
else()
  set(TEXTBOOK_CMAKE_ARGS -DBUILD_TESTING=OFF)
endif()

drake_add_external(textbook PUBLIC CMAKE
  CMAKE_ARGS ${TEXTBOOK_CMAKE_ARGS}
  INSTALL_COMMAND :
  REQUIRES PythonInterp
  SOURCE_DIR ${PROJECT_SOURCE_DIR}/drake/doc/textbook)

# yaml_cpp
drake_add_external(yaml_cpp PUBLIC CMAKE
 CMAKE_ARGS
   -DBUILD_SHARED_LIBS=ON
   -DCMAKE_EXPORT_NO_PACKAGE_REGISTRY=ON
   -DYAML_CPP_BUILD_CONTRIB=OFF
   -DYAML_CPP_BUILD_TOOLS=OFF)

# iris
drake_add_external(iris PUBLIC CMAKE
  CMAKE_ARGS
    -DBUILD_TESTING=OFF
    -DIRIS_WITH_CDD=ON
    -DIRIS_WITH_EIGEN=OFF
    -DIRIS_WITH_MOSEK=OFF
    -DMatlab_ROOT_DIR=${MATLAB_ROOT_DIR}
  DEPENDS eigen mosek)


# lcm
drake_add_external(lcm PUBLIC CMAKE
  CMAKE_ARGS
    -DBUILD_SHARED_LIBS=ON
    -DLCM_ENABLE_EXAMPLES=OFF
    -DLCM_ENABLE_LUA=OFF
    -DLCM_INSTALL_M4MACROS=OFF)

# libbot
drake_add_external(libbot PUBLIC CMAKE
  DEPENDS lcm)

# bot_core_lcmtypes
drake_add_external(bot_core_lcmtypes PUBLIC CMAKE
  DEPENDS lcm libbot) # Conflicts with libbot; ensure this is built after

# director
drake_add_external(director PUBLIC CMAKE
  SOURCE_SUBDIR distro/superbuild
  CMAKE_ARGS
    -DUSE_LCM=ON # TODO: predicate on whether we have LCM enabled?
    -DUSE_LCMGL=ON
    -DUSE_SYSTEM_LCM=ON
    -DUSE_EXTERNAL_INSTALL=ON
  INSTALL_COMMAND :
  DEPENDS bot_core_lcmtypes lcm libbot)

# signalscope
drake_add_external(signalscope PUBLIC
  DEPENDS director)

# xfoil
# The Ninja generator does not support Fortran.
drake_add_external(xfoil PUBLIC CMAKE GENERATOR "Unix Makefiles")

# Option to skip building drake proper via the superbuild. This allows the
# superbuild to build everything BUT drake, which can still be built separately
# from its build directory. This is used by the dashboards to make separate
# submissions for drake proper and the superbuild without drake. Some users may
# also find it useful, especially to build drake with ninja using fewer than
# the default number of jobs.
option(SKIP_DRAKE_BUILD "Build external projects but not drake itself" OFF)
if(SKIP_DRAKE_BUILD)
  set(DRAKE_BUILD_COMMANDS BUILD_COMMAND : INSTALL_COMMAND :)
endif()

# drake: For drake, list both compilation AND RUNTIME dependencies. Runtime
# dependencies are needed because the drake project must configure only after
# any dependencies used by MATLAB have been installed.
drake_add_external(drake LOCAL PUBLIC CMAKE ALWAYS
  SOURCE_DIR ${PROJECT_SOURCE_DIR}/drake
  BINARY_DIR ${PROJECT_BINARY_DIR}/drake
  ${DRAKE_BUILD_COMMANDS}
  CMAKE_ARGS
    -DDISABLE_MATLAB:BOOL=${DISABLE_MATLAB}
    -DWITH_AVL:BOOL=${WITH_AVL}
    -DWITH_BULLET:BOOL=${WITH_BULLET}
    -DWITH_CCD:BOOL=${WITH_CCD}
    -DWITH_DIRECTOR:BOOL=${WITH_DIRECTOR}
    -DWITH_DREAL:BOOL=${WITH_DREAL}
    -DWITH_GOOGLE_STYLEGUIDE:BOOL=${WITH_GOOGLE_STYLEGUIDE}
    -DWITH_GUROBI:BOOL=${WITH_GUROBI}
    -DWITH_IPOPT:BOOL=${WITH_IPOPT}
    -DWITH_IRIS:BOOL=${WITH_IRIS}
    -DWITH_LCM:BOOL=${WITH_LCM}
    -DWITH_LIBBOT:BOOL=${WITH_LIBBOT}
    -DWITH_MOSEK:BOOL=${WITH_MOSEK}
    -DWITH_NLOPT:BOOL=${WITH_NLOPT}
    -DWITH_OCTOMAP:BOOL=${WITH_OCTOMAP}
    -DWITH_PYTHON_3:BOOL=${WITH_PYTHON_3}
    -DWITH_SEDUMI:BOOL=${WITH_SEDUMI}
    -DWITH_SNOPT:BOOL=${WITH_SNOPT}
    -DWITH_SPOTLESS:BOOL=${WITH_SPOTLESS}
    -DWITH_XFOIL:BOOL=${WITH_XFOIL}
    -DWITH_YALMIP:BOOL=${WITH_YALMIP}
  DEPENDS
    avl
    bot_core_lcmtypes
    bullet
    ccd
    director
    dreal
    eigen
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
    sedumi
    snopt
    spdlog
    spotless
    swig_matlab
    swigmake
    yalmip
    yaml_cpp
    xfoil
)

# END external projects
###############################################################################
# BEGIN examples

# Optional examples
drake_add_example(LittleDog OFF
  "planning and control for a small quadruped robot")

# END examples
###############################################################################
