cmake_minimum_required(VERSION 3.5)
project(drake-superbuild)

include(CTest)

include(cmake/config.cmake)
include(cmake/externals.cmake)
include(cmake/examples.cmake)
include(cmake/git/hooks.cmake)

drake_setup_superbuild()
drake_setup_platform()
drake_setup_git_hooks()

###############################################################################
# BEGIN options

##########################################
# External Projects that are ON by default
##########################################
option(WITH_EIGEN "required c++ matrix library.  only disable if you have it already." ON)
option(WITH_GOOGLETEST "required c++ unit test library.  only disable if you have it already." ON)
option(WITH_GFLAGS "required c++ command-line library.  only disable if you have it already." ON)
option(WITH_GOOGLE_STYLEGUIDE "provides cpplint.py style checking" ON)
option(WITH_SWIGMAKE "helper tools to build python & MATLAB wrappers for C++ libraries with Eigen" ON)
option(WITH_BULLET "used for collision detection" ON)
option(WITH_LCM "interprocess communications protocol for visualizers, etc" ON)
option(WITH_BOT_CORE_LCMTYPES "required LCM types library. only disable if you have it already." ON)
option(WITH_SPDLOG "spdlog text logging facility; disabling will turn off text logging." ON)
if(WIN32)
  option(WITH_GTK "precompiled gtk binaries/headers for Windows" ON)  # needed for lcm on windows
else()
  option(WITH_DIRECTOR "vtk-based visualization tool and robot user interface" ON) # not win32 yet.  it builds on windows, but requires manually installation of vtk, etc.  perhaps make a precompiled director pod (a bit like snopt)
  option(WITH_LIBBOT "simple open-gl visualizer + lcmgl for director" ON) # there is hope, but i spent a long time cleaning up c/c++ language errors trying to make msvc happy.. and still had a long way to go.
  option(WITH_NLOPT "nonlinear optimization solver" ON)
  option(WITH_DREAL "nonlinear SMT solver" ON)
  option(WITH_MOSEK "convex optimization solver; free for academics" OFF)
  # IPOPT is currently disabled on Mac due to MATLAB compatibility
  # issues: https://github.com/RobotLocomotion/drake/issues/2578
  if (NOT APPLE)
    option(WITH_IPOPT "nonlinear optimization solver" ON)
  endif()
  option(WITH_SWIG_MATLAB "A version of SWIG with MATLAB support" ON)
endif()

option(WITH_SNOPT_PRECOMPILED "precompiled binaries only for snopt; the source requires a license (will be disabled if WITH_SNOPT=ON)" ON)
option(WITH_YAML_CPP "library for reading and writing yaml configuration files" ON)

##############################################################
# External Projects that are only needed when MATLAB is in use
##############################################################
include(CMakeDependentOption)
option(DISABLE_MATLAB "Don't use MATLAB even if it is present." OFF)
if(DISABLE_MATLAB)
  message(STATUS "MATLAB is disabled.")
else()
  find_program(matlab matlab)
  if(matlab)
    message(STATUS "Found MATLAB at " ${matlab})
  else()
    message(STATUS "Looked for MATLAB but could not find it.")
  endif()
endif()

## The following projects are default ON when MATLAB is present and enabled.
## Otherwise, they are hidden and default OFF.
cmake_dependent_option(WITH_SPOTLESS "polynomial optimization front-end for MATLAB" ON "NOT DISABLE_MATLAB;matlab" OFF)

## The following projects are default OFF when MATLAB is present and enabled.
## Otherwise, they are hidden and default OFF.
## Some of them may also be hidden on Windows regardless of the status of MATLAB.
cmake_dependent_option(WITH_BERTINI "solve polynomial equations; free but pod requires permissions (can't redistribute)" OFF "NOT DISABLE_MATLAB;matlab;NOT WIN32" OFF)
cmake_dependent_option(WITH_GLOPTIPOLY "free global polynomial optimization tooblox" OFF "NOT DISABLE_MATLAB;matlab;NOT WIN32" OFF)
cmake_dependent_option(WITH_IRIS "fast approximate convex segmentation" OFF "NOT DISABLE_MATLAB;matlab;NOT WIN32;WITH_MOSEK" OFF)
cmake_dependent_option(WITH_SEDUMI "semi-definite programming solver" OFF "NOT DISABLE_MATLAB;matlab;NOT WIN32" OFF)
cmake_dependent_option(WITH_YALMIP "free optimization front-end for MATLAB" OFF "NOT DISABLE_MATLAB;matlab;NOT WIN32" OFF)

###########################################
# External Projects that are OFF by default
###########################################
option(WITH_SNOPT "nonlinear optimization solver; requires access to RobotLocomotion/snopt-pod")
cmake_dependent_option(WITH_SIGNALSCOPE "live plotting tool for lcm messages" OFF "NOT WIN32;WITH_DIRECTOR" OFF)

if(NOT WIN32) # many of these might work on win32 with little or no work... they just haven't been tried
  option(WITH_AVL "use w/ AVL to compute aerodynamic coefficients for airfoils")
  option(WITH_GUROBI "convex/integer optimization solver; free for academics (will prompt you for login bits)")
  option(WITH_MESHCONVERTERS "uses vcglib to convert a few standard filetypes")
  option(WITH_OCTOMAP "provides oct-tree data structures")
  option(WITH_TEXTBOOK "pull in the Underactuated Robotics textbook and its examples")  # almost works on windows.  the update step call to git was complaining about local modifications on drake003
  option(WITH_XFOIL "use w/ XFOIL to compute aerodynamic coefficients for airfoils")
endif()

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

# END options
###############################################################################
# BEGIN external projects

# External projects in order of dependencies; 'trivial' ones first
drake_add_external(avl PUBLIC CMAKE)
drake_add_external(bertini)
drake_add_external(bullet PUBLIC CMAKE)
drake_add_external(eigen PUBLIC CMAKE)
drake_add_external(gloptipoly)
drake_add_external(gurobi)
drake_add_external(meshconverters PUBLIC)
drake_add_external(mosek PUBLIC)
drake_add_external(octomap PUBLIC)
drake_add_external(sedumi)
drake_add_external(snopt CMAKE)
drake_add_external(spdlog PUBLIC CMAKE)
drake_add_external(spotless PUBLIC CMAKE)
drake_add_external(swigmake PUBLIC CMAKE)
drake_add_external(xfoil PUBLIC CMAKE)
drake_add_external(yalmip PUBLIC)

# cmake
drake_add_external(cmake PUBLIC ALWAYS
  BUILD_COMMAND :
  SOURCE_DIR ${PROJECT_SOURCE_DIR}/drake/cmake)

# dreal
drake_add_external(dreal PUBLIC CMAKE
  CMAKE_ARGS -DUSE_NLOPT=OFF
  SOURCE_SUBDIR src)

# gflags
if(WIN32)
  # We can't figure out how to get gflags shared libraries working on Windows
  # (see #3345), so here we build it with static libraries only.  Code that
  # lives in Drake's shared libraries must be conditionalized to avoid using
  # gflags on Windows.
  drake_add_external(gflags PUBLIC CMAKE)
else()
  drake_add_external(gflags PUBLIC CMAKE
    CMAKE_ARGS -DBUILD_SHARED_LIBS=ON)
endif()
  
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
drake_add_external(nlopt PUBLIC
  PATCH_COMMAND ./autogen.sh --no-configure
  CONFIGURE_COMMAND ./configure
    --without-matlab
    --without-python
    --without-octave
    --without-guile
    --enable-maintainer-mode
    --enable-shared
    --prefix=${CMAKE_INSTALL_PREFIX}
    --includedir=${CMAKE_INSTALL_PREFIX}/include/nlopt
  INSTALL_COMMAND ${MAKE_COMMAND} install)

# swig_matlab
drake_add_external(swig_matlab PUBLIC
  PATCH_COMMAND ./autogen.sh
  CONFIGURE_COMMAND ./configure
    --prefix=${CMAKE_INSTALL_PREFIX}
    --with-matlab
  INSTALL_COMMAND ${MAKE_COMMAND} install)

# textbook
drake_add_external(textbook PUBLIC
  REQUIRES PythonInterp
  SOURCE_DIR ${PROJECT_SOURCE_DIR}/drake/doc/textbook
  BUILD_COMMAND \@PYTHON_EXECUTABLE\@
    extract_examples.py underactuated.html ./examples)

# yaml_cpp
if(APPLE OR WIN32)
  drake_add_external(yaml_cpp PUBLIC CMAKE
    CMAKE_ARGS -DBUILD_SHARED_LIBS=OFF)
else()
  drake_add_external(yaml_cpp PUBLIC CMAKE
    CMAKE_ARGS -DBUILD_SHARED_LIBS=ON)
endif()

# iris
drake_add_external(iris PUBLIC
  CONFIGURE_COMMAND ${MAKE_COMMAND} configure-cdd-only
  DEPENDS eigen mosek)

# lcm (and gtk)
if(WIN32)
  # On Windows, always build LCM in Release mode for compatibility with the
  # Python libraries.
  if(CMAKE_CONFIGURATION_TYPES)
    # If using a multi-configuration generator, replace the default build
    # commands to force the Release configuration to be built.
    set(lcm_BUILD_COMMAND ${CMAKE_COMMAND} --build . --config Release)
    set(lcm_FORCE_RELEASE_BUILD_COMMANDS
      BUILD_COMMAND ${lcm_BUILD_COMMAND}
      INSTALL_COMMAND ${lcm_BUILD_COMMAND} --target install)
  endif()

  drake_add_external(gtk PUBLIC CMAKE)
  drake_add_external(lcm PUBLIC CMAKE
    ${lcm_FORCE_RELEASE_BUILD_COMMANDS}
    CMAKE_ARGS
      -DBUILD_SHARED_LIBS=ON # Because LCM has no ABI decoration
      -DCMAKE_BUILD_TYPE=Release # See above
      -DCMAKE_PREFIX_PATH=${CMAKE_CURRENT_SOURCE_DIR}/externals/gtk/gtk3
    DEPENDS gtk)
else()
  drake_add_external(lcm PUBLIC CMAKE
    CMAKE_ARGS -DBUILD_SHARED_LIBS=ON)
endif()

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
    -DWITH_BERTINI:BOOL=${WITH_BERTINI}
    -DWITH_BULLET:BOOL=${WITH_BULLET}
    -DWITH_DIRECTOR:BOOL=${WITH_DIRECTOR}
    -DWITH_DREAL:BOOL=${WITH_DREAL}
    -DWITH_GLOPTIPOLY:BOOL=${WITH_GLOPTIPOLY}
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
    bertini
    bot_core_lcmtypes
    bullet
    cmake
    director
    dreal
    eigen
    gflags
    gloptipoly
    google_styleguide
    googletest
    gtk
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

## grab and install precompiled snopt

# TODO: look for snopt_c
if(snopt_c_FOUND OR WITH_SNOPT)
  set(WITH_SNOPT_PRECOMPILED OFF)
endif()
if(WITH_SNOPT_PRECOMPILED)
  message(STATUS "Preparing to install precompiled snopt")
  ExternalProject_Add(download-snopt-precompiled
    URL "https://s3.amazonaws.com/drake-provisioning/drakeSnopt.zip"
    URL_MD5 "7b36168cba2fb9a56b2fd6117427fc4a"
    SOURCE_DIR "${CMAKE_BINARY_DIR}/snopt-precompiled"
    CONFIGURE_COMMAND ""
    BUILD_COMMAND cmake -E copy_directory
      ${CMAKE_BINARY_DIR}/snopt-precompiled/
      ${PROJECT_SOURCE_DIR}/drake/matlab/solvers/
    INSTALL_COMMAND "")
  add_dependencies(download-all download-snopt-precompiled)
  add_dependencies(drake download-snopt-precompiled) # just in case: make sure any compiled drake version happens after precompiled install
endif()
