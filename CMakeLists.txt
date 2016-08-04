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

###############################################################################
# BEGIN external projects

# External projects in order of dependencies; 'trivial' ones first
drake_add_external(avl PUBLIC)
drake_add_external(bertini)
drake_add_external(bullet PUBLIC CMAKE)
drake_add_external(eigen PUBLIC CMAKE)
drake_add_external(gloptipoly)
drake_add_external(gurobi)
drake_add_external(meshconverters PUBLIC)
drake_add_external(mosek PUBLIC)
drake_add_external(octomap PUBLIC)
drake_add_external(snopt CMAKE)
drake_add_external(spdlog PUBLIC CMAKE)
drake_add_external(spotless PUBLIC CMAKE)
drake_add_external(swig_matlab PUBLIC CMAKE)
drake_add_external(swigmake PUBLIC CMAKE)
drake_add_external(xfoil PUBLIC)
drake_add_external(yalmip PUBLIC)

# cmake
drake_add_external(cmake PUBLIC ALWAYS
  BUILD_COMMAND :
  SOURCE_DIR ${PROJECT_SOURCE_DIR}/drake/cmake)

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
    -DWITH_DIRECTOR=${WITH_DIRECTOR}
    -DWITH_GOOGLE_STYLEGUIDE:BOOL=${WITH_GOOGLE_STYLEGUIDE}
    -DWITH_PYTHON_3:BOOL=${WITH_PYTHON_3}
  DEPENDS
    bertini
    bot_core_lcmtypes
    bullet
    cmake
    director
    eigen
    gloptipoly3
    google_styleguide
    googletest
    gurobi
    ipopt
    iris
    lcm
    libbot
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
      ${PROJECT_SOURCE_DIR}/drake/solvers/
    INSTALL_COMMAND "")
  add_dependencies(download-all download-snopt-precompiled)
  add_dependencies(drake download-snopt-precompiled) # just in case: make sure any compiled drake version happens after precompiled install
endif()
