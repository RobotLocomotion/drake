cmake_minimum_required(VERSION 3.5)
project(drake-superbuild)

include(CTest)

if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS 4.9)
  message(FATAL_ERROR "GCC version must be at least 4.9")
elseif(CMAKE_CXX_COMPILER_ID STREQUAL "AppleClang" AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS 7)
  message(FATAL_ERROR "Apple Clang version must be at least 7")
elseif(CMAKE_CXX_COMPILER_ID STREQUAL "Clang" AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS 3.7)
  message(FATAL_ERROR "Clang version must be at least 3.7")
elseif(CMAKE_CXX_COMPILER_ID STREQUAL "MSVC" AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS 19)
  message(FATAL_ERROR "MSVC version must be at least 14")
endif()

# PODs out-of-source build logic
if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
  if(POD_BUILD)
    find_file(_build_dir build PATHS ${PROJECT_SOURCE_DIR} ${PROJECT_SOURCE_DIR}/.. ${PROJECT_SOURCE_DIR}/../.. ${PROJECT_SOURCE_DIR}/../../.. ${PROJECT_SOURCE_DIR}/../../../..)
    if(_build_dir)
      set(CMAKE_INSTALL_PREFIX "${_build_dir}" CACHE STRING
        "Prefix for installation of sub-packages (note: required during build!)" FORCE)
    else()
      execute_process(COMMAND ${CMAKE_COMMAND} -E make_directory ${PROJECT_SOURCE_DIR}/build)
      set(CMAKE_INSTALL_PREFIX ${PROJECT_SOURCE_DIR}/build
      CACHE STRING "install prefix" FORCE)
    endif()
  else()
    set(CMAKE_INSTALL_PREFIX "${CMAKE_BINARY_DIR}/install" CACHE STRING
      "Prefix for installation of sub-packages (note: required during build!)" FORCE)
  endif()
endif()
message(STATUS CMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX})

include(ExternalProject)
find_package(Git REQUIRED)

function(drake_forceupdate proj)
  # CMake < 3.6 forget to mark the update step as "ALWAYS" when an explicit
  # UPDATE_COMMAND is used.  Add our own "ALWAYS" step to force updates.
  if(AUTO_UPDATE_EXTERNALS AND CMAKE_VERSION VERSION_LESS 3.6)
    ExternalProject_Add_Step(${proj} forceupdate
      DEPENDEES download
      DEPENDERS update
      ALWAYS 1
      )
  endif()
endfunction()

# Require Java at the super-build level since libbot cannot configure without finding Java
find_package(Java 1.6 REQUIRED)
include(UseJava)

# menu of extra examples
option(EXAMPLES_LITTLEDOG "planning and control for a small quadruped robot" OFF)

## External dependencies
option(AUTO_UPDATE_EXTERNALS "external projects are updated to their tag revision on compile" ON)

# Choose your python (major) version
option(WITH_PYTHON_3 "Force Drake to use python 3 instead of python 2" OFF)

# menu of external projects
option(WITH_ALL_PUBLIC_EXTERNALS "enable all externals available to public academic users" OFF)
option(WITH_ALL_SUPPORTED_EXTERNALS "enable all externals available for your platform (includes private git repositories)" OFF)
option(REMOVE_UNUSED_EXTERNALS "enable this to remove those projects from disk" OFF)

##########################################
# External Projects that are ON by default
##########################################
option(WITH_EIGEN "required c++ matrix library.  only disable if you have it already." ON)
option(WITH_GOOGLETEST "required c++ unit test library.  only disable if you have it already." ON)
option(WITH_SWIGMAKE "helper tools to build python & MATLAB wrappers for C++ libraries with Eigen" ON)
option(WITH_BULLET "used for collision detection" ON)
option(WITH_LCM "interprocess communications protocol for visualizers, etc" ON)
option(WITH_BOT_CORE_LCMTYPES "required LCM types library. only disable if you have it already." ON)
if(WIN32)
  option(WITH_GTK "precompiled gtk binaries/headers for Windows" ON)  # needed for lcm on windows
else()
  option(WITH_DIRECTOR "vtk-based visualization tool and robot user interface" ON) # not win32 yet.  it builds on windows, but requires manually installation of vtk, etc.  perhaps make a precompiled director pod (a bit like snopt)
  option(WITH_LIBBOT "simple open-gl visualizer + lcmgl for director" ON) # there is hope, but i spent a long time cleaning up c/c++ language errors trying to make msvc happy.. and still had a long way to go.
  option(WITH_NLOPT "nonlinear optimization solver" ON)
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
cmake_dependent_option(WITH_MOSEK "convex optimization solver; free for academics" OFF "NOT DISABLE_MATLAB;matlab;NOT WIN32" OFF)
cmake_dependent_option(WITH_SEDUMI "semi-definite programming solver" OFF "NOT DISABLE_MATLAB;matlab;NOT WIN32" OFF)
cmake_dependent_option(WITH_YALMIP "free optimization front-end for MATLAB" OFF "NOT DISABLE_MATLAB;matlab;NOT WIN32" OFF)
cmake_dependent_option(WITH_XFOIL "use w/ XFOIL to compute aerodynamic coefficients for airfoils" OFF "NOT DISABLE_MATLAB;matlab;NOT WIN32" OFF)

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
endif()

if(WITH_TEXTBOOK OR WITH_ALL_SUPPORTED_EXTERNALS OR WITH_ALL_PUBLIC_EXTERNALS)
  find_package(PythonInterp)
  if(NOT PYTHON_EXECUTABLE)
    message(FATAL_ERROR "could not find python, which is required for the the textbook examples")
  endif()
endif()

# list *compilation* dependencies, in alphabetical order by target (note: dependencies must come first in my foreach above)
set(lcm_dependencies gtk)
set(libbot_dependencies lcm)
set(bot_core_lcmtypes_dependencies lcm)
set(director_dependencies bot_core_lcmtypes lcm libbot)
set(iris_dependencies eigen mosek)
set(signalscope_dependencies director)

# List compilation and runtime dependencies. Runtime dependencies are needed
# because the drake project must configure only after any dependencies used by
# MATLAB have been installed.
set(drake_dependencies bertini bot_core_lcmtypes bullet cmake eigen gloptipoly3 googletest gurobi iris lcm libbot mosek nlopt ipopt octomap sedumi snopt spotless swig_matlab swigmake yalmip yaml_cpp)

# download information, in alphabetical order
set(avl_IS_PUBLIC TRUE)
set(bertini_IS_PUBLIC FALSE)
set(bullet_IS_PUBLIC TRUE)
set(bullet_IS_CMAKE_POD TRUE)
set(cmake_SOURCE_DIR "${PROJECT_SOURCE_DIR}/drake/cmake")
set(cmake_NO_BUILD TRUE)
set(cmake_IS_PUBLIC TRUE)
set(director_BUILD_COMMAND_DIR distro/pods/drake-distro)
set(director_IS_PUBLIC TRUE)
set(drake_java_args -DJava_JAVA_EXECUTABLE=${Java_JAVA_EXECUTABLE} -DJava_JAVAC_EXECUTABLE=${Java_JAVAC_EXECUTABLE} -DJava_JAVAH_EXECUTABLE=${Java_JAVAH_EXECUTABLE} -DJava_VERSION_STRING=${Java_VERSION_STRING})
set(drake_ADDITIONAL_CMAKE_CONFIGURE_ARGS ${drake_java_args} -DWITH_PYTHON_3=${WITH_PYTHON_3} -DDISABLE_MATLAB=${DISABLE_MATLAB})
set(eigen_IS_CMAKE_POD TRUE)
set(eigen_IS_PUBLIC TRUE)
set(googletest_IS_CMAKE_POD TRUE)
set(googletest_IS_PUBLIC TRUE)
set(googletest_ADDITIONAL_CMAKE_CONFIGURE_ARGS -DBUILD_SHARED_LIBS=ON -DCMAKE_INSTALL_NAME_DIR=${CMAKE_INSTALL_PREFIX}/lib -DGTEST_CREATE_SHARED_LIBRARY=1) # The "-DGTEST_CREATE_SHARED_LIBRARY=1" is needed to support parameterized Google tests on Windows.
set(gtk_IS_CMAKE_POD TRUE)
set(gtk_IS_PUBLIC TRUE)
set(gloptipoly_IS_PUBLIC FALSE)
set(gurobi_IS_PUBLIC FALSE)
set(ipopt_IS_PUBLIC TRUE)
# TODO(sam.creasey) add an alternate build command for WIN32 which
# downloads the precopiled binary and installs that.
set(ipopt_BUILD_COMMAND ./configure  --with-blas=BUILD --with-lapack=BUILD --prefix=${CMAKE_INSTALL_PREFIX} --includedir=${CMAKE_INSTALL_PREFIX}/include/ipopt --disable-shared --with-pic && make install)
set(iris_ADDITIONAL_BUILD_ARGS PATCH_COMMAND make configure-cdd-only)
set(iris_IS_PUBLIC TRUE)
set(lcm_IS_CMAKE_POD TRUE)
set(lcm_IS_PUBLIC TRUE)
set(lcm_ADDITIONAL_CMAKE_CONFIGURE_ARGS -DBUILD_SHARED_LIBS=ON)
if(WITH_GTK)
  set(lcm_ADDITIONAL_CMAKE_CONFIGURE_ARGS ${lcm_ADDITIONAL_CMAKE_CONFIGURE_ARGS} -DCMAKE_PREFIX_PATH=${CMAKE_CURRENT_SOURCE_DIR}/externals/gtk/gtk3)
endif()
set(libbot_IS_CMAKE_POD TRUE)
set(libbot_IS_PUBLIC TRUE)
set(bot_core_lcmtypes_IS_CMAKE_POD TRUE)
set(bot_core_lcmtypes_IS_PUBLIC TRUE)
set(meshconverters_IS_PUBLIC TRUE)
set(mosek_IS_PUBLIC TRUE)
set(nlopt_BUILD_COMMAND ./autogen.sh --no-configure && ./configure --without-matlab --without-python --without-octave --without-guile --enable-maintainer-mode --enable-shared --prefix=${CMAKE_INSTALL_PREFIX} --includedir=${CMAKE_INSTALL_PREFIX}/include/nlopt && make install)
set(nlopt_IS_PUBLIC TRUE)
set(octomap_IS_PUBLIC TRUE)
set(sedumi_IS_PUBLIC FALSE)
set(signalscope_IS_PUBLIC TRUE)
set(snopt_IS_CMAKE_POD TRUE)
set(snopt_IS_PUBLIC FALSE)
set(spotless_IS_CMAKE_POD TRUE)
set(spotless_IS_PUBLIC TRUE)
set(swigmake_IS_PUBLIC TRUE)
set(swigmake_IS_CMAKE_POD TRUE)
set(swigmake_NO_CLEAN TRUE)
set(swig_matlab_IS_PUBLIC TRUE)
set(swig_matlab_IS_CMAKE_POD TRUE)
set(textbook_SOURCE_DIR "${PROJECT_SOURCE_DIR}/drake/doc/textbook")
set(textbook_BUILD_COMMAND ${PYTHON_EXECUTABLE} extract_examples.py underactuated.html ./examples)
set(textbook_IS_PUBLIC TRUE)
set(yalmip_IS_PUBLIC TRUE)
set(yaml_cpp_IS_PUBLIC TRUE)
set(yaml_cpp_IS_CMAKE_POD TRUE)
if(APPLE OR WIN32)
  set(yaml_cpp_ADDITIONAL_CMAKE_CONFIGURE_ARGS -DBUILD_SHARED_LIBS=OFF)
else()
  set(yaml_cpp_ADDITIONAL_CMAKE_CONFIGURE_ARGS -DBUILD_SHARED_LIBS=ON)
endif()
set(xfoil_IS_PUBLIC TRUE)

find_program(MAKE_EXECUTABLE make)
if(NOT MAKE_EXECUTABLE)
  message(WARNING "couldn't find gnu make")
endif()
if(${CMAKE_GENERATOR} STREQUAL "Unix Makefiles")
  set(PODS_MAKE_COMMAND "$(MAKE)")   # so we can pass through commandline arguments.
else()
  set(PODS_MAKE_COMMAND ${MAKE_EXECUTABLE})
endif()

# process optional projects
# note: keep drake in this loop in case externals depend on drake (e.g. the director might in the near future)
set(EXTERNAL_PROJECTS)
foreach(proj IN ITEMS
  cmake
  textbook
  eigen
  googletest
  gtk
  lcm
  bot_core_lcmtypes
  libbot
  bullet
  spotless
  director
  signalscope
  octomap
  snopt
  gurobi
  mosek
  iris
  yalmip
  gloptipoly
  bertini
  sedumi
  avl
  xfoil
  meshconverters
  swigmake
  swig_matlab
  yaml_cpp
  ipopt
  nlopt
  drake)
  string(TOUPPER ${proj} proj_upper)
  if(${proj} STREQUAL "drake" OR
    ${proj} STREQUAL "cmake" OR
    WITH_${proj_upper} OR
    (WITH_ALL_SUPPORTED_EXTERNALS AND DEFINED WITH_${proj_upper}) OR
    (WITH_ALL_PUBLIC_EXTERNALS AND ${proj}_IS_PUBLIC AND DEFINED WITH_${proj_upper}))
    list(APPEND EXTERNAL_PROJECTS ${proj})
  elseif(REMOVE_UNUSED_EXTERNALS AND IS_DIRECTORY ${proj})
    message(STATUS "removing unused project: ${proj}")
    if(NOT ${proj}_NO_BUILD)
      execute_process(COMMAND ${MAKE_EXECUTABLE} BUILD_PREFIX=${CMAKE_INSTALL_PREFIX} BUILD_TYPE=$<CONFIGURATION> clean
          WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/${proj})
    endif()
    execute_process(COMMAND ${CMAKE_COMMAND} -E remove_directory "${PROJECT_SOURCE_DIR}/${proj}")
  endif()
endforeach()

set(COMMON_CMAKE_ARGS
  CMAKE_GENERATOR ${CMAKE_GENERATOR}
  CMAKE_ARGS
    -DCMAKE_INSTALL_PREFIX:PATH=${CMAKE_INSTALL_PREFIX}
    -DCMAKE_BUILD_TYPE:STRING=${CMAKE_BUILD_TYPE}
    -DBUILD_SHARED_LIBS:BOOL=${BUILD_SHARED_LIBS}
    -DCMAKE_C_COMPILER:STRING=${CMAKE_C_COMPILER}
    -DCMAKE_C_FLAGS:STRING=${CMAKE_C_FLAGS}
    -DCMAKE_CXX_COMPILER:STRING=${CMAKE_CXX_COMPILER}
    -DCMAKE_CXX_FLAGS:STRING=${CMAKE_CXX_FLAGS}
    -DCMAKE_EXE_LINKER_FLAGS:STRING=${CMAKE_EXE_LINKER_FLAGS}
    -DCMAKE_MODULE_LINKER_FLAGS:STRING=${CMAKE_MODULE_LINKER_FLAGS}
    -DCMAKE_SHARED_LINKER_FLAGS:STRING=${CMAKE_SHARED_LINKER_FLAGS}
    -DCMAKE_STATIC_LINKER_FLAGS:STRING=${CMAKE_STATIC_LINKER_FLAGS})

set(EXTERNAL_SOURCE_DIRS)
foreach(proj ${EXTERNAL_PROJECTS})
  set(deps)
  foreach(dep ${${proj}_dependencies})
    list(FIND EXTERNAL_PROJECTS ${dep} find_result)
    if(${dep} STREQUAL "drake" OR NOT find_result EQUAL -1)
      list(APPEND deps ${dep})
    endif()
  endforeach()

  if(NOT ${proj} STREQUAL "drake")
    if(NOT ${proj}_SOURCE_DIR)
      set(${proj}_SOURCE_DIR "${PROJECT_SOURCE_DIR}/externals/${proj}")
    endif()
    set(EXTERNAL_SOURCE_DIRS ${EXTERNAL_SOURCE_DIRS} ${${proj}_SOURCE_DIR} "\n")

    message(STATUS "Preparing to build ${proj} with dependencies: ${deps}")

    # Compute the path to the submodule for this external.
    file(RELATIVE_PATH ${proj}_GIT_SUBMODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR} ${${proj}_SOURCE_DIR})

    # Initialize the submodule configuration now so parallel downloads do not conflict later.
    execute_process(COMMAND ${GIT_EXECUTABLE} submodule init -- ${${proj}_GIT_SUBMODULE_PATH}
      WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

    # Download externals as Git submodules.
    set(${proj}_DOWNLOAD_COMMAND ${GIT_EXECUTABLE} submodule update --init --recursive -- ${${proj}_GIT_SUBMODULE_PATH})

    if(AUTO_UPDATE_EXTERNALS)
      set(${proj}_UPDATE_COMMAND ${CMAKE_COMMAND} -E chdir ${CMAKE_CURRENT_SOURCE_DIR} ${${proj}_DOWNLOAD_COMMAND})

      # Synchronize the submodule url now so parallel updates do not conflict later.
      execute_process(COMMAND ${GIT_EXECUTABLE} submodule sync -- ${${proj}_GIT_SUBMODULE_PATH}
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
    else()
      set(${proj}_UPDATE_COMMAND "")
    endif()

    # now the actual project
    if(${proj}_IS_CMAKE_POD)
      if(NOT ${proj}_BINARY_DIR)
        if(POD_BUILD)
          set(${proj}_BINARY_DIR "${${proj}_SOURCE_DIR}/pod-build")
          # Make sure the source tree is empty before cloning into it.
          set(${proj}_DOWNLOAD_COMMAND ${CMAKE_COMMAND} -E remove_directory ${${proj}_GIT_SUBMODULE_PATH}/pod-build
            COMMAND ${${proj}_DOWNLOAD_COMMAND}
            COMMAND ${CMAKE_COMMAND} -E make_directory ${${proj}_GIT_SUBMODULE_PATH}/pod-build)
        else()
          set(${proj}_BINARY_DIR "${PROJECT_BINARY_DIR}/externals/${proj}")
        endif()
      endif()
      set(PODS_VERBOSE_MAKEFILE "")
      if(CMAKE_VERBOSE_MAKEFILE)
        set(PODS_VERBOSE_MAKEFILE "-DCMAKE_VERBOSE_MAKEFILE=ON")
      endif()
      ExternalProject_Add(${proj}
        SOURCE_DIR ${${proj}_SOURCE_DIR}
        BINARY_DIR ${${proj}_BINARY_DIR}
        DOWNLOAD_COMMAND ${${proj}_DOWNLOAD_COMMAND}
        DOWNLOAD_DIR ${CMAKE_CURRENT_SOURCE_DIR}
        UPDATE_COMMAND ${${proj}_UPDATE_COMMAND}
        BUILD_ALWAYS 1
        INDEPENDENT_STEP_TARGETS update
        DEPENDS ${deps}
        ${COMMON_CMAKE_ARGS}
        ${PODS_VERBOSE_MAKEFILE}
        ${${proj}_ADDITIONAL_CMAKE_CONFIGURE_ARGS}
        ${${proj}_ADDITIONAL_BUILD_ARGS})
      drake_forceupdate(${proj})
    else() # not a CMake POD
      if(NOT ${proj}_BINARY_DIR)
        # In-source build for non-CMake projects.
        set(${proj}_BINARY_DIR "${${proj}_SOURCE_DIR}")
      endif()
      if(NOT DEFINED ${proj}_BUILD_COMMAND)
        if(${proj}_NO_BUILD)
          set(${proj}_BUILD_COMMAND "")
        else() # then use the PODS gnu make system
          set(PODS_VERBOSE_MAKEFILE "")
          if(CMAKE_VERBOSE_MAKEFILE)
            set(PODS_VERBOSE_MAKEFILE "V=1 VERBOSE=1")
          endif()
          set(${proj}_BUILD_COMMAND ${PODS_MAKE_COMMAND} ${PODS_VERBOSE_MAKEFILE} BUILD_PREFIX=${CMAKE_INSTALL_PREFIX} BUILD_TYPE=$<CONFIGURATION>)
          if(${proj}_BUILD_COMMAND_DIR)
            set(${proj}_BUILD_COMMAND ${CMAKE_COMMAND} -E chdir ${${proj}_BUILD_COMMAND_DIR} ${${proj}_BUILD_COMMAND})
          endif()
        endif()
      endif()

      if(${proj}_BUILD_COMMAND)
        set(${proj}_BUILD_ALWAYS 1)
      else()
        set(${proj}_BUILD_ALWAYS 0)
      endif()

      ExternalProject_Add(${proj}
        SOURCE_DIR ${${proj}_SOURCE_DIR}
        BINARY_DIR ${${proj}_BINARY_DIR}
        DOWNLOAD_COMMAND ${${proj}_DOWNLOAD_COMMAND}
        DOWNLOAD_DIR ${CMAKE_CURRENT_SOURCE_DIR}
        UPDATE_COMMAND ${${proj}_UPDATE_COMMAND}
        INDEPENDENT_STEP_TARGETS update
        CONFIGURE_COMMAND ""
        BUILD_COMMAND "${${proj}_BUILD_COMMAND}"
        BUILD_ALWAYS "${${proj}_BUILD_ALWAYS}"
        INSTALL_COMMAND ""
        DEPENDS ${deps}
        ${${proj}_ADDITIONAL_BUILD_ARGS})
      drake_forceupdate(${proj})
      # message(STATUS "${proj}_BUILD_COMMAND: ${${proj}_BUILD_COMMAND}")
    endif()
  else()
    message(STATUS "Preparing to build ${proj} with dependencies: ${deps}")
    set(PODS_VERBOSE_MAKEFILE "")
    if(CMAKE_VERBOSE_MAKEFILE)
      set(PODS_VERBOSE_MAKEFILE "-DCMAKE_VERBOSE_MAKEFILE=ON")
    endif()
    if(POD_BUILD)
      set(drake_BINARY_DIR "${PROJECT_SOURCE_DIR}/drake/pod-build")
    else()
      set(drake_BINARY_DIR "${PROJECT_BINARY_DIR}/drake")
    endif()
    ExternalProject_Add(${proj}
      SOURCE_DIR "${PROJECT_SOURCE_DIR}/drake"
      DOWNLOAD_COMMAND ""
      UPDATE_COMMAND ""
      BINARY_DIR ${drake_BINARY_DIR}
      BUILD_ALWAYS 1
      DEPENDS ${deps}
      ${COMMON_CMAKE_ARGS}
      ${PODS_VERBOSE_MAKEFILE}
      ${${proj}_ADDITIONAL_CMAKE_CONFIGURE_ARGS}
      ${${proj}_ADDITIONAL_BUILD_ARGS})
  endif()

endforeach()
file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/drake_external_source_dirs.txt ${EXTERNAL_SOURCE_DIRS})

# TODO: add a custom target for release_filelist

add_custom_target(download-all)
add_custom_target(clean-all)
add_custom_target(status)
set(PROJECT_LIST)
foreach(proj ${EXTERNAL_PROJECTS})
  if(NOT ${proj} STREQUAL "drake")
    add_dependencies(download-all ${proj}-update)
    ExternalProject_Get_Property(${proj} SOURCE_DIR)
    add_custom_target(status-${proj}
      COMMAND ${GIT_EXECUTABLE} status
      WORKING_DIRECTORY ${SOURCE_DIR})
    add_dependencies(status status-${proj})
  endif()

  ExternalProject_Get_Property(${proj} SOURCE_DIR)
  # Handle pods-style 'make clean' cleaning of external projects/pods
  if(NOT ${proj}_NO_BUILD)
    if(${proj}_IS_CMAKE_POD)
      ExternalProject_Get_Property(${proj} BINARY_DIR)
      # if _NO_CLEAN is set, don't attempt to run `make clean`, just wipe the build folder
      if(${proj}_NO_CLEAN)
        add_custom_target(clean-${proj}
          COMMAND ${CMAKE_COMMAND} -E remove_directory ${BINARY_DIR})
      else()
        add_custom_target(clean-${proj}
          COMMAND ${CMAKE_COMMAND} --build ${BINARY_DIR} --target clean
          COMMAND ${CMAKE_COMMAND} -E remove_directory ${BINARY_DIR})
      endif()
      add_dependencies(clean-all clean-${proj})
    else()
      add_custom_target(clean-${proj}
        COMMAND ${PODS_MAKE_COMMAND} BUILD_PREFIX=${CMAKE_INSTALL_PREFIX} BUILD_TYPE=$<CONFIGURATION> clean
        WORKING_DIRECTORY ${SOURCE_DIR})
      add_dependencies(clean-all clean-${proj})
    endif()
  endif()
  list(APPEND PROJECT_LIST ${SOURCE_DIR})
endforeach()

# process optional examples
foreach(example IN ITEMS LittleDog)
  string(TOUPPER ${example} example_upper)
  if(EXAMPLES_${example_upper})
    message(STATUS "Installation will include extra example: ${example}")

    # Initialize the submodule configuration now so parallel downloads do not conflict later.
    execute_process(COMMAND ${GIT_EXECUTABLE} submodule init -- drake/examples/${example}
      WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

    # Download examples as Git submodules.
    set(${example}_DOWNLOAD_COMMAND ${GIT_EXECUTABLE} submodule update --init --recursive -- drake/examples/${example})
    if(AUTO_UPDATE_EXTERNALS)
      set(${example}_UPDATE_COMMAND ${CMAKE_COMMAND} -E chdir ${CMAKE_CURRENT_SOURCE_DIR} ${${example}_DOWNLOAD_COMMAND})

      # Synchronize the submodule url now so parallel updates do not conflict later.
      execute_process(COMMAND ${GIT_EXECUTABLE} submodule sync -- drake/examples/${example}
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
    else()
      set(${example}_UPDATE_COMMAND "")
    endif()

    ExternalProject_Add(download-${example}
      SOURCE_DIR ${PROJECT_SOURCE_DIR}/drake/examples/${example}
      DOWNLOAD_COMMAND ${${example}_DOWNLOAD_COMMAND}
      DOWNLOAD_DIR ${CMAKE_CURRENT_SOURCE_DIR}
      UPDATE_COMMAND ${${example}_UPDATE_COMMAND}
      CONFIGURE_COMMAND ""
      BUILD_COMMAND ""
      INSTALL_COMMAND "")
    drake_forceupdate(download-${example})
    add_dependencies(drake download-${example})
    add_dependencies(download-all download-${example})
  endif()
endforeach()

string(REPLACE ";" " " PROJECT_LIST "${PROJECT_LIST}")
add_custom_target(list-project-dirs COMMAND echo "${PROJECT_LIST}")

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
    BUILD_COMMAND cmake -E copy_directory ${CMAKE_BINARY_DIR}/snopt-precompiled/ ${PROJECT_SOURCE_DIR}/drake/solvers/
    INSTALL_COMMAND "")
  add_dependencies(download-all download-snopt-precompiled)
  add_dependencies(drake download-snopt-precompiled) # just in case: make sure any compiled drake version happens after precompiled install
endif()
