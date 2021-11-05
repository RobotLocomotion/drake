# -*- mode: cmake -*-
# vi: set ft=cmake :

cmake_minimum_required(VERSION 3.10)

if(CMAKE_VERSION VERSION_GREATER_EQUAL 3.16)
  cmake_policy(VERSION 3.16)
endif()

project(drake
  DESCRIPTION "Model-based design and verification for robotics"
  LANGUAGES C CXX
)

list(INSERT CMAKE_MODULE_PATH 0 "${PROJECT_SOURCE_DIR}/cmake/modules")

include(CTest)

configure_file(CTestCustom.cmake.in CTestCustom.cmake @ONLY)

if(ANDROID OR CYGWIN OR IOS OR NOT UNIX)
  message(FATAL_ERROR
    "Android, Cygwin, iOS, and non-Unix platforms are NOT supported"
  )
endif()

if(NOT CMAKE_SYSTEM_PROCESSOR STREQUAL x86_64)
  message(FATAL_ERROR
    "System processor ${CMAKE_SYSTEM_PROCESSOR} is NOT supported"
  )
endif()

set(UNIX_DISTRIBUTION_ID)
set(UNIX_DISTRIBUTION_CODENAME)

if(APPLE)
  if(CMAKE_SYSTEM_VERSION VERSION_LESS 20)
    message(FATAL_ERROR
      "Darwin ${CMAKE_SYSTEM_VERSION} is NOT supported. Please use "
      "Darwin 20.x (macOS Big Sur) or 21.x (macOS Monterey)."
    )
  endif()
else()
  find_program(LSB_RELEASE_EXECUTABLE NAMES lsb_release)

  if(NOT LSB_RELEASE_EXECUTABLE)
    message(FATAL_ERROR "Could NOT find the lsb_release executable")
  endif()

  mark_as_advanced(LSB_RELEASE_EXECUTABLE)

  execute_process(COMMAND "${LSB_RELEASE_EXECUTABLE}" --id --short
    RESULT_VARIABLE LSB_RELEASE_ID_SHORT_RESULT_VARIABLE
    OUTPUT_VARIABLE LSB_RELEASE_ID_SHORT_OUTPUT_VARIABLE
    OUTPUT_STRIP_TRAILING_WHITESPACE
  )

  if(LSB_RELEASE_ID_SHORT_RESULT_VARIABLE EQUAL 0)
    set(UNIX_DISTRIBUTION_ID "${LSB_RELEASE_ID_SHORT_OUTPUT_VARIABLE}")
  endif()

  if(NOT UNIX_DISTRIBUTION_ID STREQUAL Ubuntu)
    message(FATAL_ERROR
      "Distribution ${UNIX_DISTRIBUTION_ID} is NOT supported"
    )
  endif()

  string(TOLOWER "${UNIX_DISTRIBUTION_ID}" UNIX_DISTRIBUTION_ID)

  execute_process(COMMAND "${LSB_RELEASE_EXECUTABLE}" --codename --short
    RESULT_VARIABLE LSB_RELEASE_CODENAME_SHORT_RESULT_VARIABLE
    OUTPUT_VARIABLE LSB_RELEASE_CODENAME_SHORT_OUTPUT_VARIABLE
    OUTPUT_STRIP_TRAILING_WHITESPACE
  )

  if(LSB_RELEASE_CODENAME_SHORT_RESULT_VARIABLE EQUAL 0)
    set(UNIX_DISTRIBUTION_CODENAME
      "${LSB_RELEASE_CODENAME_SHORT_OUTPUT_VARIABLE}"
    )
  endif()

  # The supported releases should match those listed in both doc/developers.rst
  # and tools/workspace/os.bzl.
  if(NOT UNIX_DISTRIBUTION_CODENAME MATCHES "^(bionic|focal)$")
    message(FATAL_ERROR
      "Release ${UNIX_DISTRIBUTION_CODENAME} is NOT supported. Please use "
      "Ubuntu 18.04 (Bionic) or 20.04 (Focal)."
    )
  endif()
endif()

# The version passed to find_package(Bazel) should match the
# minimum_bazel_version value in the call to versions.check() in WORKSPACE.
set(MINIMUM_BAZEL_VERSION 3.0)
find_package(Bazel ${MINIMUM_BAZEL_VERSION} MODULE REQUIRED)

get_filename_component(C_COMPILER_REALPATH "${CMAKE_C_COMPILER}" REALPATH)
get_filename_component(C_COMPILER_NAME "${C_COMPILER_REALPATH}" NAME)

get_filename_component(CXX_COMPILER_REALPATH "${CMAKE_CXX_COMPILER}" REALPATH)
get_filename_component(CXX_COMPILER_NAME "${CXX_COMPILER_REALPATH}" NAME)

if(C_COMPILER_NAME STREQUAL ccache OR CXX_COMPILER_NAME STREQUAL ccache)
  message(FATAL_ERROR
    "Compilation with ccache is NOT supported due to incompatibility with Bazel"
  )
endif()

# The minimum compiler versions should match those listed in both
# doc/developers.rst and tools/workspace/cc/repository.bzl.
set(MINIMUM_APPLE_CLANG_VERSION 12)
set(MINIMUM_CLANG_VERSION 9)
set(MINIMUM_GNU_VERSION 7.5)

if(CMAKE_C_COMPILER_ID STREQUAL AppleClang)
  if(CMAKE_C_COMPILER_VERSION VERSION_LESS ${MINIMUM_APPLE_CLANG_VERSION})
    message(FATAL_ERROR
      "Compilation with clang ${CMAKE_C_COMPILER_VERSION} is NOT supported"
    )
  endif()
elseif(CMAKE_C_COMPILER_ID STREQUAL Clang)
  if(CMAKE_C_COMPILER_VERSION VERSION_LESS ${MINIMUM_CLANG_VERSION})
    message(FATAL_ERROR
      "Compilation with clang ${CMAKE_C_COMPILER_VERSION} is NOT supported"
    )
  endif()
elseif(CMAKE_C_COMPILER_ID STREQUAL GNU)
  if(CMAKE_C_COMPILER_VERSION VERSION_LESS ${MINIMUM_GNU_VERSION})
    message(FATAL_ERROR
      "Compilation with gcc ${CMAKE_C_COMPILER_VERSION} is NOT supported"
    )
  endif()
else()
  message(WARNING
    "Compilation with ${CMAKE_C_COMPILER_ID} is NOT supported. Compilation of "
    "project drake_cxx_python may fail."
  )
endif()

if(CMAKE_CXX_COMPILER_ID STREQUAL AppleClang)
  if(CMAKE_CXX_COMPILER_VERSION VERSION_LESS ${MINIMUM_APPLE_CLANG_VERSION})
    message(FATAL_ERROR
      "Compilation with clang++ ${CMAKE_CXX_COMPILER_VERSION} is NOT supported"
    )
  endif()
elseif(CMAKE_CXX_COMPILER_ID STREQUAL Clang)
  if(CMAKE_CXX_COMPILER_VERSION VERSION_LESS ${MINIMUM_CLANG_VERSION})
    message(FATAL_ERROR
      "Compilation with clang++ ${CMAKE_CXX_COMPILER_VERSION} is NOT supported"
    )
  endif()
elseif(CMAKE_CXX_COMPILER_ID STREQUAL GNU)
  if(CMAKE_CXX_COMPILER_VERSION VERSION_LESS ${MINIMUM_GNU_VERSION})
    message(FATAL_ERROR
      "Compilation with g++ ${CMAKE_CXX_COMPILER_VERSION} is NOT supported"
    )
  endif()
else()
  # Check flag added in tools/cc_toolchain/bazel.rc is supported.
  include(CheckCXXCompilerFlag)
  set(FLAG_STD_CXX17 -std=c++17)
  check_cxx_compiler_flag(${FLAG_STD_CXX17}
    CXX_COMPILER_SUPPORTS_FLAG_STD_CXX17
  )
  if(NOT CXX_COMPILER_SUPPORTS_FLAG_STD_CXX17)
    message(FATAL_ERROR
      "Compilation with ${CMAKE_CXX_COMPILER_ID} is NOT supported because it "
      "does not accept the ${FLAG_STD_CXX17} flag"
    )
  endif()
  message(WARNING
    "Compilation with ${CMAKE_CXX_COMPILER_ID} is NOT supported. Compilation "
    "of project drake_cxx_python may fail."
  )
endif()

set(SUPPORTED_BUILD_TYPES None Debug Release RelWithDebInfo MinSizeRel)
if(CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_CONFIGURATION_TYPES ${SUPPORTED_BUILD_TYPES})
else()
  if(NOT CMAKE_BUILD_TYPE)
    string(REPLACE ";" " " SUPPORTED_BUILD_TYPES_STRING
      "${SUPPORTED_BUILD_TYPES}"
    )
    set(CMAKE_BUILD_TYPE Release CACHE STRING
      "Choose the type of build, options are ${SUPPORTED_BUILD_TYPES_STRING}"
      FORCE
    )
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY
      STRINGS "${SUPPORTED_BUILD_TYPES}"
    )
  endif()

  if(NOT CMAKE_BUILD_TYPE IN_LIST SUPPORTED_BUILD_TYPES)
    message(WARNING "Configuration ${CMAKE_BUILD_TYPE} is NOT supported")
  endif()
endif()

set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)

# The supported Python major/minor versions should match those listed in both
# doc/developers.rst and tools/workspace/python/repository.bzl.
if(APPLE)
  set(SUPPORTED_PYTHON_VERSION 3.9)
elseif(UNIX_DISTRIBUTION_CODENAME STREQUAL bionic)
  set(SUPPORTED_PYTHON_VERSION 3.6)
else()
  set(SUPPORTED_PYTHON_VERSION 3.8)
endif()

if(APPLE)
  set(FIND_PROGRAM_PYTHON3_NAMES "python${SUPPORTED_PYTHON_VERSION}")
else()
  set(FIND_PROGRAM_PYTHON3_NAMES python3)
endif()

if(CMAKE_VERSION VERSION_LESS 3.12)
  find_program(PYTHON_EXECUTABLE NAMES "${FIND_PROGRAM_PYTHON3_NAMES}")
  if(NOT PYTHON_EXECUTABLE)
    message(FATAL_ERROR "Could NOT find the python3 executable")
  endif()
  find_package(PythonInterp 3 EXACT MODULE REQUIRED)
  set(PYTHON_VERSION_MAJOR_MINOR
    "${PYTHON_VERSION_MAJOR}.${PYTHON_VERSION_MINOR}"
  )
else()
  if(PYTHON_EXECUTABLE)
    set(Python_EXECUTABLE "${PYTHON_EXECUTABLE}" CACHE FILEPATH
      "Path to the python3 executable" FORCE
    )
    unset(PYTHON_EXECUTABLE CACHE)
  endif()
  find_program(Python_EXECUTABLE NAMES "${FIND_PROGRAM_PYTHON3_NAMES}")
  if(NOT Python_EXECUTABLE)
    message(FATAL_ERROR "Could NOT find the python3 executable")
  endif()
  set(Python_FIND_STRATEGY LOCATION)
  find_package(Python 3 EXACT MODULE REQUIRED
    COMPONENTS Development Interpreter NumPy
  )
  if(NOT Python_INTERPRETER_ID STREQUAL Python)
    message(WARNING
      "Python interpreter ${Python_INTERPRETER_ID} is NOT supported. Python "
      "code in project drake_cxx_python may fail at runtime."
    )
  endif()
  set(PYTHON_VERSION_MAJOR_MINOR
    "${Python_VERSION_MAJOR}.${Python_VERSION_MINOR}"
  )
endif()

if(NOT PYTHON_VERSION_MAJOR_MINOR VERSION_EQUAL SUPPORTED_PYTHON_VERSION)
  message(WARNING
    "Python version ${PYTHON_VERSION_MAJOR_MINOR} is NOT supported. Python "
    "code in project drake_cxx_python may fail at runtime."
  )
endif()

if(CMAKE_COLOR_MAKEFILE)
  set(BAZEL_COLOR yes)
else()
  set(BAZEL_COLOR no)
endif()

if(CMAKE_VERBOSE_MAKEFILE)
  set(BAZEL_SUBCOMMANDS yes)
else()
  set(BAZEL_SUBCOMMANDS no)
endif()

set(BAZEL_ENV)

if(NOT APPLE)
  list(APPEND BAZEL_ENV
    "CC=${CMAKE_C_COMPILER}"
    "CXX=${CMAKE_CXX_COMPILER}"
  )
endif()

set(FLAG_OS -Os)
set(FLAG_G -g)
if(CMAKE_CXX_COMPILER_ID MATCHES "^(AppleClang|Clang|GNU)$")
  set(MAYBE_BAZEL_COPT_OS "--copt=${FLAG_OS} --host_copt=${FLAG_OS}")
  set(MAYBE_BAZEL_COPT_G "--copt=${FLAG_G} --host_copt=${FLAG_G}")
else()
  include(CheckCXXCompilerFlag)
  check_cxx_compiler_flag(${FLAG_OS} CXX_COMPILER_SUPPORTS_FLAG_OS)
  if(CXX_COMPILER_SUPPORTS_FLAG_OS)
    set(MAYBE_BAZEL_COPT_OS "--copt=${FLAG_OS} --host_copt=${FLAG_OS}")
  else()
    set(MAYBE_BAZEL_COPT_OS)
    if(CMAKE_CONFIGURATION_TYPES OR CMAKE_BUILD_TYPE STREQUAL MinSizeRel)
      message(WARNING
        "Compilation with ${CMAKE_CXX_COMPILER_ID} in configuration MinSizeRel "
        "is NOT supported because it does not accept the ${FLAG_OS} flag. Code "
        "will NOT be optimized for size."
      )
      if(CMAKE_CONFIGURATION_TYPES)
        list(REMOVE_ITEM CMAKE_CONFIGURATION_TYPES MinSizeRel)
      endif()
    endif()
  endif()
  check_cxx_compiler_flag(${FLAG_G} CXX_COMPILER_SUPPORTS_FLAG_G)
  if(CXX_COMPILER_SUPPORTS_FLAG_G)
    set(MAYBE_BAZEL_COPT_G "--copt=${FLAG_G} --host_copt=${FLAG_G}")
  else()
    set(MAYBE_BAZEL_COPT_G)
    if(CMAKE_CONFIGURATION_TYPES OR CMAKE_BUILD_TYPE STREQUAL RelWithDebInfo)
      message(WARNING
        "Compilation with ${CMAKE_CXX_COMPILER_ID} in configuration "
        "RelWithDebInfo is NOT supported because it does not accept the "
        "${FLAG_G} flag. Debug information will NOT be generated."
      )
      if(CMAKE_CONFIGURATION_TYPES)
        list(REMOVE_ITEM CMAKE_CONFIGURATION_TYPES RelWithDebInfo)
      endif()
    endif()
  endif()
endif()

get_filename_component(PROJECT_BINARY_DIR_REALPATH
  "${PROJECT_BINARY_DIR}" REALPATH
)
get_filename_component(PROJECT_SOURCE_DIR_REALPATH
  "${PROJECT_SOURCE_DIR}" REALPATH
)

# Check whether the PROJECT_BINARY_DIR is a subdirectory of the
# PROJECT_SOURCE_DIR.
string(FIND "${PROJECT_BINARY_DIR_REALPATH}/" "${PROJECT_SOURCE_DIR_REALPATH}/"
  STRING_FIND_RESULT_VARIABLE
)

if(STRING_FIND_RESULT_VARIABLE EQUAL 0)
  # The --output_base cannot be within the WORKSPACE (a subdirectory of
  # PROJECT_SOURCE_DIR), so fallback to the using the same parent directory
  # that Bazel uses by default for its --output_base.
  if(APPLE)
    set(BAZEL_OUTPUT_BASE "/var/tmp")
  else()
    set(BAZEL_OUTPUT_BASE "$ENV{HOME}/.cache/bazel")
  endif()
else()
  set(BAZEL_OUTPUT_BASE "${PROJECT_BINARY_DIR}")
endif()

# Compute the MD5 hash of the PROJECT_BINARY_DIR rather than the WORKSPACE
# (PROJECT_SOURCE_DIR) to avoid colliding with the directory that Bazel uses by
# default.
string(MD5 PROJECT_BINARY_DIR_MD5 "${PROJECT_BINARY_DIR_REALPATH}")
set(BAZEL_OUTPUT_BASE
  "${BAZEL_OUTPUT_BASE}/_bazel_$ENV{USER}/${PROJECT_BINARY_DIR_MD5}"
)

set(BAZEL_CONFIGS)

option(WITH_GUROBI "Build with support for Gurobi" OFF)

if(WITH_GUROBI)
  find_package(Gurobi 9.0.2 EXACT MODULE REQUIRED)

  list(APPEND BAZEL_CONFIGS --config=gurobi)

  if(NOT APPLE)
    get_filename_component(GUROBI_HOME "${Gurobi_INCLUDE_DIRS}" DIRECTORY)
    list(APPEND BAZEL_ENV "GUROBI_HOME=${GUROBI_HOME}")
  endif()
endif()

option(WITH_MOSEK "Build with support for MOSEK" OFF)

if(WITH_MOSEK)
  list(APPEND BAZEL_CONFIGS --config=mosek)
endif()

set(WITH_ROBOTLOCOMOTION_SNOPT OFF CACHE BOOL
  "Build with support for SNOPT using the RobotLocomotion/snopt private GitHub repository"
)

set(WITH_SNOPT OFF CACHE BOOL
  "Build with support for SNOPT using a SNOPT source archive at SNOPT_PATH"
)

if(WITH_ROBOTLOCOMOTION_SNOPT AND WITH_SNOPT)
  message(FATAL_ERROR
    "WITH_ROBOTLOCOMOTION_SNOPT and WITH_SNOPT options are mutually exclusive"
  )
endif()

if(WITH_ROBOTLOCOMOTION_SNOPT OR WITH_SNOPT)
  enable_language(Fortran)

  if(CMAKE_Fortran_COMPILER_ID STREQUAL GNU)
    if(CMAKE_Fortran_COMPILER_VERSION VERSION_LESS ${MINIMUM_GNU_VERSION})
      message(FATAL_ERROR
        "Compilation with gfortran ${CMAKE_Fortran_COMPILER_VERSION} is NOT "
        "supported"
      )
    endif()
  else()
    message(WARNING
      "Compilation with ${CMAKE_Fortran_COMPILER_ID} is NOT supported. "
      "Compilation of project drake_cxx_python may fail."
    )
  endif()

  list(APPEND BAZEL_CONFIGS --config=snopt)

  if(WITH_ROBOTLOCOMOTION_SNOPT)
    list(APPEND BAZEL_ENV "SNOPT_PATH=git")
  else()
    set(SNOPT_PATH SNOPT_PATH-NOTFOUND CACHE FILEPATH
      "Path to SNOPT source archive"
    )
    if(NOT EXISTS "${SNOPT_PATH}")
      message(FATAL_ERROR
        "SNOPT source archive was NOT found at '${SNOPT_PATH}'"
      )
    endif()
    mark_as_advanced(SNOPT_PATH)
    list(APPEND BAZEL_ENV "SNOPT_PATH=${SNOPT_PATH}")
  endif()
endif()

if(BAZEL_CONFIGS)
  string(REPLACE ";" " " MAYBE_BAZEL_CONFIG "${BAZEL_CONFIGS}")
  set(MAYBE_BAZEL_CONFIG "build ${MAYBE_BAZEL_CONFIG}")
else()
  set(MAYBE_BAZEL_CONFIG)
endif()

configure_file(cmake/bazel.rc.in bazel.rc @ONLY)

set(BAZEL_STARTUP_ARGS "--bazelrc=${CMAKE_CURRENT_BINARY_DIR}/bazel.rc")

if(CMAKE_CONFIGURATION_TYPES OR CMAKE_BUILD_TYPE MATCHES "^(Debug|MinSizeRel|Release|RelWithDebInfo)$")
  set(BAZEL_ARGS
    "$<$<CONFIG:Debug>:--config=Debug>$<$<CONFIG:MinSizeRel>:--config=MinSizeRel>$<$<CONFIG:Release>:--config=Release>$<$<CONFIG:RelWithDebInfo>:--config=RelWithDebInfo>"
  )
else()
  set(BAZEL_ARGS)
endif()

# N.B. If you are testing the CMake API and making changes to `install.py.in`,
# you can change this target to something more lightweight, such as
# `//tools/install/dummy:install`.
set(BAZEL_TARGETS //:install)

if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
  set(CMAKE_INSTALL_PREFIX "${PROJECT_BINARY_DIR}/install" CACHE STRING
    "Install path prefix, prepended onto install directories" FORCE
  )
endif()

set(BAZEL_TARGETS_ARGS "${CMAKE_INSTALL_PREFIX}")

if(CMAKE_COLOR_MAKEFILE)
  list(INSERT BAZEL_TARGETS_ARGS 0 "--color")
endif()

if(CMAKE_INSTALL_NAME_TOOL)
  list(INSERT BAZEL_TARGETS_ARGS 0
    "--install_name_tool"
    "${CMAKE_INSTALL_NAME_TOOL}"
  )
endif()

if(CMAKE_STRIP)
  list(INSERT BAZEL_TARGETS_ARGS 0 "--strip_tool" "${CMAKE_STRIP}")
endif()

# If CMAKE_BUILD_TYPE is Debug or RelWithDebInfo, do NOT strip symbols during
# install.
if(CMAKE_CONFIGURATION_TYPES OR CMAKE_BUILD_TYPE MATCHES "^(Debug|RelWithDebInfo)$")
  # SNOPT has restrictions for redistribution given that we are statically
  # linking it in.
  if(WITH_SNOPT OR WITH_ROBOTLOCOMOTION_SNOPT)
    message(WARNING
      "Install configurations Debug and RelWithDebInfo will STILL strip "
      "symbols because support for SNOPT is enabled"
    )
  else()
    list(INSERT BAZEL_TARGETS_ARGS 0
      "$<$<OR:$<CONFIG:Debug>,$<CONFIG:RelWithDebInfo>>:--no_strip>"
    )
  endif()
endif()

include(ExternalProject)

ExternalProject_Add(drake_cxx_python
  SOURCE_DIR "${PROJECT_SOURCE_DIR}"
  CONFIGURE_COMMAND :
  BUILD_COMMAND
    ${BAZEL_ENV}
    "${Bazel_EXECUTABLE}"
    ${BAZEL_STARTUP_ARGS}
    build
    ${BAZEL_ARGS}
    ${BAZEL_TARGETS}
  BUILD_IN_SOURCE ON
  BUILD_ALWAYS ON
  INSTALL_COMMAND
    ${BAZEL_ENV}
    "${Bazel_EXECUTABLE}"
    ${BAZEL_STARTUP_ARGS}
    run
    ${BAZEL_ARGS}
    ${BAZEL_TARGETS}
    --
    ${BAZEL_TARGETS_ARGS}
  USES_TERMINAL_BUILD ON
  USES_TERMINAL_INSTALL ON
)

set(GIT_DIR "${PROJECT_SOURCE_DIR}/.git")
set(GIT_REVISION HEAD)

find_package(Git)

if(GIT_FOUND AND EXISTS "${GIT_DIR}")
  execute_process(COMMAND
    "${GIT_EXECUTABLE}" "--git-dir=${GIT_DIR}" rev-parse HEAD
    RESULT_VARIABLE GIT_REV_PARSE_RESULT_VARIABLE
    OUTPUT_VARIABLE GIT_REV_PARSE_OUTPUT_VARIABLE
    OUTPUT_STRIP_TRAILING_WHITESPACE
  )

  if(GIT_REV_PARSE_RESULT_VARIABLE EQUAL 0)
    set(GIT_REVISION "${GIT_REV_PARSE_OUTPUT_VARIABLE}")
  endif()
endif()

string(TIMESTAMP BUILD_TIMESTAMP "%Y%m%d%H%M%S")

configure_file(tools/install/libdrake/VERSION.TXT.in VERSION.TXT @ONLY)

install(FILES "${PROJECT_BINARY_DIR}/VERSION.TXT" DESTINATION share/doc/drake)

set(CPACK_GENERATOR TGZ)
set(CPACK_INSTALLED_DIRECTORIES "${CMAKE_INSTALL_PREFIX}" .)
set(CPACK_PACKAGE_CONTACT RobotLocomotion)
set(CPACK_PACKAGE_VENDOR RobotLocomotion)
set(CPACK_PACKAGE_VERSION_MAJOR 0)
set(CPACK_PACKAGE_VERSION_MINOR 0)
set(CPACK_PACKAGE_VERSION_PATCH 0)
set(CPACK_PACKAGE_CHECKSUM SHA256)
set(CPACK_SET_DESTDIR ON)
set(CPACK_SOURCE_GENERATOR "")

if(APPLE)
  set(CPACK_SYSTEM_NAME mac)
else()
  string(TOLOWER "${CMAKE_SYSTEM_NAME}" CPACK_SYSTEM_NAME)

  list(APPEND CPACK_GENERATOR DEB)

  set(CPACK_DEBIAN_FILE_NAME DEB-DEFAULT)
  set(CPACK_DEBIAN_PACKAGE_CONTROL_STRICT_PERMISSION ON)
  set(CPACK_DEBIAN_PACKAGE_HOMEPAGE https://drake.mit.edu/)
  set(CPACK_DEBIAN_PACKAGE_RELEASE 1)
  set(CPACK_DEBIAN_PACKAGE_SECTION contrib/devel)

  file(STRINGS
    "setup/${UNIX_DISTRIBUTION_ID}/binary_distribution/packages-${UNIX_DISTRIBUTION_CODENAME}.txt"
    CPACK_DEBIAN_PACKAGE_DEPENDS
  )
  string(REGEX REPLACE "([a-z0-9\\-]+)=([a-z0-9\\-\\.~]+)" "\\1 (= \\2)"
    CPACK_DEBIAN_PACKAGE_DEPENDS "${CPACK_DEBIAN_PACKAGE_DEPENDS}"
  )
  string(REPLACE ";" ", " CPACK_DEBIAN_PACKAGE_DEPENDS
    "${CPACK_DEBIAN_PACKAGE_DEPENDS}"
  )
endif()

set(CPACK_PACKAGE_FILE_NAME
  "${PROJECT_NAME}-${BUILD_TIMESTAMP}-${CPACK_SYSTEM_NAME}"
)

include(CPack)
