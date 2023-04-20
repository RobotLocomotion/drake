# -*- mode: cmake -*-
# vi: set ft=cmake :

cmake_minimum_required(VERSION 3.16)

project(drake
  DESCRIPTION "Model-based design and verification for robotics"
  LANGUAGES C CXX
)

# The primary build system for Drake is Bazel (https://bazel.build/). For CMake,
# our objective is to accept configuration options using their standard spelling
# (e.g., `-DCMAKE_BUILD_TYPE=Release`) and install Drake using those settings.
#
# We'll do that by converting the settings to generated Bazel inputs:
# - a `WORKSPACE.bazel` file that specifies dependencies; and
# - a `.bazelrc` file that specifies configuration choices.
# and then running the `@drake//:install` program from that temporary workspace.

list(INSERT CMAKE_MODULE_PATH 0 "${PROJECT_SOURCE_DIR}/cmake/modules")

include(CTest)

configure_file(CTestCustom.cmake.in CTestCustom.cmake @ONLY)

if(ANDROID OR CYGWIN OR IOS OR NOT UNIX)
  message(FATAL_ERROR
    "Android, Cygwin, iOS, and non-Unix platforms are NOT supported"
  )
endif()

set(BAZELRC_IMPORTS "tools/bazel.rc")
set(UNIX_DISTRIBUTION_ID)
set(UNIX_DISTRIBUTION_CODENAME)

if(APPLE)
  if(CMAKE_SYSTEM_VERSION VERSION_LESS 21)
    message(WARNING
      "Darwin ${CMAKE_SYSTEM_VERSION} is NOT supported. Please use "
      "Darwin 21.x (macOS Monterey) or newer."
    )
  endif()

  list(APPEND BAZELRC_IMPORTS "tools/macos.bazelrc")

  execute_process(
    COMMAND "/usr/bin/arch"
    OUTPUT_STRIP_TRAILING_WHITESPACE
    OUTPUT_VARIABLE MACOS_ARCH)
  if(MACOS_ARCH STREQUAL "")
    message(FATAL_ERROR "Could NOT query macOS arch")
  endif()
  list(APPEND BAZELRC_IMPORTS "tools/macos-arch-${MACOS_ARCH}.bazelrc")
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
    message(WARNING
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

  # The supported releases should match those listed in both
  # doc/_pages/from_source.md and tools/workspace/os.bzl.
  if(NOT UNIX_DISTRIBUTION_CODENAME MATCHES "^(focal|jammy)$")
    message(FATAL_ERROR
      "Release ${UNIX_DISTRIBUTION_CODENAME} is NOT supported. Please use "
      "Ubuntu 20.04 (Focal) or Ubuntu 22.04 (Jammy)."
    )
  endif()

  list(APPEND BAZELRC_IMPORTS "tools/ubuntu.bazelrc")
  list(APPEND BAZELRC_IMPORTS
    "tools/ubuntu-${UNIX_DISTRIBUTION_CODENAME}.bazelrc")
endif()

# The version passed to find_package(Bazel) should match the
# minimum_bazel_version value in the call to versions.check() in WORKSPACE.
set(MINIMUM_BAZEL_VERSION 4.0)
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
# doc/_pages/from_source.md and tools/workspace/cc/repository.bzl.
set(MINIMUM_APPLE_CLANG_VERSION 14)
set(MINIMUM_CLANG_VERSION 12)
set(MINIMUM_GNU_VERSION 9.3)

if(CMAKE_C_COMPILER_ID STREQUAL AppleClang)
  if(CMAKE_C_COMPILER_VERSION VERSION_LESS ${MINIMUM_APPLE_CLANG_VERSION})
    message(WARNING
      "Compilation with clang ${CMAKE_C_COMPILER_VERSION} is NOT supported"
    )
  endif()
elseif(CMAKE_C_COMPILER_ID STREQUAL Clang)
  if(CMAKE_C_COMPILER_VERSION VERSION_LESS ${MINIMUM_CLANG_VERSION})
    message(WARNING
      "Compilation with clang ${CMAKE_C_COMPILER_VERSION} is NOT supported"
    )
  endif()
elseif(CMAKE_C_COMPILER_ID STREQUAL GNU)
  if(CMAKE_C_COMPILER_VERSION VERSION_LESS ${MINIMUM_GNU_VERSION})
    message(WARNING
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
    message(WARNING
      "Compilation with clang++ ${CMAKE_CXX_COMPILER_VERSION} is NOT supported"
    )
  endif()
elseif(CMAKE_CXX_COMPILER_ID STREQUAL Clang)
  if(CMAKE_CXX_COMPILER_VERSION VERSION_LESS ${MINIMUM_CLANG_VERSION})
    message(WARNING
      "Compilation with clang++ ${CMAKE_CXX_COMPILER_VERSION} is NOT supported"
    )
  endif()
elseif(CMAKE_CXX_COMPILER_ID STREQUAL GNU)
  if(CMAKE_CXX_COMPILER_VERSION VERSION_LESS ${MINIMUM_GNU_VERSION})
    message(WARNING
      "Compilation with g++ ${CMAKE_CXX_COMPILER_VERSION} is NOT supported"
    )
  endif()
else()
  message(WARNING
    "Compilation with ${CMAKE_CXX_COMPILER_ID} is NOT supported. Compilation "
    "of project drake_cxx_python may fail."
  )
endif()

# Determine the CMAKE_BUILD_TYPE. We'll store it as BUILD_TYPE_LOWER so that
# we can treat it as case-insensitive in our string comparisons.
get_property(IS_MULTI_CONFIG GLOBAL PROPERTY GENERATOR_IS_MULTI_CONFIG)
if(IS_MULTI_CONFIG)
  message(FATAL_ERROR "Drake does not support multi-config generators")
endif()
set(SUPPORTED_BUILD_TYPES Release RelWithDebInfo Debug MinSizeRel)
string(REPLACE ";" " " SUPPORTED_BUILD_TYPES_STRING
  "${SUPPORTED_BUILD_TYPES}"
)
set_property(CACHE CMAKE_BUILD_TYPE PROPERTY
  STRINGS "${SUPPORTED_BUILD_TYPES}"
)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING
    "Choose the type of build, options are: ${SUPPORTED_BUILD_TYPES_STRING}"
    FORCE
  )
endif()
string(TOLOWER "${CMAKE_BUILD_TYPE}" BUILD_TYPE_LOWER)
string(TOLOWER "${SUPPORTED_BUILD_TYPES}" SUPPORTED_BUILD_TYPES_LOWER)
if(NOT BUILD_TYPE_LOWER IN_LIST SUPPORTED_BUILD_TYPES_LOWER)
  message(WARNING
    "Configuration CMAKE_BUILD_TYPE='${CMAKE_BUILD_TYPE}' is NOT supported. "
    "Defaulting to Release, options are: ${SUPPORTED_BUILD_TYPES_STRING}"
  )
  set(BUILD_TYPE_LOWER release)
  set(CMAKE_BUILD_TYPE Release CACHE STRING
    "Choose the type of build, options are: ${SUPPORTED_BUILD_TYPES_STRING}"
    FORCE
  )
endif()

# TODO(jwnimmer-tri) We don't currently pass along the user's selected C++
# standard nor CMAKE_CXX_FLAGS to Bazel, but we should.
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)

# The supported Python major/minor versions should match those listed in both
# doc/_pages/from_source.md and tools/workspace/python/repository.bzl.
if(APPLE)
  set(SUPPORTED_PYTHON_VERSION 3.11)
else()
  if(UNIX_DISTRIBUTION_CODENAME STREQUAL focal)
    set(SUPPORTED_PYTHON_VERSION 3.8)
  else()  # UNIX_DISTRIBUTION_CODENAME := jammy
    set(SUPPORTED_PYTHON_VERSION 3.10)
  endif()
endif()

# Next we'll very carefully choose which Python interpreter to use.
#
# - If the user provided the legacy spelling -DPYTHON_EXECUTABLE, shift that
#   into -DPython_EXECUTABLE instead and continue (with a warning).
#
# - If the user provided -DPython_EXECUTABLE, take it at face value (and
#   therefore error out if they gave us a broken definition).
#
# - Otherwise, try to find SUPPORTED_PYTHON_VERSION and use it if found.
#
# - Otherwise, try to find any Python 3 interpreter at all.
#
# In all cases, we'll warn in case the found Python is not supported.
if(PYTHON_EXECUTABLE AND NOT Python_EXECUTABLE)
  set(Python_EXECUTABLE "${PYTHON_EXECUTABLE}" CACHE FILEPATH
    "Path to the python3 executable" FORCE
  )
  message(WARNING
    "To select a Python interpreter, you should define Python_EXECUTABLE "
    "not PYTHON_EXECUTABLE. The uppercase spelling is used for backwards "
    "compatibility only.")
  unset(PYTHON_EXECUTABLE CACHE)
endif()
if(Python_EXECUTABLE)
  find_package(Python 3 EXACT MODULE REQUIRED
    COMPONENTS Development Interpreter
  )
else()
  find_package(Python ${SUPPORTED_PYTHON_VERSION} EXACT MODULE
    COMPONENTS Development Interpreter
  )
  if(NOT Python_FOUND)
    find_package(Python 3 EXACT MODULE REQUIRED
      COMPONENTS Development Interpreter
    )
  endif()
endif()
if(NOT Python_INTERPRETER_ID STREQUAL Python)
  message(WARNING
    "Python interpreter ${Python_INTERPRETER_ID} is NOT supported. Python "
    "code in project drake_cxx_python may fail at runtime."
  )
endif()
set(PYTHON_VERSION_MAJOR_MINOR
  "${Python_VERSION_MAJOR}.${Python_VERSION_MINOR}"
)
if(NOT PYTHON_VERSION_MAJOR_MINOR VERSION_EQUAL SUPPORTED_PYTHON_VERSION)
  message(WARNING
    "The found Python version ${PYTHON_VERSION_MAJOR_MINOR} differs from "
    "Drake's preferred version ${SUPPORTED_PYTHON_VERSION} on this platform. "
    "You may experience compatibility problems that are outside the scope of "
    "Drake's continuous integration test suites."
  )
endif()

if(CMAKE_COLOR_MAKEFILE)
  set(BAZEL_COLOR yes)
else()
  set(BAZEL_COLOR no)
endif()

if(CMAKE_VERBOSE_MAKEFILE)
  set(BAZEL_SUBCOMMANDS yes)
  set(BAZEL_ANNOUNCE_RC yes)
else()
  set(BAZEL_SUBCOMMANDS no)
  set(BAZEL_ANNOUNCE_RC no)
endif()

set(BAZEL_REPO_ENV)

if(NOT APPLE)
  string(APPEND BAZEL_REPO_ENV
    " --repo_env=CC=${CMAKE_C_COMPILER}"
    " --repo_env=CXX=${CMAKE_CXX_COMPILER}"
  )
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

function(generate_external_repository_file OUTPUT)
  set(out_path
    ${CMAKE_CURRENT_BINARY_DIR}/external/workspace/${OUTPUT})
  if(ARGN)
    file(GENERATE OUTPUT ${out_path}
      INPUT ${CMAKE_CURRENT_SOURCE_DIR}/cmake/external/workspace/${ARGN})
  else()
    file(GENERATE OUTPUT ${out_path} CONTENT "")
  endif()
endfunction()

# Symlinks the C++ include path for TARGET as workspace/NAME/include, e.g.
#  workspace/eigen/include -> .../build/install/include/eigen3
function(symlink_external_repository_includes NAME TARGET)
  get_target_property(include_dir ${TARGET} INTERFACE_INCLUDE_DIRECTORIES)
  set(workspace ${CMAKE_CURRENT_BINARY_DIR}/external/workspace)
  file(MAKE_DIRECTORY ${workspace}/${NAME})
  file(CREATE_LINK ${include_dir} ${workspace}/${NAME}/include SYMBOLIC)
endfunction()

# Symlinks the C++ libraries for TARGET as workspace/NAME/lib/*, e.g.
#  workspace/fmt/lib/libfmt.so.6.1.2 -> .../build/install/lib/fmt/libfmt.so.6.1.2
#  workspace/fmt/lib/libfmt.so.6 -> .../build/install/lib/fmt/libfmt.so.6.1.2
function(symlink_external_repository_libs NAME TARGET)
  set(workspace "${CMAKE_CURRENT_BINARY_DIR}/external/workspace")
  file(MAKE_DIRECTORY "${workspace}/${NAME}/lib")
  # Link the full library name (i.e., libfmt.so.6.1.2 in the case of shared).
  get_target_property(location ${TARGET} LOCATION_${CMAKE_BUILD_TYPE})
  if(NOT location)
    message(FATAL_ERROR "Target ${TARGET} has no library in LOCATION_${CMAKE_BUILD_TYPE}")
  endif()
  get_filename_component(basename "${location}" NAME)
  file(CREATE_LINK "${location}" "${workspace}/${NAME}/lib/${basename}" SYMBOLIC)
  # Link the SONAME spelling in case of shared libraries.
  # If the basename does not match this pattern, this part is all a no-op.
  string(REGEX REPLACE "(\\.so\\.[0-9]+)\\.[0-9]+\\.[0-9]+$" "\\1"
      other_basename "${basename}")
  string(REGEX REPLACE "(\\.[0-9]+)\\.[0-9]+\\.[0-9]+\\.dylib$" "\\1.dylib"
      other_basename "${other_basename}")
  file(CREATE_LINK "${location}" "${workspace}/${NAME}/lib/${other_basename}" SYMBOLIC)
endfunction()

set(BAZEL_WORKSPACE_EXTRA)
set(BAZEL_WORKSPACE_EXCLUDES)

macro(override_repository NAME)
  set(repo "${CMAKE_CURRENT_BINARY_DIR}/external/workspace/${NAME}")
  string(APPEND BAZEL_WORKSPACE_EXTRA
    "local_repository(name = '${NAME}', path = '${repo}')\n")
  list(APPEND BAZEL_WORKSPACE_EXCLUDES "${NAME}")
endmacro()

# For Python, always override Drake's default with the CMake selection instead.
generate_external_repository_file(cmake_python/WORKSPACE)
configure_file(
  cmake/external/workspace/cmake_python/BUILD.bazel.in
  external/workspace/cmake_python/BUILD.bazel @ONLY)
override_repository(cmake_python)

option(WITH_USER_EIGEN "Use user-provided Eigen3" OFF)

if(WITH_USER_EIGEN)
  find_package(Eigen3 CONFIG REQUIRED)

  symlink_external_repository_includes(eigen Eigen3::Eigen)
  generate_external_repository_file(eigen/WORKSPACE)
  generate_external_repository_file(
    eigen/BUILD.bazel
    eigen/BUILD.bazel.in)

  override_repository(eigen)
endif()

option(WITH_USER_FMT "Use user-provided fmt" OFF)

if(WITH_USER_FMT)
  find_package(fmt CONFIG REQUIRED)

  symlink_external_repository_includes(fmt fmt::fmt)
  symlink_external_repository_libs(fmt fmt::fmt)
  generate_external_repository_file(fmt/WORKSPACE)
  generate_external_repository_file(
    fmt/BUILD.bazel
    fmt/BUILD.bazel.in)

  override_repository(fmt)
endif()

option(WITH_USER_SPDLOG "Use user-provided spdlog" OFF)

if(WITH_USER_SPDLOG)
  if(NOT WITH_USER_FMT)
    message(FATAL_ERROR
      "User-provided spdlog (WITH_USER_SPDLOG) "
      "requires user-provided fmt (WITH_USER_FMT).")
  endif()

  find_package(spdlog CONFIG REQUIRED)

  symlink_external_repository_includes(spdlog spdlog::spdlog)
  symlink_external_repository_libs(spdlog spdlog::spdlog)
  generate_external_repository_file(spdlog/WORKSPACE)
  generate_external_repository_file(
    spdlog/BUILD.bazel
    spdlog/BUILD.bazel.in)

  override_repository(spdlog)
endif()

set(BAZEL_CONFIG)

option(WITH_GUROBI "Build with support for Gurobi" OFF)

if(WITH_GUROBI)
  find_package(Gurobi 9.5 EXACT MODULE REQUIRED)

  string(APPEND BAZEL_CONFIG " --config=gurobi")

  if(NOT APPLE)
    get_filename_component(GUROBI_HOME "${Gurobi_INCLUDE_DIRS}" DIRECTORY)
    string(APPEND BAZEL_REPO_ENV " --repo_env=GUROBI_HOME=${GUROBI_HOME}")
  endif()
endif()

option(WITH_MOSEK "Build with support for MOSEK" OFF)

if(WITH_MOSEK)
  string(APPEND BAZEL_CONFIG " --config=mosek")
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

  string(APPEND BAZEL_CONFIG " --config=snopt")

  if(WITH_ROBOTLOCOMOTION_SNOPT)
    string(APPEND BAZEL_REPO_ENV " --repo_env=SNOPT_PATH=git")
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
    string(APPEND BAZEL_REPO_ENV " --repo_env=SNOPT_PATH=${SNOPT_PATH}")
  endif()
endif()

if(BUILD_TYPE_LOWER STREQUAL debug)
  string(APPEND BAZEL_CONFIG " --config=Debug")
elseif(BUILD_TYPE_LOWER STREQUAL minsizerel)
  string(APPEND BAZEL_CONFIG " --config=MinSizeRel")
elseif(BUILD_TYPE_LOWER STREQUAL release)
  string(APPEND BAZEL_CONFIG " --config=Release")
elseif(BUILD_TYPE_LOWER STREQUAL relwithdebinfo)
  string(APPEND BAZEL_CONFIG " --config=RelWithDebInfo")
endif()

# N.B. If you are testing the CMake API and making changes to `installer.py`,
# you can change this target to something more lightweight, such as
# `//tools/install/dummy:install`.
set(BAZEL_INSTALL_TARGET //:install)

if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
  set(CMAKE_INSTALL_PREFIX "${PROJECT_BINARY_DIR}/install" CACHE STRING
    "Install path prefix, prepended onto install directories" FORCE
  )
endif()

set(BAZEL_INSTALL_ARGS "${CMAKE_INSTALL_PREFIX}")

if(CMAKE_COLOR_MAKEFILE)
  list(INSERT BAZEL_INSTALL_ARGS 0 "--color")
endif()

if(CMAKE_INSTALL_NAME_TOOL)
  list(INSERT BAZEL_INSTALL_ARGS 0
    "--install_name_tool"
    "${CMAKE_INSTALL_NAME_TOOL}"
  )
endif()

if(CMAKE_STRIP)
  list(INSERT BAZEL_INSTALL_ARGS 0 "--strip_tool" "${CMAKE_STRIP}")
endif()

# If CMAKE_BUILD_TYPE is Debug or RelWithDebInfo, do NOT strip symbols during
# install.
if(BUILD_TYPE_LOWER MATCHES "^(debug|relwithdebinfo)$")
  # SNOPT has restrictions for redistribution given that we are statically
  # linking it in.
  if(WITH_SNOPT OR WITH_ROBOTLOCOMOTION_SNOPT)
    message(WARNING
      "Install configurations Debug and RelWithDebInfo will STILL strip "
      "symbols because support for SNOPT is enabled"
    )
  else()
    list(INSERT BAZEL_INSTALL_ARGS 0 --no_strip)
  endif()
endif()

set(BAZELRC_IMPORT)
foreach(import IN LISTS BAZELRC_IMPORTS)
  string(APPEND BAZELRC_IMPORT "import ${PROJECT_SOURCE_DIR}/${import}\n")
endforeach()

# We need to run Bazel in a dedicated temporary directory. The particular
# name `drake_build_cwd` isn't important, it just needs to be unique.
configure_file(cmake/bazel.rc.in drake_build_cwd/.bazelrc @ONLY)
configure_file(cmake/WORKSPACE.in drake_build_cwd/WORKSPACE.bazel @ONLY)

include(ExternalProject)

ExternalProject_Add(drake_cxx_python
  SOURCE_DIR "${PROJECT_SOURCE_DIR}"
  BINARY_DIR "${CMAKE_CURRENT_BINARY_DIR}/drake_build_cwd"
  CONFIGURE_COMMAND
    "${Bazel_EXECUTABLE}"
    info --announce_rc
  BUILD_COMMAND
    "${Bazel_EXECUTABLE}"
    build
    ${BAZEL_INSTALL_TARGET}
  BUILD_ALWAYS ON
  INSTALL_COMMAND
    "${Bazel_EXECUTABLE}"
    run
    ${BAZEL_INSTALL_TARGET}
    --
    ${BAZEL_INSTALL_ARGS}
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
