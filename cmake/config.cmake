# NOTE: The functions in this file set the basic configuration for both the
#       drake superbuild and drake proper. Keep logic that should only be in
#       one or the other in separate functions.

#------------------------------------------------------------------------------
# Check compiler version.
#
# Arguments:
#   <NAME> - Name of compiler to display in error message.
#   <VERSION> - Required compiler version.
#   [<DISPLAY_VERSION>]
#       Version string to display in error message (defaults to `<VERSION>`).
#------------------------------------------------------------------------------
function(drake_check_compiler NAME VERSION)
  if(DEFINED ARGV2)
    set(_version_string "${ARGV2}")
  else()
    set(_version_string "${VERSION}")
  endif()
  if(CMAKE_CXX_COMPILER_VERSION VERSION_LESS ${VERSION})
    message(FATAL_ERROR "${NAME} version must be at least ${_version_string} \
                         (detected version ${CMAKE_CXX_COMPILER_VERSION})")
  endif()
endfunction()

#------------------------------------------------------------------------------
# Verify minimum required compiler version and set compile options.
#------------------------------------------------------------------------------
macro(drake_setup_compiler)
  # Check minimum compiler requirements
  if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    drake_check_compiler("GCC" 5.4)
  elseif(CMAKE_CXX_COMPILER_ID STREQUAL "AppleClang")
    drake_check_compiler("Apple Clang" 8)
  elseif(CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    drake_check_compiler("Clang" 3.9)
  endif()

  # Set compiler language standard level
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endmacro()

#------------------------------------------------------------------------------
# Set up basic platform properties for building Drake.
#------------------------------------------------------------------------------
macro(drake_setup_platform)
  # Disable finding out-of-tree packages in the registry.
  # This doesn't exactly make find_package hermetic, but it's a useful step
  # in that direction.
  set(CMAKE_FIND_PACKAGE_NO_PACKAGE_REGISTRY ON)
  set(CMAKE_FIND_PACKAGE_NO_SYSTEM_PACKAGE_REGISTRY ON)

  # Disable adding packages to the registry.
  set(CMAKE_EXPORT_NO_PACKAGE_REGISTRY ON)

  # Ensure that find_package() searches in the install directory first.
  list(APPEND CMAKE_PREFIX_PATH "${CMAKE_INSTALL_PREFIX}")

  # Set RPATH for installed binaries.
  set(CMAKE_MACOSX_RPATH ON)
  set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
  set(CMAKE_INSTALL_RPATH_USE_LINK_PATH ON)

  drake_setup_compiler()

  # Set default build type
  if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING
      "The type of build. Options are: Debug Release RelWithDebInfo MinSizeRel."
      FORCE)
  endif()

  # Build shared libraries by default
  option(BUILD_SHARED_LIBS "Build shared libraries" ON)
endmacro()

#------------------------------------------------------------------------------
# Set up properties for the Drake superbuild.
#------------------------------------------------------------------------------
macro(drake_setup_superbuild)
  # Set default install prefix.
  if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
    set(CMAKE_INSTALL_PREFIX "${CMAKE_BINARY_DIR}/install" CACHE STRING
      "Prefix for installation of sub-packages (note: required during build!)"
      FORCE)
  endif()
endmacro()

###############################################################################

# Set up local module paths; this needs to be done immediately as other helper
# modules may need to include things from our local module set
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/modules")

if(CMAKE_VERSION VERSION_LESS 3.10)
  list(INSERT CMAKE_MODULE_PATH 0
    "${CMAKE_CURRENT_LIST_DIR}/../third_party/com_kitware_gitlab_cmake_cmake/3.10/Modules")
endif()
