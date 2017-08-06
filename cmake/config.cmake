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
# Find MATLAB.
#------------------------------------------------------------------------------
macro(drake_setup_matlab)
  option(DISABLE_MATLAB "Don't use MATLAB even if it is present." OFF)

  if(DISABLE_MATLAB)
    message(STATUS "MATLAB is disabled.")
    unset(MATLAB_EXECUTABLE CACHE)
    set(Matlab_FOUND OFF)
  else()
    # Look for the MATLAB executable. This does not use find_package(Matlab)
    # because that is "really good at finding MATLAB", and we only want to
    # enable matlab support if the matlab executable is in the user's PATH.
    find_program(MATLAB_EXECUTABLE matlab)
    if(MATLAB_EXECUTABLE)
      # Determine the MATLAB root.
      get_filename_component(_matlab_realpath "${MATLAB_EXECUTABLE}" REALPATH)
      get_filename_component(_matlab_bindir "${_matlab_realpath}" DIRECTORY)
      get_filename_component(Matlab_ROOT_DIR
        "${_matlab_bindir}" DIRECTORY CACHE)
      unset(_matlab_realpath)
      unset(_matlab_bindir)

      set(MATLAB_ADDITIONAL_VERSIONS "R2017a=9.2" "R2016b=9.1")

      find_package(Matlab MODULE
        COMPONENTS
          MAIN_PROGRAM
          MEX_COMPILER
          MX_LIBRARY)
    else()
      set(Matlab_FOUND OFF)
      message(STATUS "MATLAB was not found.")
    endif()
  endif()
endmacro()

#------------------------------------------------------------------------------
# Determine the version of MATLAB's JVM and set Java build flags to match.
#------------------------------------------------------------------------------
function(drake_setup_java_for_matlab)
  if(NOT MATLAB_JVM_VERSION)
    message(STATUS "Detecting MATLAB JVM version")

    # Set arguments for running MATLAB
    set(_args -nodesktop -nodisplay -nosplash)
    set(_input_file /dev/null)
    set(_logfile "${CMAKE_CURRENT_BINARY_DIR}/drake_setup_java_for_matlab.log")

    # Ask MATLAB for its JVM version
    execute_process(
      COMMAND "${Matlab_MAIN_PROGRAM}" ${_args} -logfile "${_logfile}" -r "version -java,quit"
      WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
      TIMEOUT 450
      RESULT_VARIABLE _result
      OUTPUT_QUIET
      INPUT_FILE ${_input_file})

    if(_result EQUAL 0)
      if(EXISTS ${_logfile})
        file(READ ${_logfile} _output)

        # Test for a valid result
        if(_output MATCHES "Java ([0-9]+\\.[0-9]+)\\.([0-9_.]+)")
          set(MATLAB_JVM_VERSION ${CMAKE_MATCH_1} CACHE INTERNAL "")
        else()
          message(WARNING
            "Could not determine MATLAB JVM version because regular expression was not matched")
        endif()
      else()
        message(WARNING
          "Could not determine MATLAB JVM version because MATLAB log file was not created")
      endif()
    else()
      message(WARNING
        "Could not determine MATLAB JVM version because MATLAB exited with nonzero result ${_result}")
    endif()

    if(MATLAB_JVM_VERSION)
      set(_jdk_version "${Java_VERSION_MAJOR}.${Java_VERSION_MINOR}")

      # Compare against JDK version
      if(MATLAB_JVM_VERSION VERSION_EQUAL _jdk_version)
        message(STATUS
          "The MATLAB JVM version is ${MATLAB_JVM_VERSION}")
      else()
        message(WARNING
          "MATLAB JVM version (${MATLAB_JVM_VERSION}) does not match "
          "installed Java Development Kit version (${_jdk_version})")

        # If JDK is newer, set build options to build Java code compatible
        # with the MATLAB JVM
        if(MATLAB_JVM_VERSION VERSION_LESS _jdk_version)
          set(CMAKE_JAVA_COMPILE_FLAGS
          -source ${MATLAB_JVM_VERSION} -target ${MATLAB_JVM_VERSION}
          CACHE INTERNAL "")
        endif()
      endif()
    endif()
  endif()
endfunction()

#------------------------------------------------------------------------------
# Verify minimum required compiler version and set compile options.
#------------------------------------------------------------------------------
macro(drake_setup_compiler)
  # Check minimum compiler requirements
  if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    drake_check_compiler("GCC" 4.9)
  elseif(CMAKE_CXX_COMPILER_ID STREQUAL "AppleClang")
    drake_check_compiler("Apple Clang" 7)
  elseif(CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    drake_check_compiler("Clang" 3.9)
  endif()

  # Set compiler language standard level
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endmacro()

#------------------------------------------------------------------------------
# Find and set up Java, and import utilities for using it.
#------------------------------------------------------------------------------
macro(drake_setup_java)
  # Require Java at the super-build level since libbot cannot configure without
  # finding Java
  find_package(Java 1.6 REQUIRED)
  include(UseJava)

  # If matlab is in use, try to determine its JVM version, as we need to build
  # all externals to the same version (on success, this will set the Java
  # compile flags)
  if(Matlab_FOUND AND NOT DISABLE_MATLAB)
    drake_setup_java_for_matlab()
  else()
    unset(CMAKE_JAVA_COMPILE_FLAGS CACHE)
  endif()
endmacro()

#------------------------------------------------------------------------------
# Find and set up Fortran.
#------------------------------------------------------------------------------
macro(drake_setup_fortran)
  option(DISABLE_FORTRAN "Do not use Fortran even if it is supported" OFF)
  mark_as_advanced(DISABLE_FORTRAN)

  if(NOT DISABLE_FORTRAN)
    if(CMAKE_GENERATOR STREQUAL "Unix Makefiles")
      enable_language(Fortran)

      if(CMAKE_Fortran_COMPILER_ID STREQUAL "GNU" AND CMAKE_Fortran_COMPILER_VERSION VERSION_LESS "4.9")
        message(FATAL_ERROR "GNU Fortran compiler version must be at least 4.9 \
                             (detected version ${CMAKE_Fortran_COMPILER_VERSION})")
      endif()
    else()
      # Ninja and Xcode may not support Fortran, so manually find the Fortran
      # compiler and set any flags passed in by environment variable
      find_program(CMAKE_Fortran_COMPILER
        NAMES "$ENV{FC}" gfortran-6 gfortran-5 gfortran-4.9 gfortran-4 gfortran
        DOC "Fortran compiler")
      if(CMAKE_Fortran_COMPILER)
        message(STATUS "Found Fortran compiler: ${CMAKE_Fortran_COMPILER}")
      else()
        message(FATAL_ERROR "Could NOT find Fortran compiler")
      endif()
      set(CMAKE_Fortran_FLAGS "$ENV{FFLAGS}" CACHE STRING
        "Flags for Fortran compiler")
    endif()
  endif()
endmacro()

#------------------------------------------------------------------------------
# Set up Python.
#------------------------------------------------------------------------------
macro(drake_setup_python)
  option(DISABLE_PYTHON "Do not use Python even if it is supported" OFF)
  mark_as_advanced(DISABLE_PYTHON)

  # Choose your python (major) version
  option(WITH_PYTHON_3 "Force Drake to use Python 3 instead of Python 2" OFF)

  if(WITH_PYTHON_3)
    find_package(Python 3 MODULE REQUIRED)
  else()
    find_package(Python 2.7 MODULE REQUIRED)
  endif()
endmacro()

#------------------------------------------------------------------------------
# Add local CMake modules to CMake search path.
#------------------------------------------------------------------------------
function(drake_setup_cmake BASE_PATH)
  set(CMAKE_MODULE_PATH "${BASE_PATH}")
  file(GLOB _versions RELATIVE ${BASE_PATH} "${BASE_PATH}/*/")
  foreach(_version ${_versions})
    if(IS_DIRECTORY "${BASE_PATH}/${_version}")
      if(CMAKE_VERSION VERSION_LESS ${_version})
        list(INSERT CMAKE_MODULE_PATH 0 "${BASE_PATH}/${_version}")
      endif()
    endif()
  endforeach()
  list(INSERT CMAKE_MODULE_PATH 0 ${BASE_PATH})
  set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH}" PARENT_SCOPE)
endfunction()

#------------------------------------------------------------------------------
# Set up basic platform properties for building Drake.
#------------------------------------------------------------------------------
macro(drake_setup_platform)
  include(GNUInstallDirs)

  # Disable finding out-of-tree packages in the registry.
  # This doesn't exactly make find_package hermetic, but it's a useful step
  # in that direction.
  set(CMAKE_FIND_PACKAGE_NO_PACKAGE_REGISTRY ON)
  set(CMAKE_FIND_PACKAGE_NO_SYSTEM_PACKAGE_REGISTRY ON)

  # Disable adding packages to the registry.
  set(CMAKE_EXPORT_NO_PACKAGE_REGISTRY ON)

  # Ensure that find_package() searches in the install directory first.
  list(APPEND CMAKE_PREFIX_PATH "${CMAKE_INSTALL_PREFIX}")

  # Set default lib directory name suffix
  set(LIB_SUFFIX "" CACHE STRING
    "Suffix of library install directory, e.g. '64'")
  mark_as_advanced(LIB_SUFFIX)

  # Set RPATH for installed binaries
  set(CMAKE_MACOSX_RPATH ON)
  set(CMAKE_INSTALL_RPATH
    ${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}
    ${CMAKE_INSTALL_PREFIX}/lib${LIB_SUFFIX}
    ${CMAKE_INSTALL_PREFIX}/lib)
  set(CMAKE_INSTALL_RPATH_USE_LINK_PATH ON)

  list(REMOVE_DUPLICATES CMAKE_INSTALL_RPATH)

  drake_setup_compiler()
  drake_setup_matlab()
  drake_setup_java()
  drake_setup_python()

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
  enable_testing()
  set_property(DIRECTORY PROPERTY TEST_INCLUDE_FILE
    ${CMAKE_BINARY_DIR}/CTestExternals.cmake)
  file(REMOVE ${CMAKE_BINARY_DIR}/CTestExternals.cmake)

  # Set default install prefix
  if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
    set(CMAKE_INSTALL_PREFIX "${CMAKE_BINARY_DIR}/install" CACHE STRING
      "Prefix for installation of sub-packages (note: required during build!)"
      FORCE)
  endif()
  message(STATUS CMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX})

  # Drake itself does not contain Fortran code.
  drake_setup_fortran()

  # Add homebrew path for VTK
  if(APPLE)
    list(APPEND CMAKE_PREFIX_PATH /usr/local/opt/vtk5)
  endif()

endmacro()

###############################################################################

# Set up local module paths; this needs to be done immediately as other helper
# modules may need to include things from our local module set
drake_setup_cmake(${CMAKE_CURRENT_LIST_DIR}/modules)
