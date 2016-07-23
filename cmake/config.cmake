# NOTE: The functions in this file set the basic configuration for both the
#       drake superbuild and drake proper. Keep logic that should only be in
#       one or the other in separate functions.

#------------------------------------------------------------------------------
function(drake_check_compiler NAME VERSION)
  if(DEFINED ARGV2)
    set(_version_string "${ARGV2}")
  else()
    set(_version_string "${VERSION}")
  endif()
  if(CMAKE_CXX_COMPILER_VERSION VERSION_LESS ${VERSION})
    message(FATAL_ERROR "${NAME} version must be at least ${_version_string}")
  endif()
endfunction()

#------------------------------------------------------------------------------
function(drake_get_matlab_jvm_version)
  # Find matlab; this does not use find_package(Matlab) because we only want
  # to enable matlab support if the matlab executable is in the user's PATH
  find_program(MATLAB_EXECUTABLE matlab)

  if(MATLAB_EXECUTABLE AND NOT MATLAB_JVM_VERSION)
    # Set arguments for running matlab
    set(_args -nodesktop -nodisplay -nosplash)
    if(WIN32)
      set(_args ${_args} -wait)
    endif()

    # Ask matlab for its JVM version
    execute_process(
      COMMAND ${MATLAB_EXECUTABLE} ${_args} -r "version -java,quit"
      RESULT_VARIABLE _result
      ERROR_VARIABLE _output
      OUTPUT_VARIABLE _output
      TIMEOUT 180)

    # Test for a valid result
    if(NOT _result AND _output MATCHES "Java ([0-9]+\\.[0-9]+)\\.[0-9_.]+")
      set(MATLAB_JVM_VERSION ${CMAKE_MATCH_1} CACHE INTERNAL "")
      set(_jdk_version "${Java_VERSION_MAJOR}.${Java_VERSION_MINOR}")

      # Compare against JDK version
      if(NOT MATLAB_JVM_VERSION VERSION_EQUAL _jdk_version)
        message(WARNING
        "MATLAB JVM version (${MATLAB_JVM_VERSION}) does not match "
        "installed Java Development Kit version (${_jdk_version})")

        # If JDK is newer, set build options to build Java code compatible
        # with the matlab JVM
        if(MATLAB_JVM_VERSION VERSION_LESS _jdk_version)
          set(CMAKE_JAVA_COMPILE_FLAGS
            -source ${MATLAB_JVM_VERSION} -target ${MATLAB_JVM_VERSION}
            CACHE INTERNAL "")
        endif()
      endif()
    else()
      message(WARNING "Could not determine MATLAB JVM version")
    endif()
  endif()
endfunction()

#------------------------------------------------------------------------------
macro(drake_setup_compiler)
  # Check minimum compiler requirements
  if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    drake_check_compiler("GCC" 4.9)
  elseif(CMAKE_CXX_COMPILER_ID STREQUAL "AppleClang")
    drake_check_compiler("Apple Clang" 7)
  elseif(CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    drake_check_compiler("Clang" 3.7)
  elseif(CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
    drake_check_compiler("MSVC" 19 "19 (VS 2015)")
  endif()

  # Set compiler language standard level
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endmacro()

#------------------------------------------------------------------------------
macro(drake_setup_java)
  # Require Java at the super-build level since libbot cannot configure without
  # finding Java
  find_package(Java 1.6 REQUIRED)
  include(UseJava)

  # If matlab is in use, try to determine its JVM version, as we need to build
  # all externals to the same version (on success, this will set the Java
  # compile flags)
  if(NOT DISABLE_MATLAB)
    drake_get_matlab_jvm_version()
  endif()
endmacro()

#------------------------------------------------------------------------------
macro(drake_setup_platform)
  drake_setup_compiler()
  drake_setup_java()

  # Choose your python (major) version
  option(WITH_PYTHON_3 "Force Drake to use python 3 instead of python 2" OFF)

  # Set default build
  if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING
      "The type of build. Options are: Debug Release RelWithDebInfo MinSizeRel."
      FORCE)
  endif()
endmacro()

#------------------------------------------------------------------------------
macro(drake_setup_superbuild)
  # Set default install prefix
  if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
    set(CMAKE_INSTALL_PREFIX "${CMAKE_BINARY_DIR}/install" CACHE STRING
      "Prefix for installation of sub-packages (note: required during build!)"
      FORCE)
  endif()
  message(STATUS CMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX})
endmacro()
