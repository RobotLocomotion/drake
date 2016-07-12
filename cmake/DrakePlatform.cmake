#------------------------------------------------------------------------------
function(drake_get_matlab_jvm_version)
  # Find matlab
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
        if(MATLAB_JVM_VERSION VERSION_LESS _jsdk_version)
          set(CMAKE_JAVA_COMPILE_FLAGS
            "-source ${MATLAB_JVM_VERSION} -target ${MATLAB_JVM_VERSION}"
            CACHE INTERNAL "")
        endif()
      endif()
    else()
      message(WARNING "Could not determine MATLAB JVM version")
    endif()
  endif()
endfunction()

###############################################################################

# Check minimum compiler requirements
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS 4.9)
  message(FATAL_ERROR "GCC version must be at least 4.9")
elseif(CMAKE_CXX_COMPILER_ID STREQUAL "AppleClang" AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS 7)
  message(FATAL_ERROR "Apple Clang version must be at least 7")
elseif(CMAKE_CXX_COMPILER_ID STREQUAL "Clang" AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS 3.7)
  message(FATAL_ERROR "Clang version must be at least 3.7")
elseif(CMAKE_CXX_COMPILER_ID STREQUAL "MSVC" AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS 19)
  message(FATAL_ERROR "MSVC version must be at least 19 (VS 2015)")
endif()

# Set default install prefix
if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
  set(CMAKE_INSTALL_PREFIX "${CMAKE_BINARY_DIR}/install" CACHE STRING
    "Prefix for installation of sub-packages (note: required during build!)" FORCE)
endif()
message(STATUS CMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX})

# Require Java at the super-build level since libbot cannot configure without
# finding Java
find_package(Java 1.6 REQUIRED)
include(UseJava)

# If matlab is in use, try to determine its JVM version, as we need to build
# all externals to the same version
if(NOT DISABLE_MATLAB)
  drake_get_matlab_jvm_version()
endif()

# Choose your python (major) version
option(WITH_PYTHON_3 "Force Drake to use python 3 instead of python 2" OFF)
