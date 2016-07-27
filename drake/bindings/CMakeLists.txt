if(NOT WIN32)
  # Look for the swig provided by our superbuild.  If it doesn't exist,
  # don't fall back to system swig because it may be the wrong version
  # and likely does not support matlab.
  find_program(SWIG_EXECUTABLE swig
    PATHS ${CMAKE_INSTALL_PREFIX}/bin
    NO_DEFAULT_PATH
    NO_CMAKE_ENVIRONMENT_PATH
    NO_CMAKE_PATH
    NO_SYSTEM_ENVIRONMENT_PATH
    NO_CMAKE_SYSTEM_PATH)
  if(SWIG_EXECUTABLE)
    add_subdirectory(swig)
  endif()
  if(PYTHON_EXECUTABLE)
    add_subdirectory(python)
  endif()
endif()

add_subdirectory(matlab/test)
