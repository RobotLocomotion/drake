# Look for the swig provided by our superbuild.  If it doesn't exist,
# don't fall back to system swig because it may be the wrong version
# and likely does not support matlab.
find_program(SWIG_EXECUTABLE swig
  PATHS ${CMAKE_INSTALL_PREFIX}/bin NO_DEFAULT_PATH)
if(SWIG_EXECUTABLE)
  add_subdirectory(swig)
  if(PYTHON_EXECUTABLE)
    add_subdirectory(python)
  endif()
  add_subdirectory(matlab/test)
else()
  message("
  --------------------------------------------------------------------------------
    *** IMPORTANT: swig_matlab not found; disabling all swig bindings. ***
  --------------------------------------------------------------------------------
  ")
endif()
