if(NOT WIN32)
  set(CMAKE_MODULE_PATH
    ${CMAKE_INSTALL_PREFIX}/share/cmake
    ${CMAKE_MODULE_PATH})
  # N.B. Ubuntu Xenial provides 3.0.8, so saying 3.0.9 here (which is
  # the actual version of our swig_matlab external) only succeeds if
  # our superbuild has provided swig -- we never want to use the
  # system swig.
  find_package(SWIG 3.0.9)
  if(SWIG_FOUND)
    add_subdirectory(swig)
  endif()
  if(PYTHON_EXECUTABLE)
    add_subdirectory(python)
  endif()
endif()

add_subdirectory(matlab/test)
