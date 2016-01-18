if (NOT WIN32)
  add_subdirectory(swig)
  if (PYTHON_EXECUTABLE)
    add_subdirectory(python)
  endif()
endif()

