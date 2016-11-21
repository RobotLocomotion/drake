include(FindPackageHandleStandardArgs)

if(Python_FIND_VERSION_EXACT)
  set(PYTHON_EXACT EXACT)
else()
  set(PYTHON_EXACT)
endif()

if(Python_FIND_VERSION_QUIETLY)
  set(PYTHON_QUIET QUIET)
else()
  set(PYTHON_QUIET)
endif()

find_package(PythonInterp ${Python_FIND_VERSION} ${PYTHON_EXACT} MODULE ${PYTHON_QUIET})

if(PYTHON_EXECUTABLE)
  execute_process(
    COMMAND "${PYTHON_EXECUTABLE}" -c "import sys; print(sys.exec_prefix)"
    RESULT_VARIABLE PYTHON_PREFIX_RESULT
    OUTPUT_VARIABLE PYTHON_PREFIX
    OUTPUT_STRIP_TRAILING_WHITESPACE)
  if(PYTHON_PREFIX_RESULT EQUAL 0)
    list(APPEND CMAKE_PREFIX_PATH ${PYTHON_PREFIX})
  endif()
  find_package(PythonLibs ${Python_FIND_VERSION} ${PYTHON_EXACT} MODULE ${PYTHON_QUIET})
endif()

find_package_handle_standard_args(Python
  REQUIRED_VARS PYTHON_EXECUTABLE PYTHON_INCLUDE_DIR PYTHON_LIBRARY
  VERSION_VAR PYTHON_VERSION_STRING)
