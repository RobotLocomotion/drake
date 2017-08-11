if(ENABLE_PYTHON_BINDINGS)
  drake_add_matlab_test(NAME matlab/solvers/test/testMathematicalProgram COMMAND testMathematicalProgram)
endif()
