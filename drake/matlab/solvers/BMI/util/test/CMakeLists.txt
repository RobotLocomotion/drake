add_matlab_test(NAME matlab/solvers/BMI/util/test/testReplaceBilinearProduct REQUIRES spotless COMMAND testReplaceBilinearProduct)
add_matlab_test(NAME matlab/solvers/BMI/util/test/testRotateVectorMatrix REQUIRES mosek spotless COMMAND testRotateVectorMatrix)
add_matlab_test(NAME matlab/solvers/BMI/util/test/testRotateVectorToAlign COMMAND testRotateVectorToAlign)
