add_subdirectory(solvers)

drake_add_matlab_test(NAME addpath_spotless_test COMMAND "addpath_spotless;")
