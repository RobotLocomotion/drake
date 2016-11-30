add_subdirectory(util)
add_subdirectory(solvers)
add_subdirectory(systems)

drake_add_matlab_test(NAME addpath_spotless_test COMMAND "addpath_spotless;")
