
add_library(fastQP OBJECT fastQP.cpp)

add_mex(fastQPmex fastQPmex.cpp $<TARGET_OBJECTS:fastQP>)
add_mex(gurobiQPmex gurobiQPmex.cpp $<TARGET_OBJECTS:fastQP>)

pods_use_pkg_config_packages(fastQPmex gurobi)
pods_use_pkg_config_packages(gurobiQPmex gurobi)



