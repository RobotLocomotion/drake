
add_library(QP OBJECT QP.cpp)
add_mex(fastQPmex fastQPmex.cpp $<TARGET_OBJECTS:QP>)
add_mex(gurobiQPmex gurobiQPmex.cpp $<TARGET_OBJECTS:QP>)

pods_use_pkg_config_packages(fastQPmex gurobi)
pods_use_pkg_config_packages(gurobiQPmex gurobi)



