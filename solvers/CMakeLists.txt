
# todo: still build something useful if gurobi does not exist

pods_find_pkg_config(gurobi)

if (gurobi_FOUND AND eigen3_FOUND)

	add_library(drakeQP QP.cpp)
	set_target_properties(drakeQP PROPERTIES COMPILE_FLAGS -fPIC)
	pods_use_pkg_config_packages(drakeQP gurobi)

	add_mex(fastQPmex fastQPmex.cpp)
	target_link_libraries(fastQPmex drakeQP)

	add_mex(gurobiQPmex gurobiQPmex.cpp)
	target_link_libraries(gurobiQPmex drakeQP)

	pods_install_libraries(drakeQP)
	pods_install_headers(fastQP.h gurobiQP.h DESTINATION drake)
	pods_install_pkg_config_file(drake-qp
	    LIBS -ldrakeQP
	    REQUIRES gurobi 
	    VERSION 0.0.1)

endif()


