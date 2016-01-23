
# todo: still build something useful if gurobi does not exist

pods_find_pkg_config(gurobi)

if (gurobi_FOUND)

  add_library_with_exports(LIB_NAME drakeQP SOURCE_FILES QP.cpp)
  set_target_properties(drakeQP PROPERTIES COMPILE_FLAGS -fPIC)
  pods_use_pkg_config_packages(drakeQP gurobi)

  if (MATLAB_FOUND)
    add_mex(fastQPmex fastQPmex.cpp)
    target_link_libraries(fastQPmex drakeQP)
    add_mex(gurobiQPmex gurobiQPmex.cpp)
    target_link_libraries(gurobiQPmex drakeQP)
  endif()

  pods_install_libraries(drakeQP)
  drake_install_headers(fastQP.h gurobiQP.h)
  pods_install_pkg_config_file(drake-qp
    LIBS -ldrakeQP
    REQUIRES gurobi
    VERSION 0.0.1)

endif()

pods_find_pkg_config(snopt_c)

if(MATLAB_FOUND)
  add_mex(lqrmex lqrmex.cpp)
  target_link_libraries(lqrmex drakeUtil)
endif()

set(optimization_files)
if (snopt_c_FOUND)
  list(APPEND optimization_files SNOPTWrapper.cpp)
else()
  list(APPEND optimization_files NoSNOPT.cpp)
endif(snopt_c_FOUND)
add_library_with_exports(LIB_NAME drakeOptimization SOURCE_FILES ${optimization_files})
set_target_properties(drakeOptimization PROPERTIES COMPILE_FLAGS -fPIC)
if (snopt_c_FOUND)
  pods_use_pkg_config_packages(drakeOptimization snopt_c)
endif(snopt_c_FOUND)

if(snopt_c_FOUND AND MATLAB_FOUND)
  add_mex(NonlinearProgramSnoptmex NonlinearProgramSnoptmex.cpp)
  pods_use_pkg_config_packages(NonlinearProgramSnoptmex snopt_c)
  set_target_properties(NonlinearProgramSnoptmex PROPERTIES COMPILE_FLAGS -fPIC)
endif()

add_subdirectory(qpSpline)
add_subdirectory(test)
