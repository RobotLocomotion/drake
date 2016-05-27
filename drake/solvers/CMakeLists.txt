# todo: still build something useful if gurobi does not exist

pods_find_pkg_config(gurobi)

if(gurobi_FOUND)

  add_library_with_exports(LIB_NAME drakeQP SOURCE_FILES QP.cpp)
  pods_use_pkg_config_packages(drakeQP gurobi)

  if(MATLAB_FOUND)
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

if(MATLAB_FOUND)
  add_mex(lqrmex lqrmex.cpp)
  target_link_libraries(lqrmex drakeUtil)
endif()

set(optimization_files)
list(APPEND optimization_files MathematicalProgram.cpp MobyLCP.cpp system_identification.cpp)
pods_find_pkg_config(nlopt)
if(nlopt_FOUND)
  list(APPEND optimization_files NloptSolver.cpp)
else()
  list(APPEND optimization_files NoNlopt.cpp)
endif()
pods_find_pkg_config(snopt_c)
if(snopt_c_FOUND)
  list(APPEND optimization_files SnoptSolver.cpp)
else()
  list(APPEND optimization_files NoSnopt.cpp)
endif()
add_library_with_exports(LIB_NAME drakeOptimization SOURCE_FILES ${optimization_files})
target_link_libraries(drakeOptimization drakePolynomial)
drake_install_headers(MathematicalProgram.h Optimization.h system_identification.h)
pods_install_libraries(drakeOptimization)
pods_install_pkg_config_file(drake-optimization
  LIBS -ldrakeOptimization
  REQUIRES
  VERSION 0.0.1)
if(nlopt_FOUND)
  pods_use_pkg_config_packages(drakeOptimization nlopt)
endif()
if(snopt_c_FOUND)
  pods_use_pkg_config_packages(drakeOptimization snopt_c)
endif()

if(snopt_c_FOUND AND MATLAB_FOUND)
  add_mex(NonlinearProgramSnoptmex NonlinearProgramSnoptmex.cpp)
  pods_use_pkg_config_packages(NonlinearProgramSnoptmex snopt_c)
endif()

add_subdirectory(qpSpline)
add_subdirectory(test)
add_subdirectory(+drakeFunction)
add_subdirectory(BMI)
add_subdirectory(trajectoryOptimization/test)
