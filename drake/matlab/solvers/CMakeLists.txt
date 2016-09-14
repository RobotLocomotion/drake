pods_find_pkg_config(gurobi)

if(gurobi_FOUND)
  add_mex(fastQPmex fastQPmex.cpp)
  target_link_libraries(fastQPmex drakeQP)
  add_mex(gurobiQPmex gurobiQPmex.cpp)
  target_link_libraries(gurobiQPmex drakeQP)
endif()

add_mex(lqrmex lqrmex.cpp)
target_link_libraries(lqrmex drakeUtil)

pods_find_pkg_config(snopt_c)
if(snopt_c_FOUND)
  add_mex(NonlinearProgramSnoptmex NonlinearProgramSnoptmex.cpp)
  pods_use_pkg_config_packages(NonlinearProgramSnoptmex snopt_c)

  # These warnings are not worth fixing.
  target_compile_options(NonlinearProgramSnoptmex PRIVATE
    ${CXX_FLAGS_NO_ERROR_SHADOW})
endif()

add_subdirectory(qpSpline)
add_subdirectory(test)
add_subdirectory(+drakeFunction)
add_subdirectory(BMI)
add_subdirectory(trajectoryOptimization)
