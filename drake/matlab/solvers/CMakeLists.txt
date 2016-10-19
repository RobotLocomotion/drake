pods_find_pkg_config(gurobi)

if(gurobi_FOUND)
  drake_add_mex(fastQPmex fastQPmex.cpp)
  target_link_libraries(fastQPmex drakeQP)
  pods_use_pkg_config_packages(fastQPmex gurobi)
  drake_add_mex(gurobiQPmex gurobiQPmex.cpp)
  target_link_libraries(gurobiQPmex drakeQP)
  pods_use_pkg_config_packages(gurobiQPmex gurobi)
endif()

drake_add_mex(lqrmex lqrmex.cpp $<TARGET_OBJECTS:drakeUtil>)
target_link_libraries(lqrmex drakeCommon)

find_package(snopt_c)

if(snopt_c_FOUND)
  drake_add_mex(NonlinearProgramSnoptmex NonlinearProgramSnoptmex.cpp)
  target_link_libraries(NonlinearProgramSnoptmex snopt_c)

  # These warnings are not worth fixing.
  target_compile_options(NonlinearProgramSnoptmex PRIVATE
    ${CXX_FLAGS_NO_ERROR_SHADOW})
endif()

add_subdirectory(qpSpline)
add_subdirectory(test)
add_subdirectory(+drakeFunction)
add_subdirectory(BMI)
add_subdirectory(trajectoryOptimization)
