if(gurobi_FOUND)
  drake_add_mex(fastQPmex fastQPmex.cpp)
  target_link_libraries(fastQPmex drakeQP)

  drake_add_mex(gurobiQPmex gurobiQPmex.cpp)
  target_link_libraries(gurobiQPmex drakeQP)
endif()

drake_add_mex(lqrmex lqrmex.cpp)
target_link_libraries(lqrmex drakeLinearControl)

if(snopt_FOUND)
  drake_add_mex(NonlinearProgramSnoptmex NonlinearProgramSnoptmex.cpp)
  target_link_libraries(NonlinearProgramSnoptmex snopt_c)

  # These warnings are not worth fixing.
  target_compile_options(NonlinearProgramSnoptmex PRIVATE
    ${CXX_FLAGS_NO_ERROR_SHADOW})
endif()

add_subdirectory(qpSpline)

if(BUILD_TESTING)
  add_subdirectory(test)
endif()

add_subdirectory(+drakeFunction)
add_subdirectory(BMI)
add_subdirectory(trajectoryOptimization)
