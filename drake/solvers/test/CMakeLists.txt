add_executable(testOptimizationProblem testOptimizationProblem.cpp)
target_link_libraries(testOptimizationProblem drakeOptimization)

if (snopt_c_FOUND) # todo: run tests, but catch "snopt missing" errors?
  add_test(NAME testOptimizationProblem COMMAND testOptimizationProblem)
endif (snopt_c_FOUND)