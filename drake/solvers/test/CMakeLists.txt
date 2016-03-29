if (GTEST_FOUND)
  add_executable(testOptimizationProblem testOptimizationProblem.cpp)
  target_link_libraries(testOptimizationProblem drakeOptimization ${GTEST_BOTH_LIBRARIES})

  add_executable(testMobyLCP testMobyLCP.cpp)
  target_link_libraries(testMobyLCP drakeOptimization ${GTEST_BOTH_LIBRARIES})
  add_test(NAME testMobyLCP COMMAND testMobyLCP)
endif()

add_test(NAME testOptimizationProblem COMMAND testOptimizationProblem)
