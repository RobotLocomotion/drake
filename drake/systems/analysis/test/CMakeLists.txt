add_executable(simulator_test simulator_test.cc)
target_link_libraries(simulator_test 
  drakeSystemAnalysis drakeSystemFramework drakeCommon
  ${GTEST_BOTH_LIBRARIES})
drake_add_test(NAME simulator_test COMMAND simulator_test)
