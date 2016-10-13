drake_add_cc_test(NAME valkyrie_balancing_system_test SIZE large)
target_link_libraries(valkyrie_balancing_system_test
  drakeQPController
  drakeSystemAnalysis
  GTest::GTest GTest::Main)
