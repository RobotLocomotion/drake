if(snopt_c_FOUND)
  # drake_add_cc_test(NAME valkyrie_balancing_system_test SIZE medium)  # FIXME: see #4000
  # target_link_libraries(valkyrie_balancing_system_test
  #   drakeQPController
  #   drakeSystemAnalysis
  #   GTest::GTest GTest::Main)
endif()
