if (lcm_FOUND)
  drake_add_cc_test(zmp_planner_test)
  target_link_libraries(zmp_planner_test drakeZMPTestUtil)
endif()
