if(lcm_FOUND AND WITH_HSRB_DESCRIPTION)
  drake_add_cc_test(hsrb_system_factories_test)
  target_link_libraries(hsrb_system_factories_test
      drakeHsrb)
endif()
