if(lcm_FOUND)
  add_executable(kuka_demo kuka_demo.cc)
  target_link_libraries(kuka_demo drakeRigidBodyPlant drakeSystemAnalysis
    drakeSystemControllers drakeLCMSystem2 drakeIK drakeTrajectories
    ${GTEST_BOTH_LIBRARIES})
  add_test(NAME kuka_demo COMMAND kuka_demo)
endif()
