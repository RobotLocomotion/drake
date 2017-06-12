if(lcm_FOUND)
  drake_add_cc_test(acrobot_urdf_dynamics_test)
  target_link_libraries(acrobot_urdf_dynamics_test drakeRigidBodyPlant drakeAcrobotPlant)
endif()
