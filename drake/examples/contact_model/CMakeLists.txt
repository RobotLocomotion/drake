if (lcm_FOUND AND Bullet_FOUND)
  add_executable(sliding_bricks sliding_bricks.cc)
  target_link_libraries(sliding_bricks
      drakeLcmSystem
      drakeMultibodyParsers
      drakeRigidBodyPlant
      drakeSystemAnalysis
      drakeSystemFramework
      )

  add_executable(rigid_gripper rigid_gripper.cc)
  target_link_libraries(rigid_gripper
      drakeCommon
      drakeLcmSystem
      drakeMultibodyParsers
      drakeRBM
      drakeRigidBodyPlant
      drakeSystemAnalysis
      drakeSystemControllers
      drakeSystemPrimitives
      )
endif()
