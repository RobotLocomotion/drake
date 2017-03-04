if (lcm_FOUND AND Bullet_FOUND)
  add_executable(sliding_bricks sliding_bricks.cc)
  target_link_libraries(sliding_bricks
      drakeLcmSystem
      drakeMultibodyParsers
      drakeRigidBodyPlant
      drakeSystemAnalysis
      drakeSystemFramework
      drakeSystemPrimitives
      )
endif()
