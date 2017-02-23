if (lcm_FOUND AND Bullet_FOUND)
  add_executable(sliding_brick sliding_brick.cc)
  target_link_libraries(sliding_brick
      drakeLcmSystem
      drakeMultibodyParsers
      drakeRigidBodyPlant
      drakeSystemAnalysis
      drakeSystemFramework
      drakeSystemPrimitives
      )
endif()
