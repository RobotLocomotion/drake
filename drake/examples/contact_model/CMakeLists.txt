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

  add_executable(bowling_ball bowling_ball.cc)
  target_link_libraries(bowling_ball
      drakeLcmSystem
      drakeMultibodyParsers
      drakeRigidBodyPlant
      drakeSystemAnalysis
      drakeSystemFramework
      )
endif()
