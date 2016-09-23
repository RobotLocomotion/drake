add_subdirectory(test)

add_library_with_exports(LIB_NAME drakeBouncingBall SOURCE_FILES
  ball-inl.h
  ball.h
  ball.cc
  bouncing_ball-inl.h
  bouncing_ball.h
  bouncing_ball.cc
  )
target_link_libraries(drakeBouncingBall
  drakeCommon
  drakeSystemFramework
  )
pods_install_libraries(drakeBouncingBall)
pods_install_pkg_config_file(drake-bouncing-ball
  LIBS -ldrakeBouncingBall
  REQUIRES
  VERSION 0.0.1)

add_matlab_test(NAME examples/BouncingBall/BallPlant2D.run
  COMMAND BallPlant2D.run)
