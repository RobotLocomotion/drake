if(BUILD_TESTING)
  add_subdirectory(test)
endif()

add_library_with_exports(LIB_NAME drakeBouncingBall SOURCE_FILES
  ball.cc
  bouncing_ball.cc
  )
target_link_libraries(drakeBouncingBall
  drakeCommon
  drakeSystemFramework
  )
drake_install_libraries(drakeBouncingBall)
drake_install_headers(
  bouncing_ball-inl.h
  bouncing_ball.h
  ball-inl.h
  ball.h
  )
drake_install_pkg_config_file(drake-bouncing-ball
  TARGET drakeBouncingBall
  LIBS -ldrakeBouncingBall
  REQUIRES
    drake-common
    drake-system-framework)

drake_add_matlab_test(NAME examples/bouncing_ball/BallPlant2D.run
  COMMAND BallPlant2D.run)
