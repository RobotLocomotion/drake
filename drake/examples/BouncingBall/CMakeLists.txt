add_subdirectory(test)
if(lcm_FOUND)
  include_directories(${PROJECT_SOURCE_DIR}/pod-build/lcmgen)

  add_library_with_exports(LIB_NAME drakeBouncingBall SOURCE_FILES
    ball.cc
    #bouncing_ball.cc
    )
  target_link_libraries(drakeBouncingBall
    drakeCommon
    drakeLCMSystem
    drakeLCMSystem2
    drakeRBSystem
    drakeShapes
    drakeSystemFramework
    )
  pods_install_libraries(drakeBouncingBall)
  pods_install_pkg_config_file(drake-bouncingball
    LIBS -ldrakeBouncingBall
    REQUIRES
    VERSION 0.0.1)

  add_dependencies(drakeBouncingBall drake_lcmtypes_hpp)

endif()

add_matlab_test(NAME examples/BouncingBall/BallPlant2D.run COMMAND BallPlant2D.run)
