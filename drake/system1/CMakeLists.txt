drake_install_headers(
  cascade_system.h
  feedback_system.h
  LinearSystem.h
  pd_control_system.h
  gravity_compensated_pd_position_control_system.h
  gravity_compensated_system.h
  Simulation.h
  simulation_options.h
  System.h
  vector.h
  trajectory_logger.h
  robot_state_tap.h)

drake_install_pkg_config_file(drake-system1
  REQUIRES eigen3)

if(lcm_FOUND)
  add_library_with_exports(LIB_NAME drakeLCMSystem SOURCE_FILES LCMSystem.cpp)
  target_link_libraries(drakeLCMSystem drakeLCMTypes lcm)

  drake_install_libraries(drakeLCMSystem)
  drake_install_headers(LCMSystem.h)
  drake_install_pkg_config_file(drake-lcm-system
    TARGET drakeLCMSystem
    LIBS -ldrakeLCMSystem
    REQUIRES
      eigen3
      drake-lcmtypes-cpp
      lcm)
endif()
