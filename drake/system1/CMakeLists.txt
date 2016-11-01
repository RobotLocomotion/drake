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

if(lcm_FOUND)
  add_library_with_exports(LIB_NAME drakeLCMSystem SOURCE_FILES LCMSystem.cpp)
  target_link_libraries(drakeLCMSystem drakeLCMTypes lcm)

  pods_install_libraries(drakeLCMSystem)
  drake_install_headers(LCMSystem.h)
  pods_install_pkg_config_file(drake-lcm-system
    LIBS -ldrakeLCMSystem
    REQUIRES lcm
    VERSION 0.0.1)
endif()
