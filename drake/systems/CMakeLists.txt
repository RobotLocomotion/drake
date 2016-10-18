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
  trajectory_logger.h)

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

add_subdirectory(analysis)
add_subdirectory(framework)
add_subdirectory(lcm)

# ==== Below this line is MATLAB-specific code ====

if(Matlab_SIMULINK_FOUND)
  drake_add_mex(DCSFunction DCSFunction.cpp)
endif()

add_subdirectory(frames)

# ==== Below this line is C++ and MATLAB shared code ====

add_subdirectory(plants)
add_subdirectory(trajectories)
add_subdirectory(controllers)
add_subdirectory(robotInterfaces)
add_subdirectory(test)
