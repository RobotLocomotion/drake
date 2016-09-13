add_subdirectory(test)
if(lcm_FOUND)
  include_directories(${PROJECT_SOURCE_DIR}/pod-build/lcmgen)

  add_library_with_exports(LIB_NAME drakeCars SOURCE_FILES
    car_simulation.cc
    curve2.cc
    gen/driving_command.cc
    gen/driving_command_translator.cc
    gen/euler_floating_joint_state.cc
    gen/euler_floating_joint_state_translator.cc
    gen/idm_with_trajectory_agent_state.cc
    gen/idm_with_trajectory_agent_state_translator.cc
    gen/simple_car_state.cc
    gen/simple_car_state_translator.cc
    idm_with_trajectory_agent.cc
    simple_car.cc
    trajectory_car.cc
    )
  target_link_libraries(drakeCars
    drakeCommon
    drakeLCMSystem
    drakeLCMSystem2
    drakeRBSystem
    drakeShapes
    drakeSystemAnalysis
    drakeSystemFramework
    )
  pods_install_libraries(drakeCars)
  drake_install_headers(
    curve2.h
    car_simulation.h
    simple_car.h
    simple_car-inl.h
    system1_cars_vectors.h
    trajectory_car.h
    system1_vector.h
    )
  add_subdirectory(gen)
  pods_install_pkg_config_file(drake-cars
    LIBS -ldrakeCars
    REQUIRES
    VERSION 0.0.1)

  add_dependencies(drakeCars drake_lcmtypes_hpp)
  add_executable(simple_car_demo simple_car_demo.cc)
  target_link_libraries(simple_car_demo drakeCars)

  add_executable(car_sim_lcm car_sim_lcm.cc)
  add_dependencies(car_sim_lcm drake_lcmtypes_hpp)
  target_link_libraries(car_sim_lcm drakeCars)

  add_executable(multi_car_sim_lcm multi_car_sim_lcm.cc)
  add_dependencies(multi_car_sim_lcm drake_lcmtypes_hpp)
  target_link_libraries(multi_car_sim_lcm drakeCars)

  drake_add_test(NAME multi_car_sim_lcm COMMAND multi_car_sim_lcm --duration 0.05)

  add_executable(demo_multi_car demo_multi_car.cc)
  target_link_libraries(demo_multi_car drakeCars)
endif()
