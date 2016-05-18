add_subdirectory(test)
if (LCM_FOUND)
  include_directories(${PROJECT_SOURCE_DIR}/pod-build/lcmgen)

  add_library_with_exports(LIB_NAME drakeCars SOURCE_FILES simple_car.cc)
  pods_install_libraries(drakeCars)
  drake_install_headers(
    simple_car.h
    gen/driving_command.h
    gen/euler_floating_joint_state.h
    gen/simple_car_state.h
    )
  pods_install_pkg_config_file(drake-cars
    LIBS -ldrakeCars
    REQUIRES
    VERSION 0.0.1)

  add_dependencies(drakeCars drake_lcmtypes lcmtype_agg_hpp)
  add_executable(simple_car_demo simple_car_demo.cc)
  target_link_libraries(simple_car_demo drakeCars drakeRBSystem drakeLCMSystem)

  add_executable(car_sim_lcm car_sim_lcm.cpp)
  add_dependencies(car_sim_lcm drake_lcmtypes lcmtype_agg_hpp)
  target_link_libraries(car_sim_lcm drakeRBSystem drakeLCMSystem)
endif (LCM_FOUND)
