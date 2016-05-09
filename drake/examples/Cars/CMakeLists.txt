add_subdirectory(test)
if (LCM_FOUND)
  include_directories(${PROJECT_SOURCE_DIR}/pod-build/lcmgen)

  add_library_with_exports(LIB_NAME drakeSimpleCar SOURCE_FILES simple_car.cc)
  pods_install_libraries(drakeSimpleCar)
  drake_install_headers(
    simple_car.h
    gen/driving_command.h
    gen/euler_floating_joint_state.h
    gen/simple_car_state.h
    )
  pods_install_pkg_config_file(drake-simple-car
    LIBS -ldrakeSimpleCar
    REQUIRES
    VERSION 0.0.1)

  add_dependencies(drakeSimpleCar drake_lcmtypes lcmtype_agg_hpp)
  add_executable(simple_car_demo simple_car_demo.cc)
  target_link_libraries(simple_car_demo drakeSimpleCar drakeRBSystem drakeLCMSystem)

  add_library_with_exports(LIB_NAME drakeCar SOURCE_FILES car.cc)
  pods_install_libraries(drakeCar)
  drake_install_headers(
    car.h
    )
  pods_install_pkg_config_file(drake-car
    LIBS -ldrakeCar
    REQUIRES
    VERSION 0.0.1)

  add_executable(car_sim_lcm car_sim_lcm.cpp)
  add_dependencies(car_sim_lcm drake_lcmtypes lcmtype_agg_hpp)
  target_link_libraries(car_sim_lcm drakeCar drakeRBSystem drakeLCMSystem)

  if(roscpp_FOUND)
    add_executable(car_sim_lcm_and_ros car_sim_lcm_and_ros.cpp)
    target_link_libraries(car_sim_lcm_and_ros drakeCar drakeRBSystem drakeLCMSystem ${roscpp_LIBRARIES} ${tf_LIBRARIES})
  endif()

  add_executable(publish_driving_command publish_driving_command.cpp)
  add_dependencies(publish_driving_command drake_lcmtypes lcmtype_agg_hpp)
  pods_use_pkg_config_packages(publish_driving_command lcm)
  add_test(NAME publish_driving_command COMMAND publish_driving_command 1.0 0.0)

endif (LCM_FOUND)
