#add_subdirectory(test)
if (LCM_FOUND)
  include_directories(${PROJECT_SOURCE_DIR}/pod-build/lcmgen)

  add_library_with_exports(LIB_NAME drakeTrivialCar SOURCE_FILES trivial_car.cc)
  pods_install_libraries(drakeTrivialCar)
  drake_install_headers(trivial_car.h)
  pods_install_pkg_config_file(drake-trivial-car
    LIBS -ldrakeTrivialCar
    REQUIRES
    VERSION 0.0.1)

  add_dependencies(drakeTrivialCar drake_lcmtypes lcmtype_agg_hpp)
  add_executable(demo_multi_car demo_multi_car.cc)
  target_link_libraries(demo_multi_car drakeTrivialCar drakeRBSystem drakeLCMSystem)
endif (LCM_FOUND)
