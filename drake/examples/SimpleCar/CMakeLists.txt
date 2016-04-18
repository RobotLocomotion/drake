add_subdirectory(test)
if (LCM_FOUND)
  include_directories(${PROJECT_SOURCE_DIR}/pod-build/lcmgen)

  add_library_with_exports(LIB_NAME drakeSimpleCar SOURCE_FILES simple_car.cc)
  pods_install_libraries(drakeSimpleCar)
  drake_install_headers(simple_car.h)
  pods_install_pkg_config_file(drake-simple-car
    LIBS -ldrakeSimpleCar
    REQUIRES
    VERSION 0.0.1)

  add_dependencies(drakeSimpleCar drake_lcmtypes lcmtype_agg_hpp)
  add_executable(simple_car_demo simple_car_demo.cc)
  target_link_libraries(simple_car_demo drakeSimpleCar drakeRBSystem drakeLCMSystem)
endif (LCM_FOUND)
