add_library_with_exports(LIB_NAME drakeMultibodySensors SOURCE_FILES
    ideal_depth_sensor.cc
    ideal_depth_sensor_specification.cc
    rigid_body_tree_sensor.cc)
target_link_libraries(drakeMultibodySensors
    drakeRBM
    drakeSystemFramework)

drake_install_libraries(drakeMultibodySensors)
drake_install_headers(
    ideal_depth_sensor.h
    ideal_depth_sensor_specification.h
    rigid_body_tree_sensor.h)
drake_install_pkg_config_file(drake-multibody-sensors
    TARGET drakeMultibodySensors
    LIBS -ldrakeMultibodySensors)

if(BUILD_TESTING)
  add_subdirectory(test)
endif()
