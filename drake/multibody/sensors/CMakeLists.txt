add_library_with_exports(LIB_NAME drakeMultibodySensors SOURCE_FILES
    ideal_depth_sensor.cc
    rigid_body_tree_sensor.cc)
target_link_libraries(drakeMultibodySensors
    drakeRBM
    drakeSystemFramework)

drake_install_libraries(drakeMultibodySensors)
drake_install_headers(
    ideal_depth_sensor.h
    rigid_body_tree_sensor.h)
drake_install_pkg_config_file(drake-multibody-sensors
    TARGET drakeMultibodySensors
    LIBS -ldrakeMultibodySensors)

if(BUILD_TESTING)
  add_subdirectory(test)
endif()
