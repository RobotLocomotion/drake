add_library_with_exports(LIB_NAME drakeSensors SOURCE_FILES
    depth_sensor.cc
    depth_sensor_output.cc
    depth_sensor_specification.cc
    rotary_encoders.cc)

# TODO(liang.fok): Determine whether "robotlocomotion-lcmtypes-cpp" is correct
# and why it cannot be replaced with "bot2core".
target_link_libraries(drakeSensors
    drakeRBM
    drakeSystemFramework
    robotlocomotion-lcmtypes-cpp)

drake_install_libraries(drakeSensors)
drake_install_headers(
    depth_sensor.h
    depth_sensor_output.h
    depth_sensor_specification.h
    rotary_encoders.h)
drake_install_pkg_config_file(drake-sensors
    TARGET drakeSensors
    LIBS -ldrakeSensors)

if(BUILD_TESTING)
  add_subdirectory(test)
endif()
