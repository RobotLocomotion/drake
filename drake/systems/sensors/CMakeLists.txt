add_library_with_exports(LIB_NAME drakeSensors SOURCE_FILES
    depth_sensor.cc
    depth_sensor_output.cc
    depth_sensor_specification.cc
    rgbd_camera.cc
    rotary_encoders.cc)
target_link_libraries(drakeSensors
    drakeRBM
    drakeSystemFramework
    ${VTK_LIBRARIES})

drake_install_libraries(drakeSensors)
drake_install_headers(
    depth_sensor.h
    depth_sensor_output.h
    depth_sensor_specification.h
    rgbd_camera.h
    rotary_encoders.h)
drake_install_pkg_config_file(drake-sensors
    TARGET drakeSensors
    LIBS -ldrakeSensors)

if(BUILD_TESTING)
  add_subdirectory(test)
endif()
