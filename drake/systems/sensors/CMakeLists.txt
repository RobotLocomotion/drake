set(drakeSensors_SRC_FILES
    depth_sensor.cc
    depth_sensor_output.cc
    depth_sensor_specification.cc
    rotary_encoders.cc)

set(drakeSensors_HEADERS
    depth_sensor.h
    depth_sensor_output.h
    depth_sensor_specification.h
    rotary_encoders.h)

set(drakeSensors_LINK_LIBRARIES
    drakeRBM
    drakeSystemFramework)

set(drakeSensors_INCLUDE_DIRS)

if(VTK_FOUND)
  set(drakeSensors_SRC_FILES ${drakeSensors_SRC_FILES} rgbd_camera.cc)
  set(drakeSensors_HEADERS ${drakeSensors_HEADERS} rgbd_camera.h)
  set(drakeSensors_LINK_LIBRARIES ${drakeSensors_LINK_LIBRARIES}
      ${VTK_LIBRARIES})
  set(drakeSensors_INCLUDE_DIRS ${drakeSensors_INCLUDE_DIRS}
      ${VTK_INCLUDE_DIRS})
endif()

add_library_with_exports(LIB_NAME drakeSensors SOURCE_FILES
    ${drakeSensors_SRC_FILES})

target_link_libraries(drakeSensors ${drakeSensors_LINK_LIBRARIES})

target_include_directories(drakeSensors PRIVATE ${drakeSensors_INCLUDE_DIRS})

drake_install_libraries(drakeSensors)
drake_install_headers(${drakeSensors_HEADERS})
drake_install_pkg_config_file(drake-sensors
    TARGET drakeSensors
    LIBS -ldrakeSensors)

if(BUILD_TESTING)
  add_subdirectory(test)
endif()
