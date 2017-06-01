set(drakeSensors_SRC_FILES
    accelerometer.cc
    accelerometer_output.cc
    camera_info.cc
    depth_sensor.cc
    depth_sensor_output.cc
    depth_sensor_specification.cc
    gyroscope.cc
    gyroscope_output.cc
    rotary_encoders.cc)

set(drakeSensors_HEADERS
    accelerometer.h
    accelerometer_output.h
    camera_info.h
    depth_sensor.h
    depth_sensor_output.h
    depth_sensor_specification.h
    gyroscope.h
    gyroscope_output.h
    image.h
    pixel_types.h
    rotary_encoders.h)

set(drakeSensors_TARGET_LINK_LIBRARIES
    drakeRBM
    drakeRigidBodyPlant
    drakeSystemFramework)

if(VTK_FOUND)
  list(APPEND drakeSensors_SRC_FILES rgbd_camera.cc vtk_util.cc)
  list(APPEND drakeSensors_HEADERS rgbd_camera.h vtk_util.h)
endif()

if(lcm_FOUND AND Bullet_FOUND)
  list(APPEND drakeSensors_SRC_FILES depth_sensor_to_lcm_point_cloud_message.cc)
  list(APPEND drakeSensors_HEADERS depth_sensor_to_lcm_point_cloud_message.h)
  list(APPEND drakeSensors_TARGET_LINK_LIBRARIES robotlocomotion-lcmtypes-cpp)
endif()

add_library_with_exports(LIB_NAME drakeSensors SOURCE_FILES
    ${drakeSensors_SRC_FILES})

target_link_libraries(drakeSensors
    ${drakeSensors_TARGET_LINK_LIBRARIES})

if(VTK_FOUND)
  target_link_libraries(drakeSensors drakeRendering)
  if(VTK_MAJOR_VERSION GREATER 5)
    target_link_libraries(drakeSensors
      vtkCommonCore
      vtkCommonDataModel
      vtkCommonTransforms
      vtkFiltersGeneral
      vtkFiltersSources
      vtkIOGeometry
      vtkIOImage
      vtkRenderingCore
      vtkRenderingOpenGL2)
    else()
      target_link_libraries(drakeSensors vtkIO vtkRendering)
    endif()
    target_include_directories(drakeSensors PUBLIC ${VTK_INCLUDE_DIRS})
endif()

if(LCM_FOUND)
  target_link_libraries(drakeSensors robotlocomotion-lcmtypes-cpp)
endif()

drake_install_libraries(drakeSensors)
drake_install_headers(${drakeSensors_HEADERS})
drake_install_pkg_config_file(drake-sensors
    TARGET drakeSensors
    LIBS -ldrakeSensors)

if(BUILD_TESTING)
  add_subdirectory(test)
endif()
