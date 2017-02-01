drake_add_cc_test(rotary_encoders_test)
target_link_libraries(rotary_encoders_test drakeSensors)

drake_add_cc_test(image_test)
target_link_libraries(image_test drakeSensors)

drake_add_cc_test(camera_info_test)
target_link_libraries(camera_info_test drakeSensors)

if(Bullet_FOUND)
  drake_add_cc_test(depth_sensor_test)
  target_link_libraries(depth_sensor_test
      drakeSensors)
endif()

if(BUILD_TESTING)
  add_subdirectory(accelerometer_test)
endif()

if(VTK_FOUND)
  drake_add_cc_test(rgbd_camera_test)
  target_link_libraries(rgbd_camera_test drakeSensors)
endif()
