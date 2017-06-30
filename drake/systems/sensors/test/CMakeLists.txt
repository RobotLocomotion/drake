drake_add_cc_test(rotary_encoders_test)
target_link_libraries(rotary_encoders_test drakeSensors)

drake_add_cc_test(image_test)
target_link_libraries(image_test drakeSensors)

drake_add_cc_test(camera_info_test)
target_link_libraries(camera_info_test drakeSensors)

if(robotlocomotion-lcmtypes_FOUND)
  drake_add_cc_test(image_to_lcm_image_array_t_test)
  target_link_libraries(image_to_lcm_image_array_t_test drakeSensors)
endif()

if(Bullet_FOUND)
  drake_add_cc_test(depth_sensor_test)
  target_link_libraries(depth_sensor_test
      drakeSensors)
  if(LCM_FOUND)
    drake_add_cc_test(depth_sensor_to_lcm_point_cloud_message_test)
    target_link_libraries(depth_sensor_to_lcm_point_cloud_message_test
        drakeSensors)
  endif()
endif()

if(BUILD_TESTING)
  add_subdirectory(accelerometer_test)
endif()

drake_add_cc_test(gyroscope_test)
target_link_libraries(gyroscope_test
    drakeMultibodyParsers
    drakeSensors)
