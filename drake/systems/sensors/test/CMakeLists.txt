drake_add_cc_test(rotary_encoders_test rotary_encoders_test.cc)
target_link_libraries(rotary_encoders_test drakeSensors)

if(Bullet_FOUND)
  drake_add_cc_test(depth_sensor_test depth_sensor_test.cc)
  target_link_libraries(depth_sensor_test
      drakeSensors)
endif()
