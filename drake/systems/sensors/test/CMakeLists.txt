drake_add_cc_test(rotary_encoders_test)
target_link_libraries(rotary_encoders_test drakeSensors)

if(Bullet_FOUND)
  drake_add_cc_test(depth_sensor_test)
  target_link_libraries(depth_sensor_test drakeSensors)

  drake_add_cc_test(accelerometer_test)
  target_link_libraries(accelerometer_test
      drakeLcm
      drakeLCMSystem2
      drakeMultibodyParsers
      drakeSensors
      drakeSystemAnalysis
      drakeSystemPrimitives)
endif()
