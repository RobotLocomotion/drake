drake_add_cc_test(rotary_encoders_test)
target_link_libraries(rotary_encoders_test drakeSensors)

if(Bullet_FOUND)
  drake_add_cc_test(depth_sensor_test)
  target_link_libraries(depth_sensor_test
      drakeSensors)
endif()

if(VTK_FOUND)
  drake_add_cc_test(rgbd_camera_test)
  target_link_libraries(rgbd_camera_test drakeSensors)
endif()

add_executable(image_test image_test.cc)
target_compile_definitions(image_test PRIVATE DRAKE_ENABLE_ASSERTS)
target_compile_options(image_test PRIVATE -UDRAKE_DISABLE_ASSERTS)
target_link_libraries(image_test drakeSensors GTest::GTest GTest::Main)
drake_add_test(NAME image_test COMMAND image_test)
