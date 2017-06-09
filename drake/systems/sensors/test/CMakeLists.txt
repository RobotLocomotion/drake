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

if(VTK_FOUND)
  if(Bullet_FOUND)
    # Building a manual test executable for RgbdCamera until we have CI that
    # runs Xvfb server.  #5210 and #5227 are the related issues.
    # TODO(kunimatsu-tri) Replace this with an appropriate unit test once CI
    # had Xvfb.
    add_executable(rgbd_camera_test rgbd_camera_test.cc)
    target_link_libraries(rgbd_camera_test
      drakeCollision
      drakeMultibodyParsers
      drakeRigidBodyPlant
      drakeSensors
      drakeSystemAnalysis
      GTest::GTest
      GTest::Main)
  endif()

  drake_add_cc_test(vtk_util_test)
  target_link_libraries(vtk_util_test drakeSensors)
endif()

drake_add_cc_test(gyroscope_test)
target_link_libraries(gyroscope_test
    drakeMultibodyParsers
    drakeSensors)
