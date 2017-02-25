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
  find_program(xvfb-run xvfb-run)
  if(xvfb-run)
    set(wrap_xvfb ${xvfb-run} -a -s "-screen 0 1280x1024x24")
    add_executable(rgbd_camera_test rgbd_camera_test.cc)
    target_link_libraries(rgbd_camera_test
      GTest::GTest
      GTest::Main
      drakeMultibodyParsers
      drakeRigidBodyPlant
      drakeSensors
      drakeSystemAnalysis
    )
    add_test(
      NAME rgbd_camera_test2
      COMMAND ${wrap_xvfb}
              ${PROJECT_BINARY_DIR}/systems/sensors/test/rgbd_camera_test)
  endif()

  drake_add_cc_test(vtk_util_test)
  target_link_libraries(vtk_util_test drakeSensors)
endif()

drake_add_cc_test(gyroscope_test)
target_link_libraries(gyroscope_test
    drakeMultibodyParsers
    drakeSensors)
