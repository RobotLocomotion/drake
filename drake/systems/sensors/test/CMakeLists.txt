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
  if(Bullet_FOUND)
    find_program(Xvfb Xvfb)
      if(Xvfb)
      add_executable(rgbd_camera_test rgbd_camera_test.cc)
      target_link_libraries(rgbd_camera_test
        drakeCollision
        drakeMultibodyParsers
        drakeRigidBodyPlant
        drakeSensors
        drakeSystemAnalysis
        GTest::GTest
        GTest::Main
      )
      add_test(
        NAME rgbd_camera_test2
        COMMAND
          bash ${PROJECT_SOURCE_DIR}/systems/sensors/test/runtest_with_xvfb.sh
          ${PROJECT_BINARY_DIR}/systems/sensors/test/rgbd_camera_test)
      set_tests_properties(rgbd_camera_test2
        PROPERTIES FAIL_REGULAR_EXPRESSION "FAIL")
      set_tests_properties(rgbd_camera_test2
        PROPERTIES PASS_REGULAR_EXPRESSION "PASS")
    endif()
  endif()

  drake_add_cc_test(vtk_util_test)
  target_link_libraries(vtk_util_test drakeSensors)
endif()

drake_add_cc_test(gyroscope_test)
target_link_libraries(gyroscope_test
    drakeMultibodyParsers
    drakeSensors)
