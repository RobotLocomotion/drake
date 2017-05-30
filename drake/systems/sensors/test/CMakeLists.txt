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
    add_executable(rgbd_camera_test rgbd_camera_test.cc)
    target_link_libraries(rgbd_camera_test
      drakeCollision
      drakeMultibodyParsers
      drakeRigidBodyPlant
      drakeSensors
      drakeSystemAnalysis
      GTest::GTest
      GTest::Main)

    # TODO(jamiesnape): Fix rgbd_camera_test built with VTK 5.x when using Xvfb
    # or remove VTK 5 support completely.
    if(VTK_MAJOR_VERSION GREATER 5)
      set(XVFB_RUN_COMMAND)

      if(NOT APPLE AND NOT DEFINED ENV{DISPLAY})
        find_program(XVFB_RUN_EXECUTABLE NAMES xvfb-run)

        if(XVFB_RUN_EXECUTABLE)
            set(XVFB_RUN_COMMAND
              "${XVFB_RUN_EXECUTABLE}" -a -s "-screen 0 1024x768x24")
        endif()
      endif()

      if(APPLE OR DEFINED ENV{DISPLAY} OR XVFB_RUN_COMMAND)
        add_test(NAME rgbd_camera_test COMMAND
          ${XVFB_RUN_COMMAND} $<TARGET_FILE:rgbd_camera_test>)
      else()
        message(STATUS
          "Not running rgbd_camera_test because no framebuffer is available")
      endif()
    else()
      message(STATUS
        "Not running rgbd_camera_test because this build uses VTK 5")
    endif()
  endif()

  drake_add_cc_test(vtk_util_test)
  target_link_libraries(vtk_util_test drakeSensors)
endif()

drake_add_cc_test(gyroscope_test)
target_link_libraries(gyroscope_test
    drakeMultibodyParsers
    drakeSensors)
