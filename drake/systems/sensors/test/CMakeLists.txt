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
  find_program(Xvfb Xvfb)
  if(Xvfb)
    set(ENV{DISPLAY} :99)
    execute_process(COMMAND glxinfo)
    set(wrap_xvfb "${Xvfb} :99 -ac -screen 0 1280x1024x24")
    drake_add_cc_test(rgbd_camera_test ${wrap_xvfb})
    target_link_libraries(rgbd_camera_test
      drakeMultibodyParsers
      drakeRigidBodyPlant
      drakeSensors
      drakeSystemAnalysis
    )
    set_property(TEST rgbd_camera_test PROPERTY RESOURCE_LOCK xvfb)
  endif()

  drake_add_cc_test(vtk_util_test)
  target_link_libraries(vtk_util_test drakeSensors)
endif()

drake_add_cc_test(gyroscope_test)
target_link_libraries(gyroscope_test
    drakeMultibodyParsers
    drakeSensors)
