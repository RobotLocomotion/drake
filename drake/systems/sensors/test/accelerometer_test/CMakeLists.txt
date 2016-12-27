if(Bullet_FOUND)
  add_executable(accelerometer_test
      accelerometer_example_diagram.cc
      accelerometer_test.cc
      accelerometer_test_logger.cc)
  target_link_libraries(accelerometer_test
      drakeCommon
      drakeLcm
      drakeLCMSystem2
      drakeMultibodyParsers
      drakeSensors
      drakeSystemAnalysis
      drakeSystemPrimitives
      GTest::GTest
      GTest::Main)
  drake_add_test(NAME accelerometer_test COMMAND accelerometer_test)
endif()

if(Bullet_FOUND AND lcm_FOUND)
  add_executable(accelerometer_example
      accelerometer_example_diagram.cc
      accelerometer_example.cc
      accelerometer_test_logger.cc)
  target_link_libraries(accelerometer_example
      drakeCommon
      drakeLcm
      drakeLCMSystem2
      drakeMultibodyParsers
      drakeSensors
      drakeSystemAnalysis
      drakeSystemPrimitives
      GTest::GTest
      GTest::Main)
endif()
