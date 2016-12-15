drake_add_cc_test(ideal_depth_sensor_test ideal_depth_sensor_test.cc)
target_link_libraries(ideal_depth_sensor_test
    drakeMultibodyParsers
    drakeMultibodySensors)
