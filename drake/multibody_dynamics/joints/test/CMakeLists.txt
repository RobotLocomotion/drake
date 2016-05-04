add_executable(joint_test joint_test.cc)
target_link_libraries(joint_test drakeGeometryUtil ${GTEST_BOTH_LIBRARIES})
add_test(NAME joint_test COMMAND joint_test)
