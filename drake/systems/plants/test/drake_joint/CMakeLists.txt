add_executable(drake_joint_test drake_joint_test.cc)
target_link_libraries(drake_joint_test drakeRBSystem ${GTEST_BOTH_LIBRARIES})
add_test(NAME drake_joint_test COMMAND drake_joint_test)
