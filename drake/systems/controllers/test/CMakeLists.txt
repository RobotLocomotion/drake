add_executable(gravity_compensator_test gravity_compensator_test.cc)
target_link_libraries(gravity_compensator_test
    drakeSystemControllers
    drakeSystemFramework
    drakeRBSystem ${GTEST_BOTH_LIBRARIES})
add_test(NAME gravity_compensator_test COMMAND gravity_compensator_test)


drake_add_cc_test(pid_controller_test)
target_link_libraries(pid_controller_test drakeSystemControllers drakeSystemFramework)
