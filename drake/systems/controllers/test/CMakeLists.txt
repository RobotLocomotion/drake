add_executable(gravity_compensator_test gravity_compensator_test.cc)
target_link_libraries(gravity_compensator_test
    drakeSystemControllers
    drakeSystemFramework
    drakeRBSystem GTest::GTest GTest::Main)
add_test(NAME gravity_compensator_test COMMAND gravity_compensator_test)
