add_executable(rigid_body_plant_test rigid_body_plant_test.cc)
target_link_libraries(rigid_body_plant_test
        drake_rbp drakeRBSystem ${GTEST_BOTH_LIBRARIES})
add_test(NAME rigid_body_plant_test COMMAND rigid_body_plant_test)
