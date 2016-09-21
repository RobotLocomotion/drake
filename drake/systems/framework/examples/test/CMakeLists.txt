add_executable(spring_mass_system_test spring_mass_system_test.cc)
target_link_libraries(spring_mass_system_test drakeSystemFramework
    ${GTEST_BOTH_LIBRARIES})
drake_add_test(NAME spring_mass_system_test COMMAND spring_mass_system_test)

add_executable(controlled_spring_mass_system_test
        controlled_spring_mass_system_test.cc)
target_link_libraries(controlled_spring_mass_system_test drakeSystemFramework
        ${GTEST_BOTH_LIBRARIES})
drake_add_test(NAME controlled_spring_mass_system_test
        COMMAND controlled_spring_mass_system_test)
