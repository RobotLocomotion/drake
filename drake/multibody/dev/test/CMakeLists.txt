add_library_with_exports(LIB_NAME drakeGlobalInverseKinematicsTest
        SOURCE_FILES global_inverse_kinematics_test_util.h global_inverse_kinematics_test_util.cc)
target_link_libraries(drakeGlobalInverseKinematicsTest
        drakeGlobalIK
        drakeRigidBodyPlant
        drakeMultibodyParsers
        drakeCommon
        drakeIK
        GTest::GTest)

if(gurobi_FOUND)
    drake_add_cc_test(global_inverse_kinematics_test)
    set_tests_properties(global_inverse_kinematics_test PROPERTIES TIMEOUT 200)
    target_link_libraries(global_inverse_kinematics_test
            drakeGlobalInverseKinematicsTest)

    drake_add_cc_test(global_inverse_kinematics_reachable_test)
    set_tests_properties(global_inverse_kinematics_reachable_test PROPERTIES TIMEOUT 300)
    target_link_libraries(global_inverse_kinematics_reachable_test
            drakeGlobalInverseKinematicsTest)

    drake_add_cc_test(global_inverse_kinematics_collision_avoidance_test)
    set_tests_properties(global_inverse_kinematics_collision_avoidance_test PROPERTIES TIMEOUT 600)
    target_link_libraries(global_inverse_kinematics_collision_avoidance_test
            drakeGlobalInverseKinematicsTest)
endif()