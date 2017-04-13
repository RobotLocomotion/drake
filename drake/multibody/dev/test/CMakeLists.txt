add_library_with_exports(LIB_NAME drakeGlobalInverseKinematicsTest
        SOURCE_FILES global_inverse_kinematics_test_util.h global_inverse_kinematics_test_util.cc)
target_link_libraries(drakeGlobalInverseKinematicsTest
        drakeGlobalIK
        drakeRigidBodyPlant
        drakeMultibodyParsers
        drakeCommon
        GTest::GTest)

if(gurobi_FOUND)
    drake_add_cc_test(global_inverse_kinematics_test)
    set_tests_properties(global_inverse_kinematics_test PROPERTIES TIMEOUT 200)
    target_link_libraries(global_inverse_kinematics_test
            drakeGlobalInverseKinematicsTest
            drakeIK)

    drake_add_cc_test(global_inverse_kinematics_reachable_test)
    set_tests_properties(global_inverse_kinematics_reachable_test PROPERTIES TIMEOUT 200)
    target_link_libraries(global_inverse_kinematics_reachable_test
            drakeGlobalInverseKinematicsTest)
endif()