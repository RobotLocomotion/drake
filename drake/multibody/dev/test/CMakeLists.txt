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
    drake_add_cc_test(NAME global_inverse_kinematics_test SIZE medium)
    target_link_libraries(global_inverse_kinematics_test
            drakeGlobalInverseKinematicsTest)

    drake_add_cc_test(NAME global_inverse_kinematics_reachable_test SIZE large)
    target_link_libraries(global_inverse_kinematics_reachable_test
            drakeGlobalInverseKinematicsTest)

    drake_add_cc_test(NAME global_inverse_kinematics_collision_avoidance_test
            SIZE large)
    target_link_libraries(global_inverse_kinematics_collision_avoidance_test
            drakeGlobalInverseKinematicsTest)
endif()
