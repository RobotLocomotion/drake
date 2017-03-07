if(gurobi_FOUND)
    drake_add_cc_test(global_inverse_kinematics_test)
    set_tests_properties(global_inverse_kinematics_test PROPERTIES TIMEOUT 400)
    target_link_libraries(global_inverse_kinematics_test
            drakeGlobalIK
            drakeRigidBodyPlant
            drakeMultibodyParsers
            drakeCommon)
endif()