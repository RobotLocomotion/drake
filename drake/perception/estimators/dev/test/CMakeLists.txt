if (lcm_FOUND)
    drake_add_cc_test(articulated_icp_test)
    target_link_libraries(articulated_icp_test
        drakeArticulatedIcp
        drakeCommon
        drakeRBM
        drakeRigidBodyPlant  # For drake_visualizer
        drakeMultibodyParsers
        drakeOptimization
        lcmtypes_bot2-core-cpp
        )
endif()
