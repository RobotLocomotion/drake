if (lcm_FOUND)
    add_library(drakePerceptionEstimatorsTestUtil
        test_util.cc)
    target_link_libraries(drakePerceptionEstimatorsTestUtil
        drakeArticulatedIcp
        drakeCommon
        drakeRBM
        drakeRigidBodyPlant  # For drake_visualizer
        drakeMultibodyParsers
        lcmtypes_bot2-core-cpp
        )

    drake_add_cc_test(articulated_icp_test)
    target_link_libraries(articulated_icp_test
        drakePerceptionEstimatorsTestUtil
        drakeOptimization
        )
endif()
