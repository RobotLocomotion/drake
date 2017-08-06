if (lcm_FOUND)
    add_library(drakePerceptionEstimatorsTestUtil
        test_util.cc)
    target_link_libraries(drakePerceptionEstimatorsTestUtil
        drakeCommon
        drakeLcm
        lcmtypes_bot2-core-cpp
        GTest::GTest
        )

    drake_add_cc_test(articulated_icp_test)
    target_link_libraries(articulated_icp_test
        drakePerceptionEstimatorsTestUtil
        drakeArticulatedIcp
        drakeMultibodyParsers
        drakeOptimization
        drakeRBM
        drakeRigidBodyPlant  # For drake_visualizer
        )
endif()
