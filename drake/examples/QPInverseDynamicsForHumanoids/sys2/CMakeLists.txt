add_executable(sys2_qp_test sys2_sim.cc ../qp_controller.cc ../rigid_body_tree_utils.cc ../humanoid_status.cc)
target_link_libraries(sys2_qp_test drakeRBSystem drakeOptimization drakeSide drakeLCMSystem2 drakeSystemAnalysis ${GTEST_BOTH_LIBRARIES})
