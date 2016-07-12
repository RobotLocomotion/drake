add_executable(testQP test.cc qp_controller.cc rigid_body_tree_utils.cc humanoid_status.cc)
target_link_libraries(testQP drakeRBSystem drakeOptimization drakeSide)
