add_executable(testQP test.cpp qp_controller.cc rigid_body_tree_utils.cc humanoid_state.cc)
target_link_libraries(testQP drakeRBSystem drakeOptimization)
