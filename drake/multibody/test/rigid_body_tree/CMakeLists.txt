add_library(drakeRigidBodyTreeCompareToClone STATIC
    rigid_body_tree_compare_to_clone.cc)
target_link_libraries(drakeRigidBodyTreeCompareToClone
    drakeRBM
    drakeRigidBodyActuatorCompareToClone
    drakeRigidBodyCompareToClone
    drakeRigidBodyFrameCompareToClone
    drakeRigidBodyLoopCompareToClone)

drake_add_cc_test(ball_joint_test)
target_link_libraries(ball_joint_test
    drakeMultibodyParsers
    drakeRBM)

drake_add_cc_test(NAME rigid_body_tree_creation_test SIZE medium)
target_link_libraries(rigid_body_tree_creation_test
    drakeMultibodyParsers
    drakeRBM)

drake_add_cc_test(rigid_body_tree_dynamics_test)
target_link_libraries(rigid_body_tree_dynamics_test
    drakeMultibodyParsers
    drakeRBM)

drake_add_cc_test(rigid_body_tree_completeness_test)
target_link_libraries(rigid_body_tree_completeness_test
    drakeRBM)

drake_add_cc_test(rigid_body_tree_face_extraction_test)
target_link_libraries(rigid_body_tree_face_extraction_test
    drakeRBM
    drakeMultibodyParsers
    drakeShapes)
