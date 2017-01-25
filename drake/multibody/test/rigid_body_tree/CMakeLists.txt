if(Bullet_FOUND)
  drake_add_cc_test(anchored_geometry_test)
  target_link_libraries(anchored_geometry_test
      drakeMultibodyParsers
      drakeRBM)

  drake_add_cc_test(contact_points_generation_test)
  target_link_libraries(contact_points_generation_test
      drakeMultibodyParsers
      drakeRBM)

  drake_add_cc_test(point_collision_detection_test)
  target_link_libraries(point_collision_detection_test
      drakeMultibodyParsers
      drakeRBM)

  drake_add_cc_test(rbt_collisions_test)
  target_link_libraries(rbt_collisions_test
      drakeMultibodyParsers
      drakeRBM)

  drake_add_cc_test(rigid_body_collision_clique_test)
  target_link_libraries(rigid_body_collision_clique_test
      drakeMultibodyParsers
      drakeRBM)
endif()

drake_add_cc_test(NAME rigid_body_tree_creation_test SIZE medium)
target_link_libraries(rigid_body_tree_creation_test
    drakeMultibodyParsers
    drakeRBM)

drake_add_cc_test(rigid_body_tree_kinematics_test)
target_link_libraries(rigid_body_tree_kinematics_test
    drakeMultibodyParsers
    drakeRBM
    drakeMultibodyBenchmarks)

drake_add_cc_test(rigid_body_tree_dynamics_test)
target_link_libraries(rigid_body_tree_dynamics_test
    drakeMultibodyParsers
    drakeRBM)

drake_add_cc_test(rigid_body_tree_completeness_test)
target_link_libraries(rigid_body_tree_completeness_test
    drakeRBM)
