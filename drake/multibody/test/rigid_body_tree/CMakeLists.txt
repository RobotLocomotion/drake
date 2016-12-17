if(Bullet_FOUND)
  drake_add_cc_test(rbt_collisions_test)
  target_link_libraries(rbt_collisions_test drakeMultibodyParsers drakeRBM)

  drake_add_cc_test(rigid_body_collision_clique_test)
  target_link_libraries(rigid_body_collision_clique_test drakeMultibodyParsers drakeRBM)
endif()

drake_add_cc_test(NAME rigid_body_tree_test SIZE medium)
target_link_libraries(rigid_body_tree_test drakeMultibodyParsers drakeRBM)

drake_add_cc_test(rigid_body_tree_inverse_dynamics_test)
target_link_libraries(rigid_body_tree_inverse_dynamics_test
    drakeMultibodyParsers
    drakeRBM)
