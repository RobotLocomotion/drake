find_package(Bullet)

if(BULLET_FOUND)
  drake_add_cc_test(rbt_collisions_test)
  target_link_libraries(rbt_collisions_test drakeRBM)
endif()

# TODO(jwnimmer-tri) Get valgrind passing again.
drake_add_cc_test(NAME rigid_body_tree_test NO_VALGRIND)
target_link_libraries(rigid_body_tree_test drakeRBSystem)

drake_add_cc_test(rigid_body_tree_inverse_dynamics_test)
target_link_libraries(rigid_body_tree_inverse_dynamics_test drakeRBSystem)

drake_add_cc_test(rigid_body_collision_clique_test)
target_link_libraries(rigid_body_collision_clique_test drakeRBSystem)
