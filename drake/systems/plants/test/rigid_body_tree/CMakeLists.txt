pods_find_pkg_config(bullet)

if(GTEST_FOUND AND bullet_FOUND)
  add_executable(rbt_collisions_test rbt_collisions_test.cc)
  target_link_libraries(rbt_collisions_test drakeRBM ${GTEST_BOTH_LIBRARIES})
  add_test(NAME rbt_collisions_test COMMAND rbt_collisions_test)
endif()
