find_package(Bullet)

if(BULLET_FOUND)
  # TODO(jwnimmer-tri) Get valgrind passing again.
  drake_add_cc_test(NAME model_test NO_VALGRIND)
  target_link_libraries(model_test drakeCollision)
endif()

drake_add_matlab_test(NAME systems/plants/collision/test/testActiveCollisionOptions OPTIONAL bullet COMMAND testActiveCollisionOptions)
