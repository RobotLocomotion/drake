if(Bullet_FOUND)
  drake_add_cc_test(model_test)
  target_link_libraries(model_test drakeCollision)
endif()

# ==== Below this line is MATLAB-specific code ====

drake_add_matlab_test(NAME systems/plants/collision/test/testActiveCollisionOptions OPTIONAL bullet COMMAND testActiveCollisionOptions)

# ==== Below this line is C++ and MATLAB shared code ====

