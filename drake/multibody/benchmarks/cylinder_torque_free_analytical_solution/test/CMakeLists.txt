if (Bullet_FOUND)
  drake_add_cc_test(NAME cylinder_torque_free_analytical_solution SIZE medium)
  target_link_libraries(cylinder_torque_free_analytical_solution drakeRigidBodyPlant drakeMultibodyParsers)
endif()
