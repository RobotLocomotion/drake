add_subdirectory(controlled_spring_mass_system)

drake_add_cc_test(simulator_test)
target_link_libraries(simulator_test
  drakeSystemAnalysis drakeSystemFramework drakeCommon
        drakeSpringMassSystemPlant drakeControlledSpringMassSystemPlant)

drake_add_cc_test(explicit_euler_integrator_test)
target_link_libraries(explicit_euler_integrator_test
        drakeSystemAnalysis drakeSystemFramework drakeCommon
        drakeSpringMassSystemPlant)

drake_add_cc_test(runge_kutta2_integrator_test)
target_link_libraries(runge_kutta2_integrator_test
        drakeSystemAnalysis drakeSystemFramework drakeCommon
        drakeSpringMassSystemPlant)

# TODO(edrumwri): remove this test when evaluating derivatives for
# RigidBodyPlant does not fail when no geometry is present (specifically for
# the minimal build case). See issue #4294.
if(Bullet_FOUND)
  drake_add_cc_test(NAME runge_kutta3_integrator_test SIZE medium)
  target_link_libraries(runge_kutta3_integrator_test
          drakeSystemAnalysis drakeSystemFramework drakeCommon
          drakeSpringMassSystemPlant drakeRigidBodyPlant drakeRBSystem)
endif()
