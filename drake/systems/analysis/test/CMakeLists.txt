add_subdirectory(controlled_spring_mass_system)

drake_add_cc_test(NAME simulator_test EXTENSION cc)
drake_add_cc_test(NAME explicit_euler_integrator_test EXTENSION cc)
drake_add_cc_test(NAME runge_kutta2_integrator_test EXTENSION cc)
drake_add_cc_test(NAME runge_kutta3_integrator_test EXTENSION cc)
target_link_libraries(simulator_test
  drakeSystemAnalysis drakeSystemFramework drakeCommon
        drakeSpringMassSystemPlant drakeControlledSpringMassSystemPlant)
target_link_libraries(explicit_euler_integrator_test
        drakeSystemAnalysis drakeSystemFramework drakeCommon
        drakeSpringMassSystemPlant)
target_link_libraries(runge_kutta2_integrator_test
        drakeSystemAnalysis drakeSystemFramework drakeCommon
        drakeSpringMassSystemPlant)
target_link_libraries(runge_kutta3_integrator_test
        drakeSystemAnalysis drakeSystemFramework drakeCommon
        drakeSpringMassSystemPlant)
