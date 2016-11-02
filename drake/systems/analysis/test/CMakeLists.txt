add_subdirectory(controlled_spring_mass_system)

add_executable(simulator_test simulator_test.cc)
add_executable(explicit_euler_integrator_test explicit_euler_integrator_test.cc)
add_executable(runge_kutta2_integrator_test runge_kutta2_integrator_test.cc)
target_link_libraries(simulator_test
  drakeSystemAnalysis drakeSystemFramework drakeCommon
        drakeSpringMassSystemPlant drakeControlledSpringMassSystemPlant
        GTest::GTest GTest::Main)
target_link_libraries(explicit_euler_integrator_test
        drakeSystemAnalysis drakeSystemFramework drakeCommon
        drakeSpringMassSystemPlant GTest::GTest GTest::Main)
target_link_libraries(runge_kutta2_integrator_test
        drakeSystemAnalysis drakeSystemFramework drakeCommon
        drakeSpringMassSystemPlant GTest::GTest GTest::Main)
drake_add_test(NAME simulator_test COMMAND simulator_test)
drake_add_test(NAME explicit_euler_integrator_test COMMAND explicit_euler_integrator_test)
drake_add_test(NAME runge_kutta2_integrator_test COMMAND runge_kutta2_integrator_test)
