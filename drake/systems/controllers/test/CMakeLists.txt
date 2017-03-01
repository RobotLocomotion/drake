drake_add_cc_test(inverse_dynamics_test)
target_link_libraries(inverse_dynamics_test
    drakeSystemControllers
    drakeSystemFramework
    drakeRBSystem)

drake_add_cc_test(inverse_dynamics_controller_test)
target_link_libraries(inverse_dynamics_controller_test
    drakeSystemControllers
    drakeSystemFramework
    drakeRBSystem)

drake_add_cc_test(pid_controlled_system_test)
target_link_libraries(pid_controlled_system_test
    drakeSystemControllers
    drakeSystemFramework)

drake_add_cc_test(pid_controller_test)
target_link_libraries(pid_controller_test drakeSystemControllers drakeSystemFramework)

drake_add_cc_test(linear_quadratic_regulator_test)
target_link_libraries(linear_quadratic_regulator_test drakeLinearControl)
