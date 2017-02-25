drake_add_cc_test(endless_road_car_test)
target_link_libraries(endless_road_car_test
                      drakeAutomotive)

drake_add_cc_test(endless_road_car_to_euler_floating_joint_test)
target_link_libraries(endless_road_car_to_euler_floating_joint_test
                      drakeAutomotive)

drake_add_cc_test(endless_road_oracle_test)
target_link_libraries(endless_road_oracle_test
                      drakeAutomotive)

drake_add_cc_test(infinite_circuit_road_test)
target_link_libraries(infinite_circuit_road_test drakeAutomotive)
