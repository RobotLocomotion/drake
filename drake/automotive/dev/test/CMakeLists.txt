drake_add_cc_test(endless_road_car_test)
target_link_libraries(endless_road_car_test
                      drakeAutomotive)

drake_add_cc_test(endless_road_oracle_test)
target_link_libraries(endless_road_oracle_test
                      drakeAutomotive)
