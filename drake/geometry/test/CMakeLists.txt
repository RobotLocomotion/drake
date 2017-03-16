# Build tests for geometry

drake_add_cc_test(type_safe_int_id_test type_safe_int_id_test.cc)
target_link_libraries(type_safe_int_id_test
    drakeCommon
    )
