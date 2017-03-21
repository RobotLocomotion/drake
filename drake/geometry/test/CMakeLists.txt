# Build tests for geometry

drake_add_cc_test(identifier_test identifier_test.cc)
target_link_libraries(identifier_test
    drakeCommon
    )
