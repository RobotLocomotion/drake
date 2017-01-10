drake_add_cc_test(NAME parser_common_test
    WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}/../")
target_link_libraries(parser_common_test drakeMultibodyParsers)
