add_executable(parser_common_test
    parser_common_test.cc)
target_link_libraries(parser_common_test
    drakeMultibodyParsers
    GTest::GTest GTest::Main)
drake_add_test(
    NAME parser_common_test
    COMMAND parser_common_test
    WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}/../")

