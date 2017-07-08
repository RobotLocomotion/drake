drake_add_cc_test(NAME package_map_test
    WORKING_DIRECTORY "${PROJECT_BINARY_DIR}/../")
target_link_libraries(package_map_test drakeMultibodyParsers)
