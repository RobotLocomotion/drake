drake_add_cc_test(package_map_test)
# TODO(liang.fok) package_map_test should link against a parser-specific library
# once it exists.
target_link_libraries(package_map_test drakeRBM)
