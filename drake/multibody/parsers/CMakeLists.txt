add_library_with_exports(LIB_NAME drakeMultibodyParsers SOURCE_FILES
    model_instance_id_table.cc
    package_map.cc
    parser_common.cc
    sdf_parser.cc
    urdf_parser.cc
    xml_util.cc)
target_link_libraries(drakeMultibodyParsers
    drakeCommon
    drakeRBM
    spruce
    tinyxml2)
drake_install_libraries(drakeMultibodyParsers)
drake_install_headers(
    model_instance_id_table.h
    package_map.h
    parser_common.h
    sdf_parser.h
    urdf_parser.h
    xml_util.h)
drake_install_pkg_config_file(drakeMultibodyParsers
    LIBS -ldrakeRBM -ltinyxml2 -lspruce
    REQUIRES
        drake-rbm)

if(BUILD_TESTING)
  add_subdirectory(test)
endif()
