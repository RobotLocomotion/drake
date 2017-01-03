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
    Eigen3::Eigen
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
drake_install_pkg_config_file(drake-multibody-parsers
    TARGET drakeMultibodyParsers
    LIBS -ldrakeMultibodyParsers
    REQUIRES
        drake-rbm
        eigen3
        spruce
        tinyxml2)

if (yaml-cpp_FOUND)
  add_library_with_exports(LIB_NAME drakeRigidBodyAliasGroups SOURCE_FILES
    rigid_body_tree_alias_groups.cc)
  target_link_libraries(drakeRigidBodyAliasGroups
    drakeRBM
    yaml-cpp)
  drake_install_libraries(drakeRigidBodyAliasGroups)
  drake_install_headers(rigid_body_tree_alias_groups.h)
  drake_install_pkg_config_file(drake-rigidbody-kinematic-property
    TARGET drakeRigidBodyAliasGroups
    LIBS -ldrakeRigidBodyAliasGroups
    REQUIRES
        drakeRBM
        yaml-cpp)
endif()

if(BUILD_TESTING)
  add_subdirectory(test)
endif()
