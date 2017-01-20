LIST(APPEND CMAKE_MODULE_PATH "${PROJECT_BINARY_DIR}/../install/lib/cmake/sdformat")
include(SDFormatConfig)

add_executable(urdf_kinematics_test urdf_kinematics_test.cc)
target_link_libraries(urdf_kinematics_test
    drakeMultibodyParsers
    drakeRBM)
if (Bullet_FOUND)
  add_executable(urdf_collision_test urdf_collision_test.cc)
  target_link_libraries(urdf_collision_test drakeMultibodyParsers drakeRBM)
endif ()

add_executable(urdf_manipulator_dynamics_test urdf_manipulator_dynamics_test.cc)
target_link_libraries(urdf_manipulator_dynamics_test
    drakeMultibodyParsers
    drakeRBM)

drake_add_cc_test(xml_util_test)
target_link_libraries(xml_util_test drakeMultibodyParsers)

if (sdformat_FOUND)
  drake_add_cc_test(sdformat_test)
  target_include_directories(sdformat_test PUBLIC ${SDFormat_INCLUDE_DIRS})
  target_link_libraries(sdformat_test sdformat)
endif ()

add_subdirectory(package_map_test)
add_subdirectory(parser_common_test)
add_subdirectory(parsers_frames_test)
add_subdirectory(urdf_parser_test)
