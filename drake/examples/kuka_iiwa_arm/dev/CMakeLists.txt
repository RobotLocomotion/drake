add_library_with_exports(LIB_NAME drakeKukaIiwaArmIK SOURCE_FILES
    iiwa_ik_planner.cc)
target_link_libraries(drakeKukaIiwaArmIK
    drakeCommon
    drakeIK
    drakeRBM
    drakeMultibodyParsers
    drakeTrajectories)
drake_install_libraries(drakeKukaIiwaArmIK)
drake_install_headers(
    iiwa_ik_planner.h)
drake_install_pkg_config_file(drake-kuka-iiwa-arm-ik
    TARGET drakeKukaIiwaArmIK
    LIBS -ldrakeKukaIiwaArmIK
    REQUIRES drake-lcm-system drake-lcmtypes-cpp drake-rbm)

add_subdirectory(pick_and_place)
add_subdirectory(tools)

if(BUILD_TESTING)
  add_subdirectory(test)
endif()
