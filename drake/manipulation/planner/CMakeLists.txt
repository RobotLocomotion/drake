add_library_with_exports(LIB_NAME drakeManipulationPlanner SOURCE_FILES
    constraint_relaxing_ik.cc)
target_link_libraries(drakeManipulationPlanner
    drakeCommon
    drakeIK
    drakeRBM
    drakeMultibodyParsers
    drakeTrajectories)
drake_install_libraries(drakeManipulationPlanner)
drake_install_headers(
    constraint_relaxing_ik.h)
drake_install_pkg_config_file(drake-manipulation-planner
    TARGET drakeManipulationPlanner
    LIBS -ldrakeManipulationPlanner
    REQUIRES drake-rbm)

if(BUILD_TESTING)
  add_subdirectory(test)
endif()
