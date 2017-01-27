add_library_with_exports(LIB_NAME drakeRendering SOURCE_FILES
    pose_aggregator.cc
    pose_bundle.cc)
target_link_libraries(drakeRendering
    drakeRBM
    drakeSystemFramework)

drake_install_libraries(drakeRendering)
drake_install_headers(
    pose_aggregator.h
    pose_bundle.h)
drake_install_pkg_config_file(drake-rendering
    TARGET drakeRendering
    LIBS -ldrakeRendering)

if(BUILD_TESTING)
    add_subdirectory(test)
endif()
