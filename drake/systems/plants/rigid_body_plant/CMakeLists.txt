set(source_files
    rigid_body_plant.cc
    rigid_body_tree_visualizer_lcm.cc
    viewer_draw_translator.cc)

add_library_with_exports(LIB_NAME drake_rbp SOURCE_FILES ${source_files})
target_link_libraries(drake_rbp
        drakeCommon
        drakeLCMSystem2
        drakeSystemFramework
        drakeRBM
        drakeOptimization)

drake_install_headers(
    rigid_body_plant.h
    rigid_body_tree_visualizer_lcm.h
    viewer_draw_translator.h)
pods_install_libraries(drake_rbp)
pods_install_pkg_config_file(drake-rbs
        CFLAGS -I${CMAKE_INSTALL_PREFIX}/include
        LIBS -ldrake_rbp -ldrakeRBM
        VERSION 0.0.1)

add_subdirectory(test)
