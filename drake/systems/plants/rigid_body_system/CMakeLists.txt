set(source_files rigid_body_system.cc)

add_library_with_exports(LIB_NAME drake_rbs SOURCE_FILES ${source_files})
target_link_libraries(drake_rbs drakeRBM drakeSystemFramework)

drake_install_headers(rigid_body_system.h)
pods_install_libraries(drake_rbs)
pods_install_pkg_config_file(drake-rbs
        CFLAGS -I${CMAKE_INSTALL_PREFIX}/include
        LIBS -ldrake_rbs -ldrakeRBM
        VERSION 0.0.1)

add_subdirectory(test)
