add_library_with_exports(LIB_NAME drakeSystemFramework
                         SOURCE_FILES value.cc value.h)
pods_install_libraries(drakeSystemFramework)
drake_install_headers(value.h)
pods_install_pkg_config_file(drake-system-framework
    LIBS -ldrakeSystemFramework
    REQUIRES
    VERSION 0.0.1)

add_subdirectory(test)
