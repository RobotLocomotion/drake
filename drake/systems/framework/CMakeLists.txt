add_library_with_exports(LIB_NAME drakeSystemFramework
    SOURCE_FILES value.cc value.h
                 vector_interface.cc vector_interface.h
                 basic_vector.cc basic_vector.h
                 named_value_vector.cc named_value_vector.h
                 cache.cc cache.h
                 context.cc context.h
                 system_output.cc system_output.h
                 system_interface.cc system_interface.h
                 primitives/adder.cc primitives/adder.h)

drake_install_headers(value.h
                      vector_interface.h
                      basic_vector.h
                      named_value_vector.h
                      cache.h
                      context.h
                      system_output.h
                      system_interface.h
                      primitives/adder.h)

pods_install_libraries(drakeSystemFramework)
pods_install_pkg_config_file(drake-system-framework
    LIBS -ldrakeSystemFramework
    REQUIRES
    VERSION 0.0.1)

add_subdirectory(test)
add_subdirectory(primitives/test)
