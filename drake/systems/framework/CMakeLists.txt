add_library_with_exports(LIB_NAME drakeSystemFramework
    SOURCE_FILES basic_vector.cc
                 cache.cc
                 context.cc
                 named_value_vector.cc
                 state.cc
                 system_interface.cc
                 system_output.cc
                 value.cc
                 vector_interface.cc
                 primitives/adder.cc)

drake_install_headers(basic_vector.h
                      cache.h
                      context.h
                      named_value_vector.h
                      system_interface.h
                      system_output.h
                      value.h
                      vector_interface.h
                      primitives/adder.h)

pods_install_libraries(drakeSystemFramework)
pods_install_pkg_config_file(drake-system-framework
    LIBS -ldrakeSystemFramework
    REQUIRES
    VERSION 0.0.1)

add_subdirectory(test)
add_subdirectory(primitives/test)
