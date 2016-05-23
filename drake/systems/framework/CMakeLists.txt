# Source files used to build drakeSystemFramework.
set(sources 
    basic_vector.cc
    cache.cc
    context.cc
    named_value_vector.cc
    state.cc
    system_interface.cc
    system_output.cc
    value.cc
    vector_interface.cc
    primitives/adder.cc)

# Headers that should be installed with Drake so that they
# are available elsewhere via #include.
set(installed_headers
    basic_vector.h
    cache.h
    context.h
    named_value_vector.h
    state.h
    system_interface.h
    system_output.h
    value.h
    vector_interface.h
    primitives/adder.h)

# Headers that are needed by code here but should not
# be exposed anywhere else.
set(private_headers 
    )

add_library_with_exports(LIB_NAME drakeSystemFramework
    SOURCE_FILES ${sources} ${installed_headers} ${private_headers})

drake_install_headers(${installed_headers})

pods_install_libraries(drakeSystemFramework)
pods_install_pkg_config_file(drake-system-framework
    LIBS -ldrakeSystemFramework
    REQUIRES
    VERSION 0.0.1)

add_subdirectory(test)
add_subdirectory(primitives/test)
