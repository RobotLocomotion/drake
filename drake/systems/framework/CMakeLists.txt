# Source files used to build drakeSystemFramework.
set(shared_sources
  basic_vector.cc
  value.cc
  vector_interface.cc
)

set(system2_sources
  basic_state_vector.cc
  continuous_system.cc
  state.cc
  state_subvector.cc
  state_vector.cc
  system_input.cc
  system_interface.cc
  system_output.cc
  primitives/adder.cc
)

set(system3_sources
  cache3.cc
  context3.cc
  system3.cc
  primitives/adder3.cc
  primitives/gain3.cc
)

set(sources
  ${shared_sources}
  ${system2_sources}
  ${system3_sources}
)

# Headers that should be installed with Drake so that they
# are available elsewhere via #include.
set(shared_installed_headers
  basic_vector.h
  value.h
  vector_interface.h
)

set(system2_installed_headers
  basic_state_vector.h
  cache.h
  context.h
  continuous_system.h
  continuous_system_interface.h
  state.h
  state_subvector.h
  state_vector.h
  system_input.h
  system_interface.h
  system_output.h
  primitives/adder.h
)

set(system3_installed_headers
  cache3.h
  context3.h
  system3.h
  system3_input.h
  system3_output.h
  value_listener3.h
  primitives/adder3.h
  primitives/cascade3.h
  primitives/diagram3.h
  primitives/gain3.h
  primitives/vector_constant3.h
)

set(installed_headers
  ${shared_installed_headers}
  ${system2_installed_headers}
  ${system3_installed_headers}
)

# Headers that are needed by code here but should not
# be exposed anywhere else.
set(private_headers)

add_library_with_exports(LIB_NAME drakeSystemFramework
  SOURCE_FILES ${sources} ${installed_headers} ${private_headers})
target_link_libraries(drakeSystemFramework drakeCommon)

drake_install_headers(${installed_headers})

pods_install_libraries(drakeSystemFramework)
pods_install_pkg_config_file(drake-system-framework
  LIBS -ldrakeSystemFramework -ldrakeCommon
  REQUIRES
  VERSION 0.0.1)

add_subdirectory(test)
add_subdirectory(primitives/test)
