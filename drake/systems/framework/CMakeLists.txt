# Source files used to build drakeSystemFramework.
set(sources
  context_base.cc
  basic_vector.cc
  cache.cc
  context.cc
  continuous_system.cc
  diagram.cc
  diagram_context.cc
  state.cc
  state_subvector.cc
  state_vector.cc
  system_interface.cc
  system_input.cc
  system_output.cc
  basic_state_vector.cc
  value.cc
  vector_interface.cc
  diagram.cc
  primitives/adder.cc
  primitives/integrator.cc)

# Headers that should be installed with Drake so that they
# are available elsewhere via #include.
set(installed_headers
  context_base.h
  basic_vector.h
  cache.h
  context.h
  continuous_system.h
  continuous_system_interface.h
  diagram.h
  diagram_context.h
  state.h
  state_subvector.h
  state_vector.h
  system_interface.h
  system_input.h
  system_output.h
  basic_state_vector.h
  value.h
  vector_interface.h
  diagram.h
  primitives/adder.h
  primitives/integrator.h)

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
