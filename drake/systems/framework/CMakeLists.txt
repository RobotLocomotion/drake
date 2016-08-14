# Source files used to build drakeSystemFramework.
set(sources
  context_base.cc
  basic_vector.cc
  cache.cc
  context.cc
  diagram.cc
  diagram_context.cc
  state.cc
  state_subvector.cc
  state_supervector.cc
  state_vector.cc
  system.cc
  system_input.cc
  system_output.cc
  basic_state_vector.cc
  value.cc
  vector_interface.cc
  diagram.cc
  primitives/adder.cc
  primitives/constant_vector_source.cc
  primitives/integrator.cc)

# Headers that should be installed with Drake so that they
# are available elsewhere via #include.
#
# TODO(david-german-tri): Some of these headers are implementation details that
# should never be directly included outside of systems/framework. They ought to
# be private, but we can't make them private yet because they are included in
# API headers. (For example, state_subvector.h is used in state.h.) Once the
# System2 framework template implementations have been moved into .cc files
# using explicit instantiation, tighten this list.
set(installed_headers
  context_base.h
  basic_vector.h
  cache.h
  context.h
  diagram.h
  diagram_context.h
  leaf_state_vector.h
  state.h
  state_subvector.h
  state_supervector.h
  state_vector.h
  system.h
  system_input.h
  system_output.h
  basic_state_vector.h
  value.h
  vector_interface.h
  diagram.h
  primitives/adder.h
  primitives/constant_vector_source.h
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
