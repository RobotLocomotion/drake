# Source files used to build drakeSystemFramework.
set(sources
  basic_vector.cc
  cache.cc
  context.cc
  diagram.cc
  diagram_builder.cc
  diagram_context.cc
  examples/spring_mass_system.cc
  leaf_context.cc
  leaf_system.cc
  input_port_evaluator_interface.cc
  output_port_listener_interface.cc
  primitives/adder.cc
  primitives/constant_value_source.cc
  primitives/constant_vector_source.cc
  primitives/demultiplexer.cc
  primitives/gain.cc
  primitives/integrator.cc
  primitives/pass_through.cc
  primitives/pid_controller.cc
  state.cc
  state_subvector.cc
  state_supervector.cc
  system.cc
  system_input.cc
  system_output.cc
  system_port_descriptor.cc
  value.cc
  vector_base.cc)

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
  basic_vector.h
  cache.h
  context.h
  diagram.h
  diagram_builder.h
  diagram_context.h
  leaf_context.h
  leaf_system.h
  input_port_evaluator_interface.h
  output_port_listener_interface.h
  state.h
  state_subvector.h
  state_supervector.h
  system.h
  system_input.h
  system_output.h
  system_port_descriptor.h
  value.h
  vector_base.h)

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

add_subdirectory(examples)
add_subdirectory(primitives)
add_subdirectory(test)
