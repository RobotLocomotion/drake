# This directory has no library targets. All the source code here is part
# drakeSystemFramework, which is declared in systems/framework.

drake_install_headers(
  adder.h
  constant_value_source.h
  constant_value_source-inl.h
  constant_vector_source.h
  demultiplexer.h
  gain.h
  gain-inl.h
  integrator.h
  pass_through.h
  pass_through-inl.h
  pid_controller.h)

add_subdirectory(test)
