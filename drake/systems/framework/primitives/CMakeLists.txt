# This directory has no library targets. All the source code here is part
# drakeSystemFramework, which is declared in systems/framework.

drake_install_headers(
  adder.h
  affine_system.h
  constant_value_source.h
  constant_value_source-inl.h
  constant_vector_source.h
  demultiplexer.h
  gain.h
  gain-inl.h
  integrator.h
  linear_system.h
  matrix_gain.h
  multiplexer.h
  pass_through.h
  pass_through-inl.h
  trajectory_source.h
  zero_order_hold.h)

add_subdirectory(test)
