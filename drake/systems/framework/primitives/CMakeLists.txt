# This directory has no library targets. All the source code here is part
# drakeSystemFramework, which is declared in systems/framework.

drake_install_headers(
  adder.h
  constant_vector_source.h
  gain.h
  gain-inl.h
  integrator.h
  pass_through.h
  pass_through-inl.h)

add_subdirectory(test)
