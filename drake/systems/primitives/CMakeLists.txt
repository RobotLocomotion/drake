# Source files used to build drakeSystemPrimitives.
set(sources
    adder.cc
    affine_system.cc
    constant_value_source.cc
    constant_vector_source.cc
    demultiplexer.cc
    first_order_low_pass_filter.cc
    gain.cc
    integrator.cc
    linear_system.cc
    matrix_gain.cc
    multiplexer.cc
    pass_through.cc
    saturation.cc
    signal_log.cc
    signal_logger.cc
    trajectory_source.cc
    zero_order_hold.cc)

drake_install_headers(
  adder.h
  affine_system.h
  constant_value_source.h
  constant_value_source-inl.h
  constant_vector_source.h
  demultiplexer.h
  first_order_low_pass_filter.h
  gain.h
  gain-inl.h
  integrator.h
  linear_system.h
  matrix_gain.h
  multiplexer.h
  pass_through.h
  pass_through-inl.h
  random_source.h
  saturation.h
  signal_log.h
  signal_logger.h
  trajectory_source.h
  zero_order_hold.h
  zero_order_hold-inl.h
  )

# Headers that are needed by code here but should not
# be exposed anywhere else.
set(private_headers)

add_library_with_exports(LIB_NAME drakeSystemPrimitives
    SOURCE_FILES ${sources} ${private_headers})
target_link_libraries(drakeSystemPrimitives drakeSystemFramework drakeCommon)

drake_install_libraries(drakeSystemPrimitives)
drake_install_pkg_config_file(drake-system-primitives
    TARGET drakeSystemPrimitives
    LIBS -ldrakeSystemPrimitives
    REQUIRES
    drake-common
    drake-trajectories
    eigen3)

if(BUILD_TESTING)
  add_subdirectory(test)
endif()
