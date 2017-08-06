drake_install_headers(
    spatial_acceleration.h
    spatial_force.h
    spatial_velocity.h
)

if(BUILD_TESTING)
  add_subdirectory(test)
endif()
