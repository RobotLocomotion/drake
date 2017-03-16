
# List all source files used to build libdrakeGeometry.
set(sources
    )

# List headers that should be installed with Drake so that they
# are available elsewhere via #include "drake/geometry/xxx.h".
set(installed_headers
    type_safe_int_id.h
    )

# List headers that are needed by code here but should not
# be exposed anywhere else.
set(private_headers
    )

# Create the library target and note its dependencies.
add_library_with_exports(LIB_NAME drakeGeometry
    SOURCE_FILES ${sources} ${installed_headers} ${private_headers})

target_link_libraries(drakeGeometry
    drakeCommon
    )

if(BUILD_TESTING)
  add_subdirectory(test)
endif()