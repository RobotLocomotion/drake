#
# Drake common library
#

# List all source files used to build libdrakeCommon.
set(sources
  drake_assert.cc
  nice_type_name.cc
  )

# List headers that should be installed with Drake so that they
# are available elsewhere via #include "drake/common/xxx.h".
set(installed_headers
  drake_assert.h
  drake_deprecated.h
  nice_type_name.h
  eigen_types.h
  )

# List headers that are needed by code here but should not
# be exposed anywhere else.
set(private_headers
  )

# Create the library target and note its dependencies. Also generates
# a header defining DRAKECOMMON_EXPORT for dealing with .dlls on Windows.
add_library_with_exports(LIB_NAME drakeCommon
  SOURCE_FILES ${sources} ${installed_headers} ${private_headers})

# Install all externally-visible headers.
drake_install_headers(${installed_headers})

# Install the library and note Drake's dependency on it.
pods_install_libraries(drakeCommon)
pods_install_pkg_config_file(drake-common
  LIBS -ldrakeCommon
  REQUIRES
  VERSION 0.0.1)

add_subdirectory(test)
