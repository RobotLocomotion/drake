# Source files used to build drakeMath.
set(sources
  autodiff.cc
  expmap.cc
  gradient.cc)

# Headers that should be installed with Drake so that they
# are available elsewhere via #include.
set(installed_headers
  autodiff.h
  expmap.h
  gradient.h)

# Headers that are needed by code here but should not
# be exposed anywhere else.
set(private_headers)

add_library_with_exports(LIB_NAME drakeMath
  SOURCE_FILES ${sources} ${installed_headers} ${private_headers})
target_link_libraries(drakeMath drakeCommon)

drake_install_headers(${installed_headers})

pods_install_libraries(drakeMath)
pods_install_pkg_config_file(drake-math
  LIBS -ldrakeMath -ldrakeCommon
  REQUIRES
  VERSION 0.0.1)

add_subdirectory(test)
