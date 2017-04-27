if(BUILD_TESTING)
  add_subdirectory(test)
endif()

add_library_with_exports(LIB_NAME drakeBeadOnAWire SOURCE_FILES
  bead_on_a_wire.cc
  )
target_link_libraries(drakeBeadOnAWire
  drakeCommon
  drakeSystemFramework
  )
drake_install_libraries(drakeBeadOnAWire)
drake_install_headers(
  bead_on_a_wire-inl.h
  bead_on_a_wire.h
  )
drake_install_pkg_config_file(drake-bead_on_a_wire
  TARGET drakeBeadOnAWire
  LIBS -ldrakeBeadOnAWire
  REQUIRES
    drake-common
    drake-system-framework)

