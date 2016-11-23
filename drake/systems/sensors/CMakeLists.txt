
add_library_with_exports(LIB_NAME drakeSensors SOURCE_FILES rotary_encoders.cc)
target_link_libraries(drakeSensors drakeSystemFramework)

drake_install_libraries(drakeSensors)
drake_install_headers(rotary_encoders.h)
drake_install_pkg_config_file(drake-sensors
    TARGET drakeSensors
    LIBS -ldrakeSensors)

if(BUILD_TESTING)
  add_subdirectory(test)
endif()
