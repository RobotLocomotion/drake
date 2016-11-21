
add_library_with_exports(LIB_NAME drakeSensors SOURCE_FILES encoders.cc)
target_link_libraries(drakeSensors drakeSystemFramework)
pods_install_libraries(drakeSensors)
drake_install_headers(encoders.h)
drake_install_pkg_config_file(drake-sensors
    TARGET drakeSensors
    LIBS -ldrakeSensors)

if(BUILD_TESTING)
  add_subdirectory(test)
endif()
