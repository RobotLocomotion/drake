add_library_with_exports(LIB_NAME drakeSpringMassSystemPlant SOURCE_FILES
    spring_mass_system.cc)
target_link_libraries(drakeSpringMassSystemPlant
    drakeSystemFramework)
drake_install_libraries(drakeSpringMassSystemPlant)
drake_install_headers(
    spring_mass_system.h)

drake_install_pkg_config_file(drake-spring-mass-system-plant
    TARGET drakeSpringMassSystemPlant
    LIBS -ldrakeSpringMassSystemPlant
    REQUIRES drake-system-framework)

if(BUILD_TESTING)
  add_subdirectory(test)
endif()
