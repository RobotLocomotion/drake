add_library_with_exports(LIB_NAME drakeSpringMassSystemPlant SOURCE_FILES
    spring_mass_system.cc)
target_link_libraries(drakeSpringMassSystemPlant
    drakeSystemFramework)
pods_install_libraries(drakeSpringMassSystemPlant)
drake_install_headers(
    spring_mass_system.h)

pods_install_pkg_config_file(drake-spring-mass-system-plant
    LIBS -ldrakeSpringMassSystemPlant -ldrakeSystemFramework
    REQUIRES
    VERSION 0.0.1)
