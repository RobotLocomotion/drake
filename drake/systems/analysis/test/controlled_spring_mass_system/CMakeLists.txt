add_library_with_exports(LIB_NAME drakeControlledSpringMassSystemPlant SOURCE_FILES
    controlled_spring_mass_system.cc)
target_link_libraries(drakeControlledSpringMassSystemPlant
    drakeSpringMassSystemPlant
    drakeSystemControllers
    drakeSystemFramework)
pods_install_libraries(drakeControlledSpringMassSystemPlant)
drake_install_headers(controlled_spring_mass_system.h)

pods_install_pkg_config_file(drake-controlled-spring-mass-system-plant
    LIBS -ldrakeControlledSpringMassSystemPlant -ldrakeSpringMassSystemPlant -drakeSystemControllers -ldrakeSystemFramework
    REQUIRES
    VERSION 0.0.1)

drake_add_cc_test(controlled_spring_mass_system_test)
target_link_libraries(controlled_spring_mass_system_test drakeControlledSpringMassSystemPlant drakeSystemFramework)
