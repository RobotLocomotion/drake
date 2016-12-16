if(lcm_FOUND AND WITH_HSRB_DESCRIPTION)
  # Defines a shared library for use by Drake demo1 of the Toyota Human Support
  # Robot revision B (HSRb).
  add_library_with_exports(LIB_NAME drakeHsrb SOURCE_FILES
      hsrb_diagram_factories.cc)
  target_link_libraries(drakeHsrb
      drakeLcm
      drakeMultibodyParsers
      drakeRBM
      drakeRigidBodyPlant
      drakeSystemFramework
      drakeSystemPrimitives)
  drake_install_libraries(drakeHsrb)
  drake_install_headers(
      hsrb_diagram_factories.h)
  drake_install_pkg_config_file(drake-toyota-hsrb
      TARGET drakeHsrb
      LIBS -ldrakeHsrb
      REQUIRES
        drake-lcm
        drake-multibody-parsers
        drake-rbm
        drake-rigid-body-plant
        drake-system-framework
        drake-system-primitives)
endif()

if(BUILD_TESTING)
  add_subdirectory(test)
endif()
