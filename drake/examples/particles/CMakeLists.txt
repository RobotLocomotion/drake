add_library_with_exports(LIB_NAME drakeParticles SOURCE_FILES
  particle.cc
  utilities.cc
  )
target_link_libraries(drakeParticles
  drakeCommon
  drakeSystemPrimitives
  drakeSystemFramework
  )
drake_install_libraries(
  drakeParticles
  )
drake_install_headers(
  particle.h
  utilities.h
  )
drake_install_pkg_config_file(drake-particles
  TARGET drakeParticles
  LIBS -ldrakeParticles
  REQUIRES
    drake-common
    drake-system-framework
  )

if(lcm_FOUND)
  add_executable(uniformly_accelerated_particle_demo uniformly_accelerated_particle.cc)
  target_link_libraries(uniformly_accelerated_particle_demo
    drakeParticles
    drakeRBM
    drakeLcmSystem
    drakeRigidBodyPlant
    drakeSystemAnalysis
    drakeSystemPrimitives
    drakeMultibodyParsers
    drakeRendering
    )
  drake_install_executables(TARGETS uniformly_accelerated_particle_demo)
endif()

if(BUILD_TESTING)
  add_subdirectory(test)
endif()
