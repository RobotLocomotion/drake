if(BUILD_TESTING)
  add_subdirectory(test)
endif()

add_library_with_exports(LIB_NAME drakeParticles SOURCE_FILES
  particle.cc
  degenerate_euler_joint.cc
  )
target_link_libraries(drakeParticles
  drakeCommon
  drakeSystemFramework
  )
drake_install_libraries(
  drakeParticles
  )
drake_install_headers(
  particle.h
  degenerate_euler_joint.h
  )
drake_install_pkg_config_file(drake-particles
  TARGET drakeParticles
  LIBS -ldrakeParticles
  REQUIRES
    drake-common
    drake-system-framework
  )

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
