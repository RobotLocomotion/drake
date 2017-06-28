set(drakeRigidContact_SRC_FILES
    rigid_contact_solver.cc
    )

add_library_with_exports(LIB_NAME drakeRigidContact SOURCE_FILES ${drakeRigidContact_SRC_FILES})

target_link_libraries(drakeRigidContact
  drakeCommon
  drakeSystemFramework
  drakeOptimization
  Eigen3::Eigen)

drake_install_libraries(drakeRigidContact)
drake_install_headers(rigid_contact.h
    rigid_contact_problem_data.h 
    rigid_contact_solver.h 
    )
drake_install_pkg_config_file(drake-rigid-contact
  TARGET drakeRigidContact
  LIBS -ldrakeRigidContact
  REQUIRES
    drake-common
    drake-optimization
    drake-system-framework
    eigen3
    )

if(BUILD_TESTING)
  add_subdirectory(test)
endif()
