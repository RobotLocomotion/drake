set(source_files kinematics_results.cc rigid_body_plant.cc)

add_library_with_exports(LIB_NAME drakeRigidBodyPlant
        SOURCE_FILES ${source_files})
target_link_libraries(drakeRigidBodyPlant
        drakeCommon drakeSystemFramework drakeRBM drakeOptimization)

drake_install_headers(kinematics_results.h rigid_body_plant.h)
pods_install_libraries(drakeRigidBodyPlant)
pods_install_pkg_config_file(drake-rbs
        CFLAGS -I${CMAKE_INSTALL_PREFIX}/include
        LIBS -ldrakeRigidBodyPlant -ldrakeRBM
        VERSION 0.0.1)

add_subdirectory(test)
