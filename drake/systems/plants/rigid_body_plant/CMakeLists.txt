set(source_files kinematics_results.cc rigid_body_plant.cc)

# The following variables are necessary to conditionally include
# RigidBodyTreeLcmPublisher in libdrakeRigidBodyPlant.so if LCM exists.
set(lcm_dependent_header_files)
if (lcm_FOUND)
  set(lcm_dependent_header_files
      rigid_body_tree_lcm_publisher.h
      viewer_draw_translator.h)
endif()

set(lcm_dependent_source_files)
if (lcm_FOUND)
  set(lcm_dependent_source_files
      rigid_body_tree_lcm_publisher.cc
      viewer_draw_translator.cc)
endif()

set(lcm_dependent_link_libraries)
if (lcm_FOUND)
  set(lcm_dependent_link_libraries
      drakeLCMSystem2)
endif()

add_library_with_exports(LIB_NAME drakeRigidBodyPlant SOURCE_FILES
    ${source_files}
    ${lcm_dependent_source_files})
target_link_libraries(drakeRigidBodyPlant
    ${lcm_dependent_link_libraries}
    drakeCommon
    drakeSystemFramework
    drakeRBM
    drakeOptimization)

drake_install_headers(
    ${lcm_dependent_header_files}
    kinematics_results.h rigid_body_plant.h)
pods_install_libraries(drakeRigidBodyPlant)

add_subdirectory(test)
