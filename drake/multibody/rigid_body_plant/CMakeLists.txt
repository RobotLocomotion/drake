set(source_files
    contact_detail.cc
    contact_force.cc
    contact_info.cc
    contact_resultant_force_calculator.cc
    contact_results.cc
    kinematics_results.cc
    point_contact_detail.cc
    rigid_body_plant.cc)

# The following variables are necessary to conditionally include
# DrakeVisualizer and ViewerDrawTranslator in libdrakeRigidBodyPlant.so if LCM
# exists.
set(lcm_dependent_header_files)
if (lcm_FOUND)
  set(lcm_dependent_header_files
    contact_results_to_lcm.h
    drake_visualizer.h
    rigid_body_plant_that_publishes_xdot.h
    viewer_draw_translator.h)
endif ()

set(lcm_dependent_source_files)
if (lcm_FOUND)
  set(lcm_dependent_source_files
    contact_results_to_lcm.cc
    drake_visualizer.cc
    rigid_body_plant_that_publishes_xdot.cc
    viewer_draw_translator.cc)
endif ()

set(lcm_dependent_link_libraries)
set(lcm_dependent_requires)
if (lcm_FOUND)
  set(lcm_dependent_link_libraries
    drakeLcmSystem
    drakeSystemPrimitives
    drakeRenderingLcm)
  set(lcm_dependent_requires drake-lcm-system2 drake-rendering-lcm)
endif ()

add_library_with_exports(LIB_NAME drakeRigidBodyPlant SOURCE_FILES
  ${lcm_dependent_source_files}
  ${source_files})
target_link_libraries(drakeRigidBodyPlant
  drakeCommon
  drakeOptimization
  drakeRBM
  drakeSystemFramework
  Eigen3::Eigen
  ${lcm_dependent_link_libraries})

drake_install_headers(
    ${lcm_dependent_header_files}
    contact_detail.h
    contact_force.h
    contact_info.h
    contact_results.h
    kinematics_results.h
    rigid_body_plant.h)

drake_install_libraries(drakeRigidBodyPlant)
drake_install_pkg_config_file(drake-rigid-body-plant
  TARGET drakeRigidBodyPlant
  CFLAGS -I${CMAKE_INSTALL_PREFIX}/include
  LIBS -ldrakeRigidBodyPlant
  REQUIRES
    ${lcm_dependent_requires}
    drake-common
    drake-optimization
    drake-rbm
    drake-system-framework
    eigen3)

if(BUILD_TESTING)
  add_subdirectory(test)
endif()
