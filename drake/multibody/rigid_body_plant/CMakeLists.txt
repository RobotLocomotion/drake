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
    drake_visualizer.h
    viewer_draw_translator.h)
endif ()

set(lcm_dependent_source_files)
if (lcm_FOUND)
  set(lcm_dependent_source_files
    drake_visualizer.cc
    viewer_draw_translator.cc)
endif ()

set(lcm_dependent_link_libraries)
set(lcm_dependent_requires)
if (lcm_FOUND)
  set(lcm_dependent_link_libraries drakeLCMSystem2)
  set(lcm_dependent_requires drake-lcm-system2)
endif ()

add_library_with_exports(LIB_NAME drakeRigidBodyPlant SOURCE_FILES
  ${lcm_dependent_source_files}
  ${source_files})
target_link_libraries(drakeRigidBodyPlant
  ${lcm_dependent_link_libraries}
  drakeCommon
  drakeSystemFramework
  drakeRBM
  drakeOptimization)

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
