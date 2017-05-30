add_library_with_exports(LIB_NAME drakeRendering SOURCE_FILES
    frame_velocity.cc
    pose_aggregator.cc
    pose_bundle.cc
    pose_vector.cc)
target_link_libraries(drakeRendering
    drakeRBM
    drakeSystemFramework)

drake_install_libraries(drakeRendering)
drake_install_headers(
    frame_velocity.h
    pose_aggregator.h
    pose_bundle.h
    pose_vector.h)
drake_install_pkg_config_file(drake-rendering
    TARGET drakeRendering
    LIBS -ldrakeRendering
    REQUIRES
        drake-rbm
        drake-system-framework
)

if(lcm_FOUND)
  add_library_with_exports(LIB_NAME drakeRenderingLcm SOURCE_FILES
      drake_visualizer_client.cc
      pose_bundle_to_draw_message.cc
      pose_stamped_t_pose_vector_translator.cc)
  drake_install_libraries(drakeRenderingLcm)
  target_link_libraries(drakeRenderingLcm
      drakeLcmSystem
      drakeLCMTypes
      drakeRendering
      drakeShapes
      drakeSystemFramework
      robotlocomotion-lcmtypes-cpp)

  drake_install_headers(
      drake_visualizer_client.h
      pose_bundle_to_draw_message.h
      pose_stamped_t_pose_vector_translator.h)
  drake_install_pkg_config_file(drake-rendering-lcm
      TARGET drakeRenderingLcm
      LIBS -ldrakeRenderingLcm
      REQUIRES
          drakeLcmSystem
          drake_lcmtypes
          drake-shapes
          robotlocomotion-lcmtypes-cpp
  )
endif()

if(BUILD_TESTING)
    add_subdirectory(test)
endif()
