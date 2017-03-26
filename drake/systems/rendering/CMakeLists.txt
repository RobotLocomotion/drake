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
      pose_bundle_to_draw_message.cc)

  drake_install_libraries(drakeRenderingLcm)
  target_link_libraries(drakeRenderingLcm
      drakeLCMTypes
      drakeRendering
      drakeShapes
      drakeSystemFramework)
  drake_install_headers(
      drake_visualizer_client.h
      pose_bundle_to_draw_message.h)
  drake_install_pkg_config_file(drake-rendering-lcm
      TARGET drakeRenderingLcm
      LIBS -ldrakeRenderingLcm
      REQUIRES
          drake_lcmtypes
          drake-shapes
  )
endif()

if(BUILD_TESTING)
    add_subdirectory(test)
endif()
