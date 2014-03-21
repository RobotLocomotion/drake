
include_directories ( .. )

pods_find_pkg_config(bot2-vis)
if (${bot2-vis_FOUND} AND ${Boost_FOUND} )
#   add_lcmtype(lcmt_viewer_command.lcm)
#   add_lcmtype(lcmt_viewer_draw.lcm)
#   add_lcmtype(lcmt_viewer_geometry_data.lcm)
#   add_lcmtype(lcmt_viewer_link_data.lcm)
#   add_lcmtype(lcmt_viewer_load_robot.lcm)

   add_executable(drake_viewer main.cpp)
   pods_use_pkg_config_packages(drake_viewer bot2-lcmgl-renderer)
   pods_install_executables(drake_viewer)

   add_executable(lcmgl_test lcmgl_test.cpp)
   pods_use_pkg_config_packages(lcmgl_test bot2-lcmgl-client)

   add_library(drake_urdf_renderer drake_urdf_renderer.cpp)
   target_link_libraries(drake_urdf_renderer drake_urdf_renderer ${LCMTYPES_C_LIBRARY} ${Boost_LIBRARIES})
   pods_use_pkg_config_packages(drake_urdf_renderer bot2-lcmgl-renderer)
   pods_install_libraries(drake_urdf_renderer)
  
   target_link_libraries(drake_viewer drake_urdf_renderer)
endif()
