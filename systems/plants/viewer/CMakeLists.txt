
include_directories ( .. )

pods_find_pkg_config(bot2-vis)
if (bot2-vis_FOUND) 

  add_executable(drake_viewer main.cpp)
  pods_use_pkg_config_packages(drake_viewer bot2-lcmgl-renderer)
  pods_install_executables(drake_viewer)

   add_executable(lcmgl_test lcmgl_test.cpp)
   pods_use_pkg_config_packages(lcmgl_test bot2-lcmgl-client)

   add_library(drake_urdf_renderer drake_urdf_renderer.cpp)
   target_link_libraries(drake_viewer drake_urdf_renderer lcmtypes_drake URDFRigidBodyManipulator)

endif()
