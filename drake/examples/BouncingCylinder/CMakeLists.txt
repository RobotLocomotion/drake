pods_find_pkg_config(bot2-lcmgl-client)

if (LCM_FOUND AND bot2-lcmgl-client_FOUND)
  add_executable(bouncing_cylinder bouncing_cylinder.cc cylinder_system.cc)
  pods_use_pkg_config_packages(bouncing_cylinder lcm bot2-lcmgl-client)
  target_link_libraries(bouncing_cylinder
          drakeRBSystem drakeLCMSystem drake_lcmtypes)
  # add_test(NAME runAtlasDynamics COMMAND runAtlasDynamics 0.4)
endif()
