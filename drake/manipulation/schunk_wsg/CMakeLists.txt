if(lcm_FOUND)
  add_library_with_exports(LIB_NAME drakeSchunkWsg SOURCE_FILES
    gen/schunk_wsg_trajectory_generator_state_vector.cc
    schunk_wsg_lcm.cc)
  target_link_libraries(drakeSchunkWsg
    drakeLcmSystem
    drakeMultibodyParsers)
  drake_install_libraries(drakeSchunkWsg)
  drake_install_headers(
    # N.B. The gen/*.h headers are installed by gen/CMakeLists.txt.
    schunk_wsg_lcm.cc)
  drake_install_pkg_config_file(drake-schunk-wsg
    TARGET drakeSchunkWsg
    LIBS -ldrakeSchunkWsg
    REQUIRES
      drake-lcm-system
      drake-lcmtypes-cpp
      drake-multibody-parsers)
endif()

add_subdirectory(gen)

if(BUILD_TESTING)
  add_subdirectory(test)
endif()
