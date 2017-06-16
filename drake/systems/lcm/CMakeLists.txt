if(lcm_FOUND)
  add_library_with_exports(LIB_NAME drakeLcmSystem SOURCE_FILES
    lcm_and_vector_base_translator.cc
    lcm_publisher_system.cc
    lcm_subscriber_system.cc
    lcm_translator_dictionary.cc
    lcmt_drake_signal_translator.cc
    serializer.cc)
  target_link_libraries(drakeLcmSystem
    drakeSystemFramework
    drakeLCMTypes
    drakeLcm
    Threads::Threads)

  drake_install_libraries(drakeLcmSystem)
  drake_install_headers(
    lcm_publisher_system.h
    lcm_subscriber_system.h
    lcm_translator_dictionary.h
    lcm_and_vector_base_translator.h
    lcmt_drake_signal_translator.h
    serializer.h
    translator_system.h)
  drake_install_pkg_config_file(drake-lcm-system2
    TARGET drakeLcmSystem
    LIBS -ldrakeLcmSystem ${CMAKE_THREAD_LIBS_INIT}
    REQUIRES
      drake-lcm
      drake-lcmtypes-cpp
      drake-system-framework)

  add_library_with_exports(LIB_NAME drakeLcmDrivenLoop SOURCE_FILES
    lcm_driven_loop.cc)
  target_link_libraries(drakeLcmDrivenLoop
    drakeLcmSystem
    drakeSystemAnalysis)

  drake_install_libraries(drakeLcmDrivenLoop)
  drake_install_headers(
    lcm_driven_loop.h)
  drake_install_pkg_config_file(drake-lcm-driven-loop
    TARGET drakeLcmDrivenLoop
    LIBS -ldrakeLcmDrivenLoop ${CMAKE_THREAD_LIBS_INIT}
    REQUIRES
      drake-lcm-system2)
endif()

if(BUILD_TESTING)
  add_subdirectory(test)
endif()
