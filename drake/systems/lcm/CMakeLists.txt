if(lcm_FOUND)
  add_library_with_exports(LIB_NAME drakeLCMSystem2 SOURCE_FILES
    lcm_and_vector_base_translator.cc
    lcm_publisher_system.cc
    lcm_subscriber_system.cc
    lcm_translator_dictionary.cc
    lcmt_drake_signal_translator.cc
    serializer.cc)
  target_link_libraries(drakeLCMSystem2
    drakeSystemFramework
    drakeLCMTypes
    drakeLcm
    ${CMAKE_THREAD_LIBS_INIT})

  pods_install_libraries(drakeLCMSystem2)
  drake_install_headers(
    lcm_publisher_system.h
    lcm_subscriber_system.h
    lcm_translator_dictionary.h
    lcm_and_vector_base_translator.h
    lcmt_drake_signal_translator.h)
  pods_install_pkg_config_file(drake-lcm-system2
    LIBS -ldrakeLCMSystem2
    REQUIRES lcm
    VERSION 0.0.1)
endif()

add_subdirectory(test)
