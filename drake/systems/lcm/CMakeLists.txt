if(lcm_FOUND)
  add_library_with_exports(LIB_NAME drakeLCMSystem2 SOURCE_FILES
    lcm_and_vector_base_translator.cc
    lcm_publisher_system.cc
    lcm_receive_thread.cc
    lcm_subscriber_system.cc
    lcm_translator_dictionary.cc
    translator_between_lcmt_drake_signal.cc)
  add_dependencies(drakeLCMSystem2 drake_lcmtypes_hpp)
  target_link_libraries(drakeLCMSystem2
    drakeSystemFramework
    lcm
    ${CMAKE_THREAD_LIBS_INIT})
  if(WIN32)
    target_link_libraries(drakeLCMSystem2 ws2_32)
  endif()

  pods_install_libraries(drakeLCMSystem2)
  drake_install_headers(
    lcm_receive_thread.h
    lcm_publisher_system.h
    lcm_subscriber_system.h
    lcm_translator_dictionary.h
    lcm_and_vector_base_translator.h
    translator_between_lcmt_drake_signal.h)
  pods_install_pkg_config_file(drake-lcm-system2
    LIBS -ldrakeLCMSystem2
    REQUIRES lcm
    VERSION 0.0.1)
endif()

add_subdirectory(test)
