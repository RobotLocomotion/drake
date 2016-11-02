# Creates lists of header files, source files, and libraries that should always
# be included regardless of whether LCM is present.
set(lcm_independent_header_files
    drake_lcm_interface.h
    drake_mock_lcm.h)

set(lcm_independent_source_files
    drake_mock_lcm.cc)

set(lcm_independent_link_libraries
    drakeCommon)

# Creates lists of header files, source files, and libraries that should only be
# included if LCM is present.
set(lcm_dependent_header_files)
if (lcm_FOUND)
  set(lcm_dependent_header_files
      drake_lcm.h
      lcm_receive_thread.h
      lcmt_drake_signal_utils.h)
endif()

set(lcm_dependent_source_files)
if (lcm_FOUND)
  set(lcm_dependent_source_files
      drake_lcm.cc
      lcm_receive_thread.cc
      lcmt_drake_signal_utils.cc)
endif()

set(lcm_dependent_link_libraries)
if (lcm_FOUND)
  set(lcm_dependent_link_libraries
      ${CMAKE_THREAD_LIBS_INIT}
      drakeLCMTypes
      lcm)
endif()

add_library_with_exports(LIB_NAME drakeLcm SOURCE_FILES
    ${lcm_dependent_source_files}
    ${lcm_independent_source_files})

target_link_libraries(drakeLcm
    ${lcm_dependent_link_libraries}
    ${lcm_independent_link_libraries})

if (lcm_FOUND AND WIN32)
  target_link_libraries(drakeLcm ws2_32)
endif()

drake_install_headers(
    ${lcm_dependent_header_files}
    ${lcm_independent_header_files})
pods_install_libraries(drakeLcm)
pods_install_pkg_config_file(drake-lcm
    CFLAGS -I${CMAKE_INSTALL_PREFIX}/include
    LIBS -ldrakeLcm
    VERSION 0.0.1)

add_subdirectory(test)
