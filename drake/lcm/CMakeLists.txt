# Creates lists of header and source files that should ways be included.
set(source_files
    drake_lcm_mock.cc
    lcm_receive_thread.cc)
set(header_files
    drake_lcm_interface.h
    drake_lcm_mock.h
    lcm_receive_thread.h)

# Creates lists of header files, source files, and libraries that should only be
# included if LCM is present.
set(lcm_dependent_header_files)
if (lcm_FOUND)
  set(lcm_dependent_header_files
      drake_lcm.h)
endif()

set(lcm_dependent_source_files)
if (lcm_FOUND)
  set(lcm_dependent_source_files
      drake_lcm.cc)
endif()

set(lcm_dependent_link_libraries)
if (lcm_FOUND)
  set(lcm_dependent_link_libraries
      lcm)
endif()

add_library_with_exports(LIB_NAME drakeLcm SOURCE_FILES
    ${source_files}
    ${lcm_dependent_source_files})
target_link_libraries(drakeLcm
    ${lcm_dependent_link_libraries}
    drakeCommon
    drakeSystemFramework
    drakeRBM
    drakeOptimization)

drake_install_headers(
    ${header_files}
    ${lcm_dependent_header_files})
pods_install_libraries(drakeLcm)
pods_install_pkg_config_file(drake-lcm
    CFLAGS -I${CMAKE_INSTALL_PREFIX}/include
    LIBS -ldrakeLcm
    VERSION 0.0.1)

add_subdirectory(test)
