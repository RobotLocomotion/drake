# Assuming lcm_FOUND

set(lcm_dependent_link_libraries
    drakeLCMTypes
    lcm
    Threads::Threads)
set(lcm_dependent_libs ${CMAKE_THREAD_LIBS_INIT})
set(lcm_dependent_requires lcm)

add_library_with_exports(LIB_NAME robotlocomotionTranslators SOURCE_FILES
    point_t_translator.cc
    pose_t_translator.cc
    quaternion_t_translator.cc)

target_link_libraries(robotlocomotionTranslators
    Eigen3::Eigen
    ${lcm_dependent_link_libraries}
    robotlocomotion-lcmtypes-cpp)

drake_install_headers(
    point_t_translator.h
    pose_t_translator.h
    quaternion_t_translator.h)

drake_install_libraries(robotlocomotionTranslators)
drake_install_pkg_config_file(robotlocomotion-translators
    TARGET robotlocomotionTranslators
    CFLAGS -I${CMAKE_INSTALL_PREFIX}/include
    LIBS -lrobotlocomotionTranslators ${lcm_dependent_libs}
    REQUIRES eigen3 ${lcm_dependent_requires})

if(BUILD_TESTING)
  add_subdirectory(test)
endif()
