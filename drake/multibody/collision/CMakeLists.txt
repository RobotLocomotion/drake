set(drakeCollision_SRC_FILES drake_collision.cc element.cc model.cc)

if(Bullet_FOUND)
  set(drakeCollision_SRC_FILES ${drakeCollision_SRC_FILES} bullet_model.cc)
  set(bullet "yes" CACHE STRING "yes")
endif()
set(drakeCollision_SRC_FILES ${drakeCollision_SRC_FILES} unusable_model.cc)

add_library_with_exports(LIB_NAME drakeCollision SOURCE_FILES ${drakeCollision_SRC_FILES})

if(Bullet_FOUND)
  target_compile_definitions(drakeCollision PUBLIC BULLET_COLLISION)
  target_include_directories(drakeCollision PUBLIC ${BULLET_INCLUDE_DIRS})
  target_link_libraries(drakeCollision ${BULLET_LIBRARIES})
endif()
target_link_libraries(drakeCollision drakeShapes drakeCommon)

pods_install_libraries(drakeCollision)
drake_install_headers(drake_collision.h point_pair.h element.h model.h)
pods_install_pkg_config_file(drake-collision
  LIBS -ldrakeCollision
  REQUIRES
  VERSION 0.0.1)

if(BUILD_TESTING)
  add_subdirectory(test)
endif()
