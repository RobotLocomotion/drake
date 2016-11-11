set(drakeCollision_SRC_FILES drake_collision.cc element.cc model.cc)

if(Bullet_FOUND)
  set(drakeCollision_SRC_FILES ${drakeCollision_SRC_FILES} bullet_model.cc)
  set(bullet "yes" CACHE STRING "yes")
endif()
set(drakeCollision_SRC_FILES ${drakeCollision_SRC_FILES} unusable_model.cc)

add_library_with_exports(LIB_NAME drakeCollision SOURCE_FILES ${drakeCollision_SRC_FILES})

set(drakeCollision_REQUIRES)
if(Bullet_FOUND)
  target_compile_definitions(drakeCollision PUBLIC BULLET_COLLISION)
  target_include_directories(drakeCollision PUBLIC ${BULLET_INCLUDE_DIRS})
  target_link_libraries(drakeCollision ${BULLET_LIBRARIES})
  set(drakeCollision_REQUIRES bullet)
endif()
target_link_libraries(drakeCollision drakeShapes drakeCommon)

pods_install_libraries(drakeCollision)
drake_install_headers(drake_collision.h point_pair.h element.h model.h)
drake_install_pkg_config_file(drake-collision
  LIBS -ldrakeCollision
  REQUIRES ${drakeCollision_REQUIRES})

if(BUILD_TESTING)
  add_subdirectory(test)
endif()
