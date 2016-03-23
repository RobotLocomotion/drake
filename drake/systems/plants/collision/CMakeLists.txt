set(drakeCollision_SRC_FILES DrakeCollision.cpp Element.cpp PointPair.cpp Model.cpp BulletHeightMapTerrain.cpp)
pods_find_pkg_config(bullet)

if (bullet_FOUND)
  add_definitions( -DBULLET_COLLISION -DBT_USE_DOUBLE_PRECISION )   
  set( drakeCollision_SRC_FILES ${drakeCollision_SRC_FILES} BulletModel.cpp BulletResultCollector.cpp )   
  set(bullet "yes" CACHE STRING "yes" )
endif()

add_library_with_exports(LIB_NAME drakeCollision SOURCE_FILES ${drakeCollision_SRC_FILES})

if (bullet_FOUND)
  pods_use_pkg_config_packages(drakeCollision bullet)
endif()
target_link_libraries(drakeCollision drakeShapes)

enable_testing()
add_subdirectory(test)

pods_install_libraries(drakeCollision)
drake_install_headers(DrakeCollision.h PointPair.h Element.h Model.h)
  
