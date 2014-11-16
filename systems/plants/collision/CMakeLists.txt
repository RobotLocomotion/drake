include_directories ( .. )

set(drakeCollision_SRC_FILES DrakeCollision.cpp simpleClosestPointFunctions.cpp)
pods_find_pkg_config(bullet)

if (bullet_FOUND)
  add_definitions( -DBULLET_COLLISION -DBT_USE_DOUBLE_PRECISION )   
  set( drakeCollision_SRC_FILES ${drakeCollision_SRC_FILES} BulletModel.cpp BulletElement.cpp BulletResultCollector.cpp PointPair.cpp )   
  set(bullet "yes" CACHE STRING "yes" )
endif()

if (MSVC)
   add_library(drakeCollision STATIC ${drakeCollision_SRC_FILES})
else()
   add_library(drakeCollision SHARED ${drakeCollision_SRC_FILES})
endif()
if (bullet_FOUND)
  pods_use_pkg_config_packages(drakeCollision bullet)
endif()

enable_testing()
add_subdirectory(test)

pods_install_libraries(drakeCollision)
pods_install_headers(DrakeCollision.h DESTINATION drake/collision)
  
