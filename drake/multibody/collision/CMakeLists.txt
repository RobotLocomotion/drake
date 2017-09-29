add_library(drakeCollision
    drake_collision.cc
    element.cc
    model.cc
    collision_filter.cc
    unusable_model.cc)
target_compile_definitions(drakeCollision PUBLIC DRAKE_DISABLE_FCL)

target_link_libraries(drakeCollision
  drakeCommon
  drakeShapes
  Eigen3::Eigen)

drake_install_libraries(drakeCollision)
drake_install_headers(drake_collision.h
    point_pair.h
    element.h
    model.h
    collision_filter.h
    )
