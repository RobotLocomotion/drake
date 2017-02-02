add_library_with_exports(LIB_NAME drakeMaliputDragway SOURCE_FILES
  branch_point.cc
  junction.cc
  lane.cc
  northbound_lane.cc
  road_geometry.cc
  segment.cc
  southbound_lane.cc)

target_link_libraries(drakeMaliputDragway
  drakeCommon
  # drakeSystemAnalysis
  # drakeSystemFramework
  # drakeSystemPrimitives
  # Eigen3::Eigen
  # yaml-cpp
)

drake_install_libraries(drakeMaliputDragway)

drake_install_headers(
  branch_point.h
  junction.h
  lane.h
  northbound_lane.h
  road_geometry.h
  segment.h
  southbound_lane.h)

if(BUILD_TESTING)
  add_subdirectory(test)
endif()