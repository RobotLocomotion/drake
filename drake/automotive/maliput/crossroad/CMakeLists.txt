add_library_with_exports(LIB_NAME drakeMaliputCrossroad SOURCE_FILES
  branch_point.cc
  junction.cc
  lane.cc
  road_geometry.cc
  segment.cc)

target_link_libraries(drakeMaliputCrossroad
  drakeCommon
)

drake_install_libraries(drakeMaliputCrossroad)

drake_install_headers(
  branch_point.h
  junction.h
  lane.h
  road_geometry.h
  segment.h)

if(BUILD_TESTING)
  add_subdirectory(test)
endif()
