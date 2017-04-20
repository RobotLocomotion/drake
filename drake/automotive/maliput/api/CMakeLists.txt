drake_install_headers(
  branch_point.h
  junction.h
  lane.h
  lane_data.h
  road_geometry.h
  segment.h
  to_string_stream.h
  )

if(BUILD_TESTING)
  add_subdirectory(test)
endif()
