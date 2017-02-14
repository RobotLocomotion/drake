if(BUILD_TESTING)
  add_subdirectory(test)
endif()

drake_install_headers(
  arc_lane.h
  branch_point.h
  builder.h
  junction.h
  lane.h
  line_lane.h
  road_geometry.h
  road_section_builder.h
  segment.h
  )
