add_library_with_exports(LIB_NAME drakeMaliputDragway SOURCE_FILES
  branch_point.cc
  junction.cc
  lane.cc
  road_geometry.cc
  segment.cc)

target_link_libraries(drakeMaliputDragway
  drakeAutomotive
  drakeCommon
)

drake_install_libraries(drakeMaliputDragway)

drake_install_headers(
  branch_point.h
  junction.h
  lane.h
  road_geometry.h
  segment.h)

add_executable(dragway_to_urdf dragway_to_urdf.cc)
target_link_libraries(dragway_to_urdf
    drakeAutomotive
    drakeCommon
    drakeMaliputDragway
    spruce)

if(BUILD_TESTING)
  add_subdirectory(test)
endif()
