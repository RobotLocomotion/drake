drake_add_cc_test(dragway_test)
target_link_libraries(dragway_test drakeMaliputDragway)

add_executable(dragway_to_urdf dragway_to_urdf.cc)
target_link_libraries(dragway_to_urdf
    drakeAutomotive
    drakeMaliputDragway)
