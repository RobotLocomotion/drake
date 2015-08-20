
include_directories(${PROJECT_SOURCE_DIR}/util)
include_directories(..)

if (MATLAB_FOUND)

add_mex(CoordinateFrame.new new.cpp OUTPUT_NAME new)
target_link_libraries(CoordinateFrame.new drakeMexUtil)

add_mex(CoordinateFrame.name name.cpp OUTPUT_NAME name)
target_link_libraries(CoordinateFrame.name drakeMexUtil)

add_mex(CoordinateFrame.dim dim.cpp OUTPUT_NAME dim)
target_link_libraries(CoordinateFrame.dim drakeMexUtil)

add_mex(CoordinateFrame.getCoordinateName getCoordinateName.cpp OUTPUT_NAME getCoordinateName)
target_link_libraries(CoordinateFrame.getCoordinateName drakeMexUtil)

add_mex(CoordinateFrame.getCoordinateNames getCoordinateNames.cpp OUTPUT_NAME getCoordinateNames)
target_link_libraries(CoordinateFrame.getCoordinateNames drakeMexUtil)

add_mex(CoordinateFrame.setCoordinateNames setCoordinateNames.cpp OUTPUT_NAME setCoordinateNames)
target_link_libraries(CoordinateFrame.setCoordinateNames drakeMexUtil)

endif()
