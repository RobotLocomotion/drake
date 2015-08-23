
include_directories(${PROJECT_SOURCE_DIR}/util)
include_directories(..)

if (MATLAB_FOUND)

add_mex(CoordinateFrame.new new.cpp OUTPUT_NAME new)
target_link_libraries(CoordinateFrame.new drakeSystem drakeMexUtil)

add_mex(CoordinateFrame.name name.cpp OUTPUT_NAME name)
target_link_libraries(CoordinateFrame.name drakeSystem drakeMexUtil)

add_mex(CoordinateFrame.dim dim.cpp OUTPUT_NAME dim)
target_link_libraries(CoordinateFrame.dim drakeSystem drakeMexUtil)

add_mex(CoordinateFrame.getCoordinateName getCoordinateName.cpp OUTPUT_NAME getCoordinateName)
target_link_libraries(CoordinateFrame.getCoordinateName drakeSystem drakeMexUtil)

add_mex(CoordinateFrame.getCoordinateNames getCoordinateNames.cpp OUTPUT_NAME getCoordinateNames)
target_link_libraries(CoordinateFrame.getCoordinateNames drakeSystem drakeMexUtil)

add_mex(CoordinateFrame.setCoordinateNames setCoordinateNames.cpp OUTPUT_NAME setCoordinateNames)
target_link_libraries(CoordinateFrame.setCoordinateNames drakeSystem drakeMexUtil)

endif()
