
include_directories(${PROJECT_SOURCE_DIR}/util)
include_directories(..)

add_mex(CoordinateFrameTest.new new.cpp OUTPUT_NAME new)
target_link_libraries(CoordinateFrameTest.new drakeMexUtil)

add_mex(CoordinateFrameTest.name name.cpp OUTPUT_NAME name)
target_link_libraries(CoordinateFrameTest.name drakeMexUtil)