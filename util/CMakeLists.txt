
add_mex(realtime realtime.cpp)
add_mex(drakeUtil OBJECT drakeUtil.cpp)
 
add_mex(deleteMexPointer deleteMexPointer.cpp $<TARGET_OBJECTS:drakeUtil>)
add_mex(drakeDebugMex SHARED drakeDebugMex.cpp)

add_subdirectory(test)

