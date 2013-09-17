
add_mex(realtime realtime.cpp)

# todo: use this again once I can assume everyone has CMAKE version >= 2.8.8
#add_mex(drakeUtil OBJECT drakeUtil.cpp)
#add_mex(deleteMexPointer deleteMexPointer.cpp $<TARGET_OBJECTS:drakeUtil>)

add_mex(drakeUtil SHARED drakeUtil.cpp)

add_mex(drakeDebugMex SHARED drakeDebugMex.cpp)

add_mex(drakeQuatUtil SHARED drakeQuatUtil.cpp)
add_subdirectory(test)

