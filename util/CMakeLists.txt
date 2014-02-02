
add_mex(realtime realtime.cpp)

# todo: use this again once I can assume everyone has CMAKE version >= 2.8.8
#add_mex(drakeUtil OBJECT drakeUtil.cpp)
#add_mex(deleteMexPointer deleteMexPointer.cpp $<TARGET_OBJECTS:drakeUtil>)

add_mex(drakeUtil SHARED drakeUtil.cpp)
pods_install_libraries(drakeUtil)

add_mex(debugMex SHARED debugMexLib.cpp)
#set_property( SOURCE debugMexLib.cpp PROPERTY COMPILE_FLAGS -DMX_COMPAT_32 )
add_mex(drake_debug_mex EXECUTABLE drakeDebugMex.cpp)
set_target_properties(drake_debug_mex PROPERTIES LINK_FLAGS -ldl )

if (eigen3_FOUND)
  add_mex(drakeQuatUtil SHARED drakeQuatUtil.cpp)
  add_mex(test/testQuatmex test/testQuatmex.cpp) 
  target_link_libraries(test/testQuatmex drakeQuatUtil)
  pods_install_libraries(drakeQuatUtil)
endif()

add_subdirectory(test)



