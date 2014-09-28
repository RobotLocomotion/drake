
if (simulink_FOUND)
  add_mex(realtime realtime.cpp)
  endif()

add_mex(barycentricInterpolation barycentricInterpolation.cpp)

if (LCM_FOUND)
  if (simulink_FOUND)
    add_mex(lcmLogger lcmLogger.cpp)
    pods_use_pkg_config_packages(lcmLogger lcm)
  endif()

  add_mex(publishLCMLog publishLCMLog.cpp)
  pods_use_pkg_config_packages(publishLCMLog lcm)
endif()

if (eigen3_FOUND)
  add_mex(drakeUtil SHARED drakeUtil.cpp)

  # todo: use this again once I can assume everyone has CMAKE version >= 2.8.8
  #add_mex(drakeUtil OBJECT drakeUtil.cpp)

  pods_install_libraries(drakeUtil)

  if (NOT WIN32) # todo: fix this

  add_library(drakeGradientUtil SHARED drakeGradientUtil.cpp)
  pods_install_libraries(drakeGradientUtil)
  pods_install_headers(drakeGradientUtil.h DESTINATION drake)

  endif(NOT WIN32)

  add_library(drakeGeometryUtil SHARED drakeGeometryUtil.cpp)
  if (NOT WIN32)
  target_link_libraries(drakeGeometryUtil drakeGradientUtil)
  endif()
  pods_install_libraries(drakeGeometryUtil)
  pods_install_headers(drakeGeometryUtil.h DESTINATION drake)

  if (NOT WIN32)
  add_mex(test/testQuatmex test/testQuatmex.cpp)
  target_link_libraries(test/testQuatmex drakeGeometryUtil)

  add_mex(test/testGeometryGradientsmex test/testGeometryGradientsmex.cpp)
  target_link_libraries(test/testGeometryGradientsmex drakeGeometryUtil)

  add_mex(test/testGeometryConversionFunctionsmex test/testGeometryConversionFunctionsmex.cpp)
  target_link_libraries(test/testGeometryConversionFunctionsmex drakeGeometryUtil)
  endif()
endif()

add_subdirectory(test)


# Mex debugging tools:
if (NOT WIN32)
add_mex(debugMex SHARED debugMexLib.cpp)
target_link_libraries(debugMex -ldl)
#set_property( SOURCE debugMexLib.cpp PROPERTY COMPILE_FLAGS -DMX_COMPAT_32 )
add_mex(drake_debug_mex EXECUTABLE drakeDebugMex.cpp)
target_link_libraries(drake_debug_mex -ldl)

message(STATUS "Writing drake_debug_mex.sh")
file(WRITE ${CMAKE_BINARY_DIR}/bin/drake_debug_mex.sh
	   "#!/bin/bash\n"
     "\n"
     "# Usage:\n"
     "#   % drake_debug_mex.sh [args]\n"
     "# will set up the environment and then run:\n"
     "#   % args pod-build/bin/drake-debug-mex\n"
     "#\n"
     "# For example,\n"
     "#   % drake_debug_mex.sh\n"
     "# will simply run the executable,\n"
     "#   % drake_debug_mex.sh gdb\n"
     "# will run gdb on drake-debug-mex, and\n"
     "#   % drake_debug_mex.sh valgrind --leak-check=full --dsymutil=yes --track-origins=yes --xml=yes\n"
     "# will run valgrind with the appropriate arguments passed in.\n"
     "#\n"
     "# It's not pretty, but seems to work for the use\n"
     "# cases I've imagined so far. - RussT\n"
     "\n"
    )

if (APPLE)
  file(APPEND ${CMAKE_BINARY_DIR}/bin/drake_debug_mex.sh
       "export DYLD_LIBRARY_PATH=$DYLD_LIBRARY_PATH:${MATLAB_ROOT}/bin/${MATLAB_CPU}\n"
       "export DYLD_FORCE_FLAT_NAMESPACE=1\n"
       "export DYLD_INSERT_LIBRARIES=${CMAKE_BINARY_DIR}/lib/libdebugMex.dylib\n"
      )
else()
  file(APPEND ${CMAKE_BINARY_DIR}/bin/drake_debug_mex.sh
       "export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:${MATLAB_ROOT}/bin/${MATLAB_CPU}\n"
       "export LD_PRELOAD=${CMAKE_BINARY_DIR}/lib/libdebugMex.so\n"
      )
endif()

file(APPEND ${CMAKE_BINARY_DIR}/bin/drake_debug_mex.sh
     "\n"
     "\"\$@\" ${CMAKE_BINARY_DIR}/bin/drake_debug_mex\n"
    )

install(FILES ${CMAKE_BINARY_DIR}/bin/drake_debug_mex.sh
        DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
        PERMISSIONS OWNER_READ OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE)
endif(NOT WIN32)
