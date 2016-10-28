if(Matlab_FOUND)
  drake_add_mex(drakeMexUtil SHARED drakeMexUtil.cpp MexWrapper.cpp)
  target_link_libraries(drakeMexUtil drakeCommon)

  drake_install_headers(drakeMexUtil.h MexWrapper.h)
  pods_install_libraries(drakeMexUtil)
  pods_install_pkg_config_file(drake-mex-util
    LIBS -ldrakeMexUtil
    REQUIRES
    VERSION 0.0.1)

  drake_add_mex(barycentricInterpolation barycentricInterpolation.cpp)
  
  drake_add_mex(cylindrical2cartesianmex cylindrical2cartesianmex.cpp
    cylindrical.cc)
  target_link_libraries(cylindrical2cartesianmex drakeMath)
  drake_add_mex(cartesian2cylindricalmex cartesian2cylindricalmex.cpp
    cylindrical.cc)
  target_link_libraries(cartesian2cylindricalmex drakeMath)

  drake_add_mex(quat2expmapmex quat2expmapmex.cpp)
  target_link_libraries(quat2expmapmex drakeMexUtil drakeGeometryUtil)

  drake_add_mex(expmap2quatImplmex expmap2quatImplmex.cpp)
  target_link_libraries(expmap2quatImplmex drakeMexUtil drakeGeometryUtil)

  drake_add_mex(flipExpmapmex flipExpmapmex.cpp)
  target_link_libraries(flipExpmapmex drakeMexUtil drakeGeometryUtil)

  drake_add_mex(unwrapExpmapmex unwrapExpmapmex.cpp)
  target_link_libraries(unwrapExpmapmex drakeMexUtil drakeGeometryUtil)

  drake_add_mex(quat2expmapSequencemex quat2expmapSequencemex.cpp)
  target_link_libraries(quat2expmapSequencemex drakeMexUtil drakeGeometryUtil)

  drake_add_mex(closestExpmapmex closestExpmapmex.cpp)
  target_link_libraries(closestExpmapmex drakeMexUtil drakeGeometryUtil)

  drake_add_mex(resolveCenterOfPressuremex resolveCenterOfPressuremex.cpp $<TARGET_OBJECTS:drakeUtil>)
  target_link_libraries(resolveCenterOfPressuremex drakeMexUtil)

  if(Matlab_SIMULINK_FOUND)
    drake_add_mex(realtime realtime.cpp)
  endif()

  if(lcm_FOUND)
    if(Matlab_SIMULINK_FOUND)
      drake_add_mex(lcmLogger lcmLogger.cpp)
      target_link_libraries(lcmLogger lcm)
    endif()

    drake_add_mex(publishLCMLog publishLCMLog.cpp)
    target_link_libraries(publishLCMLog lcm)
  endif()
endif()

# Mex debugging tools:
if(0)  # NOT WIN32
  drake_add_mex(drake_debug_mex EXECUTABLE drakeDebugMex.cpp)
  target_link_libraries(drake_debug_mex drakeMexUtil)

  message(STATUS "Writing drake_debug_mex.sh")
  file(WRITE ${CMAKE_BINARY_DIR}/bin/drake_debug_mex.sh
    "#!/bin/bash\n"
    "\n"
    "# Usage:\n"
    "#   % drake_debug_mex.sh [args]\n"
    "# will set up the environment and then run:\n"
    "#   % args drake-debug-mex\n"
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
    "\n")

  if(APPLE)
    file(APPEND ${CMAKE_BINARY_DIR}/bin/drake_debug_mex.sh
      "export DYLD_LIBRARY_PATH=$DYLD_LIBRARY_PATH:${MATLAB_ROOT}/bin/${MATLAB_CPU}\n"
      "export DYLD_FORCE_FLAT_NAMESPACE=1\n"
      "export DYLD_INSERT_LIBRARIES=${CMAKE_BINARY_DIR}/lib/libdebugMex.dylib\n")
  else()
    file(APPEND ${CMAKE_BINARY_DIR}/bin/drake_debug_mex.sh
      "export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:${MATLAB_ROOT}/bin/${MATLAB_CPU}\n"
      "export LD_PRELOAD=${CMAKE_BINARY_DIR}/lib/libdebugMex.so\n")
  endif()

  file(APPEND ${CMAKE_BINARY_DIR}/bin/drake_debug_mex.sh
    "\n"
    "\"\$@\" ${CMAKE_BINARY_DIR}/bin/drake_debug_mex\n")

  install(FILES ${CMAKE_BINARY_DIR}/bin/drake_debug_mex.sh
    DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
    PERMISSIONS OWNER_READ OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE)
endif()  # NOT WIN32

add_subdirectory(geometry/test)

add_subdirectory(test)
