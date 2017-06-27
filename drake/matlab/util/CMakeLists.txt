if(Matlab_FOUND)
  # Used by matlab/util/systemWCMakeEnv.m
  if(APPLE)
    if(DEFINED ENV{DYLD_LIBRARY_PATH})
      set(DYLD_LIBRARY_PATH "$ENV{DYLD_LIBRARY_PATH}" CACHE INTERNAL "" FORCE)
    else()
      set(DYLD_LIBRARY_PATH "" CACHE INTERNAL "" FORCE)
    endif()
    if(DEFINED ENV{DYLD_FRAMEWORK_PATH})
      set(DYLD_FRAMEWORK_PATH "$ENV{DYLD_FRAMEWORK_PATH}" CACHE INTERNAL "" FORCE)
    else()
      set(DYLD_FRAMEWORK_PATH "" CACHE INTERNAL "" FORCE)
    endif()
  else()
    if(DEFINED ENV{LD_LIBRARY_PATH})
      set(LD_LIBRARY_PATH "$ENV{LD_LIBRARY_PATH}" CACHE INTERNAL "" FORCE)
    else()
      set(LD_LIBRARY_PATH "" CACHE INTERNAL "" FORCE)
    endif()
  endif()

  drake_add_mex(drakeMexUtil SHARED drakeMexUtil.cpp MexWrapper.cpp)
  target_link_libraries(drakeMexUtil Eigen3::Eigen)

  drake_install_headers(drakeMexUtil.h MexWrapper.h)
  drake_install_libraries(drakeMexUtil)
  drake_install_pkg_config_file(drake-mex-util
    TARGET drakeMexUtil
    LIBS -ldrakeMexUtil
    REQUIRES drake-common)

  drake_add_mex(barycentricInterpolation barycentricInterpolation.cpp)

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

    include_directories(${PROJECT_BINARY_DIR}/..) # to find protobuf generated headers
    drake_add_mex(call_matlab_client call_matlab_client.cc)
    target_link_libraries(call_matlab_client
        drakeCommon
        drakeMexUtil)
  endif()
endif()

# Mex debugging tools:
if(0)
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
endif()

add_subdirectory(geometry/test)

if(BUILD_TESTING)
  add_subdirectory(test)
endif()
