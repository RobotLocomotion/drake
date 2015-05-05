
if (simulink_FOUND)
  add_mex(realtime realtime.cpp)
  endif()

add_mex(barycentricInterpolation barycentricInterpolation.cpp)

if (LCM_FOUND AND MATLAB_FOUND)
  if (simulink_FOUND)
    add_mex(lcmLogger lcmLogger.cpp)
    pods_use_pkg_config_packages(lcmLogger lcm)
  endif()

  add_mex(publishLCMLog publishLCMLog.cpp)
  pods_use_pkg_config_packages(publishLCMLog lcm)
endif()


add_library(drakeUtil SHARED drakeUtil.cpp)
pods_install_headers(drakeUtil.h  DESTINATION drake)
pods_install_libraries(drakeUtil)
pods_install_pkg_config_file(drake-util
  LIBS -ldrakeUtil
  REQUIRES
  VERSION 0.0.1)

if (MATLAB_FOUND)
  add_mex(drakeMexUtil SHARED drakeMexUtil.cpp MexWrapper.cpp)
  # todo: use this again once I can assume everyone has CMAKE version >= 2.8.8
  #add_mex(drakeUtil OBJECT drakeUtil.cpp)

  pods_install_headers(drakeMexUtil.h MexWrapper.h DESTINATION drake)
  pods_install_libraries(drakeMexUtil)
  pods_install_pkg_config_file(drake-mex-util
    LIBS -ldrakeMexUtil
    REQUIRES
    VERSION 0.0.1)
endif()

if (LCM_FOUND)
  # note: requires drakeUtil header file, but not linking against it because
  # we only need methods that are templated and defined in the header
  include_directories (${PROJECT_SOURCE_DIR}/systems/trajectories )
  include_directories (${PROJECT_SOURCE_DIR}/solvers/polynomial )
  include_directories (${PROJECT_BINARY_DIR}/lcmgen/lcmtypes )
  add_library(drakeLCMUtil SHARED lcmUtil.cpp)
  target_link_libraries(drakeLCMUtil drakeTrajectories)
  pods_install_headers(lcmUtil.h DESTINATION drake)
  pods_install_libraries(drakeLCMUtil)
endif()

add_library(drakeGradientUtil SHARED drakeGradientUtil.cpp)
pods_install_libraries(drakeGradientUtil)
pods_install_headers(drakeGradientUtil.h DESTINATION drake)

add_library(drakeGeometryUtil SHARED drakeGeometryUtil.cpp expmap2quat.cpp)
target_link_libraries(drakeGeometryUtil drakeGradientUtil)
pods_install_libraries(drakeGeometryUtil)
pods_install_headers(drakeGeometryUtil.h DESTINATION drake)
pods_install_pkg_config_file(drake-geometry-util
  LIBS -ldrakeGeometryUtil
  REQUIRES
  VERSION 0.0.1)

pods_install_headers(drakeFloatingPointUtil.h DESTINATION drake)

pods_install_headers(GradientVar.h DESTINATION drake)

if (MATLAB_FOUND)
  add_mex(cylindrical2cartesianmex cylindrical2cartesianmex.cpp)
  target_link_libraries(cylindrical2cartesianmex drakeGeometryUtil drakeGradientUtil)
  add_mex(cartesian2cylindricalmex cartesian2cylindricalmex.cpp)
  target_link_libraries(cartesian2cylindricalmex drakeGeometryUtil drakeGradientUtil)

  add_mex(quat2expmapmex quat2expmapmex.cpp)
  target_link_libraries(quat2expmapmex drakeMexUtil drakeGeometryUtil drakeGradientUtil)

  add_mex(expmap2quatImplmex expmap2quatImplmex.cpp)
  target_link_libraries(expmap2quatImplmex drakeMexUtil drakeGeometryUtil drakeGradientUtil)

  add_mex(flipExpmapmex flipExpmapmex.cpp)
  target_link_libraries(flipExpmapmex drakeMexUtil drakeGeometryUtil drakeGradientUtil)

  add_mex(unwrapExpmapmex unwrapExpmapmex.cpp)
  target_link_libraries(unwrapExpmapmex drakeMexUtil drakeGeometryUtil drakeGradientUtil)

  add_mex(quat2expmapSequencemex quat2expmapSequencemex.cpp)
  target_link_libraries(quat2expmapSequencemex drakeMexUtil drakeGeometryUtil drakeGradientUtil)
endif()

add_subdirectory(test)


# Mex debugging tools:
if (0) #NOT WIN32)
add_mex(debugMex SHARED debugMexLib.cpp)
target_link_libraries(debugMex -ldl)
#set_property( SOURCE debugMexLib.cpp PROPERTY COMPILE_FLAGS -DMX_COMPAT_32 )
add_mex(drake_debug_mex EXECUTABLE drakeDebugMex.cpp)
target_link_libraries(drake_debug_mex drakeUtil -ldl)

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
endif(0) #NOT WIN32)
