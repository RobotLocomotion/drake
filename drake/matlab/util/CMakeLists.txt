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
  target_link_libraries(drakeMexUtil drakeCommon)

  drake_install_headers(drakeMexUtil.h MexWrapper.h)
  drake_install_libraries(drakeMexUtil)
  drake_install_pkg_config_file(drake-mex-util
    TARGET drakeMexUtil
    LIBS -ldrakeMexUtil
    REQUIRES drake-common)

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
  target_link_libraries(resolveCenterOfPressuremex
    drakeMexUtil
    Eigen3::Eigen)

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

add_subdirectory(geometry/test)

if(BUILD_TESTING)
  add_subdirectory(test)
endif()
