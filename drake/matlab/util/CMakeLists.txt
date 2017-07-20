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

  if(lcm_FOUND)
    drake_add_mex(call_matlab_client call_matlab_client.cc)
    target_include_directories(call_matlab_client PRIVATE
      ${PROJECT_BINARY_DIR}/..) # to find protobuf generated headers
    target_link_libraries(call_matlab_client
        drakeCommon
        drakeCommonProto
        drakeMexUtil)
  endif()
endif()

add_subdirectory(geometry/test)

if(BUILD_TESTING)
  add_subdirectory(test)
endif()
