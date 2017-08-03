if(Matlab_FOUND)
  drake_add_mex(drakeMexUtil SHARED
    mex_util.cc
    mex_util.h)
  target_link_libraries(drakeMexUtil Eigen3::Eigen)

  drake_install_headers(mex_util.h)
  drake_install_libraries(drakeMexUtil)

  drake_add_mex(call_matlab_client call_matlab_client.cc)
  target_include_directories(call_matlab_client PRIVATE
    ${PROJECT_BINARY_DIR}/..) # to find protobuf generated headers
  target_link_libraries(call_matlab_client
    drakeCommon
    drakeCommonProto
    drakeMexUtil)
endif()

