drake_matlab_add_mex(mex_util mex_util.cc mex_util.h)
target_link_libraries(mex_util Eigen3::Eigen)
drake_install_libraries(mex_util)

drake_matlab_add_mex(call_matlab_client call_matlab_client.cc)
target_include_directories(call_matlab_client PRIVATE
    ${PROJECT_BINARY_DIR}/..) # to find protobuf generated headers
target_link_libraries(call_matlab_client
  drakeCommon
  drakeCommonProto
  mex_util)

if(BUILD_TESTING)
  add_subdirectory(test)
endif()
