protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS matlab_rpc.proto)

set(sources
  ${PROTO_SRCS}
  call_matlab.cc
  protobuf.cc
  )

set(installed_headers
  ${PROTO_HDRS}
  call_matlab.h
  protobuf.h
  )

add_library_with_exports(LIB_NAME drakeCommonProto
  SOURCE_FILES ${sources} ${installed_headers})
target_include_directories(drakeCommonProto PUBLIC ${PROTOBUF_INCLUDE_DIRS})
target_link_libraries(drakeCommonProto
  ${PROTOBUF_LIBRARIES}
  drakeCommon
  Eigen3::Eigen
  )
set(drakeCommonProto_CFLAGS)
set(drakeCommonProto_REQUIRES)

drake_install_headers(${installed_headers})
drake_install_libraries(drakeCommonProto)

# N.B. The tests are not built with CMake.
