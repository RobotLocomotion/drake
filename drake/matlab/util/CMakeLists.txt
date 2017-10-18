# -*- mode: cmake -*-
# vi: set ft=cmake :

drake_matlab_add_mex(mex_util mex_util.cc mex_util.h)
target_link_libraries(mex_util drake::drake Eigen3::Eigen)

drake_matlab_add_mex(call_matlab_client call_matlab_client.cc)
target_link_libraries(call_matlab_client drake::drake mex_util)

install(TARGETS call_matlab_client mex_util DESTINATION share/drake/matlab)

if(BUILD_TESTING)
  add_subdirectory(test)
endif()
