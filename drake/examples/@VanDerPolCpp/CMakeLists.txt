if(Matlab_FOUND)
  drake_add_mex(dynamicsRHS dynamicsRHS.cpp)
  target_link_libraries(dynamicsRHS drakeMexUtil drakeCommon)
endif()
