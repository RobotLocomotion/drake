if (LCM_FOUND) # only needed because it's using Pendulum.h for examples...
  add_executable(testVector testVector.cpp)
  add_dependencies(testVector drake_lcmtypes lcmtype_agg_hpp)
  add_test(NAME testVector COMMAND testVector)
endif (LCM_FOUND)
