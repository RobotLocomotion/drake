
if (eigen3_FOUND)
   add_mex(PPTmex PPTmex.cpp)
   add_mex(ExpPlusPPTrajectoryEvalmex ExpPlusPPTrajectoryEvalmex.cpp)
endif()
