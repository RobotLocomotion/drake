
add_mex(PPTmex PPTmex.cpp)
add_mex(ExpPlusPPTrajectoryEvalmex ExpPlusPPTrajectoryEvalmex.cpp)

pods_use_pkg_config_packages(PPTmex
  eigen3)
