
add_mex(PPTmex PPTmex.cpp)
add_mex(ExpPlusPPTrajectoryEvalmex ExpPlusPPTrajectoryEvalmex.cpp)

# only need to set this once, since eigen gets added to the include_directories for all files
pods_use_pkg_config_packages(PPTmex
  eigen3)
