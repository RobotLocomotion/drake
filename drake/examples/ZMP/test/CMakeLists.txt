drake_add_matlab_test(NAME examples/ZMP/test/foh_zmp_traj_test REQUIRES spotless OPTIONAL bullet COMMAND foh_zmp_traj_test)
drake_add_matlab_test(NAME examples/ZMP/test/testZMPCachedLQR COMMAND testZMPCachedLQR)
drake_add_matlab_test(NAME examples/ZMP/test/testZMPplan_coordinates REQUIRES spotless OPTIONAL bullet COMMAND testZMPplan_coordinates)
drake_add_matlab_test(NAME examples/ZMP/test/testZMPplanners REQUIRES spotless OPTIONAL bullet COMMAND testZMPplanners)

if (lcm_FOUND)
  drake_add_cc_test(zmp_planner_test zmp_planner_test.cc)
  target_link_libraries(zmp_planner_test drakeZMPTestUtil)
endif()
