drake_add_matlab_test(NAME systems/trajectories/test/expplusppFnderTest COMMAND expplusppFnderTest)
drake_add_matlab_test(NAME systems/trajectories/test/feedbackTest COMMAND feedbackTest)
drake_add_matlab_test(NAME systems/trajectories/test/ppRateTransitionTest COMMAND ppRateTransitionTest)
drake_add_matlab_test(NAME systems/trajectories/test/ppdertest COMMAND ppdertest)
drake_add_matlab_test(NAME systems/trajectories/test/ppmathtest REQUIRES spotless COMMAND ppmathtest)
drake_add_matlab_test(NAME systems/trajectories/test/pprefinetest COMMAND pprefinetest)

