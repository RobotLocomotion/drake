drake_add_cc_test(NAME testPiecewisePolynomial EXTENSION cpp)
target_link_libraries(testPiecewisePolynomial drakeTrajectories)

drake_add_cc_test(NAME testExponentialPlusPiecewisePolynomial EXTENSION cpp)
target_link_libraries(testExponentialPlusPiecewisePolynomial drakeTrajectories)

# ==== Below this line is MATLAB-specific code ====

drake_add_matlab_test(NAME systems/trajectories/test/expplusppFnderTest COMMAND expplusppFnderTest)
drake_add_matlab_test(NAME systems/trajectories/test/feedbackTest COMMAND feedbackTest)
drake_add_matlab_test(NAME systems/trajectories/test/ppRateTransitionTest COMMAND ppRateTransitionTest)
drake_add_matlab_test(NAME systems/trajectories/test/ppdertest COMMAND ppdertest)
drake_add_matlab_test(NAME systems/trajectories/test/ppmathtest REQUIRES spotless COMMAND ppmathtest)
drake_add_matlab_test(NAME systems/trajectories/test/pprefinetest COMMAND pprefinetest)

# ==== Below this line is C++ and MATLAB shared code ====

