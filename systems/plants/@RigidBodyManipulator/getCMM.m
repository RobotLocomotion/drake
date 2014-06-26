function [A, Adot_times_v, dA, dAdot_times_v] = getCMM(robot, kinsol)
error('getCMM is deprecated. Use centroidalMomentumMatrix for A and its gradient and centroidalMomentumMatrixDotTimesV for dA/dt * v and its gradient');
end