function coefs = cubicSplineCoefficients(tf, q0, qf, qdot0, qdotf)
% cubicSplineCoefficients: compute the coefficients for a cubic polynomial spline
% See Craig, J. Introduction to Robotics: Mechanics and Control, 2005, p207, eq 7.11
% These coefficients assume that q0 and qdot0 occur at time = 0.
%
% Note that these are exactly the same coefficients computed by pchipDeriv,
% but this code is 5 to 10 times faster. For a comparison of the results and
% performance, see test/testCubicSplineCoefficients.m

a0 = q0;
a1 = qdot0;
a2 = (3 / tf^2) * (qf - q0) - (2 / tf) * qdot0 - (1 / tf) * qdotf;
a3 = (-2 / tf^3) * (qf - q0) + (1 / tf^2) * (qdotf + qdot0);
coefs = cat(3, a3, a2, a1, a0);
end