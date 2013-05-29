function d = angleDiff(phi1, phi2)
% 
% Compute the angular difference between two angles, paying careful attention to wraparound. Note that for scalar phi1 and phi2, the result should be the same as diff(unwrap([phi1, phi2])). 
%
% Note: about 10-15x faster than using matlab's unwrap function.
%
% Test case provided in test/angleDiffTest.m

phi2 = mod(phi2, 2*pi);
phi1 = mod(phi1, 2*pi);

d = phi2 - phi1;
d(d > pi) = d(d > pi) -2*pi;
d(d < -pi) = d(d < -pi) + 2*pi;
