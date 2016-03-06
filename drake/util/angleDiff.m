function d = angleDiff(phi1, phi2)
% 
% Compute the angular difference between two angles, paying careful attention to wraparound. Note that for scalar phi1 and phi2, the result should be the same as diff(unwrap([phi1, phi2])). 
%
% Note: about 10-15x faster than using matlab's unwrap function.
%
% Test case provided in test/angleDiffTest.m

d = mod(phi2 - phi1 + pi, 2*pi) - pi;
