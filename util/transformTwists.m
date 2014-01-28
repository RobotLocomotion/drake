function ret = transformTwists(H, twists)
% more efficient implementation of transformAdjoint(H) * twists

ret = transformAdjoint(H) * twists;

% SLOWER:
% R = H(1 : 3, 1 : 3);
% p = H(1 : 3, 4);
% angularPart = R * twists(1 : 3, :);
% pHat = vectorToSkewSymmetric(p);
% linearPart = pHat * angularPart + R * twists(4 : 6, :);
% ret = [angularPart; linearPart];

% SLOWER:
% R = H(1 : 3, 1 : 3);
% p = H(1 : 3, 4);
% 
% angularPart = R * twists(1 : 3, :);
% linearPart = cross(repmat(p, 1, size(angularPart, 2)), angularPart) + R * twists(4 : 6, :);
% ret = [angularPart; linearPart];

end

