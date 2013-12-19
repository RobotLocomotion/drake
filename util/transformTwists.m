function ret = transformTwists(H, twists)
% more efficient implementation of transformAdjoint(H) * twists

sizecheck(twists, [6, nan]);

R = H(1 : 3, 1 : 3);
p = H(1 : 3, 4);
angularPart = R * twists(1 : 3, :);
pHat = vectorToSkewSymmetric(p);
linearPart = pHat * angularPart + R * twists(4 : 6, :);
ret = [angularPart; linearPart];

end

