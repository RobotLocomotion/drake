function JAnalytical = analyticalJacobian(obj, kinsol, base, endEffector, points, rotationType)

% NOTE: even though geometricJacobian is mexed, I need kinsol.T in what
% follows, so I can't use a mex kinsol.
if (kinsol.mex)
    error('Drake:RigidBodyManipulator:NoMex','This method has not been mexed yet.');
end

[JGeometric, vIndices] = geometricJacobian(obj, kinsol, base, endEffector, base);
JOmega = JGeometric(1 : 3, :);
JV = JGeometric(4 : 6, :);

pointSize = size(points, 1);
nPoints = size(points, 2);

% transform points from end effector frame to base frame
transform = kinsol.T{base} \ kinsol.T{endEffector};
points = homogTransMult(transform, points);

xHats = zeros(nPoints * pointSize, pointSize);
for i = 1 : nPoints
  point = points(:, i);
  xHats(pointSize * (i - 1) + 1 : pointSize * i, :) = vectorToSkewSymmetric(point);
end

JX = -xHats * JOmega + repmat(JV, nPoints, 1);
vSize = obj.num_velocities;

switch (rotationType)
  case 0 % no rotation included
    JRot = zeros(0, size(JOmega, 2));
  case 1 % output Euler angle
    R = transform(1:3, 1:3);
    rpy = rotmat2rpy(R);
    JRot = angularvel2rpydotMatrix(rpy) * JOmega;
  case 2 % output quaternion
    R = transform(1:3, 1:3);
    quat = rotmat2quat(R);
    JRot = angularvel2quatdotMatrix(quat) * JOmega;
  otherwise
    error('rotationType not recognized')
end

chiSize = pointSize + size(JRot, 1);
xRowIndices = repeatVectorIndices(1 : pointSize, chiSize, nPoints);
rotRowIndices = repeatVectorIndices(pointSize + 1 : chiSize, chiSize, nPoints);

JAnalytical = zeros(length(xRowIndices) + length(rotRowIndices), vSize);
JAnalytical(xRowIndices, vIndices) = JX;
JAnalytical(rotRowIndices, vIndices) = repmat(JRot, nPoints, 1);

end

function ret = repeatVectorIndices(subvectorIndices, subvectorSize, nRepeats)
subvectorIndicesRepeated = repmat(subvectorIndices, 1, nRepeats);
offsets = reshape(repmat(0 : subvectorSize : (nRepeats - 1) * subvectorSize,length(subvectorIndices),1),1,[]);
ret = subvectorIndicesRepeated + offsets;
end