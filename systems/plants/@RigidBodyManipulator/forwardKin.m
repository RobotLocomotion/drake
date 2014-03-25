function [x, J, JdotV] = forwardKin(obj, kinsol, body_or_frame_ind, points, rotationType, base)
% computes the position of pts (given in the body frame) in the global frame
%
% @param kinsol solution structure obtained from doKinematics
% @param body_or_frame_ind, an integer ID for a RigidBody or RigidBodyFrame
% (obtained via e.g., findLinkInd or findFrameInd)
% @param rotation_type integer flag indicated whether rotations and
% derivatives should be computed (0 - no rotations, 1 - rpy, 2 - quat)
% @retval x the position of pts (given in the body frame) in the global
% frame. If rotation_type, x is 6-by-num_pts where the final 3
% components are the roll/pitch/yaw of the body frame (same for all points 
% on the body) 
% @retval J the Jacobian, dxdq
% @retval JdotV the time derivative of the Jacobian multiplied by the
% velocity vector, dJdt * v
%
% rotation_type  -- 0, no rotation included
%                -- 1, output Euler angle
%                -- 2, output quaternion
% if rotation_type = 0:
% if pts is a 3xm matrix, then x will be a 3xm matrix
%  and (following our gradient convention) J will be a ((3xm)x(q))
%  matrix, with [J1;J2;...;Jm] where Ji = dxidq
% if rotation_type = 1:
% x will be a 6xm matrix and (following our gradient convention) J will be 
% a ((6xm)x(q)) matrix, with [J1;J2;...;Jm] where Ji = dxidq
% if rotation_type = 2:
% x will be a 7xm matrix and (following out gradient convention) J will be
% a ((7xm)*(q)) matrix with [J1;J2;....;Jm] where Ji = dxidq

if nargout < 6
  base = 1;
end
endEffector = body_or_frame_ind;
expressedIn = base; % todo: pass this in as an argument

computeJdotV = nargout > 2;
[JGeometric, vIndices] = geometricJacobian(obj, kinsol, base, endEffector, expressedIn);
JOmega = JGeometric(1 : 3, :);
JV = JGeometric(4 : 6, :);

pointSize = size(points, 1);
nPoints = size(points, 2);

% transform points from end effector frame to base frame
transform = kinsol.T{base} \ kinsol.T{endEffector};
R = transform(1:3, 1:3);
points = homogTransMult(transform, points);
if computeJdotV
  twist = relativeTwist(kinsol.T, kinsol.twists, base, endEffector, expressedIn);
  JGeometricdotV = geometricJacobianDotV(obj, kinsol, base, endEffector, expressedIn);
end

rHats = zeros(nPoints * pointSize, pointSize);
for i = 1 : nPoints
  point = points(:, i);
  rHats(pointSize * (i - 1) + 1 : pointSize * i, :) = vectorToSkewSymmetric(point);
end

JX = -rHats * JOmega + repmat(JV, nPoints, 1);
vSize = obj.num_velocities;

switch (rotationType)
  case 0 % no rotation included
    x = points;
    Phi = zeros(0, 3);
    if computeJdotV
      Phid = zeros(0, 3);
    end
  case 1 % output Euler angle
    rpy = rotmat2rpy(R);
    x = [points; repmat(rpy, 1, nPoints)];
    if computeJdotV
      [Phi, Phid] = angularvel2rpydotMatrix(rpy, twist(1 : 3));
    else
      Phi = angularvel2rpydotMatrix(rpy);
    end
  case 2 % output quaternion
    quat = rotmat2quat(R);
    x = [points; repmat(quat, 1, nPoints)];
    Phi = angularvel2quatdotMatrix(quat);
    if computeJdotV
      quatd = Phi * twist(1 : 3);
      Phid = angularvel2quatdotMatrix(quatd);
    end
  otherwise
    error('rotationType not recognized')
end
JRot = Phi * JOmega;

xSize = pointSize + size(JRot, 1);
posRowIndices = repeatVectorIndices(1 : pointSize, xSize, nPoints);
rotRowIndices = repeatVectorIndices(pointSize + 1 : xSize, xSize, nPoints);

J = zeros(length(posRowIndices) + length(rotRowIndices), vSize);
J(posRowIndices, vIndices) = JX;
J(rotRowIndices, vIndices) = repmat(JRot, nPoints, 1);

if computeJdotV
  JRotdotv = Phid * twist(1 : 3) + Phi * JGeometricdotV(1 : 3);
  JdotV = zeros(length(posRowIndices) + length(rotRowIndices), 1);
  rdots = reshape(-rHats * twist(1 : 3) + repmat(twist(4 : 6), nPoints, 1), pointSize, nPoints);
  XBardotJv = reshape((cross(-rdots, repmat(twist(1 : 3), 1, nPoints))), length(posRowIndices), 1);
  XBarJdotV = -rHats * JGeometricdotV(1 : 3) + repmat(JGeometricdotV(4 : 6), nPoints, 1);
  JdotV(posRowIndices, :) = XBardotJv + XBarJdotV;
  JdotV(rotRowIndices, :) = repmat(JRotdotv, nPoints, 1);
end


end

function ret = repeatVectorIndices(subvectorIndices, subvectorSize, nRepeats)
subvectorIndicesRepeated = repmat(subvectorIndices, 1, nRepeats);
offsets = reshape(repmat(0 : subvectorSize : (nRepeats - 1) * subvectorSize,length(subvectorIndices),1),1,[]);
ret = subvectorIndicesRepeated + offsets;
end