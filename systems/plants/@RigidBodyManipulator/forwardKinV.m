function [x, Jv, Jvdot_times_v] = forwardKinV(obj, kinsol, body_or_frame_ind, points, rotationType, base_ind)
% computes the position of pts (given in the body frame) in the global
% frame, as well as the Jacobian Jv that maps the joint velocity vector v
% to xdot, and d/dt(Jv) * v.
%
% @param kinsol solution structure obtained from doKinematics
% @param body_or_frame_ind, an integer ID for a RigidBody or RigidBodyFrame
% (obtained via e.g., findLinkInd or findFrameInd)
% @param rotation_type integer flag indicated whether rotations and
% derivatives should be computed (0 - no rotations, 1 - rpy, 2 - quat)
% @param base_ind index of the base rigid body. Default is 1 (world).
% @retval x the position of pts (given in the body frame) in the base frame
% frame. If rotation_type, x is 6-by-num_pts where the final 3
% components are the roll/pitch/yaw of the body frame (same for all points 
% on the body) 
% @retval Jv the Jacobian, that maps the joint velocity vector v to xdot
% @retval Jvdot_times_v the time derivative of the Jacobian Jv multiplied
% by the joint velocity vector, d/dt(Jv) * v
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

if nargin < 6
  base_ind = 1;
end
base = base_ind;
endEffector = body_or_frame_ind;
expressedIn = base; % todo: pass this in as an argument

computeJvdot_times_v = nargout > 2;
[JGeometric, vIndices] = geometricJacobian(obj, kinsol, base, endEffector, expressedIn);
JOmega = JGeometric(1 : 3, :);
JV = JGeometric(4 : 6, :);

pointSize = size(points, 1);
nPoints = size(points, 2);

% transform points from end effector frame to base frame
transform = kinsol.T{base} \ kinsol.T{endEffector};
R = transform(1:3, 1:3);
points = homogTransMult(transform, points);
if computeJvdot_times_v
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
    if computeJvdot_times_v
      Phid = zeros(0, 3);
    end
  case 1 % output rpy
    rpy = rotmat2rpy(R);
    x = [points; repmat(rpy, 1, nPoints)];
    if computeJvdot_times_v
      [Phi, Phid] = angularvel2rpydotMatrix(rpy, twist(1 : 3));
    else
      Phi = angularvel2rpydotMatrix(rpy);
    end
  case 2 % output quaternion
    quat = rotmat2quat(R);
    x = [points; repmat(quat, 1, nPoints)];
    Phi = angularvel2quatdotMatrix(quat);
    if computeJvdot_times_v
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

Jv = zeros(length(posRowIndices) + length(rotRowIndices), vSize);
Jv(posRowIndices, vIndices) = JX;
Jv(rotRowIndices, vIndices) = repmat(JRot, nPoints, 1);

if computeJvdot_times_v
  JRotdot_times_v = Phid * twist(1 : 3) + Phi * JGeometricdotV(1 : 3);
  Jvdot_times_v = zeros(length(posRowIndices) + length(rotRowIndices), 1);
  rdots = reshape(-rHats * twist(1 : 3) + repmat(twist(4 : 6), nPoints, 1), pointSize, nPoints);
  XBardotJv = reshape((cross(-rdots, repmat(twist(1 : 3), 1, nPoints))), length(posRowIndices), 1);
  XBarJdotV = -rHats * JGeometricdotV(1 : 3) + repmat(JGeometricdotV(4 : 6), nPoints, 1);
  Jvdot_times_v(posRowIndices, :) = XBardotJv + XBarJdotV;
  Jvdot_times_v(rotRowIndices, :) = repmat(JRotdot_times_v, nPoints, 1);
end
end

function ret = repeatVectorIndices(subvectorIndices, subvectorSize, nRepeats)
subvectorIndicesRepeated = repmat(subvectorIndices, 1, nRepeats);
offsets = reshape(repmat(0 : subvectorSize : (nRepeats - 1) * subvectorSize,length(subvectorIndices),1),1,[]);
ret = subvectorIndicesRepeated + offsets;
end