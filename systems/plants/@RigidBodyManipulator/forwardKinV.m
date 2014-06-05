function [x, J, Jdot_times_v] = forwardKinV(obj, kinsol, body_or_frame_ind, points, rotation_type, base_ind)
% computes the position of pts (given in the body frame) in the global
% frame, as well as the Jacobian J that maps the joint velocity vector v
% to xdot, and d/dt(J) * v.
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
% @retval J the Jacobian, that maps the joint velocity vector v to xdot
% @retval Jdot_times_v the time derivative of the Jacobian J multiplied
% by the joint velocity vector, d/dt(J) * v
%
% rotation_type  -- 0, no rotation included
%                -- 1, output Euler angle
%                -- 2, output quaternion
% if rotation_type = 0:
% if pts is a 3xm matrix, then x will be a 3xm matrix
%  and (following our gradient convention) J will be a ((3xm)x(q))
%  matrix, with [J1;J2;...;Jm] where Ji = dxidq
% if rotation_type = 1 or 2:
% x will be a 6xm matrix and (following our gradient convention) J will be 
% a ((6xm)x(q)) matrix, with [J1;J2;...;Jm] where Ji = dxidq

if nargin < 5, rotation_type = 0; end
if nargin < 6, base_ind = 1; end
computeJdot_times_v = nargout > 2;

base = base_ind;
if (body_or_frame_ind < 0)
  frame = obj.frame(-body_or_frame_ind);
  end_effector = frame.body_ind;
  Tframe = frame.T;
else
  end_effector = body_or_frame_ind;
  Tframe=eye(4);
end
expressed_in = base; % todo: pass this in as an argument
[point_size, npoints] = size(points);

% transform points from end effector frame to base frame
transform = kinsol.T{base} \ kinsol.T{end_effector} * Tframe;
points = transform(1:3, :) * [points; ones(1, npoints)];

% compute geometric Jacobian
[JGeometric, vIndices] = geometricJacobian(obj, kinsol, base, end_effector, expressed_in);
Jomega = JGeometric(1 : 3, :);
Jv = JGeometric(4 : 6, :);

% compute geometric Jacobian dot times v.
if computeJdot_times_v
  twist = relativeTwist(kinsol.T, kinsol.twists, base, end_effector, expressed_in);
  J_geometric_dot_v = geometricJacobianDotV(obj, kinsol, base, end_effector, expressed_in);
end

% compute position Jacobian
r_hats = zeros(npoints * point_size, point_size) * kinsol.q(1); % for TaylorVar
for i = 1 : npoints
  point = points(:, i);
  r_hats(point_size * (i - 1) + 1 : point_size * i, :) = vectorToSkewSymmetric(point);
end
Jpos = -r_hats * Jomega + repmat(Jv, npoints, 1);

% compute orientation Jacobian
switch (rotation_type)
  case 0 % no rotation included
    x = points;
    Phi = zeros(0, 3);
    if computeJdot_times_v
      Phid = zeros(0, 3);
    end
  case 1 % output rpy
    R = transform(1:3, 1:3);
    rpy = rotmat2rpy(R);
    x = [points; repmat(rpy, 1, npoints)];
    if computeJdot_times_v
      [Phi, Phid] = angularvel2rpydotMatrix(rpy, twist(1 : 3));
    else
      Phi = angularvel2rpydotMatrix(rpy);
    end
  case 2 % output quaternion
    R = transform(1:3, 1:3);
    quat = rotmat2quat(R);
    x = [points; repmat(quat, 1, npoints)];
    Phi = angularvel2quatdotMatrix(quat);
    if computeJdot_times_v
      quatd = Phi * twist(1 : 3);
      Phid = angularvel2quatdotMatrix(quatd);
    end
  otherwise
    error('rotationType not recognized')
end
Jrot = Phi * Jomega;

% compute J from JPos and JRot
x_size = point_size + size(Jrot, 1);
pos_row_indices = repeatVectorIndices(1 : point_size, x_size, npoints);
rot_row_indices = repeatVectorIndices(point_size + 1 : x_size, x_size, npoints);

vSize = obj.num_velocities;
J = zeros(length(pos_row_indices) + length(rot_row_indices), vSize) * kinsol.q(1); % for TaylorVar
J(pos_row_indices, vIndices) = Jpos;
J(rot_row_indices, vIndices) = repmat(Jrot, npoints, 1);

% compute Jdot times v
if computeJdot_times_v
  Jrotdot_times_v = Phid * twist(1 : 3) + Phi * J_geometric_dot_v(1 : 3);
  Jdot_times_v = zeros(length(pos_row_indices) + length(rot_row_indices), 1);
  rdots = reshape(-r_hats * twist(1 : 3) + repmat(twist(4 : 6), npoints, 1), point_size, npoints);
  XBardotJv = reshape((cross(-rdots, repmat(twist(1 : 3), 1, npoints))), length(pos_row_indices), 1);
  XBarJdotV = -r_hats * J_geometric_dot_v(1 : 3) + repmat(J_geometric_dot_v(4 : 6), npoints, 1);
  Jdot_times_v(pos_row_indices, :) = XBardotJv + XBarJdotV;
  Jdot_times_v(rot_row_indices, :) = repmat(Jrotdot_times_v, npoints, 1);
end
end

function ret = repeatVectorIndices(subvectorIndices, subvectorSize, nRepeats)
subvectorIndicesRepeated = repmat(subvectorIndices, 1, nRepeats);
offsets = reshape(repmat(0 : subvectorSize : (nRepeats - 1) * subvectorSize,length(subvectorIndices),1),1,[]);
ret = subvectorIndicesRepeated + offsets;
end