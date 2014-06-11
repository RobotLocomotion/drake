function [x, J, Jdot_times_v, dJ] = forwardKinV(obj, kinsol, body_or_frame_ind, points, rotation_type, base_ind)
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
compute_Jdot_times_v = nargout > 2;
compute_gradient = nargout > 3;

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
transform_world_to_base = homogTransInv(kinsol.T{base});
transform_frame_to_world = kinsol.T{end_effector} * Tframe;
transform_frame_to_base = transform_world_to_base * transform_frame_to_world;
R_frame_to_base = transform_frame_to_base(1:3, 1:3);
p_frame_to_base = transform_frame_to_base(1:3, 4);
points_base = R_frame_to_base * points + repmat(p_frame_to_base, [1 npoints]);
if compute_gradient
  nq = obj.getNumPositions();
  dtransform_world_to_base = dinvT(kinsol.T{base}, kinsol.dTdq{base});
  dTframe = zeros(numel(Tframe), nq);
  dtransform_frame_to_world = matGradMultMat(kinsol.T{end_effector}, Tframe, kinsol.dTdq{end_effector}, dTframe); % TODO: can be made more efficient due to zero gradient
  dtransform_frame_to_base = matGradMultMat(transform_world_to_base, transform_frame_to_world, dtransform_world_to_base, dtransform_frame_to_world);
  dR_frame_to_base = getSubMatrixGradient(dtransform_frame_to_base, 1:3, 1:3, size(transform_frame_to_base));
  dp_frame_to_base = getSubMatrixGradient(dtransform_frame_to_base, 1:3, 4, size(transform_frame_to_base));
  dpoints = zeros(numel(points), nq);
  dpoints_base = matGradMultMat(R_frame_to_base, points, dR_frame_to_base, dpoints) + repmat(dp_frame_to_base, [npoints, 1]); % TODO: can be made more efficient due to zero gradient
end

% compute geometric Jacobian
if compute_gradient
  [J_geometric, v_indices, dJ_geometric] = geometricJacobian(obj, kinsol, base, end_effector, expressed_in);
else
  [J_geometric, v_indices] = geometricJacobian(obj, kinsol, base, end_effector, expressed_in);
end

Jomega = J_geometric(1 : 3, :);
Jv = J_geometric(4 : 6, :);
if compute_gradient
  dJomega = getSubMatrixGradient(dJ_geometric, 1:3, 1:size(J_geometric,2), size(J_geometric));
  dJv = getSubMatrixGradient(dJ_geometric, 4:6, 1:size(J_geometric,2), size(J_geometric));
end

% compute geometric Jacobian dot times v.
if compute_Jdot_times_v
  if compute_gradient
    [twist, dtwist] = relativeTwist(kinsol.T, kinsol.twists, base, end_effector, expressed_in, kinsol.dTdq, kinsol.dtwistsdq);
    [J_geometric_dot_v, dJ_geometric_dot_v] = geometricJacobianDotV(obj, kinsol, base, end_effector, expressed_in);
  else
    twist = relativeTwist(kinsol.T, kinsol.twists, base, end_effector, expressed_in);
    J_geometric_dot_v = geometricJacobianDotV(obj, kinsol, base, end_effector, expressed_in);
  end
end

% compute position Jacobian
r_hats = zeros(npoints * point_size, point_size) * kinsol.q(1); % for TaylorVar
if compute_gradient
  dr_hats = zeros(numel(r_hats), nq) * kinsol.q(1); % for TaylorVar
end

for i = 1 : npoints
  point = points_base(:, i);
  r_rows = point_size * (i - 1) + 1 : point_size * i;
  r_hats(r_rows, :) = vectorToSkewSymmetric(point);
  if compute_gradient
    dpoint = getSubMatrixGradient(dpoints_base, 1:size(points_base,1), i, size(points_base));
    dr_hats = setSubMatrixGradient(dr_hats, dvectorToSkewSymmetric(dpoint), r_rows, 1:size(r_hats,2), size(r_hats));
  end
end
Jpos = -r_hats * Jomega + repmat(Jv, npoints, 1);
if compute_gradient
  dJpos = matGradMultMat(-r_hats, Jomega, -dr_hats, dJomega) + repmat(dJv, npoints, 1);
end

% compute orientation Jacobian
switch (rotation_type)
  case 0 % no rotation included
    x = points_base;
    Phi = zeros(0, 3);
    if compute_gradient
      dPhi = zeros(numel(Phi), nq);
    end
    if compute_Jdot_times_v
      Phid = zeros(0, 3);
      if compute_gradient
        dPhid = zeros(numel(Phid), nq);
      end
    end
  case 1 % output rpy
    R = transform_frame_to_base(1:3, 1:3);
    rpy = rotmat2rpy(R);
    x = [points_base; repmat(rpy, 1, npoints)];
    if compute_Jdot_times_v
      [Phi, Phid] = angularvel2rpydotMatrix(rpy, twist(1 : 3));
    else
      Phi = angularvel2rpydotMatrix(rpy);
    end
  case 2 % output quaternion
    R = transform_frame_to_base(1:3, 1:3);
    quat = rotmat2quat(R);
    x = [points_base; repmat(quat, 1, npoints)];
    Phi = angularvel2quatdotMatrix(quat);
    if compute_Jdot_times_v
      quatd = Phi * twist(1 : 3);
      Phid = angularvel2quatdotMatrix(quatd);
    end
  otherwise
    error('rotationType not recognized')
end
Jrot = Phi * Jomega;
if compute_gradient
  dJrot = matGradMultMat(Phi, Jomega, dPhi, dJomega);
end

% compute J from JPos and JRot
x_size = point_size + size(Jrot, 1);
pos_row_indices = repeatVectorIndices(1 : point_size, x_size, npoints);
rot_row_indices = repeatVectorIndices(point_size + 1 : x_size, x_size, npoints);

vSize = obj.num_velocities;
J = zeros(length(pos_row_indices) + length(rot_row_indices), vSize) * kinsol.q(1); % for TaylorVar
J(pos_row_indices, v_indices) = Jpos;
J(rot_row_indices, v_indices) = repmat(Jrot, npoints, 1);
if compute_gradient
  dJ = zeros(numel(J), nq) * kinsol.q(1); % for TaylorVar
  dJ = setSubMatrixGradient(dJ, dJpos, pos_row_indices, v_indices, size(J)); % FIXME: v_indices might not be ordered
  dJ = setSubMatrixGradient(dJ, repmat(dJrot, npoints, 1), rot_row_indices, v_indices, size(J)); % FIXME: v_indices might not be ordered
end

% compute Jdot times v
if compute_Jdot_times_v
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