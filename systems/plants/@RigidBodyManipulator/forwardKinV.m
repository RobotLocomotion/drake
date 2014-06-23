function [x, J, Jdot_times_v, dJ, dJdot_times_v] = forwardKinV(obj, kinsol, body_or_frame_ind, points, rotation_type, base_or_frame_ind)
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
if nargin < 6, base_or_frame_ind = 1; end
compute_Jdot_times_v = nargout > 2;
compute_gradient = nargout > 3;
nq = obj.getNumPositions();

expressed_in = base_or_frame_ind; % TODO
[point_size, npoints] = size(points);

if compute_gradient
  [Tbodyframe_to_baseframe, dTbodyframe_to_baseframe] = relativeTransform(obj, kinsol, base_or_frame_ind, body_or_frame_ind);
else
  Tbodyframe_to_baseframe = relativeTransform(obj, kinsol, base_or_frame_ind, body_or_frame_ind);
end

Rbodyframe_to_baseframe = Tbodyframe_to_baseframe(1:3, 1:3);
pbodyframe_to_baseframe = Tbodyframe_to_baseframe(1:3, 4);
points_base = Rbodyframe_to_baseframe * points + repmat(pbodyframe_to_baseframe, 1, npoints);
if compute_gradient
  dRbodyframe_to_baseframe = getSubMatrixGradient(dTbodyframe_to_baseframe, 1:3, 1:3, size(Tbodyframe_to_baseframe));
  dpbodyframe_to_baseframe = getSubMatrixGradient(dTbodyframe_to_baseframe, 1:3, 4, size(Tbodyframe_to_baseframe));
  dpoints = zeros(numel(points), nq); % TODO: can be made more efficient due to zero gradient
  dpoints_base = matGradMultMat(Rbodyframe_to_baseframe, points, dRbodyframe_to_baseframe, dpoints) + repmat(dpbodyframe_to_baseframe, [npoints, 1]);
end

% compute geometric Jacobian
if compute_gradient
  [J_geometric, v_indices, dJ_geometric] = geometricJacobian(obj, kinsol, base_or_frame_ind, body_or_frame_ind, expressed_in);
else
  [J_geometric, v_indices] = geometricJacobian(obj, kinsol, base_or_frame_ind, body_or_frame_ind, expressed_in);
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
    [twist, dtwist] = relativeTwist(kinsol, base_or_frame_ind, body_or_frame_ind, expressed_in);
    [J_geometric_dot_v, dJ_geometric_dot_v] = geometricJacobianDotV(obj, kinsol, base_or_frame_ind, body_or_frame_ind, expressed_in);
  else
    twist = relativeTwist(kinsol, base_or_frame_ind, body_or_frame_ind, expressed_in);
    J_geometric_dot_v = geometricJacobianDotV(obj, kinsol, base_or_frame_ind, body_or_frame_ind, expressed_in);
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
  block_sizes = repmat(size(Jv, 1), npoints, 1);
  blocks = repmat({dJv}, npoints, 1);
  dJpos = matGradMultMat(-r_hats, Jomega, -dr_hats, dJomega) + interleaveRows(block_sizes, blocks);
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
    if compute_gradient
      [rpy, drpy] = rotmat2rpy(Rbodyframe_to_baseframe, dRbodyframe_to_baseframe);
    else
      rpy = rotmat2rpy(Rbodyframe_to_baseframe);
    end
    x = [points_base; repmat(rpy, 1, npoints)];
    if compute_Jdot_times_v
      if compute_gradient
        [Phi, dPhidrpy, ddPhidrpy] = angularvel2rpydotMatrix(rpy);
        dPhi = dPhidrpy * drpy;
        ddPhidrpy = reshape(ddPhidrpy, [], numel(rpy));
        ddPhidrpydq = ddPhidrpy * drpy;
      else
        [Phi, dPhidrpy] = angularvel2rpydotMatrix(rpy);
      end
      omega = twist(1:3);
      rpyd = Phi * omega;
      Phid = reshape(dPhidrpy * rpyd, size(Phi));
      if compute_gradient
        domega = dtwist(1:3, :);
        drpyd = Phi * domega + matGradMult(dPhi, omega);
        dPhid = dPhidrpy * drpyd + matGradMult(ddPhidrpydq, rpyd);
      end
    else
      Phi = angularvel2rpydotMatrix(rpy);
    end
  case 2 % output quaternion
    if compute_gradient
      [quat, dquat] = rotmat2quat(Rbodyframe_to_baseframe, dRbodyframe_to_baseframe);
    else
      quat = rotmat2quat(Rbodyframe_to_baseframe);
    end

    if compute_Jdot_times_v || compute_gradient
      [Phi, dPhidquat] = angularvel2quatdotMatrix(quat);
    else
      Phi = angularvel2quatdotMatrix(quat);
    end

    x = [points_base; repmat(quat, 1, npoints)];
    if compute_Jdot_times_v
      omega = twist(1 : 3);
      quatd = Phi * omega;
      Phid = reshape(dPhidquat * quatd, size(Phi));
      if compute_gradient
        dPhi = dPhidquat * dquat;
        domega = dtwist(1:3, :);
        dquatd = Phi * domega + matGradMult(dPhi, omega);
        % ddphidquatdq = 0:
        dPhid = dPhidquat * dquatd;
      else

      end
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

if compute_gradient
  dJ = zeros(numel(J), nq) * kinsol.q(1); % for TaylorVar
  dJ = setSubMatrixGradient(dJ, dJpos, pos_row_indices, v_indices, size(J));
end

if rotation_type ~= 0
  J(rot_row_indices, v_indices) = repmat(Jrot, npoints, 1);
  if compute_gradient
    block_sizes = repmat(size(Jrot, 1), npoints, 1);
    blocks = repmat({dJrot}, npoints, 1);
    dJ = setSubMatrixGradient(dJ, interleaveRows(block_sizes, blocks), rot_row_indices, v_indices, size(J));
  end
end

% compute Jdot times v
if compute_Jdot_times_v
  omega = twist(1 : 3);
  v_twist = twist(4 : 6);
  Jrotdot_times_v = Phid * omega + Phi * J_geometric_dot_v(1 : 3);
  Jdot_times_v = zeros(length(pos_row_indices) + length(rot_row_indices), 1) * kinsol.q(1); % for TaylorVar
  rdots = reshape(-r_hats * omega + repmat(v_twist, npoints, 1), point_size, npoints);
  omega_hat = vectorToSkewSymmetric(omega);
  XBardotJv = reshape(omega_hat * rdots, length(pos_row_indices), 1);
  XBarJdotV = -r_hats * J_geometric_dot_v(1 : 3) + repmat(J_geometric_dot_v(4 : 6), npoints, 1);
  Jdot_times_v(pos_row_indices, :) = XBardotJv + XBarJdotV;

  if rotation_type ~= 0
    Jdot_times_v(rot_row_indices, :) = repmat(Jrotdot_times_v, npoints, 1);
  end
  if compute_gradient
    domega = dtwist(1:3, :);
    dv_twist = dtwist(4:6, :);
    dJdot_times_v = zeros(numel(Jdot_times_v), nq);
    drdots = -r_hats * domega + matGradMult(-dr_hats, omega) + repmat(dv_twist, npoints, 1);
    domega_hat = dvectorToSkewSymmetric(domega);
    dXBardotJv = matGradMultMat(omega_hat, rdots, domega_hat, drdots);
    dXBarJdotV = -r_hats * dJ_geometric_dot_v(1:3, :) + matGradMult(-dr_hats, J_geometric_dot_v(1:3)) + repmat(dJ_geometric_dot_v(4:6, :), npoints, 1);
    allcols = 1:size(Jdot_times_v, 2);
    dJdot_times_v = setSubMatrixGradient(dJdot_times_v, dXBardotJv + dXBarJdotV, pos_row_indices, allcols, size(Jdot_times_v));
    
    if rotation_type ~= 0
      dJrotdot_times_v = Phid * domega + matGradMult(dPhid, omega) + Phi * dJ_geometric_dot_v(1:3, :) + matGradMult(dPhi, J_geometric_dot_v(1:3));
      block_sizes = repmat(size(Jrotdot_times_v, 1), npoints, 1);
      blocks = repmat({dJrotdot_times_v}, npoints, 1);
      dJdot_times_v = setSubMatrixGradient(dJdot_times_v, interleaveRows(block_sizes, blocks), rot_row_indices, allcols, size(Jdot_times_v));
    end
  end
end
end

function ret = repeatVectorIndices(subvectorIndices, subvectorSize, nRepeats)
subvectorIndicesRepeated = repmat(subvectorIndices, 1, nRepeats);
offsets = reshape(repmat(0 : subvectorSize : (nRepeats - 1) * subvectorSize,length(subvectorIndices),1),1,[]);
ret = subvectorIndicesRepeated + offsets;
end
