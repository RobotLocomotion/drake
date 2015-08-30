function [Jdot_times_v, dJdot_times_v] = forwardJacDotTimesV(obj, kinsol, body_or_frame_id, points, rotation_type, base_or_frame_id)
% Computes Jd * v, where J is a matrix that maps from the joint velocity
% vector v to the time derivative of x (x and J of format specified in
% forwardKinV). Gradient output is also available.
% Note that this is in general <b>not</b> the same as dx/dq * qd when the
% mapping from v to qd is time-variant!
%
% @param body_or_frame_ind, an integer ID for a RigidBody or RigidBodyFrame
% (obtained via e.g., findLinkInd or findFrameInd)
% @param points a 3 x m matrix where each column represents a point in the
% frame specified by \p body_or_frame_ind
% @param rotation_type integer flag indicated whether rotations and
% derivatives should be computed (0 - no rotations, 1 - rpy, 2 - quat)
% @param base_or_frame_ind an integer ID for a RigidBody or RigidBodyFrame
% (obtained via e.g., findLinkInd or findFrameInd) @default 1 (world)
%
% @see forwardKin

if (nargin<6), base_or_frame_id=1; end
if (nargin<5), rotation_type=0; end
compute_gradient = nargout > 1;

if kinsol.mex
  if (obj.mex_model_ptr==0)
    error('Drake:RigidBodyManipulator:InvalidKinematics','This kinsol is no longer valid because the mex model ptr has been deleted.');
  end
  if compute_gradient
    [Jdot_times_v, dJdot_times_v] = forwardJacDotTimesVmex(obj.mex_model_ptr, kinsol.mex_ptr, body_or_frame_id, points, rotation_type, base_or_frame_id);
  else
    Jdot_times_v = forwardJacDotTimesVmex(obj.mex_model_ptr, kinsol.mex_ptr, body_or_frame_id, points, rotation_type, base_or_frame_id);
  end
else
  [point_size, npoints] = size(points);
  
  forwardkin_options.rotation_type = rotation_type;
  forwardkin_options.in_terms_of_qdot = true;
  forwardkin_options.base_or_frame_id = base_or_frame_id;
  
  if compute_gradient
    [x, dx] = forwardKin(obj, kinsol, body_or_frame_id, points, forwardkin_options);
  else
    x = forwardKin(obj, kinsol, body_or_frame_id, points, forwardkin_options);
  end
  points_base = x(1 : point_size, :);
  qrot = x(point_size + 1 : end, 1);
  if compute_gradient
    x_size = size(x);
    dpoints_base = getSubMatrixGradient(dx, 1 : 3, 1 : npoints, x_size);
    dqrot = getSubMatrixGradient(dx, point_size + 1 : point_size + length(qrot), 1, x_size);
    [r_hats, dr_hats] = vectorToSkewSymmetric(points_base, dpoints_base);
    [Phi, dPhidqrot, dPhi, ddPhidqrotdq] = angularvel2RepresentationDotMatrix(rotation_type, qrot, dqrot);
  else
    r_hats = vectorToSkewSymmetric(points_base);
    [Phi, dPhidqrot] = angularvel2RepresentationDotMatrix(rotation_type, qrot);
  end

  expressed_in = base_or_frame_id;
  if compute_gradient
    [twist, dtwist] = relativeTwist(kinsol, base_or_frame_id, body_or_frame_id, expressed_in);
    [J_geometric_dot_v, dJ_geometric_dot_v] = geometricJacobianDotTimesV(obj, kinsol, base_or_frame_id, body_or_frame_id, expressed_in);
  else
    twist = relativeTwist(kinsol, base_or_frame_id, body_or_frame_id, expressed_in);
    J_geometric_dot_v = geometricJacobianDotTimesV(obj, kinsol, base_or_frame_id, body_or_frame_id, expressed_in);
  end
  
  omega = twist(1 : 3);
  v_twist = twist(4 : 6);
  
  qrotdot = Phi * omega;
  Phid = reshape(dPhidqrot * qrotdot, size(Phi));
  
  x_size = point_size + size(Phi, 1);
  pos_row_indices = repeatVectorIndices(1 : point_size, x_size, npoints);
  rot_row_indices = repeatVectorIndices(point_size + 1 : x_size, x_size, npoints);
  
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
    
    dqrotdot = Phi * domega + matGradMult(dPhi, omega);
    dPhid = dPhidqrot * dqrotdot + matGradMult(ddPhidqrotdq, qrotdot);
    
    dJdot_times_v = zeros(numel(Jdot_times_v), length(kinsol.q));
    drdots = -r_hats * domega + matGradMult(-dr_hats, omega) + repmat(dv_twist, npoints, 1);
    [omega_hat, domega_hat] = vectorToSkewSymmetric(omega, domega);
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
