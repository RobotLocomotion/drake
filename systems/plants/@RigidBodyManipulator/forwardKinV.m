function [x, J, dJ] = forwardKinV(obj, kinsol, body_or_frame_ind, points, rotation_type, base_or_frame_ind)
% Transforms \p points given in a frame identified by \p body_or_frame_ind
% to a frame identified by \p base_or_frame_ind, and also computes a
% representation of the relative rotation (of type specified by
% \p rotation_type), stacked in a matrix \p x.
% Also returns the Jacobian \p J that maps the joint velocity vector v to
% xdot, as well as the gradient of J with respect to the joint
% configuration vector q.
%
% @param kinsol solution structure obtained from doKinematics
% @param body_or_frame_ind, an integer ID for a RigidBody or RigidBodyFrame
% (obtained via e.g., findLinkInd or findFrameInd)
% @param points a 3 x m matrix where each column represents a point in the
% frame specified by \p body_or_frame_ind
% @param rotation_type integer flag indicated whether rotation output
% should be included in the return values (0 - no rotation, 1 - rpy,
% 2 - quat).
% @param base_or_frame_ind an integer ID for a RigidBody or RigidBodyFrame
% (obtained via e.g., findLinkInd or findFrameInd) @default 1 (world).
%
% @retval x a matrix with m columns, such that column i is
% [points_base_i; q_rot], where points_base_i is points(:, i) transformed
% to the frame identified by \p body_or_frame_ind, and q_rot is a
% representation of the relative rotation (the same for all m columns).
% That is, if rotation_type = 0 then \p x will be a 3xm matrix with column
% i equal to [points_base_i]. If rotation_type = 1 \p x will be 6xm, with
% column i equal to [points_base_i; rpy], and if rotation_type = 2, \p will
% be 7xm with column i equal to [points_base_i; quat]
% @retval J the Jacobian that maps the joint velocity vector v to xd, the
% time derivative of x.
% @retval dJ the gradient of J with respect to the coordinate vector q.

if ~obj.use_new_kinsol
  error('method is only for use with new kinsol and will be phased in soon.')
end

if nargin < 5, rotation_type = 0; end
if nargin < 6, base_or_frame_ind = 1; end
compute_J = nargout > 1;
compute_gradient = nargout > 2;
expressed_in = base_or_frame_ind; % TODO

if (kinsol.mex)
  if (obj.mex_model_ptr==0)
    error('Drake:RigidBodyManipulator:InvalidKinematics','This kinsol is no longer valid because the mex model ptr has been deleted.');
  end
  if nargout > 2
    [x, J, dJ] = forwardKinVmex(obj.mex_model_ptr, body_or_frame_ind, points, rotation_type, base_or_frame_ind);
    dJ = reshape(dJ, size(J, 1), []); % convert to strange second derivative output format
  elseif nargout > 1
    [x, J] = forwardKinVmex(obj.mex_model_ptr, body_or_frame_ind, points, rotation_type, base_or_frame_ind);
  else
    x = forwardKinVmex(obj.mex_model_ptr, body_or_frame_ind, points, rotation_type, base_or_frame_ind);
  end
else
  nq = obj.getNumPositions();
  % transform points to base frame
  if compute_gradient
    [T, dT] = relativeTransform(obj, kinsol, base_or_frame_ind, body_or_frame_ind);
  else
    T = relativeTransform(obj, kinsol, base_or_frame_ind, body_or_frame_ind);
  end
  
  [point_size, npoints] = size(points);
  R = T(1:3, 1:3);
  p = T(1:3, 4);
  points_base = R * points + repmat(p, 1, npoints);
  % points_base = homogTransMult(T, points); % doesn't work with TaylorVar yet
  if compute_gradient
    dR = getSubMatrixGradient(dT, 1:3, 1:3, size(T));
    dp = getSubMatrixGradient(dT, 1:3, 4, size(T));
    dpoints_base = matGradMult(dR, points) + repmat(dp, [npoints, 1]);
  end
  
  % compute rotation representation
  if compute_gradient
    [qrot, dqrot] = rotmat2Representation(rotation_type, R, dR);
  else
    qrot = rotmat2Representation(rotation_type, R);
  end
  
  % compute x output
  x = [points_base; repmat(qrot, 1, npoints)];
  
  if compute_J
    % compute geometric Jacobian
    body = extractFrameInfo(obj, body_or_frame_ind);
    base = extractFrameInfo(obj, base_or_frame_ind);
    if compute_gradient
      [J_geometric, v_indices, dJ_geometric] = geometricJacobian(obj, kinsol, base, body, expressed_in);
    else
      [J_geometric, v_indices] = geometricJacobian(obj, kinsol, base, body, expressed_in);
    end
    
    % split up into rotational and translational parts
    Jomega = J_geometric(1 : 3, :);
    Jv = J_geometric(4 : 6, :);
    if compute_gradient
      dJomega = getSubMatrixGradient(dJ_geometric, 1:3, 1:size(J_geometric,2), size(J_geometric));
      dJv = getSubMatrixGradient(dJ_geometric, 4:6, 1:size(J_geometric,2), size(J_geometric));
    end
    
    % compute position Jacobian
    if compute_gradient
      [r_hats, dr_hats] = vectorToSkewSymmetric(points_base, dpoints_base);
    else
      r_hats = vectorToSkewSymmetric(points_base);
    end
    Jpos = -r_hats * Jomega + repmat(Jv, npoints, 1);
    if compute_gradient
      block_sizes = repmat(size(Jv, 1), npoints, 1);
      blocks = repmat({dJv}, npoints, 1);
      dJpos = matGradMultMat(-r_hats, Jomega, -dr_hats, dJomega) + interleaveRows(block_sizes, blocks);
    end
    
    % compute rotation Jacobian
    if compute_gradient
      [Phi, ~, dPhi, ~] = angularvel2RepresentationDotMatrix(rotation_type, qrot, dqrot);
      dJrot = matGradMultMat(Phi, Jomega, dPhi, dJomega);
    else
      Phi = angularVel2RepresentationDotMatrix(rotation_type, qrot);
    end
    Jrot = Phi * Jomega;
    
    % compute J from JPos and JRot
    x_size = point_size + size(Phi, 1);
    pos_row_indices = repeatVectorIndices(1 : point_size, x_size, npoints);
    rot_row_indices = repeatVectorIndices(point_size + 1 : x_size, x_size, npoints);
    
    nv = obj.num_velocities;
    J = zeros(length(pos_row_indices) + length(rot_row_indices), nv) * kinsol.q(1); % for TaylorVar
    J(pos_row_indices, v_indices) = Jpos;
    
    if compute_gradient
      dJ = zeros(numel(J), nq) * kinsol.q(1); % for TaylorVar
      dJ = setSubMatrixGradient(dJ, dJpos, pos_row_indices, v_indices, size(J));
    end
    
    if rotation_type ~= 0
      J(rot_row_indices, v_indices) = Jrot(reshape(bsxfun(@times,(1:size(Jrot, 1))',ones(1,npoints)), [], 1), :);
      if compute_gradient
        block_sizes = repmat(size(Jrot, 1), npoints, 1);
        blocks = repmat({dJrot}, npoints, 1);
        dJ = setSubMatrixGradient(dJ, interleaveRows(block_sizes, blocks), rot_row_indices, v_indices, size(J));
      end
    end
    if compute_gradient
      dJ = reshape(dJ, size(J, 1), []); % convert to strange second derivative output format
    end
  end
end

end

function ret = repeatVectorIndices(subvector_indices, subvector_size, nrepeats)
subvector_indices_repeated = reshape(bsxfun(@times,subvector_indices',ones(1,nrepeats)), [], 1);
offsets = reshape(bsxfun(@times,ones(length(subvector_indices),nrepeats),(0:subvector_size:(nrepeats-1) * subvector_size)), [], 1);
ret = subvector_indices_repeated + offsets;
end
