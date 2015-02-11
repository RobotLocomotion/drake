function [J, v_or_qdot_indices, dJ] = geometricJacobian(obj, kinsol, base, end_effector, expressed_in, in_terms_of_qdot)
%GEOMETRICJACOBIAN Computes the geometric Jacobian from base to end_effector
% expressed in frame attached to expressedIn
%   The general contract of this method is that for joint velocity vector
%   v, the twist of endEffector with respect to base, expressed in
%   expressedIn, can be computed as J * (v(vIndices)).

if nargin < 6
  in_terms_of_qdot = false;
end

if (kinsol.mex)
  if (obj.mex_model_ptr==0)
    error('Drake:RigidBodyManipulator:InvalidKinematics','This kinsol is no longer valid because the mex model ptr has been deleted.');
  end
  if nargout > 2
    [J, v_or_qdot_indices, dJ] = geometricJacobianmex(obj.mex_model_ptr, base, end_effector, expressed_in, in_terms_of_qdot);
  else
    [J, v_or_qdot_indices] = geometricJacobianmex(obj.mex_model_ptr, base, end_effector, expressed_in, in_terms_of_qdot);
  end
else
  if obj.use_new_kinsol
    if nargout > 2
      [J, v_or_qdot_indices, dJ] = geometricJacobianNew(obj, kinsol, base, end_effector, expressed_in, in_terms_of_qdot);
    else
      [J, v_or_qdot_indices] = geometricJacobianNew(obj, kinsol, base, end_effector, expressed_in, in_terms_of_qdot);
    end
  else
    if in_terms_of_qdot
      error('not implemented when use_new_kinsol is false');
    end
    [~, joint_path, signs] = findKinematicPath(obj, base, end_effector);
    v_or_qdot_indices = vertcat(obj.body(joint_path).velocity_num);
    if isempty(joint_path)
      J = zeros(6,0);
      return;
    end
    
    motionSubspaces = cell(1, length(joint_path));
    for i = 1 : length(joint_path)
      body = obj.body(joint_path(i));
      motionSubspaces{i} = motionSubspace(body, kinsol.q(body.position_num));
    end
    
    transformedMotionSubspaces = cellfun(@transformMotionSubspace, ...
      kinsol.T(joint_path), motionSubspaces, num2cell(signs'), 'UniformOutput', ...
      false); % change frame from body to world
    J = cell2mat(transformedMotionSubspaces);
    J = transformTwists(inv(kinsol.T{expressed_in}), J); % change frame from world to expressedIn
  end
end
end

function ret = transformMotionSubspace(H, S, sign)
ret = sign * transformTwists(H, S);
end


function [J, v_or_qdot_indices, dJdq] = geometricJacobianNew(obj, kinsol, base, end_effector, expressed_in, in_terms_of_qdot)
% Computes the geometric Jacobian from base to end_effector
% expressed in frame expressed_in
%
% @param kinsol solution structure obtained from doKinematics
% @param base index of `base' rigid body
% @param end_effector index of `end effector' rigid body
% @param expressed_in an integer ID for a RigidBody or RigidBodyFrame
% (obtained via e.g., findLinkId or findFrameInd) signifying in which
% frame the geometric Jacobian should be expressed
% @param in_terms_of_qdot boolean specifying whether to return the mapping
% from qdot to twist or v to twist
%
% @retval J the geometric Jacobian such that the twist of end_effector with
% respect to base, expressed in expressed_in is J * (v(v_or_qdot_indices))
% or J * (qdot(v_or_qdot_indices)) if in_terms_of_qdot
% @retval v_or_qdot_indices vector of indices into the robot's joint velocity
% vector v. v_or_qdot_indices is guaranteed to be `in order' in the following
% two ways:
% 1) if the joint associated with `body' is on the path between base and
% endEffector, then the indices in body.velocity_num or body.position_num 
% will appear in the same order in v_or_qdot_indices
% 2) if body1 precedes body2 in the joint path going from base to
% end_effector, then body1.velocity_num/position_num will precede
% body2.velocity_num/position_num in v_or_qdot_indices.
% Note that this guarantees that the last
% length(end_effector.velocity_num)/length(end_effector.position_num)
% indices of v_or_qdot_indices are exactly
% body.velocity_num/body.position_num.
% @retval dJdq gradient of J with respect to joint configuration vector q


compute_gradient = nargout > 2;

[~, joint_path, signs] = findKinematicPath(obj, base, end_effector);
if isempty(joint_path)
  v_or_qdot_indices = zeros(0, 1); % size fix
else
  if in_terms_of_qdot
    v_or_qdot_indices = vertcat(obj.body(joint_path).position_num);
  else
    v_or_qdot_indices = vertcat(obj.body(joint_path).velocity_num);
  end
end

if isempty(joint_path)
  J = zeros(6,0);
  if compute_gradient
    dJdq = zeros(0, obj.num_positions);
  end
  return;
end

% mtimes to please MSSPoly for TrigPoly
if in_terms_of_qdot
  JBlocks = cellfun(@(x, y, z) x * y * z, num2cell(signs'), kinsol.J(joint_path), kinsol.qdotToV(joint_path), 'UniformOutput', false);
else
  JBlocks = cellfun(@mtimes, kinsol.J(joint_path), num2cell(signs'), 'UniformOutput', false);
end
J = [JBlocks{:}];
% J = cell2mat(JBlocks); % doesn't work with TaylorVar

if compute_gradient
  if in_terms_of_qdot
    dJdqBlocks = cellfun(@(sign, a, b, da, db) sign * matGradMultMat(a, b, da, db), ...
      num2cell(signs'), kinsol.J(joint_path), kinsol.qdotToV(joint_path), kinsol.dJdq(joint_path), kinsol.dqdotToVdq(joint_path),...
      'UniformOutput', false);
  else
    dJdqBlocks = cellfun(@times, kinsol.dJdq(joint_path), num2cell(signs'), 'UniformOutput', false);
  end
  dJdq = vertcat(dJdqBlocks{:});
end

if expressed_in ~= 1
  % change frame from world to expressedIn
  if compute_gradient
    [T, dTdq] = relativeTransform(obj, kinsol, expressed_in, 1);
    dJdq = dTransformSpatialMotion(T, J, dTdq, dJdq);
  else
    T = relativeTransform(obj, kinsol, expressed_in, 1);
  end
  J = transformAdjoint(T) * J;
end

end
