function [J, v_indices] = geometricJacobian(obj, kinsol, base, end_effector, expressed_in)
%GEOMETRICJACOBIAN Computes the geometric Jacobian from base to endEffector
% expressed in frame attached to expressedIn
%   The general contract of this method is that for joint velocity vector
%   v, the twist of endEffector with respect to base, expressed in
%   expressedIn, can be computed as J * (v(vIndices)).


if (kinsol.mex)
  if (obj.mex_model_ptr==0)
    error('Drake:RigidBodyManipulator:InvalidKinematics','This kinsol is no longer valid because the mex model ptr has been deleted.');
  end
  [J, v_indices] = geometricJacobianmex(obj.mex_model_ptr, base, end_effector, expressed_in);
else
  if obj.use_new_kinsol
    [J, v_indices] = geometricJacobianNew(obj, kinsol, base, end_effector, expressed_in);
  else
    [~, jointPath, signs] = findKinematicPath(obj, base, end_effector);
    v_indices = velocityVectorIndices(obj.body, jointPath);
    
    motionSubspaces = cell(1, length(jointPath));
    for i = 1 : length(jointPath)
      body = obj.body(jointPath(i));
      motionSubspaces{i} = motionSubspace(body, kinsol.q(body.position_num));
    end
    
    transformedMotionSubspaces = cellfun(@transformMotionSubspace, ...
      kinsol.T(jointPath), motionSubspaces, num2cell(signs'), 'UniformOutput', ...
      false); % change frame from body to world
    J = cell2mat(transformedMotionSubspaces);
    J = transformTwists(inv(kinsol.T{expressed_in}), J); % change frame from world to expressedIn
  end
end
end

function ret = transformMotionSubspace(H, S, sign)
ret = sign * transformTwists(H, S);
end

function vIndices = velocityVectorIndices(bodies, jointIndices)
vIndices = vertcat(bodies(jointIndices).velocity_num);
end



function [J, v_indices, dJdq] = geometricJacobianNew(obj, kinsol, base, end_effector, expressed_in)
% Computes the geometric Jacobian from base to end_effector
% expressed in frame expressed_in
%
% @param kinsol solution structure obtained from doKinematics
% @param base index of `base' rigid body
% @param end_effector index of `end effector' rigid body
% @param expressed_in an integer ID for a RigidBody or RigidBodyFrame
% (obtained via e.g., findLinkId or findFrameInd) signifying in which
% frame the geometric Jacobian should be expressed
%
% @retval J the geometric Jacobian such that the twist of end_effector with
% respect to base, expressed in expressed_in is J * (v(v_indices))
% @retval v_indices vector of indices into the robot's joint velocity
% vector v. v_indices is guaranteed to be `in order' in the following
% two ways:
% 1) if the joint associated with `body' is on the path between base and
% endEffector, then the velocity indices in body.velocity_num will appear
% in the same order in vIndices
% 2) if body1 precedes body2 in the joint path going from base to
% end_effector, then body1.velocity_num will precede body2.velocity_num
% in v_indices.
% Note that this guarantees that the last
% length(end_effector.velocity_num) indices of v_indices are exactly
% body.velocity_num
% @retval dJdq gradient of J with respect to joint configuration vector q


compute_gradient = nargout > 2;

[~, joint_path, signs] = findKinematicPath(obj, base, end_effector);
v_indices = vertcat(obj.body(joint_path).velocity_num);

if isempty(joint_path)
  J = zeros(6,0);
  dJdq = zeros(0, obj.num_positions);
  return;
end

% mtimes to please MSSPoly for TrigPoly
JBlocks = cellfun(@mtimes, kinsol.J(joint_path), num2cell(signs'), 'UniformOutput', false);
J = [JBlocks{:}];
% J = cell2mat(JBlocks); % doesn't work with TaylorVar

if compute_gradient
  dJdqBlocks = cellfun(@times, kinsol.dJdq(joint_path), num2cell(signs'), 'UniformOutput', false);
  dJdq = vertcat(dJdqBlocks{:});
end

if expressed_in ~= 1
  % change frame from world to expressedIn
  if compute_gradient
    [T, dTdq] = relativeTransform(obj, kinsol, expressed_in, 1);
    dJdq = dTransformAdjoint(T, J, dTdq, dJdq);
  else
    T = relativeTransform(obj, kinsol, expressed_in, 1);
  end
  J = transformAdjoint(T) * J;
end

end
