function [J, v_or_qdot_indices, dJ] = geometricJacobian(obj, kinsol, base, end_effector, expressed_in, in_terms_of_qdot)
%GEOMETRICJACOBIAN Computes the geometric Jacobian from base to end_effector
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

if nargin < 6
  in_terms_of_qdot = false;
end

if (kinsol.mex)
  if (obj.mex_model_ptr==0)
    error('Drake:RigidBodyManipulator:InvalidKinematics','This kinsol is no longer valid because the mex model ptr has been deleted.');
  end
  J = geometricJacobianmex(obj.mex_model_ptr, kinsol.mex_ptr, base - 1, end_effector - 1, expressed_in - 1, in_terms_of_qdot);
  if kinsol.has_gradients
    [J, dJ] = eval(J);
    nq = length(kinsol.q);
    if isempty(dJ)
      dJ = zeros(numel(J), nq);
    else
      dJ = dJ(:, 1 : nq);
    end
  end
  if nargout > 1
    % temporary solution
    [~, joint_path] = findKinematicPath(obj, base, end_effector);
    v_or_qdot_indices = vOrQdotIndices(obj, joint_path, in_terms_of_qdot);
  end
else
  if nargout > 2
    [J, v_or_qdot_indices, dJ] = geometricJacobianMatlab(obj, kinsol, base, end_effector, expressed_in, in_terms_of_qdot);
  else
    [J, v_or_qdot_indices] = geometricJacobianMatlab(obj, kinsol, base, end_effector, expressed_in, in_terms_of_qdot);
  end
end
end

function [J, v_or_qdot_indices, dJdq] = geometricJacobianMatlab(obj, kinsol, base, end_effector, expressed_in, in_terms_of_qdot)

compute_gradient = nargout > 2;

[~, joint_path, signs] = findKinematicPath(obj, base, end_effector);
v_or_qdot_indices = vOrQdotIndices(obj, joint_path, in_terms_of_qdot);

if isempty(joint_path)
  J = zeros(6,0);
  if compute_gradient
    dJdq = zeros(0, obj.getNumPositions());
  end
  return;
end

if in_terms_of_qdot
  JBlocks = cellfun(@(x, y, z) x * y * z, num2cell(signs'), kinsol.J(joint_path), kinsol.qdotToV(joint_path), 'UniformOutput', false);
else
  JBlocks = cellfun(@mtimes, kinsol.J(joint_path), num2cell(signs'), 'UniformOutput', false);
end
J = [JBlocks{:}];

if expressed_in ~= 1
  T = relativeTransform(obj, kinsol, expressed_in, 1);
  J = transformAdjoint(T) * J;
end

if compute_gradient
  dJdq = zeros(numel(J), obj.num_positions);
  J_size = [6, length(v_or_qdot_indices)];
  col = 1;
  AdH1 = transformAdjoint(obj.relativeTransform(kinsol, expressed_in, 1));
  for i = 1 : length(joint_path)
    j = joint_path(i);
    sign = signs(i);
    bodyJ = obj.body(j);
    Jj = sign * kinsol.J{j};
    if in_terms_of_qdot
      Jj = Jj * kinsol.qdotToV{j};
    end
    
    dSjdqj = sign * kinsol.dSdq{j};
    if in_terms_of_qdot
      dSjdqj = matGradMultMat(sign * kinsol.S{j}, kinsol.qdotToV{j}, dSjdqj, kinsol.dqdotToVdqi{j});
    end
    AdHj = transformAdjoint(obj.relativeTransform(kinsol, expressed_in, j));
    [Jj1, qdot_ind_ij] = obj.geometricJacobian(kinsol, expressed_in, j, 1, true);
    
    for Jj_col = 1 : size(Jj, 2)
      block = AdHj * getSubMatrixGradient(dSjdqj, 1:6, Jj_col, size(Jj));
      dJdq = setSubMatrixGradient(dJdq, block, 1:6, col, J_size, bodyJ.position_num);
      
      block = -AdH1 * crm(Jj(:, Jj_col)) * Jj1;
      block = block + getSubMatrixGradient(dJdq, 1:6, col, J_size, qdot_ind_ij);
      dJdq = setSubMatrixGradient(dJdq, block, 1:6, col, J_size, qdot_ind_ij);
      col = col + 1;
    end
  end
end
end

function ret = vOrQdotIndices(model, joint_path, in_terms_of_qdot)
if isempty(joint_path)
  ret = zeros(0, 1); % size fix
else
  if in_terms_of_qdot
    ret = vertcat(model.body(joint_path).position_num);
  else
    ret = vertcat(model.body(joint_path).velocity_num);
  end
end
end
