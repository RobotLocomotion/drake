function [J, v_indices, dJdq] = geometricJacobian(obj, kinsol, base, end_effector, expressed_in)
%GEOMETRICJACOBIAN Computes the geometric Jacobian from base to endEffector
% expressed in frame attached to expressedIn
%   The general contract of this method is that for joint velocity vector
%   v, the twist of end_effector with respect to base, expressed in
%   expressed_in, can be computed as J * (v(v_indices)).
%   In addition, v_indices is guaranteed to be `in order' in the following
%   two ways:
%   1) if the joint associated with `body' is on the path between base and
%   endEffector, then the velocity indices in body.velocity_num will appear
%   in the same order in vIndices
%   2) if body1 precedes body2 in the joint path going from base to
%   end_effector, then body1.velocity_num will precede body2.velocity_num
%   in v_indices.
%   Note that this guarantees that the last
%   length(end_effector.velocity_num) indices of v_indices are exactly
%   body.velocity_num

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
