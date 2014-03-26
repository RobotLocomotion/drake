function [J, vIndices] = geometricJacobian(obj, kinsol, base, endEffector, expressedIn)
%GEOMETRICJACOBIAN Computes the geometric Jacobian from base to endEffector
% expressed in frame attached to expressedIn
%   The general contract of this method is that for joint velocity vector
%   v, the twist of endEffector with respect to base, expressed in
%   expressedIn, can be computed as J * (v(vIndices)).

[~, jointPath, signs] = findKinematicPath(obj, base, endEffector);
vIndices = vertcat(obj.body(jointPath).velocity_num);

if isempty(jointPath)
  J = zeros(6,0);
  return;
end

J = cell2mat(cellfun(@times, kinsol.J(jointPath), num2cell(signs'), 'UniformOutput', false));

if expressedIn ~= 1
  J = transformTwists(inv(kinsol.T{expressedIn}), J); % change frame from world to expressedIn
end
end
