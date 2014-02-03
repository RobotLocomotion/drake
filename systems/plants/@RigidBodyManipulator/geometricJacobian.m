function [J, vIndices] = geometricJacobian(obj, kinsol, base, endEffector, expressedIn)
%GEOMETRICJACOBIAN Computes the geometric Jacobian from base to endEffector
% expressed in frame attached to expressedIn
%   The general contract of this method is that for joint velocity vector
%   v, the twist of endEffector with respect to base, expressed in
%   expressedIn, can be computed as J * (v(vIndices)).

[~, jointPath, signs] = findKinematicPath(obj, base, endEffector);
vIndices = velocityVectorIndices(obj.body, jointPath);

motionSubspaces = cell(1, length(jointPath));
for i = 1 : length(jointPath)
  body = obj.body(jointPath(i));
  motionSubspaces{i} = motionSubspace(body, kinsol.q(body.dofnum));
end

transformedMotionSubspaces = cellfun(@transformMotionSubspace, ...
  kinsol.T(jointPath), motionSubspaces, num2cell(signs'), 'UniformOutput', ...
  false); % change frame from body to world
J = cell2mat(transformedMotionSubspaces); 
J = transformTwists(inv(kinsol.T{expressedIn}), J); % change frame from world to expressedIn
end

function ret = transformMotionSubspace(H, S, sign)
ret = sign * transformTwists(H, S);
end

function vIndices = velocityVectorIndices(bodies, jointIndices)
vIndices = vertcat(bodies(jointIndices).dofnum);
end