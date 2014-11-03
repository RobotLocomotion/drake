function [J, v_indices] = geometricJacobian(obj, kinsol, base, endEffector, expressedIn)
%GEOMETRICJACOBIAN Computes the geometric Jacobian from base to endEffector
% expressed in frame attached to expressedIn
%   The general contract of this method is that for joint velocity vector
%   v, the twist of endEffector with respect to base, expressed in
%   expressedIn, can be computed as J * (v(vIndices)).

if (kinsol.mex)
  if (obj.mex_model_ptr==0)
    error('Drake:RigidBodyManipulator:InvalidKinematics','This kinsol is no longer valid because the mex model ptr has been deleted.');
  end
  [J, v_indices] = geometricJacobianmex(obj.mex_model_ptr, base, endEffector, expressedIn);
else
  [~, jointPath, signs] = findKinematicPath(obj, base, endEffector);
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
  J = transformTwists(inv(kinsol.T{expressedIn}), J); % change frame from world to expressedIn
end
end

function ret = transformMotionSubspace(H, S, sign)
ret = sign * transformTwists(H, S);
end

function vIndices = velocityVectorIndices(bodies, jointIndices)
vIndices = vertcat(bodies(jointIndices).velocity_num);
end