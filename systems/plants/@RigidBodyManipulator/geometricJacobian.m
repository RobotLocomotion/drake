function [J, vIndices] = geometricJacobian(obj, kinsol, base, endEffector, expressedIn)
%GEOMETRICJACOBIAN Computes the geometric Jacobian from base to endEffector
% expressed in frame attached to expressedIn
%   The general contract of this method is that for joint velocity vector
%   v, the twist of endEffector with respect to base, expressed in
%   expressedIn, can be computed as J * (v(vIndices)).

[~, jointPath, signs] = obj.findKinematicPath(base, endEffector);
vIndices = velocityVectorIndices(obj.body, jointPath);

% motionSubspaces = arrayfun(@motionSubspace, obj.body(jointPath), jointpath, 'UniformOutput', false);
motionSubspaces = cell(1, length(jointPath));
for i = 1 : length(jointPath)
  body = obj.body(jointPath(i));
  motionSubspaces{i} = motionSubspace(body, kinsol.q(body.dofnum));
end

transformedMotionSubspaces = cellfun(@transformMotionSubspace, kinsol.T(jointPath), motionSubspaces, num2cell(signs'), 'UniformOutput', false);
J = cell2mat(transformedMotionSubspaces); % change frame from body to world
J = adjoint(inv(kinsol.T{expressedIn})) * J; % change frame from world to expressedIn
end

function ret = transformMotionSubspace(H, S, sign)
% this could be sped up by not explicitly computing adjoint(H) and
% exploiting its structure
ret = sign * adjoint(H) * S;
end

function S = motionSubspace(body, qBody)
if body.floating == 1 % roll, pitch, yaw
  rpy = qBody(4 : 6);
  E = rpydot2angularvelMatrix(rpy);
  R = rpy2rotmat(rpy);
  S = [zeros(3, 3), R' * E;
    R' , zeros(3, 3)];
elseif body.floating == 2 % quaternion
  quat = qBody(4 : 7);
  R = quat2rotmat(quat);
  S = [zeros(3, 3), R' * quatdot2angularvelMatrix(quat);
       R', zeros(3, 4)];
  warning('Drake:RigidBodyManipulator:NotTested','Motion subspace not tested for case of quaternion floating joint.');
elseif body.floating ~= 0
  % twistSize = 6;
  % S = eye(twistSize);
  % currently not true. This would be the case if we were using twist
  % as the parameterization of velocity across a 6-DoF joint
  error('Drake:RigidBodyManipulator:NotImplemented','Motion subspace not implemented for this type of floating joint.');
else
  pitch = body.pitch;
  axis = body.joint_axis;
  if isinf(pitch) % prismatic joint
    S = [zeros(3, 1); axis];
  else % helical or revolute joint
    S = [axis; pitch * axis];
  end
end
end

function ret = adjoint(H)
R = H(1 : 3, 1 : 3);
p = H(1 : 3, 4);
pHat = vectorToSkewSymmetric(p);
ret = [R, zeros(3, 3);
  pHat * R, R];
end

function vIndices = velocityVectorIndices(bodies, jointIndices)
allVIndices = arrayfun(@(x) x.dofnum, bodies, 'UniformOutput', false)';
vIndices = cell2mat(allVIndices(jointIndices));
end