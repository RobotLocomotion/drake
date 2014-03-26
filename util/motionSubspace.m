function S = motionSubspace(body, qBody)
if body.floating == 1
  % configuration parameterization: origin position in base frame and rpy
  % velocity parameterization: origin velocity in base frame and rpydot
  
  rpy = qBody(4 : 6);
  E = rpydot2angularvelMatrix(rpy);
  R = rpy2rotmat(rpy);
  S = [zeros(3, 3), R' * E;
    R' , zeros(3, 3)];
elseif body.floating == 2
  % configuration parameterization: origin position in base frame and quat
  % velocity parameterization: twist in body frame
  
  S = eye(6);

  % previously:
%   % configuration parameterization: origin position in base frame and quat
%   % velocity parameterization: origin velocity and quatdot
%   quat = qBody(4 : 7);
%   R = quat2rotmat(quat);
%   S = [zeros(3, 3), R' * quatdot2angularvelMatrix(quat);
%        R', zeros(3, 4)];
%   warning('Drake:RigidBodyManipulator:NotTested','Motion subspace not tested for case of quaternion floating joint.');
elseif body.floating ~= 0
  error('Drake:RigidBodyManipulator:NotImplemented','Motion subspace not implemented for this type of floating joint.');
else
  % one dof joint
  
  pitch = body.pitch;
  axis = body.joint_axis;
  if isinf(pitch) % prismatic joint
    S = [zeros(3, 1); axis];
  else % helical or revolute joint
    S = [axis; pitch * axis];
  end
end
end