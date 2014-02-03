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