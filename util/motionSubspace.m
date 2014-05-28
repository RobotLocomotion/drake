function [S, dSdq] = motionSubspace(body, qBody)
if body.floating == 1
  % configuration parameterization: origin position in base frame and rpy
  % velocity parameterization: origin velocity in base frame and rpydot
  
  rpy = qBody(4 : 6);
  E = rpydot2angularvelMatrix(rpy);
  R = rpy2rotmat(rpy);
  S = [zeros(3, 3), R' * E;
    R' , zeros(3, 3)];
  
  % TODO: clean up
  if nargout > 1
    [roll, pitch, yaw] = deal(rpy(1), rpy(2), rpy(3));
    cr = cos(roll);
    sr = sin(roll);
    cp = cos(pitch);
    sp = sin(pitch);
    cy = cos(yaw);
    sy = sin(yaw);
    
%     Generated using:
%     body.floating = 1;
%     syms roll pitch yaw real;
%     syms x y z real;
%     qBody = [x; y; z; roll; pitch; yaw];
%     S = motionSubspace(body, qBody);
%     dSdq = jacobian(S(:), qBody);
%     matlabFunction(simple(dSdq))
    dSdq = reshape([0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,sr.*sy+cr.*cy.*sp,cr.*sy-cy.*sp.*sr,0,0,0,0,-cy.*sr+cr.*sp.*sy,-cr.*cy-sp.*sr.*sy,0,0,0,0,cp.*cr,-cp.*sr,0,0,0,0,0,0,0,-sr,-cr,0,0,0,0,cp.*cr,-cp.*sr,0,0,0,0,0,0,-cy.*sp,cp.*cy.*sr,cp.*cr.*cy,0,0,0,-sp.*sy,cp.*sr.*sy,cp.*cr.*sy,0,0,0,-cp,-sp.*sr,-cr.*sp,0,0,0,0,0,0,0,0,0,0,0,0,-cp,-sp.*sr,-cr.*sp,0,0,0,0,0,0,-cp.*sy,-cr.*cy-sp.*sr.*sy,cy.*sr-cr.*sp.*sy,0,0,0,cp.*cy,-cr.*sy+cy.*sp.*sr,sr.*sy+cr.*cy.*sp,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],[36,6]);
  end

elseif body.floating == 2
  % configuration parameterization: origin position in base frame and quat
  % velocity parameterization: twist in body frame
  
  S = eye(6);
  
  if nargout > 1
    dSdq = zeros(numel(S), 7);
  end

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
  
  p = body.pitch;
  axis = body.joint_axis;
  if isinf(p) % prismatic joint
    S = [zeros(3, 1); axis];
  else % helical or revolute joint
    S = [axis; p * axis];
  end
  
  if nargout > 1
    dSdq = zeros(numel(S), 1);
  end
end
end