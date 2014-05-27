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
%     syms r p y real;
%     syms x y z real;
%     qBody = [x; y; z; r; p; y];
%     S = motionSubspace(body, qBody);
%     dSdq = jacobian(S(:), qBody);
%     matlabFunction(dSdq)
    dSdq = reshape([0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,-cp*sy,-cr*cy-sp*sr*sy,cy*sr-cr*sp*sy,0,0,0,cp*cy,-cr*sy+cy*sp*sr,sr*sy+cr*cy*sp,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,sr*sy+cr*cy*sp,cr*sy-cy*sp*sr,0,0,0,0,-cy*sr+cr*sp*sy,-cr*cy-sp*sr*sy,0,0,0,0,cp*cr,-cp*sr,0,cp*cy*(sr*sy+cr*cy*sp)-cp*sy*(cy*sr-cr*sp*sy)-cp*cr*sp,cp*cy*(cr*sy-cy*sp*sr)-cp*sy*(cr*cy+sp*sr*sy)+cp*sp*sr,0,0,0,0,-cy*(cy*sr-cr*sp*sy)-sy*(sr*sy+cr*cy*sp),-cy*(cr*cy+sp*sr*sy)-sy*(cr*sy-cy*sp*sr),0,0,0,0,cp*cr,-cp*sr,0,0,0,0,0,0,-cy*sp,cp*cy*sr,cp*cr*cy,0,0,0,-sp*sy,cp*sr*sy,cp*cr*sy,0,0,0,-cp,-sp*sr,-cr*sp,cp*sp*2.0-cp*cy.^2*sp*2.0-cp*sp*sy.^2*2.0,-cp.^2*sr+sp.^2*sr+cy*sp*(cr*sy-cy*sp*sr)+cp.^2*cy.^2*sr-sp*sy*(cr*cy+sp*sr*sy)+cp.^2*sr*sy.^2,-cp.^2*cr+cr*sp.^2+cp.^2*cr*cy.^2-cy*sp*(sr*sy+cr*cy*sp)+cp.^2*cr*sy.^2+sp*sy*(cy*sr-cr*sp*sy),0,0,0,0,0,0,0,0,0,-cp,-sp*sr,-cr*sp,0,0,0,0,0,0,-cp*sy,-cr*cy-sp*sr*sy,cy*sr-cr*sp*sy,0,0,0,cp*cy,-cr*sy+cy*sp*sr,sr*sy+cr*cy*sp,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],[36,6]);
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
  
  pitch = body.pitch;
  axis = body.joint_axis;
  if isinf(pitch) % prismatic joint
    S = [zeros(3, 1); axis];
  else % helical or revolute joint
    S = [axis; pitch * axis];
  end
  
  if nargout > 1
    dSdq = zeros(numel(S), 1);
  end
end
end