function [S, dSdqbody] = motionSubspace(body, qbody)
% Computes the `motion subspace' of a joint, expressed in joint successor
% body frame, i.e. the matrix that maps the joint's velocity vector to the
% twist of the its successor with respect to its predecessor, expressed
% in the joint successor body frame.
%
% @param body a RigidBody object
% @param qbody the joint configuration vector associated with \p body
%
% @retval S the motion subspace associated with the joint of which \p body
% is the successor
% @retval dSdq the gradient of S with respect to qbody

if body.floating == 1
  % configuration parameterization: origin position in base frame and rpy
  % velocity parameterization: origin velocity in base frame and rpydot
  
  rpy = qbody(4 : 6);
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
    dSdqbody = reshape([0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,sr.*sy+cr.*cy.*sp,cr.*sy-cy.*sp.*sr,0,0,0,0,-cy.*sr+cr.*sp.*sy,-cr.*cy-sp.*sr.*sy,0,0,0,0,cp.*cr,-cp.*sr,0,0,0,0,0,0,0,-sr,-cr,0,0,0,0,cp.*cr,-cp.*sr,0,0,0,0,0,0,-cy.*sp,cp.*cy.*sr,cp.*cr.*cy,0,0,0,-sp.*sy,cp.*sr.*sy,cp.*cr.*sy,0,0,0,-cp,-sp.*sr,-cr.*sp,0,0,0,0,0,0,0,0,0,0,0,0,-cp,-sp.*sr,-cr.*sp,0,0,0,0,0,0,-cp.*sy,-cr.*cy-sp.*sr.*sy,cy.*sr-cr.*sp.*sy,0,0,0,cp.*cy,-cr.*sy+cy.*sp.*sr,sr.*sy+cr.*cy.*sp,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],[36,6]);
  end
elseif body.floating == 2
  % configuration parameterization: origin position in base frame and quat
  % velocity parameterization: twist in body frame
  
  S = eye(6);
  
  if nargout > 1
    dSdqbody = zeros(numel(S), 7);
  end
elseif body.floating ~= 0
  error('Drake:RigidBodyManipulator:NotImplemented','Motion subspace not implemented for this type of floating joint.');
else
  % one-dof joint
  p = body.pitch;
  axis = body.joint_axis;
  if isinf(p) % prismatic joint
    S = [zeros(3, 1); axis];
  else % helical or revolute joint
    S = [axis; p * axis];
  end
  
  if nargout > 1
    dSdqbody = zeros(numel(S), 1);
  end
end
end