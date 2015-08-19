function [VqInvJoint, dVqInvJoint] = jointV2qdot(body, q_body)
% Computes the matrix that maps from the velocity vector of a joint
% associated with a RigidBody object to the derivative of its configuration
% vector.
%
% @param body a RigidBody object
% @param q_body joint configuration vector for body
% @retval VqInvJoint matrix such that qd = Vq * v, where v is the velocity
% vector for body and qd is the derivative of the configuration vector for
% body.
% @retval dVqInvJoint gradient output

compute_gradient = nargout > 1;

if body.floating == 1
  VqInvJoint = eye(6);
  if compute_gradient
    dVqInvJoint = zeros(numel(VqInvJoint), size(VqInvJoint, 1));
  end
elseif body.floating == 2
  quat = q_body(4 : 7);
  if compute_gradient
    [R, dR] = quat2rotmat(quat);
    [M, dM] = angularvel2quatdotMatrix(quat);
  else
    R = quat2rotmat(quat);
    M = angularvel2quatdotMatrix(quat);
  end
  
  VqInvJoint = [zeros(3, 3), R;
    M * R, zeros(4, 3)];
  
  if compute_gradient
    dVqInvJoint = zeros(numel(VqInvJoint), size(VqInvJoint, 1)) * q_body(1);
    dVqInvJoint = setSubMatrixGradient(dVqInvJoint, dR, 1:3, 4:6, size(VqInvJoint), 4:7);
    dMR = matGradMultMat(M, R, dM, dR);
    dVqInvJoint = setSubMatrixGradient(dVqInvJoint, dMR, 4:7, 1:3, size(VqInvJoint), 4:7);
  end
elseif body.floating ~= 0
  error('case not handled');
else
  VqInvJoint = 1;
  if compute_gradient
    dVqInvJoint = 0;
  end
end
end