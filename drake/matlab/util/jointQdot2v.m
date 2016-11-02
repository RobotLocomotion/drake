function [VqJoint, dVqJoint] = jointQdot2v(body, q_body)
% Computes the matrix that maps from the derivative of the position vector
% of a joint associated with a RigidBody object to its velocity vector.
%
% @param body a RigidBody object
% @param q_body joint configuration vector for body
% @retval VqJoint matrix such that v = Vq * qd, where v is the velocity
% vector for body and qd is the derivative of the configuration vector for
% body
% @retval dVqJoint gradient output

compute_gradient = nargout > 1;

if body.floating == 1
  VqJoint = eye(6);
  if compute_gradient
    dVqJoint = zeros(numel(VqJoint), size(VqJoint, 2));
  end
elseif body.floating == 2
  quat = q_body(4 : 7);
  
  if compute_gradient
    [~, dquattildedquat, ddquattildedquat] = normalizeVec(quat);
    [R, dR] = quat2rotmat(quat);
    [M, dM] = quatdot2angularvelMatrix(quat);
  else
    [~, dquattildedquat] = normalizeVec(quat);
    R = quat2rotmat(quat);
    M = quatdot2angularvelMatrix(quat);
  end
  
  RTransposeM = R' * M; % TODO: directly use body frame representation
  VqJoint = [...
    zeros(3, 3), RTransposeM * dquattildedquat;
    R', zeros(3, 4)];
  
  if compute_gradient
    dRTranspose = transposeGrad(dR, size(R));
    dRTransposeM = matGradMultMat(R', M, dRTranspose, dM);
    ddquattildedquat = reshape(ddquattildedquat, numel(dquattildedquat), numel(quat));
    dRTransposeMdquattildedquat = matGradMultMat(RTransposeM, dquattildedquat, dRTransposeM, ddquattildedquat);
    dVqJoint = zeros(numel(VqJoint), length(q_body)) * q_body(1); % for TaylorVar
    dVqJoint = setSubMatrixGradient(dVqJoint, dRTranspose, 4:6, 1:3, size(VqJoint), 4:7);
    dVqJoint = setSubMatrixGradient(dVqJoint, dRTransposeMdquattildedquat, 1:3, 4:7, size(VqJoint), 4:7);
  end
elseif body.floating ~= 0
  error('case not handled');
else
  VqJoint = 1;
  if compute_gradient
    dVqJoint = 0;
  end
end

end

