function [VqInvJoint, dVqInvJoint] = jointVToqdot(body, q_body)
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