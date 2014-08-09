function [Vq, dVq] = jointQdotTov(body, q_body)
compute_gradient = nargout > 1;

if body.floating == 1
  Vq = eye(6);
  if compute_gradient
    dVq = zeros(numel(Vq), size(Vq, 2));
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
  Vq = [...
    zeros(3, 3), RTransposeM * dquattildedquat;
    R', zeros(3, 4)];
  
  if compute_gradient
    dRTranspose = transposeGrad(dR, size(R));
    dRTransposeM = matGradMultMat(R', M, dRTranspose, dM);
    ddquattildedquat = reshape(ddquattildedquat, numel(dquattildedquat), numel(quat));
    dRTransposeMdquattildedquat = matGradMultMat(RTransposeM, dquattildedquat, dRTransposeM, ddquattildedquat);
    dVq = zeros(numel(Vq), length(q_body)) * q_body(1); % for TaylorVar
    dVq = setSubMatrixGradient(dVq, dRTranspose, 4:6, 1:3, size(Vq), 4:7);
    dVq = setSubMatrixGradient(dVq, dRTransposeMdquattildedquat, 1:3, 4:7, size(Vq), 4:7);
  end
elseif body.floating ~= 0
  error('case not handled');
else
  Vq = 1;
  if compute_gradient
    dVq = 0;
  end
end

end

