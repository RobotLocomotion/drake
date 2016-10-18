function [rpy, drpy] = rotmat2rpy(R, dR)

compute_gradient = nargout > 1;
if compute_gradient && nargin < 2
  error('need dR to compute drpy');
end

if sizecheck(R,[2 2])
  rpy = atan2(R(2,1),R(1,1));
  if compute_gradient
    sqterm = R(1,1)^2 + R(2,1)^2;
    drpy = [ -R(2,1)/sqterm, R(1,1)/sqterm, 0, 0] * dR;
  end
else
  % NOTE: assumes we're using an X-Y-Z convention to construct R
  rpy = [atan2(R(3,2),R(3,3)); ...
    atan2(-R(3,1),sqrt(R(3,2)^2 + R(3,3)^2)); ...
    atan2(R(2,1),R(1,1)) ];
  
  if compute_gradient
    nq = size(dR, 2);
    drpy = zeros(numel(rpy), nq) * R(1);
    
    dR11_dq = getSubMatrixGradient(dR,1,1,size(R));
%     dR12_dq = getSubMatrixGradient(dR,1,2,size(R));
%     dR13_dq = getSubMatrixGradient(dR,1,3,size(R));
    dR21_dq = getSubMatrixGradient(dR,2,1,size(R));
%     dR22_dq = getSubMatrixGradient(dR,2,2,size(R));
%     dR23_dq = getSubMatrixGradient(dR,2,3,size(R));
    dR31_dq = getSubMatrixGradient(dR,3,1,size(R));
    dR32_dq = getSubMatrixGradient(dR,3,2,size(R));
    dR33_dq = getSubMatrixGradient(dR,3,3,size(R));

    sqterm = R(3,2)^2 + R(3,3)^2;

    % droll_dq
    drpy(1,:) = (R(3,3)*dR32_dq - R(3,2)*dR33_dq)/sqterm;
    
    % dpitch_dq
    drpy(2,:) = (-sqrt(sqterm)*dR31_dq + R(3,1)/sqrt(sqterm)*(R(3,2)*dR32_dq + R(3,3)*dR33_dq) )/(R(3,1)^2 + R(3,2)^2 + R(3,3)^2);
    
    % dyaw_dq
    sqterm = R(1,1)^2 + R(2,1)^2;
    drpy(3,:) = (R(1,1)*dR21_dq - R(2,1)*dR11_dq)/sqterm;
    
  end
end

