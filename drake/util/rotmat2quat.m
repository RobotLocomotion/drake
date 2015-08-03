function [q, dq] = rotmat2quat(R, dR)
% convert rotation matrix to quaternion, based on 
% http://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/
% Pay close attention that there are two quaternions for the same rotation
% matrix!!!, namely quat2rotmat(q) = quat2rotmat(-q).

compute_gradient = nargout > 1;
if compute_gradient && nargin < 2
  error('need dM to compute dq');
end

[val,ind] = max([1 1 1; 1 -1 -1; -1 1 -1; -1 -1 1]*diag(R));

switch(ind)
  case 1  % val = trace(M)
    w = sqrt(1+val)/2;
    w4 = w*4;
    x = (R(3,2)-R(2,3))/w4;
    y = (R(1,3)-R(3,1))/w4;
    z = (R(2,1)-R(1,2))/w4;
  case 2 % val = M(1,1) - M(2,2) - M(3,3)
    s = 2*sqrt(1+val);
    w = (R(3,2)-R(2,3))/s;
    x = 0.25*s;
    y = (R(1,2)+R(2,1))/s;
    z = (R(1,3)+R(3,1))/s;
  case 3 % val = M(2,2) - M(1,1) - M(3,3)
    s = 2*(sqrt(1+val));
    w = (R(1,3)-R(3,1))/s;
    x = (R(1,2)+R(2,1))/s;
    y = 0.25*s;
    z = (R(2,3)+R(3,2))/s;
  otherwise % val = M(3,3) - M(2,2) - M(1,1) 
    s = 2*(sqrt(1+val));
    w = (R(2,1)-R(1,2))/s;
    x = (R(1,3)+R(3,1))/s;
    y = (R(2,3)+R(3,2))/s;
    z = 0.25*s;
end

q = [w;x;y;z];

if compute_gradient
  dR11_dq = getSubMatrixGradient(dR,1,1,size(R));
  dR12_dq = getSubMatrixGradient(dR,1,2,size(R));
  dR13_dq = getSubMatrixGradient(dR,1,3,size(R));
  dR21_dq = getSubMatrixGradient(dR,2,1,size(R));
  dR22_dq = getSubMatrixGradient(dR,2,2,size(R));
  dR23_dq = getSubMatrixGradient(dR,2,3,size(R));
  dR31_dq = getSubMatrixGradient(dR,3,1,size(R));
  dR32_dq = getSubMatrixGradient(dR,3,2,size(R));
  dR33_dq = getSubMatrixGradient(dR,3,3,size(R));
  
  switch(ind)
    case 1  % val = trace(M)
      dvaldq = dR11_dq + dR22_dq + dR33_dq;
      dwdq = dvaldq/(4*sqrt(1+val));
      w = q(1);
      wsquare4 = 4*w^2;
      dxdq = ((dR32_dq-dR23_dq)*w-(R(3,2)-R(2,3))*dwdq)/wsquare4;
      dydq = ((dR13_dq-dR31_dq)*w-(R(1,3)-R(3,1))*dwdq)/wsquare4;
      dzdq = ((dR21_dq-dR12_dq)*w-(R(2,1)-R(1,2))*dwdq)/wsquare4;
    case 2 % val = M(1,1) - M(2,2) - M(3,3)
      dvaldq = dR11_dq - dR22_dq - dR33_dq;
      s = 2*sqrt(1+val); ssquare = s^2;
      dsdq = dvaldq/sqrt(1+val);
      dwdq = ((dR32_dq-dR23_dq)*s - (R(3,2)-R(2,3))*dsdq)/ssquare; % w = (M(3,2)-M(2,3))/s;
      dxdq = .25*dsdq; % qx = 0.25*s;
      dydq = ((dR12_dq+dR21_dq)*s - (R(1,2)+R(2,1))*dsdq)/ssquare; % y = (M(1,2)+M(2,1))/s;
      dzdq = ((dR13_dq+dR31_dq)*s - (R(1,3)+R(3,1))*dsdq)/ssquare; % z = (M(1,3)+M(3,1))/s;
    case 3 % val = M(2,2) - M(1,1) - M(3,3)
      dvaldq = - dR11_dq + dR22_dq - dR33_dq;
      s = 2*(sqrt(1+val)); ssquare = s^2;
      dsdq = dvaldq/sqrt(1+val);
      dwdq = ((dR13_dq-dR31_dq)*s - (R(1,3)-R(3,1))*dsdq)/ssquare; % w = (M(1,3)-M(3,1))/s;
      dxdq = ((dR12_dq+dR21_dq)*s - (R(1,2)+R(2,1))*dsdq)/ssquare; % x = (M(1,2)+M(2,1))/s;
      dydq = .25*dsdq; % y = 0.25*s;
      dzdq = ((dR23_dq+dR32_dq)*s - (R(2,3)+R(3,2))*dsdq)/ssquare; % z = (M(2,3)+M(3,2))/s;
    otherwise % val = M(3,3) - M(2,2) - M(1,1)
      dvaldq = - dR11_dq - dR22_dq + dR33_dq;
      s = 2*(sqrt(1+val)); ssquare = s^2;
      dsdq = dvaldq/sqrt(1+val);
      dwdq = ((dR21_dq-dR12_dq)*s - (R(2,1)-R(1,2))*dsdq)/ssquare; % w = (M(2,1)-M(1,2))/s;
      dxdq = ((dR13_dq+dR31_dq)*s - (R(1,3)+R(3,1))*dsdq)/ssquare; % x = (M(1,3)+M(3,1))/s;
      dydq = ((dR23_dq+dR32_dq)*s - (R(2,3)+R(3,2))*dsdq)/ssquare; % y = (M(2,3)+M(3,2))/s;
      dzdq = .25*dsdq; % z = 0.25*s;
  end
  dq = [dwdq;dxdq;dydq;dzdq];
end

end