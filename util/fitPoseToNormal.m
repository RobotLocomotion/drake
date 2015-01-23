function pos = fitPoseToNormal(pos, normal)
% Given a 6 DOF pose and a normal, reorient the pose to match the normal but preserve its intrinsic yaw.

sizecheck(pos(:,1), [6,1]);

for j = 1:length(pos(1,:))
  M0 = rpy2rotmat(pos(4:6,j));
  x0 = M0 * [1;0;0];
  z0 = M0 * [0;0;1];
  ax = fastCross3(z0, normal(:,j));
  costheta = normal(:,j)' * z0 / norm(normal(:,j));
  theta = real(acos(costheta));
  q = axis2quat([ax;theta]);
  Mf = quat2rotmat(q);
  xf = Mf * x0;
  zf = normal(:,j);
  yf = fastCross3(zf, xf);
  Mx = xf / norm(xf);
  Mz = fastCross3(xf, yf);
  Mz = Mz / norm(Mz);
  My = fastCross3(Mz, xf);
  My = My / norm(My);
  M = [Mx, My, Mz];
  new_rpy = [atan2(M(3,2),M(3,3)); ... 
             atan2(-M(3,1),sqrt(M(3,2)^2 + M(3,3)^2)); ...
             atan2(M(2,1),M(1,1)) ];
  if ~any(isnan(new_rpy))
    pos(4:5,j) = new_rpy(1:2);
  end
end
