function pose = fitPoseToNormal(pose, normal)
% Given a 6 DOF pose and a normal, reorient the pose to match the normal but preserve its intrinsic yaw.

sizecheck(pose(:,1), [6,1]);

for j = 1:length(pose(1,:))
  M0 = rpy2rotmat(pose(4:6,j));
  x0 = M0 * [1;0;0];
  z0 = M0 * [0;0;1];
  ax = cross(z0, normal(:,j));
  costheta = normal(:,j)' * z0 / norm(normal(:,j));
  theta = real(acos(costheta));
  q = axis2quat([ax;theta]);
  Mf = quat2rotmat(q);
  xf = Mf * x0;
  zf = normal(:,j);
  yf = cross(zf, xf);
  Mx = xf / norm(xf);
  Mz = cross(xf, yf);
  Mz = Mz / norm(Mz);
  My = cross(Mz, xf);
  My = My / norm(My);
  M = [Mx, My, Mz];
  new_rpy = [atan2(M(3,2),M(3,3)); ... 
             atan2(-M(3,1),sqrt(M(3,2)^2 + M(3,3)^2)); ...
             atan2(M(2,1),M(1,1)) ];
  if ~any(isnan(new_rpy))
    pose(4:5,j) = new_rpy(1:2);
  end
end
