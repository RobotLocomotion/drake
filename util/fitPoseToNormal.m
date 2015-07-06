function pose = fitPoseToNormal(pose, normal)
% Given a 6 DOF pose and a normal, reorient the pose to match the normal but preserve its intrinsic yaw.

normal = normal / norm(normal);

for j = 1:size(pose, 2)
  M = rpy2rotmat(pose(4:6,j));
  n = M * [0;0;1];
  ax = cross(n, normal);
  sin_theta = norm(ax);
  if sin_theta > 1e-6
    theta = asin(sin_theta);
    R = axis2rotmat([reshape(ax, 3, 1); theta]);
    M = R * M;
    pose(4:6) = rotmat2rpy(M);
  end
end
