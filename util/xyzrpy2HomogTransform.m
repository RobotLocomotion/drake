% computes the homogeneous transform from the given 6-DOF position to the world frame
function T = xyzrpy2HomogTransform(pos)
  T = zeros(4,4);
  T(1:3,1:3) = rpy2rotmat(pos(4:6));
  T(1:3,4) = pos(1:3);
  T(4,4) = 1;
end