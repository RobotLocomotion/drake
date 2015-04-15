function T = xyz_quat_2_tform(xyzquat)
  T = zeros(4,4);
  R = quat2rotmat(xyzquat(4:7));
  T(1:3,1:3) = R;
  T(1:3,4) = xyzquat(1:3);
  T(4,4) = 1;
end

