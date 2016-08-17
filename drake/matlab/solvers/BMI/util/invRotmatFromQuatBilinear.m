function R = invRotmatFromQuatBilinear(Quat)
% For a quaternion product Quat = quat*quat', return the matrix that would be
% the inverse of the rotation matrix corresponding to quat
Q_inv = [1 -ones(1,3);-ones(3,1) eye(3)].*Quat;
R = rotmatFromQuatBilinear(Q_inv);
end