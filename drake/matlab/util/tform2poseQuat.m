function pose_quat = tform2poseQuat(T)
% Convert a homogeneous transformation matrix to a pose specified as [xyz; quaternion]

pose_quat = [T(1:3,4); rotmat2quat(T(1:3,1:3))];