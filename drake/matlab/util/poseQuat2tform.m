function T = poseQuat2tform(pose)
% Convert a pose specified as [xyz; quaternion] to a homogeneous transformation matrix

T = [quat2rotmat(pose(4:7)), reshape(pose(1:3), 3, 1); [0,0,0,1]];