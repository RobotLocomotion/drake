function T = poseRPY2tform(pose)
% Convert a 6-DOF pose to a homogeneous transformation matrix

T = [rpy2rotmat(pose(4:6)), reshape(pose(1:3), 3, 1); [0,0,0,1]];
