function pose = tform2poseRPY(T)
% Convert a homogeneous transformation matrix to a 6-DOF pose

pose = [T(1:3,4); rotmat2rpy(T(1:3,1:3))];
