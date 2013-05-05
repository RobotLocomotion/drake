function rotMat = rpy2rotmat(rpy)
% equivalent to rotz(rpy(3))*roty(rpy(2))*rotx(rpy(1))

rotMat = [cos(rpy(3))*cos(rpy(2)) cos(rpy(3))*sin(rpy(2))*sin(rpy(1))-sin(rpy(3))*cos(rpy(1)) cos(rpy(3))*sin(rpy(2))*cos(rpy(1))+sin(rpy(3))*sin(rpy(1));...
    sin(rpy(3))*cos(rpy(2)) sin(rpy(3))*sin(rpy(2))*sin(rpy(1))+cos(rpy(3))*cos(rpy(1)) sin(rpy(3))*sin(rpy(2))*cos(rpy(1))-cos(rpy(3))*sin(rpy(1));...
    -sin(rpy(2)) cos(rpy(2))*sin(rpy(1)) cos(rpy(2))*cos(rpy(1))];
  
end