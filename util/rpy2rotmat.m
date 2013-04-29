function rotMat = rpy2rotmat(rpy)
% use LaValle's order, see http://planning.cs.uiuc.edu/ch3.pdf page 99 for
% reference
rotMat = [cos(rpy(3))*cos(rpy(2)) cos(rpy(3))*sin(rpy(2))*sin(rpy(1))-sin(rpy(3))*cos(rpy(1)) cos(rpy(3))*sin(rpy(2))*cos(rpy(1))+sin(rpy(3))*sin(rpy(1));...
    sin(rpy(3))*cos(rpy(2)) sin(rpy(3))*sin(rpy(2))*sin(rpy(1))+cos(rpy(3))*cos(rpy(1)) sin(rpy(3))*sin(rpy(2))*cos(rpy(1))-cos(rpy(3))*sin(rpy(1));...
    -sin(rpy(2)) cos(rpy(2))*sin(rpy(1)) cos(rpy(2))*cos(rpy(1))];
end