function M = rotz(theta)
% 3D rotation matrix (about the z axis)

c=cos(theta); 
s=sin(theta);
M = [c,-s,0; s,c,0; 0,0,1];
