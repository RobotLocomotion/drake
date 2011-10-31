function M = roty(theta)
% 3D rotation matrix (about the Y axis)

c=cos(theta); 
s=sin(theta);
M = [c,0,-s; 0,1,0; s,0,c];
