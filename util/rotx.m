function M = rotx(theta)
% 3D rotation matrix (about the x axis)

c=cos(theta); 
s=sin(theta);
M = [1,0,0; 0,c,-s; 0,s,c];
