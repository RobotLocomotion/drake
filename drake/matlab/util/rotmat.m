function M = rotmat(theta)
% 2D rotation matrix

c=cos(theta); 
s=sin(theta);
M = [c,-s; s,c];
