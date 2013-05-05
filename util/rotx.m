function [M,dM,ddM] = rotx(theta)
% 3D rotation matrix (about the x axis)

c=cos(theta); 
s=sin(theta);
M = [1,0,0; 0,c,-s; 0,s,c];
dM = [0,0,0; 0,-s,-c; 0,c,-s];
ddM = [0,0,0; 0,-c,s; 0,-s,-c];