function [M,dM,ddM] = roty(theta)
% 3D rotation matrix (about the Y axis)
% Note that, because we are using vehicle notation (where positive y is
% INTO the screen when viewing the x-z plane), the signs of this are
% flipped from the usual rotation matrix convention.

theta=-theta; % flip the signs!

c=cos(theta); 
s=sin(theta);
M = [c,0,-s; 0,1,0; s,0,c];
dM = -[-s,0,-c; 0,0,0; c,0,-s];
ddM = [-c,0,s; 0,0,0; -s,0,-c];