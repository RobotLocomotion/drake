function  X = Xrotz( theta )

% Xrotz  spatial coordinate transform (Z-axis rotation).
% Xrotz(theta) calculates the coordinate transform matrix from A to B
% coordinates for spatial motion vectors, where coordinate frame B is
% rotated by an angle theta (radians) relative to frame A about their
% common Z axis.

c = cos(theta);
s = sin(theta);
% 
% X = [  c  s  0  0  0  0 ;
%       -s  c  0  0  0  0 ;
%        0  0  1  0  0  0 ;
%        0  0  0  c  s  0 ;
%        0  0  0 -s  c  0 ;
%        0  0  0  0  0  1
%     ];

X = 0*theta(1)+eye(6);
X_ind = sub2ind([6 6],[1,1,2,2,4,4,5,5],[1,2,1,2,4,5,4,5]);
X(X_ind) = [c,s,-s,c,c,s,-s,c];