function  X = Xtrans( r )

% Xtrans  spatial coordinate transform (translation of origin).
% Xtrans(r) calculates the coordinate transform matrix from A to B
% coordinates for spatial motion vectors, in which frame B is translated by
% an amount r (3D vector) relative to frame A.

% X = [  1     0     0    0  0  0 ;
%        0     1     0    0  0  0 ;
%        0     0     1    0  0  0 ;
%        0     r(3) -r(2) 1  0  0 ;
%       -r(3)  0     r(1) 0  1  0 ;
%        r(2) -r(1)  0    0  0  1
%     ];

X = 0*r(1)+eye(6);
X_ind = sub2ind([6 6],[4,4,5,5,6,6],[2,3,1,3,1,2]);
X(X_ind) = [r(3),-r(2),-r(3),r(1),r(2),-r(1)];