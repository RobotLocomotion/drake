function  X = Xpln( theta, r )

% Xpln  coordinate transform for planar vectors.
% Xpln(theta,r) calculates the coordinate transform matrix from A to B
% coordinates for planar motion vectors, where coordinate frame B is
% located at point r (2D vector expressed in A coords) relative to frame A,
% and is rotated by an angle theta (radians) relative to A.

c = cos(theta);
s = sin(theta);

X = [  1               0  0 ;
       s*r(1)-c*r(2)   c  s ;
       c*r(1)+s*r(2)  -s  c
    ];
