function  X = Xrotz( theta )

% Xrotz  spatial coordinate transform (Z-axis rotation).
% Xrotz(theta) calculates the coordinate transform matrix from A to B
% coordinates for spatial motion vectors, where coordinate frame B is
% rotated by an angle theta (radians) relative to frame A about their
% common Z axis.

c = cos(theta);
s = sin(theta);

X = [  c  s  0  0  0  0 ;
      -s  c  0  0  0  0 ;
       0  0  1  0  0  0 ;
       0  0  0  c  s  0 ;
       0  0  0 -s  c  0 ;
       0  0  0  0  0  1
    ];
