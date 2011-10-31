function  X = Xrotx( theta )

% Xrotx  spatial coordinate transform (X-axis rotation).
% Xrotx(theta) calculates the coordinate transform matrix from A to B
% coordinates for spatial motion vectors, where coordinate frame B is
% rotated by an angle theta (radians) relative to frame A about their
% common X axis.

c = cos(theta);
s = sin(theta);

X = [ 1  0  0  0  0  0 ;
      0  c  s  0  0  0 ;
      0 -s  c  0  0  0 ;
      0  0  0  1  0  0 ;
      0  0  0  0  c  s ;
      0  0  0  0 -s  c
    ];
