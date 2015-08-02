function  rbi = mcI( m, c, I )

% mcI  spatial rigid-body inertia from mass, CoM and rotational inertia.
% mcI(m,c,I) calculates the spatial inertia matrix of a rigid body from its
% mass, centre of mass (3D vector) and rotational inertia (3x3 matrix)
% about its centre of mass.

C = [  0,    -c(3),  c(2);
       c(3),  0,    -c(1);
      -c(2),  c(1),  0 ];

rbi = [ I + m*C*C', m*C; m*C', m*eye(3) ];
