function  rbi = mcIp( m, c, I )

% mcIp  planar rigid-body inertia from mass, CoM and rotational inertia.
% mcIp(m,c,I) calculates the planar inertia matrix of a rigid body from its
% mass, centre of mass (2D vector) and rotational inertia (scalar) about
% its centre of mass.

rbi = [  I+m*dot(c,c), -m*c(2), m*c(1);
	  -m*c(2),        m,      0;
	   m*c(1),        0,      m
      ];
