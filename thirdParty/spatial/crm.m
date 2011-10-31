function  vcross = crm( v )

% crm  spatial cross-product operator (motion).
% crm(v) calculates the 6x6 matrix such that the expression crm(v)*m is the
% cross product of the spatial motion vectors v and m.

vcross = [  0    -v(3)  v(2)   0     0     0    ;
	    v(3)  0    -v(1)   0     0     0    ;
	   -v(2)  v(1)  0      0     0     0    ;
	    0    -v(6)  v(5)   0    -v(3)  v(2) ;
	    v(6)  0    -v(4)   v(3)  0    -v(1) ;
	   -v(5)  v(4)  0     -v(2)  v(1)  0
	 ];
