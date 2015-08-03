function  vcross = crmp( v )

% crmp  planar cross-product operator (motion).
% crmp(v) calculates the 3x3 matrix such that the expression crmp(v)*m is
% the cross product of the planar motion vectors v and m.

vcross = [  0     0     0    ;
	    v(3)  0    -v(1) ;
	   -v(2)  v(1)  0
	 ];
