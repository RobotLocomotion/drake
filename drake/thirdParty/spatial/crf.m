function  vcross = crf( v )

% crf  spatial cross-product operator (force).
% crf(v) calculates the 6x6 matrix such that the expression crf(v)*f is the
% cross product of the spatial motion vector v with the spatial force
% vector f.


%vcross = -crm(v)';

vcross = [  0    -v(3)  v(2)   0     -v(6)    v(5)  ;
	    v(3)  0    -v(1)  v(6)   0     -v(4)    ;
	   -v(2)  v(1)  0     -v(5)  v(4)     0    ;
	    0     0     0     0     -v(3)  v(2) ;
	    0     0     0     v(3)   0    -v(1) ;
	    0     0     0     -v(2)  v(1)  0
	 ];
