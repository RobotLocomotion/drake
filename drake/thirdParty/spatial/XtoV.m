function v = XtoV( X )

% XtoV  obtain spatial vector from small-angle transform.
% XtoV(X) interprets X as the coordinate transform from A to B coordinates,
% and calculates the constant spatial velocity of a third frame, C(t), that
% travels from A to B in one time unit.  Thus, C(0)=A, C(1)=B, dC/dt is a
% constant, and the return value is the spatial velocity of C.  The
% calculation uses a small-angle approximation, and is therefore only exact
% if A and B are parallel.  The return value, v, is an invariant of X
% (i.e., v=X*v), and can therefore be regarded as being expressed in either
% A or B coordinates.

v = 0.5 * [ X(2,3) - X(3,2);
	    X(3,1) - X(1,3);
	    X(1,2) - X(2,1);
	    X(5,3) - X(6,2);
	    X(6,1) - X(4,3);
	    X(4,2) - X(5,1) ];
