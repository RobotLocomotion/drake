function  v = XtoVp( X )

% XtoVp  obtain planar vector from small-angle transform.
% XtoVp(X) interprets X as the coordinate transform from A to B
% coordinates, and calculates the constant planar velocity of a third
% frame, C(t), that travels from A to B in one time unit.  Thus, C(0)=A,
% C(1)=B and dC/dt is a constant.  The return value is the planar velocity
% of C.  The calculation uses a small-angle approximation, and is therefore
% only exact if A and B are parallel.  The return value is an invariant of
% X (i.e., v=X*v), and can therefore be regarded as being expressed in
% either A or B coordinates.

v = [  X(2,3);
      (X(3,1) + X(2,2)*X(3,1) + X(2,3)*X(2,1))/2;
     (-X(2,1) - X(2,2)*X(2,1) + X(2,3)*X(3,1))/2  ];
