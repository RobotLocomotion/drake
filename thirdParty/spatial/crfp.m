function  vcross = crfp( v )

% crfp  planar cross-product operator (force).
% crfp(v) calculates the 3x3 matrix such that the expression crfp(v)*f is
% the cross product of the planar motion vector v with the planar force
% vector f.

vcross = -crmp(v)';
