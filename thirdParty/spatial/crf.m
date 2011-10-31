function  vcross = crf( v )

% crf  spatial cross-product operator (force).
% crf(v) calculates the 6x6 matrix such that the expression crf(v)*f is the
% cross product of the spatial motion vector v with the spatial force
% vector f.

vcross = -crm(v)';
