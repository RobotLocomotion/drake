function [A, b] = poly2lincon(X,Y)
% Convert a 2D polygon represented by coordinates X and Y into a set of linear constraints such that (A*[u;v] <= b) iff [u;v] is in the convex hull of the polygon.

x = reshape(X,1,[]);
y = reshape(Y,1,[]);

k = convhull(x,y, 'simplify', true);
A = [(y(k(2:end)) - y(k(1:end-1)))', (x(k(1:end-1)) - x(k(2:end)))'];
b = sum(A' .* [x(k(1:end-1)); y(k(1:end-1))], 1)';