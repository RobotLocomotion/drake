function [center, radius] = maxInnerSphere(Ain,bin)
% Find the maximal inner sphere of an polyhedron Pobj
A = [Ain sqrt(sum(Ain.^2,2))];
b = bin;
[x,~,exitflag] = linprog([0;0;0;-1],A,b,[],[]);
center = x(1:3);
radius = x(4);
end