function p = lagrangePoly(breaks,values)
% This code is not very trustworthy, since I find that the roots of the
% lagrange polynomial pi is not the breaks themsevles, may due to
% truncation of the high order polynomial coefficients.
% breaks    - a row vector with breaks(i) being the mesh point i
% values    - a row vector with values(i) being the value at breaks(i)
breaks = breaks(:)';
order = length(breaks)-1;
if(size(values,2)~=order+1)
    error('Inconsistent values size with breaks size')
end
pi = zeros(order+1,order+1);
for i = 1:order+1
    pi(i,:) = poly(breaks([1:i-1 i+1:end]));
    pi(i,:) = pi(i,:)/polyval(pi(i,:),breaks(i));
end
p = values*pi;
end