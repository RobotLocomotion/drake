function  y = mpyLi( L, lambda, x )

% mpyLi  multiply vector by inverse of L factor from LTL or LTDL
% mpyLi(L,lambda,x) computes inv(L)*x where L is the lower-triangular
% matrix from either LTL or LTDL and lambda is the parent array describing
% the sparsity pattern in L.

n = size(L,1);

for i = 1:n
  j = lambda(i);
  while j ~= 0
    x(i) = x(i) - L(i,j) * x(j);
    j = lambda(j);
  end
  x(i) = x(i) / L(i,i);
end

y = x;
