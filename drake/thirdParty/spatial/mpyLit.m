function  y = mpyLit( L, lambda, x )

% mpyLit  multiply vector by inverse transpose of L factor from LTL or LTDL
% mpyLit(L,lambda,x) computes inv(L')*x where L is the lower-triangular
% matrix from either LTL or LTDL and lambda is the parent array describing
% the sparsity pattern in L.

n = size(L,1);

for i = n:-1:1
  x(i) = x(i) / L(i,i);
  j = lambda(i);
  while j ~= 0
    x(j) = x(j) - L(i,j) * x(i);
    j = lambda(j);
  end
end

y = x;
