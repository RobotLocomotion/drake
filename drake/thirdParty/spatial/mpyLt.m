function  y = mpyLt( L, lambda, x )

% mpyLt  multiply vector by transpose of L factor from LTL or LTDL
% mpyLt(L,lambda,x) computes L'*x where L is the lower-triangular matrix
% from either LTL or LTDL and lambda is the parent array describing the
% sparsity pattern in L.

n = size(L,1);
y = x;           % to give y same dimensions as x

for i = 1:n
  y(i) = L(i,i) * x(i);
  j = lambda(i);
  while j ~= 0
    y(j) = y(j) + L(i,j) * x(i);
    j = lambda(j);
  end
end
