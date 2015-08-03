function  y = mpyL( L, lambda, x )

% mpyL  multiply vector by L factor from LTL or LTDL
% mpyL(L,lambda,x) computes L*x where L is the lower-triangular matrix from
% either LTL or LTDL and lambda is the parent array describing the sparsity
% pattern in L.

n = size(L,1);
y = x;           % to give y same dimensions as x

for i = 1:n
  y(i) = L(i,i) * x(i);
  j = lambda(i);
  while j ~= 0
    y(i) = y(i) + L(i,j) * x(j);
    j = lambda(j);
  end
end
