function  L = LTL( H, lambda )

% LTL  factorize H -> L'*L exploiting branch-induced sparsity
% LTL(H,lambda) returns a lower-triangular matrix L satisfying L'*L = H,
% where H is a symmetric, positive-definite matrix having the property that
% the nonzero elements on row i below the main diagonal appear only in
% columns lambda(i), lambda(lambda(i)), and so on.  This is the pattern of
% branch-induced sparsity; and H and lambda can be regarded as the
% joint-space inertia matrix and parent array of a kinematic tree.  lambda
% must satisfy 0<=lambda(i)<i for all i.

n = size(H,1);

for k = n : -1 : 1
  H(k,k) = sqrt(H(k,k));
  i = lambda(k);
  while i ~= 0
    H(k,i) = H(k,i) / H(k,k);
    i = lambda(i);
  end
  i = lambda(k);
  while i ~= 0
    j = i;
    while j ~= 0
      H(i,j) = H(i,j) - H(k,i) * H(k,j);
      j = lambda(j);
    end
    i = lambda(i);
  end
end

L = H;
for i = 1:n-1  				% zero the upper triangle
  L(i,i+1:n) = zeros(1,n-i);
end
