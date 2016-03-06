function  [L,D] = LTDL( H, lambda )

% LTDL  factorize H -> L'*D*L exploiting branch-induced sparsity
% [L,D]=LTDL(H,lambda) returns a unit-lower-triangular matrix L and a
% diagonal matrix D that satisfy L'*D*L = H, where H is a symmetric,
% positive-definite matrix having the property that the nonzero elements on
% row i below the main diagonal appear only in columns lambda(i),
% lambda(lambda(i)), and so on.  This is the pattern of branch-induced
% sparsity; and H and lambda can be regarded as the joint-space inertia
% matrix and parent array of a kinematic tree.  lambda must satisfy
% 0<=lambda(i)<i for all i.

n = size(H,1);

for k = n : -1 : 1
  i = lambda(k);
  while i ~= 0
    a = H(k,i) / H(k,k);
    j = i;
    while j ~= 0
      H(i,j) = H(i,j) - a * H(k,j);
      j = lambda(j);
    end
    H(k,i) = a;
    i = lambda(i);
  end
end

D = diag(diag(H));
L = eye(n);
for i = 2:n
  L(i,1:(i-1)) = H(i,1:(i-1));
end
