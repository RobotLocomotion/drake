function  y = mpyH( H, lambda, x )

% mpyH  calculate H*x exploiting branch-induced sparsity in H
% mpyH(H,lambda,x) computes H*x where x is a vector and H is a symmetric,
% positive-definite matrix having the property that the nonzero elements on
% row i below the main diagonal appear only in columns lambda(i),
% lambda(lambda(i)), and so on.  This is the pattern of branch-induced
% sparsity; and H and lambda can be regarded as the joint-space inertia
% matrix and parent array of a kinematic tree.  lambda must satisfy
% 0<=lambda(i)<i for all i.

n = size(H,1);
y = x;           % to give y same dimensions as x

for i = 1:n
  y(i) = H(i,i) * x(i);
end
for i = n:-1:1
  j = lambda(i);
  while j ~= 0
    y(i) = y(i) + H(i,j) * x(j);
    y(j) = y(j) + H(i,j) * x(i);
    j = lambda(j);
  end
end
