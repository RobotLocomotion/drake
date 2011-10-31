function  newLambda = expandLambda( lambda, nf )

% expandLambda  expand a parent array
% expandLambda(lambda,nf) calculates the expanded parent array, for use in
% sparse factorization algorithms, from a given parent array and an array
% of joint motion freedoms.  nf(i) is the degree of motion freedom allowed
% by joint i.

N = length(lambda);
n = sum(nf);

newLambda = 0:(n-1);

map(1) = 0;				% (matlab won't let us use map(0))
for i = 1:N
  map(i+1) = map(i) + nf(i);
end

for i = 1:N
  newLambda(map(i)+1) = map(lambda(i)+1);
end
