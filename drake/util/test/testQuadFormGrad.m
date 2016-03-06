function testQuadFormGrad()
n = 5; m = 10; nq = 15;
P = randn(n, n);
q = randn(nq, 1);
option.grad_method = 'taylorvar';
[X, dX] = geval(1, @(q) getX(n, m, q), q, option);
dQuadForm = quadFormGrad(X, P, dX);
[~, dQuadFormGeval] = geval(1, @(q) getQuadForm(P, n, m, q), q, option);
valuecheck(dQuadFormGeval, dQuadForm, 1e-12);
end

function ret = getX(n, m, q)
rng(12512, 'twister');
ret = zeros(n, m) * q(1);
for i = 1 : n
  for j = 1 : m
    for k = 1 : length(q)
      ret(i, j) = ret(i, j) + randn * q(k);
    end
  end
end
end

function ret = getQuadForm(P, n, m, q)
X = getX(n, m, q);
ret = X' * P * X;
end