function taylorVarSqTest()
n = 2;
nq = 2;
f = randn(n, 1);
df = {randn(n, nq), randn(n, nq^2)};
a = TaylorVar(f, df);
b = a .* a; % fine
c = a.^2; % error
end