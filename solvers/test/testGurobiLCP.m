function testGurobiLCP()

checkDependency('gurobi');

for n = 3:7
  % generate a random diagonally dominant, symmetric matrix (to ensure a unique solution to the LCP)
  % lovingly ripped off from http://math.stackexchange.com/a/358092
  M = rand(n,n);
  M = M + M';
  M = M + n * eye(n);

  q = rand(n,1) * 10;

  l = -rand(n,1) * 10;

  z = gurobi_lcp(M, q, l);

  z_path = pathlcp(M, q, l);

  valuecheck(z, z_path, 1e-5);
end

