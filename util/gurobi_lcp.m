function z = gurobi_lcp(M, q, l)
% Solve a Linear Complimentarity Problem as a mixed-integer linear program using Gurobi.
% This is meant to be a drop-in replacement for pathlcp(M, q, l). For the full documentation
% see pathlcp.m
% In general, this function is ~10x slower than pathlcp, but it does provide a guarantee
% of global optimality, subject to numerical precision issues. 
% WARNING: in order to produce a well-scaled formulation, we hard-limit z and (Mz + q) to
% be less than a predefined constant called 'big'. 


% checkDependency('gurobi');

big = 1e2;
DEBUG = false;
persistent x_seed;

n = size(M, 2);
nv = n * 3;
z_ndx = 1:n;
z_abs_ndx = n + (1:n);
z_zero_ndx = 2*n + (1:n);
lb = [l; zeros(n, 1); zeros(n, 1)];
ub = [inf + zeros(2*n, 1); ones(n, 1)];
vtype = [repmat('C', 2*n, 1); repmat('B', n, 1)];

A = zeros(5*n, nv);
b = zeros(size(A, 1), 1);
offset = 0;

% z_abs >= abs(z)
% z_abs >= z, z_abs >= -z
A(offset+(1:n), z_ndx) = eye(n);
A(offset+(1:n), z_abs_ndx) = -eye(n);
offset = offset + n;
A(offset+(1:n), z_ndx) = -eye(n);
A(offset+(1:n), z_abs_ndx) = -eye(n);
offset = offset + n;

% z_zero -> z == l
% z - l <= 1e6 * (1-z_zero)
A(offset+(1:n), z_ndx) = eye(n);
A(offset+(1:n), z_zero_ndx) = big * eye(n);
b(offset+(1:n)) = big + l;
% -inf as a lower bound creates a special case in which z_zero must be false
inf_mask = isinf(l);
inf_idx = find(inf_mask);
A(offset+inf_idx, :) = 0;
b(offset+inf_idx) = 0;
ub(z_zero_ndx(inf_mask)) = 0;
offset = offset + n;

% Mz + q >= 0
A(offset+(1:n), z_ndx) = -M;
b(offset+(1:n)) = q;
offset = offset + n;

% ~z_zero -> Mz + q == 0
% Mz + q <= 1e6 * z_zero
A(offset+(1:n), z_ndx) = M;
A(offset+(1:n), z_zero_ndx) = -big * eye(n);
% again, -inf as a lower bound creates a special case where z_zero is always false
A(offset+inf_idx, z_zero_ndx) = 0;
b(offset+(1:n)) = -q;
offset = offset + n;

if DEBUG
  assert(offset == size(A, 1));
end

c = zeros(nv, 1);
c(z_abs_ndx) = 1;

model = struct('A', sparse(A), 'rhs', b, 'sense', '<', 'obj', c, 'lb', lb, 'ub', ub, 'vtype', vtype);
if ~isempty(x_seed) && length(x_seed) == length(lb)
  model.start = x_seed;
end
params = struct('outputflag', 0, 'Threads', 1);%, 'IntFeasTol', 1e-5, 'FeasibilityTol', 1e-6);

result = gurobi(model, params);
z = result.x(z_ndx);
x_seed = result.x;

if DEBUG
  try
    assert(all(z >= l - 1e-4));
    assert(all(M * z + q >= -1e-3), sprintf('%f', min(M * z + q)));
    assert(all((abs(z - l) <= 1e-3) | (abs(M * z + q) <= 1e-3)));
  catch e
    e.getReport()
    keyboard();
  end
end

