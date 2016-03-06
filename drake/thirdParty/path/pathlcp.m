function [z,mu] = pathlcp(M,q,l,u,z,A,b,t,mu)
% pathlcp(M,q,l,u,z,A,b,t,mu)
%
% Solve the standard linear complementarity problem using PATH:
%     z >= 0, Mz + q >= 0, z'*(Mz + q) = 0
%
% Required input:
%     M(n,n)  - matrix
%     q(n)    - vector
% 
% Output:
%     z(n)    - solution
%     mu(m)   - multipliers (if polyhedral constraints are present)
%
% Optional input:
%     l(n)    - lower bounds                       default: zero
%     u(n)    - upper bounds                       default: infinity
%     z(n)    - starting point                     default: zero
%     A(m,n)  - polyhedral constraint matrix       default: empty
%     b(m)    - polyhedral right-hand side         default: empty
%     t(m)    - type of polyhedral constraint      default: 1
%                  < 0: less than or equal
%                    0: equation
%                  > 0: greater than or equal
%     mu(m)   - starting value for multipliers     default: zero
%
% The optional lower and upper bounds are used to define a linear mixed 
% complementarity problem (box constrained variational inequality).
%       l <= z <= u
%       where l_i < z_i < u_i  => (Mz + q)_i = 0
%             l_i = z          => (Mz + q)_i >= 0
%             u_i = z          => (Mz + q)_i <= 0
% 
% The optional constraints are used to define a polyhedrally constrained
% variational inequality.  These are transformed internally to a standard
% mixed complementarity problem.  The polyhedral constraints are of the
% form
%       Ax ? b
% where ? can be <=, =, or >= depending on the type specified for each
% constraint.

Big = 1e20;

if (nargin < 2)
  error('PathLCP:BadInputs','two input arguments required for lcp(M, q)');
end

if (~issparse(M))
  M = sparse(M);	% Make sure M is sparse
end
q = full(q(:)); 	% Make sure q is a column vector

[mm,mn] = size(M);	% Get the size of the inputs
n = length(q);

if (mm ~= mn | mm ~= n) 
  error('PathLCP:BadInputs','dimensions of M and q must match');
end

if (n == 0)
  error('PathLCP:BadInputs','empty model');
end

if (nargin < 3 | isempty(l))
  l = zeros(n,1);
end

if (nargin < 4 | isempty(u))
  u = Big*ones(n,1);
end

if (nargin < 5 | isempty(z))
  z = zeros(n,1);
end

z = full(z(:)); l = full(l(:)); u = full(u(:));
if (length(z) ~= n | length(l) ~= n | length(u) ~= n)
  error('PathLCP:BadInputs','Input arguments are of incompatible sizes');
end

l = max(l,-Big*ones(n,1));
u = min(u,Big*ones(n,1));
z = min(max(z,l),u);

m = 0;
if (nargin > 5)
  if (nargin < 7)
    error('PathLCP:BadConstraints','Polyhedral constraints require A and b');
  end

  if (~issparse(A))
    A = sparse(A);
  end
  b = full(b(:));

  m = length(b);

  if (m > 0)

    [am, an] = size(A);

    if (am ~= m | an ~= n)
      error('PathLCP:BadConstraints','Polyhedral constraints of incompatible sizes');
    end

    if (nargin < 8 | isempty(t))
      t = ones(m,1);
    end

    if (nargin < 9 | isempty(mu))
      mu = zeros(m,1);
    end

    t = full(t(:)); mu = full(mu(:));
    if (length(t) ~= m | length(mu) ~= m)
      error('PathLCP:BadConstraints','Polyhedral input arguments are of incompatible sizes');
    end

    l_p = -Big*ones(m,1);
    u_p =  Big*ones(m,1);

    idx = find(t > 0);
    if (length(idx) > 0)
      l_p(idx) = zeros(length(idx),1);
    end

    idx = find(t < 0);
    if (length(idx) > 0)
      u_p(idx) = zeros(length(idx),1);
    end

    mu = min(max(mu,l_p),u_p);

    M = [M -A'; A sparse(m,m)];
    q = [q; -b];

    z = [z; mu];
    l = [l; l_p];
    u = [u; u_p];
  else
    if (nargin >= 9 & ~isempty(mu))
      error('PathLCP:BadConstraints','No polyhedral constraints -- multipliers set.');
    end

    if (nargin >= 8 & ~isempty(t))
      error('PathLCP:BadConstraints','No polyhedral constraints -- equation types set.');
    end
  end
end

idx = find(l > u);
if length(idx) > 0
  error('PathLCP:BadConstraints','Bounds infeasible.');
end

nnzJ = nnz(M);

[status, ttime] = lcppath(n+m, nnzJ, z, l, u, M, q);

if (status ~= 1) 
  status,
  type logfile.tmp
  error('PathLCP:FailedToSolve','Path fails to solve problem');
end

mu = [];
if (m > 0)
  mu = z(n+1:n+m);
  z = z(1:n);
end

return;

