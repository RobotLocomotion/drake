function [z,f,J,mu] = pathmcp(z,l,u,cpfj,A,b,t,mu)
% pathmcp(z,l,u,cpfj,A,b,t,mu)
%
% Solve a polyhedrally constrained variational inequality using PATH
%
% Calling syntax: [z,f,J] = pathmcp(z,l,u,cpfunjac,A,b,t,mu)
%
% Input:
%  z - starting point
%  l - lower bounds on z
%  u - upper bounds on z
%
%  cpfunjac - the name of the m-file for  evaluating the function F and its 
%             Jacobian J (without .m-extension).
%
%             The following m-file must be supplied (where default name is
%             'mcp_funjac.m' unless stated otherwise in the variable cpfunjac).
%
%             'mcp_funjac.m' contains function [f,J,domerr]=cpfunjac(z,jacflag)
%             that computes the function F and if jacflag=1 the sparse 
%             Jacobian J at the point z. domerr returns the number of domain 
%             violations.
%
%  A - constraint matrix
%  b - right hand side of the constraints
%  t - types of the constraints
%      <0 : less than or equal
%      =0 : equal to
%      >0 : greater than or equal
%
%      We have Ax ? b, ? is the type of constraint
%
% Output:
%  z      - solution
%  mu     - multipliers on the constraints
%  f      - function evaluation at the solution
%  J      - jacobian evaluation at the solution


Big = 1e20;

if (nargin < 1)
  error('one input arguments required for mcp(z)');
end

z = full(z(:)); 
n = length(z);

if (n == 0)
  error('empty model');
end

if (nargin < 2 | isempty(l))
  l = zeros(n,1);
end

if (nargin < 3 | isempty(u))
  u = Big*ones(n,1);
end

l = full(l(:)); u = full(u(:));
if (length(l) ~= n | length(u) ~= n)
  error('Input arguments are of incompatible sizes');
end

l = max(l,-Big*ones(n,1));
u = min(u,Big*ones(n,1));
z = min(max(z,l),u);

if (nargin < 4 | isempty(cpfj))
  cpfj = 'mcp_funjac';
end

m   = 0;
mu  = [];
l_p = [];
u_p = [];

if (nargin > 4)
  if (nargin < 6)
    error('Polyhedral constraints require A and b');
  end

  if (~issparse(A))
    A = sparse(A);
  end
  b = full(b(:));

  m = length(b);

  if (m > 0)

    [am, an] = size(A);

    if (am ~= m | an ~= n)
      error('Polyhedral constraints of incompatible sizes');
    end

    if (nargin < 7 | isempty(t))
      t = ones(m,1);
    end

    if (nargin < 8 | isempty(mu))
      mu = zeros(m,1);
    end

    t = full(t(:)); mu = full(mu(:));
    if (length(t) ~= m | length(mu) ~= m)
      error('Polyhedral input arguments are of incompatible sizes');
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
  else
    if (nargin >= 8 & ~isempty(mu))
      error('No polyhedral constraints -- multipliers set.');
    end

    if (nargin >= 7 & ~isempty(t))
      error('No polyhedral constraints -- equation types set.');
    end
  end
else
  A = [];
end

% this is a fix, nnz may be bigger than this
[f,J,domerr] = feval(cpfj,z+1e-5*ones(size(z))+1e-5*abs(z),1);

if (domerr > 0)
  [f,J,domerr] = feval(cpfj,z,1);
end 

if (domerr > 0)
  error([cpfj ' not defined at starting point']);
end

if ~issparse(J)
  error([cpfj ' must return a sparse Jacobian']);
end

nnzJ = nzmax(J);

row = n + m;
ele = nnzJ + 2*nzmax(A);

init = [z; mu];
low  = [l; l_p];
upp  = [u; u_p];

if m > 0
  global mcp_vifunc;
  global mcp_viconn;
  global mcp_viconm; 
  global mcp_viconA; 
  global mcp_viconb;

  mcp_vifunc = cpfj;
  mcp_conn = n;
  mcp_conm = m;
  mcp_conA = A;
  mcp_conb = b;

  [status, ttime, f, J] = mcppath(row, ele, init, low, upp, 'mcp_vifunjac');
else
  [status, ttime, f, J] = mcppath(row, ele, init, low, upp, cpfj);
end

if (status ~= 1) 
  status,
  error('Path fails to solve problem');
end

mu = [];
z = init;

if m > 0
  mu = init(n+1:n+m);
  z = init(1:n);

  J = J(1:n,1:n);
  f = f(1:n) + A'*mu;
end

return;
