% Simple example of common lyapunov functions
% for robust stability of linear systems

checkDependency('spotless');
checkDependency('mosek');

n = 4;
k = 2;

% generate random stable matrices
for i=1:k
  d = -rand(n,1);
  v = randn(n);
  A{i} = v*diag(d)/v;
end

% create the optimization program
prog = spotsosprog;

% construct an n-by-n positive semi-definite matrix as the decision
% variables
[prog,P] = prog.newSym(n);
prog = prog.withPSD(P-.01*eye(n));

% add the common Lyapunov conditions
for i=1:k
  prog = prog.withPSD(- A{i}'*P - P*A{i});
end

% run the optimization
sol = prog.minimize(trace(P),@spot_mosek);

if sol.isPrimalFeasible()
  % display the results
  P = double(sol.eval(P))
  %isPositiveDefinite(P)
  eig(P)
else
  fprintf('Could not find a common Lyapunov function.\nThis is expected to occur with some probability:  not all\n random sets of stable matrices will have a common Lyapunov\n function\n');
end

% another example from the lecture notes:
%a = randn;  ab = 2*rand - 1;  b=ab/a;
%A{i} = [-1 a; b -1];
%A{1} = [-1 .5; -3 -1];
%A{2} = [-1 .1; -10 -1];

