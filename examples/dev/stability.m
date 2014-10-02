n = 4;
k = 2;

% generate random stable matrices
for i=1:k
  d = -rand(n,1);
  v = randn(n);
  A{i} = v*diag(d)/v;
end

% example from lecture notes:
%a = randn;  ab = 2*rand - 1;  b=ab/a;
%A{i} = [-1 a; b -1];
%A{1} = [-1 .5; -3 -1];
%A{2} = [-1 .1; -10 -1];

cvx_begin
  variable P(n,n) symmetric;
  %find P;
  minimize (trace(P));
  for i=1:k, - A{i}'*P - P*A{i} == semidefinite(n); end
  P - 0.01*eye(n) == semidefinite(n);
cvx_end

P