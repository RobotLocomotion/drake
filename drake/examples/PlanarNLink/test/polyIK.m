function polyIK(N)

oldpath=addpath(fullfile(pwd,'..'));

if (nargin<1)
  N = 4; %1+ceil(5*rand);
end

r = PlanarNLink(N);

checkDependency('spotless');

q = TrigPoly('q','s','c',N);
kinsol = doKinematics(r,q);
hand = [0;-1.2];
x = forwardKin(r,kinsol,N+1,hand);

q0 = .5*ones(N,1);  % prefer to bend counter-clockwise

% desired objective function:
% qerr = sin((q-q0)/2);  objective = qerr'*qerr;
% but sin(q/2) is ugly
% (see http://mathworld.wolfram.com/Half-AngleFormulas.html)
% fortunately, sin^2(q/2) is not...
%   sin^2(q/2) = .5*(1-cos(q))
% so instead, I'll write the objective as 
objective = getmsspoly(sum(.5*(1-cos(q-q0))));
% note: general case also needs to be more careful about 
% only doing it for the trig variables.

v = getTrigPolyBasis([x;objective]);

decision_vars = getmsspoly(v);
equality_constraints = [getmsspoly(x) - [.9*N;0]; getUnitCircleConstraints(x)];

prog = PolynomialProgram(decision_vars,objective,[],equality_constraints);

[x,objval,exitflag] = gloptipoly(prog);
q = atan2(x(1:2:end),x(2:2:end)); 

v = constructVisualizer(r);
clf;
draw(v,0,q);

path(oldpath);
