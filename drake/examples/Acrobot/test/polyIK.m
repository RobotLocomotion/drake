function polyIK

w = warning('off','Drake:RigidBody:SimplifiedCollisionGeometry');
r = PlanarRigidBodyManipulator('../Acrobot.urdf');
warning(w);
hand = findFrameId(r,'hand');

checkDependency('spotless');
q = TrigPoly('q','s','c',2);

kinsol = doKinematics(r,q);
x = forwardKin(r,kinsol,hand,zeros(2,1));

q0 = [0;.5];  

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
equality_constraints = [getmsspoly(x) - [1.5;0]; getUnitCircleConstraints(x)];

prog = PolynomialProgram(decision_vars,objective,[],equality_constraints);

[x,objval,exitflag] = gloptipoly(prog)
q = atan2(x(1:2:end),x(2:2:end)); 

v = constructVisualizer(r);
clf;
draw(v,0,q);