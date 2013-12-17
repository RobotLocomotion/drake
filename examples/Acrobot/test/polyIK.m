function polynomialIKtest


w = warning('off','Drake:RigidBody:SimplifiedCollisionGeometry');
r = PlanarRigidBodyManipulator('../Acrobot.urdf');
warning(w);
hand = findFrameId(r,'hand');

q = TrigPoly('q','s','c',2);

kinsol = doKinematics(r,q);
x = forwardKin(r,kinsol,hand,zeros(2,1));

q0 = [0;.5];  
% note: general case needs to be more careful about only
% doing this for the trig variables:
qerr = sin((q-q0)/2);  
objective = qerr'*qerr;

v = getTrigPolyBasis([x;objective]);

decision_vars = getmsspoly(v);
objective = (decision_vars-v0)'*(decision_vars-v0);
equality_constraints = [getmsspoly(x); getUnitCircleConstraints(x)];

prog = PolynomialProgram(decision_vars,objective,[],equality_constraints);

[x,objval,exitflag] = bertini(prog)