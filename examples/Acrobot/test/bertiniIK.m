function polynomialIKtest


w = warning('off','Drake:RigidBody:SimplifiedCollisionGeometry');
r = PlanarRigidBodyManipulator('../Acrobot.urdf');
warning(w);
hand = findFrameId(r,'hand');

q = TrigPoly('q','s','c',2);

kinsol = doKinematics(r,q);
x = forwardKin(r,kinsol,hand,zeros(2,1));

v = getTrigPolyBasis(x)
v0 = eval(v,randn(2,1))

decision_vars = getmsspoly(v);
objective = (decision_vars-v0)'*(decision_vars-v0);
equality_constraints = [getmsspoly(x); getUnitCircleConstraints(x)];

prog = PolynomialProgram(decision_vars,objective,[],equality_constraints);

[x,objval,exitflag] = bertini(prog)