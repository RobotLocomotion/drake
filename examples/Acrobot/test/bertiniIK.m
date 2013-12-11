function bertiniIK

checkDependency('bertini');

w = warning('off','Drake:RigidBody:SimplifiedCollisionGeometry');
r = PlanarRigidBodyManipulator('../Acrobot.urdf');
warning(w);
hand = findFrameId(r,'hand');

q = TrigPoly('q','s','c',2);

kinsol = doKinematics(r,q);
x = forwardKin(r,kinsol,hand,zeros(2,1));

xs = getsym(x)
