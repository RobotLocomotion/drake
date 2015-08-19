function kintest

p = PlanarRigidBodyManipulator('CompassGait.urdf');

x=randn(8,1); %p.getInitialState();
q=x(1:4);qd=x(5:8);
[phi,J]=p.positionConstraints(q);

psi = p.velocityConstraints(q,qd);

qd(2)+[qd(3)*sin(q(3)); qd(4)*sin(q(4))]

J*qd
psi
J*qd - psi
