function kintest

p = PlanarRigidBodyManipulator('../Pendulum.urdf');

x=randn(2,1); %p.getInitialState();
q=x(1);qd=x(1);
[phi,J]=p.positionConstraints(q);

psi = p.velocityConstraints(q,qd);

qd*[cos(q);sin(q)]

J*qd
psi
J*qd - psi
