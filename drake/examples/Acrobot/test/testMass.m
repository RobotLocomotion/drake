function testMass

% tests getMass() method of RigidBodyManipulator

r = PlanarRigidBodyManipulator('../Acrobot.urdf');

valuecheck(r.getMass(),2);

