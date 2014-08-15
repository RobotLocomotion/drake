function testMass

% tests getMass() method of RigidBodyManipulator

r = PlanarRigidBodyManipulator(fullfile(getDrakePath, '+examples', '+Acrobot', 'Acrobot.urdf'));

valuecheck(r.getMass(),2);

