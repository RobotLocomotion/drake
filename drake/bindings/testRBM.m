function testRBM()

addpath('matlab');
r = pydrake.RigidBodyManipulator('../examples/Pendulum/Pendulum.urdf')

kinsol = r.doKinematics(zeros(7,1), zeros(7,1), 1, true)

com = r.centerOfMass(kinsol, 0)
com{1}

x = r.forwardKin(kinsol, zeros(3,1), 0, 1, 0, 1)

x{1}
x{2}