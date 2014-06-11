function multiRobotTest

urdf = '../urdf/atlas_minimal_contact.urdf';
options.view = 'right';
r = PlanarRigidBodyManipulator('',options);

w = warning('off','Drake:RigidBodyManipulator:UnsupportedJointLimits');
warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits');
r = addRobotFromURDF(r,urdf,[-1;2]);
r = addRobotFromURDF(r,urdf,[1;2]);
warning(w);
xtraj = simulate(r,[0 3]);

v = r.constructVisualizer();
v.playback(xtraj);
