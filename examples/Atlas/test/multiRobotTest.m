function multiRobotTest

urdf = '../urdf/atlas_minimal_contact.urdf';
options.view = 'right';
r = PlanarRigidBodyManipulator('',options);

r = addRobotFromURDF(r,urdf,[-1;2]);
r = addRobotFromURDF(r,urdf,[1;2]);
xtraj = simulate(r,[0 3]);

v = r.constructVisualizer();
v.playback(xtraj);
