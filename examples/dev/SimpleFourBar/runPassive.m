function runPassive

p = PlanarRigidBodyManipulator('FourBar.urdf');
v = PlanarRigidBodyVisualizer('FourBar.urdf',[-8 8 -4 10]);

xtraj = p.simulate([0 10]);

v.playback(xtraj);
