function runPassive

p = PlanarRigidBodyManipulator('FourBar.urdf');
v = p.constructVisualizer();
v.xlim = [-8 8]; v.ylim = [-4 10];

xtraj = p.simulate([0 10]);

v.playback(xtraj);
