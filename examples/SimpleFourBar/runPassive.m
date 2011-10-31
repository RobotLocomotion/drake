function runPassive

p = PlanarURDFManipulator('FourBar.urdf');
v = PlanarURDFVisualizer('FourBar.urdf',[-8 8 -4 10]);

xtraj = p.simulate([0 5]);

v.playback(xtraj);
