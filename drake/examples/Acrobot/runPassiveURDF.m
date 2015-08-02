function runPassiveWRL

p = PlanarRigidBodyManipulator('Acrobot.urdf');
v = p.constructVisualizer;

ytraj = p.simulate([0 5],randn(4,1));
v.playback(ytraj);

