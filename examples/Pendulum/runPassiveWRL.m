function runPassiveWRL

p = PlanarRigidBodyManipulator('Pendulum.urdf');
v = p.constructWRLVisualizer;

x = p.simulate([0 5],randn(2,1));
v.playback(x);