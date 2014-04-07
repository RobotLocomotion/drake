function runPassiveWRL
options.terrain = [];
p = PlanarRigidBodyManipulator('Pendulum.urdf',options);
x = p.simulate([0 5],randn(2,1));

v = constructVisualizer(p);
v.playback(x);
