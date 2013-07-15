function runPassiveURDF

r = PlanarRigidBodyManipulator('SimpleDoublePendulum.urdf');
v = r.constructVisualizer();
v.axis = [-2 2 -2 2];

x0 = randn(4,1);
xtraj = simulate(r,[0 5],x0);

v.playback(xtraj);