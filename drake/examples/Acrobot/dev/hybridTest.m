function hybridTest

m = PlanarRigidBodyManipulator('../Acrobot.urdf');
m.body(2).joint_limit_min = -pi/2;
m.body(3).joint_limit_min = -pi/2;
m.body(2).joint_limit_max = pi/2;
m.body(3).joint_limit_max = pi/2;

p = HybridRigidBodyManipulator(m);

x0 = resolveConstraints(p,[1;randn(4,1)]);
xtraj = simulate(p,[0 4],x0);

v = p.constructVisualizer();
v.playback(xtraj);
