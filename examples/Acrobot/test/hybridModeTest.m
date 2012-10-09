function hybridModeTest

m = RigidBodyModel('../Acrobot.urdf');
m.body(2).joint_limit_min = -pi/2;
m.body(3).joint_limit_min = -pi/2;
m.body(2).joint_limit_max = pi/2;
m.body(3).joint_limit_max = pi/2;

p = HybridRigidBodyMode(m,[0; 1],[]);

x0 = resolveConstraints(p,randn(4,1));
xtraj = simulate(p,[0 4],x0);
v = p.constructVisualizer();
v.playback(xtraj);
