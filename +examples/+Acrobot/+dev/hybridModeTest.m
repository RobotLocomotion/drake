function hybridModeTest

p = HybridRigidBodyMode('../Acrobot.urdf',[0; 1],[]);
p.body(2).joint_limit_min = -pi/2;
p.body(3).joint_limit_min = -pi/2;
p.body(2).joint_limit_max = pi/2;
p.body(3).joint_limit_max = pi/2;
p = compile(p);

x0 = resolveConstraints(p,randn(4,1));
xtraj = simulate(p,[0 4],x0);
v = p.constructVisualizer();
v.playback(xtraj);
