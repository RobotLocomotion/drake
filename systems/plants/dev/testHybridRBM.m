function testHybridMode

p = HybridRigidBodyMode(fullfile(getDrakePath,'examples','Acrobot','Acrobot.urdf'));
p = setJointLimits(p,[-inf;-1.5],[inf;1.5]);
p = compile(p);

p = HybridRigidBodyManipulator(p);

xtraj = simulate(p,[0,4],[1;0;pi/4;0;5]+[0;randn(4,1)]);
v = p.constructVisualizer();
v.playback(xtraj);


