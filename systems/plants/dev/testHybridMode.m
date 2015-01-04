function testHybridMode

p = HybridRigidBodyMode(fullfile(getDrakePath,'examples','Acrobot','Acrobot.urdf'));
p = setJointLimits(p,[-inf;-1.5],[inf;1.5]);
p = compile(p);
xtraj = simulate(p,[0,4],[0;pi/4;0;5]+randn(4,1));
v = p.constructVisualizer();
v.playback(xtraj);

return;

options.floating = true;
p = HybridRigidBodyMode('FallingBrick.urdf',zeros(6,1),[1;zeros(7,1)],options);
xtraj = simulate(p,[0 4]);
v = p.constructVisualizer();
v.playback(xtraj);

p = HybridRigidBodyMode('FallingBrick.urdf',zeros(6,1),[1;1;zeros(6,1)],options);
xtraj = simulate(p,[0 4]);
v.playback(xtraj);
