function testHybridMode

p = RigidBodyManipulator(fullfile(getDrakePath,'examples','Acrobot','Acrobot.urdf'));
p = setJointLimits(p,[-inf;1.5],[inf;1.5]);
p = compile(p);
v = p.constructVisualizer();
p = HybridRigidBodyMode(p);
% simulates it with both lb and ub on the joint limit set to active (so
% thetaddot(2) will = 0, but currently the initial velocity will not be dissipated
x0 = [0;2;0;pi/4;0;0]+3*[zeros(2,1);randn(3,1);0];
x0 = resolveConstraints(p,x0);
ytraj = simulate(p,[0,4],x0);
v.playback(ytraj);

return;

options.floating = true;
p = HybridRigidBodyMode('FallingBrick.urdf',zeros(6,1),[1;zeros(7,1)],options);
xtraj = simulate(p,[0 4]);
v = p.constructVisualizer();
v.playback(xtraj);

p = HybridRigidBodyMode('FallingBrick.urdf',zeros(6,1),[1;1;zeros(6,1)],options);
xtraj = simulate(p,[0 4]);
v.playback(xtraj);
