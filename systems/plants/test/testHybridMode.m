function testHybridMode

options.floating = true;
p = HybridRigidBodyMode('FallingBrick.urdf',zeros(3,1),[1;zeros(3,1)],options);

xtraj = simulate(p,[0 4]);
v = p.constructVisualizer();
v.playback(xtraj);