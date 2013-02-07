function testHybridMode

options.floating = true;
p = HybridRigidBodyMode('FallingBrick.urdf',zeros(6,1),[1;zeros(7,1)],options);

xtraj = simulate(p,[0 4]);
v = p.constructVisualizer();
v.playback(xtraj);