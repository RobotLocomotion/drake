function testHybridMode

m = PlanarRigidBodyModel('FallingBrick.urdf',struct('floating',true));
p = HybridRigidBodyMode(m,zeros(3,1),[1;zeros(3,1)]);

xtraj = simulate(p,[0 4]);
v = p.constructVisualizer();
v.playback(xtraj);