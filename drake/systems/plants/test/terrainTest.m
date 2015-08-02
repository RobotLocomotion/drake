function terrainTest

options.z_scale = 10;
options.terrain = RigidBodyHeightMapTerrain.loadFromImage('terrainTest.png',[-10 30 -10 30],eye(4),options);
options.floating = true;
r = TimeSteppingRigidBodyManipulator('FallingBrick.urdf',.01,options);

v = r.constructVisualizer();
x0 = .6*randn(12,1)+[0;0;20;zeros(9,1)];
v.draw(0,x0);

traj = simulate(r,[0 5],x0);
v.playback(traj);
