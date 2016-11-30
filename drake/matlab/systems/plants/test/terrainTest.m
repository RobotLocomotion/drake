function terrainTest

options.z_scale = 10;
options.terrain = RigidBodyHeightMapTerrain.loadFromImage('terrainTest.png',[-10 30 -10 30],eye(4),options);
options.floating = 'quat';
r = TimeSteppingRigidBodyManipulator('FallingBrick.urdf',.01,options);

v = r.constructVisualizer();
x0 = [getRandomConfiguration(r.getManipulator());zeros(6,1)];
x0(3) = x0(3)+20;
v.draw(0,x0);

traj = simulate(r,[0 5],x0);
v.playback(traj);

% TIMEOUT 1500
