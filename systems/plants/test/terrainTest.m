function terrainTest

r = TimeSteppingRigidBodyManipulator('FallingBrick.urdf',.01,struct('floating',true));
r = setTerrain(r,'terrainTest.png',[-10;10;0],[20,20,20]);

v = r.constructVisualizer();
x0 = .6*randn(12,1)+[0;0;20;zeros(9,1)];
v.draw(0,x0);

traj = simulate(r,[0 5],x0);
v.playback(traj);
