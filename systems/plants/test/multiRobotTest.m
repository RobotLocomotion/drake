function multiRobotTest

urdf = 'FallingBrick.urdf';
options.terrain = RigidBodyFlatTerrain();
r = TimeSteppingRigidBodyManipulator('',.01,options);

options.floating = 'quat';
n = 1+randi(4);
for i=1:n
  r = addRobotFromURDF(r,urdf,zeros(3,1),zeros(3,1),options);
end

x0 = r.resolveConstraints([getRandomConfiguration(r); randn(r.getNumVelocities(), 1)]);
xtraj = simulate(r,[0 3],x0);

v = r.constructVisualizer();
v.playback(xtraj);
end