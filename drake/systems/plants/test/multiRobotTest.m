function multiRobotTest

urdf = 'FallingBrick.urdf';
options.terrain = RigidBodyFlatTerrain();
r = TimeSteppingRigidBodyManipulator('',.01,options);

options.floating = 'quat';
n = 1+randi(4);
for i=1:n
  r = addRobotFromURDF(r,urdf,zeros(3,1),zeros(3,1),options);
end

v = r.constructVisualizer();

x0 = randn(13*n,1);
x0(3:7:7*n) = x0(3:7:7*n) + (1:n)';
x0 = r.resolveConstraints(x0,v);
xtraj = simulate(r,[0 3],x0);

v.playback(xtraj);
