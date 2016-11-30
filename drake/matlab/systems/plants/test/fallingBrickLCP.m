function fallingBrickLCP

options.floating = 'quat';
options.terrain = RigidBodyFlatTerrain();
% options.ignore_self_collisions = true;
% options.use_bullet = false;
s = 'FallingBrickContactPoints.urdf';
% s = 'FallingBrickBetterCollisionGeometry.urdf';
p = TimeSteppingRigidBodyManipulator(s,.01,options);
p = p.addRobotFromURDF(s,[],[],options);
% x0 = [0;1;2;rpy2quat(randn(3,1));randn(6,1)];
x0 = [0;1;2;rpy2quat(randn(3,1));2;1;2;rpy2quat(randn(3,1));randn(12,1)];
x0 = p.resolveConstraints(x0);

if 0 
  v = p.constructVisualizer();
  sys = cascade(p,v);
  sys.simulate([0 8],x0);
  return;
end

v = p.constructVisualizer();
v.drawWrapper(0,x0);
xtraj = p.simulate([0 4],x0);
v.playback(xtraj);