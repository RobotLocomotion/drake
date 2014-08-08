function fallingBrickLCP

testFallingBrick('rpy');
% testFallingBrick('quat');

end

function testFallingBrick(floating_type)
options.floating = floating_type;
options.terrain = RigidBodyFlatTerrain();
p = TimeSteppingRigidBodyManipulator('FallingBrickBetterCollisionGeometry.urdf',.01,options);
x0 = p.resolveConstraints([0;1+rand;randn(10,1)]);
% x0 = p.resolveConstraints([getRandomConfiguration(p); randn(p.getNumVelocities(), 1)]); % TODO: use something like this to get quat to work

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

for t=xtraj.getBreaks()
  x=xtraj.eval(t);
  q=x(1:p.getNumPositions());
  phi = p.contactConstraints(q);
  if any(phi<-0.05)
    phi
    error('penetration');
  end
end
end
