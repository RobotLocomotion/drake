function fallingBrickLCP

options.floating = true;
options.terrain = RigidBodyFlatTerrain();
p = TimeSteppingRigidBodyManipulator('FallingBrickBetterCollisionGeometry.urdf',.01,options);
x0 = p.resolveConstraints([0;1+rand;randn(10,1)]);

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
  phi = p.contactConstraints(x(1:6));
  if any(phi<-0.05)
    phi
    error('penetration');
  end
end
