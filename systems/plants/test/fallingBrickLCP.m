function fallingBrickLCP

options.floating = true;
m = RigidBodyModel('FallingBrick.urdf',options);

p = TimeSteppingRigidBodyManipulator(m,.01);
v = p.constructVisualizer();
x0 = p.manip.resolveConstraints([0;1+rand;randn(10,1)]);

%  sys = cascade(p,v);
%  sys.simulate([0 4],x0);
  
xtraj = p.simulate([0 4],x0);
v.playback(xtraj);

for t=xtraj.getBreaks()
  x=xtraj.eval(t);
  phi = p.manip.contactConstraints(x(1:6));
  if any(phi<-0.05)
    phi
    error('penetration');
  end
end