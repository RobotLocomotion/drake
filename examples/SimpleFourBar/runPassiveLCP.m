function runPassiveLCP

p = TimeSteppingRigidBodyManipulator(PlanarRigidBodyModel('FourBar.urdf'),.01);
v = p.constructVisualizer();
v.xlim = [-8 8]; v.ylim = [-4 10];

xtraj = p.simulate([0 10]);

v.playback(xtraj);

x = xtraj.eval(xtraj.getBreaks);
for i=1:size(x,2)
  valuecheck(p.stateConstraints(x(:,i)),zeros(4,1),.1);  % note the comically loose tolerance
end