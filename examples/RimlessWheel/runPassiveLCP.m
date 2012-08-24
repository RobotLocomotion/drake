function runPassiveLCP

options.floating = true;
m = PlanarRigidBodyModel('RimlessWheel.urdf',options);

%p = PlanarRigidBodyManipulator(m);
%x0 = p.resolveConstraints(randn(6,1));

p = PlanarTimeSteppingRBM(m,.01);
x0 = p.manip.resolveConstraints(randn(6,1));

xtraj = p.simulate([0 10],x0);

v = p.constructVisualizer();
v.playback(xtraj);
