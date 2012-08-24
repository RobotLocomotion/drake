function runPassiveLCP

options.floating = true;
m = PlanarRigidBodyModel('RimlessWheel.urdf',options);

%p = PlanarRigidBodyManipulator(m);
%x0 = p.resolveConstraints(randn(6,1));

p = TimeSteppingRigidBodyManipulator(m,.01);
x0 = p.manip.resolveConstraints([0;1+rand;randn;5*rand;randn;5*rand]);

xtraj = p.simulate([0 10],x0);

v = p.constructVisualizer();
v.axis = [0 5 -.1 3];
v.playback(xtraj);
