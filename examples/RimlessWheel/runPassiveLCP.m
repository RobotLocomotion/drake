function runPassiveLCP

options.floating = true;
m = PlanarRigidBodyModel('RimlessWheel.urdf',options);
p = PlanarRigidBodyManipulator(m);
%p = PlanarTimeSteppingRBM(m,.01);
v = p.constructVisualizer();
%v.xlim = [-8 8]; v.ylim = [-4 10];

xtraj = p.simulate([0 10]);
v.playback(xtraj);
