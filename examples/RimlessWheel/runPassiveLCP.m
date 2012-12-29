function runPassiveLCP

options.floating = true;
options.twoD = true;
p = TimeSteppingRigidBodyManipulator('RimlessWheel.urdf',.01,options);
x0 = p.manip.resolveConstraints([0;1+rand;randn;5*rand;randn;5*rand]);

xtraj = p.simulate([0 10],x0);

v = p.constructVisualizer();
v.axis = [0 5 -.1 3];
v.playback(xtraj);
