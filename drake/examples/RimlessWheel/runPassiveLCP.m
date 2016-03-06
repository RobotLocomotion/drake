function runPassiveLCP

options.floating = true;
options.twoD = true;
options.terrain = RigidBodyFlatTerrain();
w = warning('off','Drake:RigidBody:SimplifiedCollisionGeometry');
p = TimeSteppingRigidBodyManipulator('RimlessWheel.urdf',.01,options);
warning(w);
x0 = p.resolveConstraints([0;1+rand;randn;5*rand;randn;5*rand]);

[ytraj,xtraj] = p.simulate([0 10],x0);

v = p.constructVisualizer();
v.axis = [0 5 -.1 3];
v.playback(xtraj);
