function runPassiveLCP

options.twoD = false;
options.floating = false;
w = warning('off','Drake:RigidBody:SimplifiedCollisionGeometry');
p = TimeSteppingRigidBodyManipulator('Strandbeest.urdf',.01,options);
% p = p.setTerrain(RigidBodyFlatTerrain()).compile();
warning(w);
v = p.constructVisualizer();
v.inspector()

% x0 = p.getInitialState();
% x0(3) = x0(3) + 1;

% xtraj = p.simulate([0 5], x0);

% v.playback(xtraj);
