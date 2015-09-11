function runPassiveLCP

options.twoD = false;
% options.floating = true;
w = warning('off','Drake:RigidBody:SimplifiedCollisionGeometry');
p = TimeSteppingRigidBodyManipulator('Strandbeest.urdf',.01,options);
warning(w);
% p = p.setTerrain(RigidBodyFlatTerrain()).compile();
v = p.constructVisualizer();
% load('x0_no_floating_base.mat', 'x0');
% nq = p.getNumPositions();
% x0 = [0;0;2;0;0;0; x0(1:nq); zeros(6,1); x0(nq+1:end)];
v.inspector();

% sys = cascade(p, v);

% xtraj = sys.simulate([0 5], x0);

% v.playback(xtraj, struct('slider', true));
