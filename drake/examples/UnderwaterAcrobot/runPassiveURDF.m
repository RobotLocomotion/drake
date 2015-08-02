function runPassiveURDF()
% Simulate the passive acrobot, as pulled from URDF

w = warning('off','Drake:RigidBody:SimplifiedCollisionGeometry');
d = PlanarRigidBodyManipulator('UnderwaterAcrobot.urdf');
warning(w);
v = d.constructVisualizer();

traj = simulate(d,[0 5],.5*randn(4,1));
playback(v,traj);

end
