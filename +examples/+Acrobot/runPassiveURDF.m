function runPassiveURDF()
% Simulate the passive acrobot

w = warning('off','Drake:RigidBody:SimplifiedCollisionGeometry');
d = PlanarRigidBodyManipulator('Acrobot.urdf');
warning(w);
v = d.constructVisualizer();

traj = simulate(d,[0 5],.5*randn(4,1));
playback(v,traj);

end
