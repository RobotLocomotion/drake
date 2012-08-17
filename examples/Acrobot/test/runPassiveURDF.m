function runPassiveURDF()
% Simulate the passive acrobot

%r = PlanarRigidBodyManipulator('../Acrobot.urdf');
r = HybridPlanarRigidBodyManipulator('../Acrobot.urdf');
v = r.constructVisualizer;

traj = simulate(r,[0 5],[1;pi/4;pi/6;0;4]);
playback(v,traj);

end
