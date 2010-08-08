function runPassive
% Simulate the passive acrobot

d = AcrobotDynamics;
v = AcrobotVisualizer(d);
c = ConstantControl(0);

traj = simulate(d,c,[0 5]);
playback(v,traj);
