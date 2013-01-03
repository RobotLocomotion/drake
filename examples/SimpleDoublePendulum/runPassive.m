function runPassive
% Simulate the passive acrobot

d = DoublePendPlant;
v = DoublePendVisualizer(d);

traj = simulate(d,[0 5],[.5*pi;0;0;0]);
playback(v,traj);
