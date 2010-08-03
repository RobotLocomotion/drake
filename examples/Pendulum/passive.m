function passive
% Simulate the passive pendulum

pd = PendulumDynamics;
pv = PendulumVisualizer;

traj = simulate(pd,[0 5]);
playback(pv,traj);

