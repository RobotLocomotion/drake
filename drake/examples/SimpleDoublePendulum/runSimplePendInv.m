function runSimplePendInv
% Simulate the passive acrobot

d = DoublePendPlant;
c = SimplePendInv(d);
v = DoublePendVisualizer(d);

sys = feedback(d,c);

traj = simulate(sys,[0 5],[pi/2;0;0;0]);
playback(v,traj);
