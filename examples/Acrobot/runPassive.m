function runPassive()
% Simulate the passive acrobot

d = AcrobotPlant;
v = AcrobotVisualizer(d);

traj = simulate(d,[0 5],.5*randn(4,1));
playback(v,traj);

end
