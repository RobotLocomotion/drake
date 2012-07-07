function runPassive()
% runs the passive system

pd = PendulumPlant;
pv = PendulumVisualizer(pd);
traj = simulate(pd,[0 5],randn(2,1));
playback(pv,traj);

end

