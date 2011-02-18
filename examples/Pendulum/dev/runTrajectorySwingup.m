function runTrajectorySwingup

pd = PendulumPlant;
pv = PendulumVisualizer;
c = TrajectorySwingup(pd);

for i=1:5
  xtraj = simulate(pd,c,[0 6],.01*randn(2,1));
  playback(pv,xtraj);
end
