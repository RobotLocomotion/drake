function energyShaping

pd = PendulumDynamics;
pv = PendulumVisualizer;
c = PendulumEnergyShaping(pd);

for i=1:5
  xtraj = simulate(pd,[0 6],randn(2,1),c);
  playback(pv,xtraj);
end
