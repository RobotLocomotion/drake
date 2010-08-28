function energyShaping

pd = PendulumPlant;
pv = PendulumVisualizer;
c = PendulumEnergyShaping(pd);

for i=1:5
  xtraj = simulate(pd,c,[0 6]);
  playback(pv,xtraj);
end
