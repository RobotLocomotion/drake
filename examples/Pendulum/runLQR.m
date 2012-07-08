function runLQR()

% run the lqr controller from a handful of initial conditions
pd = PendulumPlant;
pv = PendulumVisualizer();
c = balanceLQR(pd);
sys = feedback(pd,c);
for i=1:5
  xtraj=simulate(sys,[0 4],[pi;0]+0.2*randn(2,1));
  pv.playback(xtraj);
end

end
