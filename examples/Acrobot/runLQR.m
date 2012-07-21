function runLQR()
%% run the lqr controller from a handful of initial conditions

p = AcrobotPlant;
v = AcrobotVisualizer(p);
c = balanceLQR(p);
sys = feedback(p,c);
for i=1:5
  xtraj=simulate(sys,[0 4],[pi;0;0;0]+0.05*randn(4,1));
  v.playback(xtraj);
end

end
