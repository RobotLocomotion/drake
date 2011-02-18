function runDLQR

pd = sampledData(PendulumPlant,.1);
pv = PendulumVisualizer;
c = PendulumLQR(pd);
sys = cascade(feedback(pd,c),pv);

for i=1:5
  simulate(sys,[0 4],[pi;0]+0.2*randn(2,1));
end
