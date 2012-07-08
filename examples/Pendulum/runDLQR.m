function runDLQR()
%% runs lqr on a sampled-data version of the plant dynamics

pd = sampledData(PendulumPlant,.1);
pv = PendulumVisualizer();
x0=[pi;0]; u0=0;
Q = diag([10 1]); R = 1;
c = tilqr(pd,x0,u0,Q,R);
sys = feedback(pd,c);
for i=1:5
  xtraj=simulate(sys,[0 4],[pi;0]+0.2*randn(2,1));
  pv.playback(xtraj);
end

end
