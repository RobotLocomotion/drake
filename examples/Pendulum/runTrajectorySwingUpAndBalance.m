function runTrajectorySwingUpAndBalance()
% tilqr + funnel at the top, traj opt + tvlqr + funnel for swingup

pd = PendulumPlant;
pd = pd.setInputLimits(-inf,inf);  % for now
pv = PendulumVisualizer();
c = trajectorySwingUpAndBalance(pd);
sys = feedback(pd,c);
for i=1:5
  xtraj=simulate(sys,[0 6]);
  pv.playback(xtraj);
end

end
