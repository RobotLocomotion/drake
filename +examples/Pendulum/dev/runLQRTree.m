function runTrajectorySwingUpAndBalance()
% tilqr + funnel at the top, traj opt + tvlqr + funnel for swingup

pd = PendulumPlant;
pv = PendulumVisualizer();
c = balanceLQRTree(pd);
sys = feedback(pd,c);
for i=1:5
  xtraj=simulate(sys,[0 6]);
  pv.playback(xtraj);
end

end
