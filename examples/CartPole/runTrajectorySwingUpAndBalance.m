function runTrajectorySwingUpAndBalance()
% tilqr + funnel at the top, traj opt + tvlqr + funnel for swingup

pd = CartPolePlant;
pd = pd.setInputLimits(-inf,inf);  % for now
pv = CartPoleVisualizer(pd);
c = trajectorySwingUpAndBalance(pd);
%[c,V] = trajectorySwingUpAndBalance(pd);  % use this version if you want the verification
sys = feedback(pd,c);
for i=1:5
  xtraj=simulate(sys,[0 6]);
  pv.playback(xtraj);
end

end

