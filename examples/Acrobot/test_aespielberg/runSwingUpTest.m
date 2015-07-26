function runSwingUpTest()
%% runs trajectory optimization and animates open-loop playback

p = AcrobotPlantTest;
v = AcrobotVisualizer(p);
[utraj,xtraj] = swingUpTrajectory(p);
%      sys = cascade(utraj,p);
%      xtraj=simulate(sys,utraj.tspan,zeros(4,1));
v.playback(xtraj);

end
