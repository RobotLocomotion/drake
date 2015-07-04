function runSwingUpKinematicTest()
%% runs trajectory optimization and animates open-loop playback

p = AcrobotPlantTest;
v = AcrobotVisualizer(p);
[xtraj] = swingUpTrajectoryKinematic(p);
%      sys = cascade(utraj,p);
%      xtraj=simulate(sys,utraj.tspan,zeros(4,1));
v2 = p.constructVisualizer()
v2.playback(xtraj);
%v.playback(xtraj);

end
