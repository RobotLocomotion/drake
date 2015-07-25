function runSwingUpKinematicTest()
%% runs trajectory optimization and animates open-loop playback

p = AcrobotPlantTest;
v = AcrobotVisualizer(p);
[xtraj,z,F,info] = swingUpTrajectoryKinematic(p);
%      sys = cascade(utraj,p);
%      xtraj=simulate(sys,utraj.tspan,zeros(4,1));
p = p.setParams(z(end-5:end)); %TODO: make this cleaner
v2 = p.constructVisualizer()
v2.playback(xtraj);
%v.playback(xtraj);

end
