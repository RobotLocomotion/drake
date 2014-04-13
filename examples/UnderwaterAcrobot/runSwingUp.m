function runSwingUp()
%% runs trajectory optimization and animates open-loop playback

p = UnderwaterAcrobotPlant;
v = UnderwaterAcrobotVisualizer(p);
[utraj,xtraj] = swingUpTrajectory(p);
%      sys = cascade(utraj,p);
%      xtraj=simulate(sys,utraj.tspan,zeros(4,1));
v.playback(xtraj);

%playbackAVI(v,xtraj,'swingup');
end
