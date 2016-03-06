function runSwingUp()
%% runs trajectory optimization and animates open-loop playback

p = UnderwaterAcrobotPlant;
v = UnderwaterAcrobotVisualizer(p);
[utraj,xtraj] = swingUpTrajectory(p);
%      sys = cascade(utraj,p);
%      xtraj=simulate(sys,utraj.tspan,zeros(4,1));
v.playback(xtraj);

% t = linspace(xtraj.tspan(1),xtraj.tspan(2),100);
% figure()
% plot(t,eval(utraj,t));

%playbackAVI(v,xtraj,'swingup');
end
