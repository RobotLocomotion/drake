function runSwingUp()
% runs trajectory optimization and animates open-loop playback

pd = PendulumPlant;
pv = PendulumVisualizer(pd);
utraj = swingUpTrajectory(pd);
sys = cascade(utraj,pd);
xtraj=simulate(sys,utraj.tspan,[0;0]);
pv.playback(xtraj);

end
