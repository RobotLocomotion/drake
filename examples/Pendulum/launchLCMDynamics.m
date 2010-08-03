function timerobj=launchLCMDynamics
% Runs the pendulum dynamics as an lcm client

pd = PendulumDynamics;
plcm = PendulumLCMCoder;
pv = PendulumVisualizer;

timerobj=simulateLCM(pd,plcm,.01,rand(2,1),pv);