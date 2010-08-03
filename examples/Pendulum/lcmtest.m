function timerobj=lcmtest
% Runs the pendulum dynamics as an lcm client

pd = PendulumDynamics;
plcm = PendulumLCMCoder;

timerobj=simulateLCM(pd,plcm);