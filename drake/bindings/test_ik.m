addpath('drake_matlab');

lc = lcm_wrapper.LCM()
p = pendulum_wrapper.PendulumWithBotVis(lc);
controller = p.balanceLQR();