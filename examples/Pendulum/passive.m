function passive

pd = PendulumDynamics;

traj = simulate(pd,[0 5]);

fnplt(traj);