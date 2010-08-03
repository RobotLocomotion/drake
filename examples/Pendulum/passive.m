function passive

pd = PendulumDynamics;

traj = simulate(pd);

fnplt(traj);