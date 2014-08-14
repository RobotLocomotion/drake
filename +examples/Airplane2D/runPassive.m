function runPassive

pp = PlanePlant();
pv = PlaneVisualizer(pp);

x0 = [0; 0; 0; 1];


traj = simulate(pp,[0 .5], x0);
playback(pv,traj);
