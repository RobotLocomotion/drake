function runPassive

gp = GliderPlant();
gv = GliderVisualizer(gp);

traj = simulate(gp,[0 .5]);
playback(gv,traj);
