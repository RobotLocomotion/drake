function runPassive

gp = GliderPlant();
gv = GliderVisualizer();

traj = simulate(gp,[0 .5]);
playback(gv,traj);
