function runSpong96

d = AcrobotDynamics;
c = Spong96(d);
v = AcrobotVisualizer(d);

xtraj = simulate(d,c,[0 10]);
playback(v,xtraj);
