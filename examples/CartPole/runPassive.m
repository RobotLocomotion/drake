function runPassive

d = CartPolePlant;
c = ConstantControl(0);
v = CartPoleVisualizer(d);

xtraj = simulate(d,c,[0 5]);

playback(v,xtraj);
