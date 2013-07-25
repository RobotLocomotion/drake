function runPassive()

d = CartPolePlant;
v = CartPoleVisualizer(d);
xtraj = simulate(d,[0 5]);
playback(v,xtraj);

end
