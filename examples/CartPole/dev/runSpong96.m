function runLQR

d = CartPolePlant;
v = CartPoleVisualizer;
c = Spong96(d);

xtraj = simulate(d,c,[0 10],[]);
playback(v,xtraj);

end




