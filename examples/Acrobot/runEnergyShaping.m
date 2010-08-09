function runSpong96

d = AcrobotDynamics;
d.ode_options = odeset('RelTol',1e-6);
d.setInputLimits(-inf,inf);

c = EnergyShaping(d);
v = AcrobotVisualizer(d);

xtraj = simulate(d,c,[0 10],.001*randn(4,1));
v.playback_speed = .5;
playback(v,xtraj);
