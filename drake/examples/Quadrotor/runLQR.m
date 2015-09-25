function runLQR

%r = Quadrotor();
r = QuadPlantPenn();  % sim is faster with this
v = r.constructVisualizer();

x0 = [0;0;1;zeros(9,1)];
u0 = double(nominalThrust(r));

Q = diag([10*ones(6,1); ones(6,1)]);
R = .1*eye(4);

c = tilqr(r,x0,u0,Q,R);

sys = feedback(r,c);

for i=1:5
  xtraj = simulate(sys,[0 4],double(x0)+[.5*randn(6,1);zeros(6,1)]);
  v.playback(xtraj);
end