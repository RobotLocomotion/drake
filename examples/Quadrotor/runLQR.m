function runLQR

r = Quadrotor();
v = r.constructVisualizer();

x0 = [0;0;.5;zeros(9,1)];
u0 = nominalThrust(r);

Q = diag([10*ones(6,1); ones(6,1)]);
R = .1*eye(4);
c = tilqr(r,x0,u0,Q,R);

%sys = cascade(setOutputFrame(ConstantTrajectory(u0),getInputFrame(r)),r);
sys = feedback(r,c);

xtraj = simulate(sys,[0 1],double(x0)+.1*randn(12,1));
v.playback(xtraj);
