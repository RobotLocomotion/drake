function runPDtest

r = PlanarRigidBodyManipulator('SimpleDoublePendulum.urdf');
v = r.constructVisualizer();
v.axis = [-2 2 -2 2];

sys = pdcontrol(r,diag([100 100]),diag([10 10]));

theta_d = FunctionHandleTrajectory(@(t)[0;sin(t/2)],2,[0 5]);
theta_d = setOutputFrame(theta_d,sys.getInputFrame);

x0 = randn(4,1);
tf = 15;
xtraj = simulate(cascade(theta_d,sys),[0 tf],x0);

%v.playback(xtraj);

ts = 0:.1:tf;
theta_ds = eval(theta_d,ts);
thetas = eval(xtraj(1:2),ts);
figure(1); clf;
subplot(2,1,1);
plot(ts,theta_ds(1,:),ts,thetas(1,:));
subplot(2,1,2);
plot(ts,theta_ds(2,:),ts,thetas(2,:));

if (norm(thetas(:,end)-theta_ds(:,end))>.2)
  error('PD controller has large position error at the end of the simulation');
end