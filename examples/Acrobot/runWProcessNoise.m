function runWProcessNoise

r = RigidBodyManipulator('Acrobot.urdf');
for i=1:2, % add position sensors
  b = r.getBody(i);
  b.has_position_sensor = true;
  r = r.setBody(i,b);
end
r = compile(r);

sys = DrakeSystemWGaussianNoise(r,0.1*diag([0 0 1 1]),[],zeros(2),.01);

x0 = randn(4,1);
ytraj = simulate(r,[0 5],x0);
figure(1); clf;
subplot(2,1,1); fnplt(ytraj,1); hold on;
subplot(2,1,2); fnplt(ytraj,2); hold on;

for i=1:5
  ytraj = simulate(sys,[0 5],x0);
  figure(1);
  subplot(2,1,1); fnplt(ytraj,1);
  subplot(2,1,2); fnplt(ytraj,2);
end

