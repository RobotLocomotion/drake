function runWProcessNoise

r = RigidBodyManipulator('Acrobot.urdf');
sys = DrakeSystemWGaussianNoise(r,0.1*diag([0 0 1 1]),[],zeros(4),.01);

x0 = randn(4,1);
xtraj = simulate(r,[0 5],x0);
figure(1); clf;
subplot(2,1,1); fnplt(xtraj,1); hold on;
subplot(2,1,2); fnplt(xtraj,2); hold on;

for i=1:5
  xtraj = simulate(sys,[0 5],x0);
  figure(1);
  subplot(2,1,1); fnplt(xtraj,1);
  subplot(2,1,2); fnplt(xtraj,2);
end

