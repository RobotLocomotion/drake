function testHybridRBM

p = RigidBodyManipulator(fullfile(getDrakePath,'examples','Acrobot','Acrobot.urdf'));
p = setJointLimits(p,[-inf;-1.5],[inf;1.5]);
p = compile(p);

p = HybridRigidBodyManipulator(HybridRigidBodyMode(p));

x0 = [1;zeros(2,1);0;pi/4;0;5]+[0;zeros(2,1);randn(4,1)];
x0 = resolveConstraints(p,x0);
[ytraj,xtraj] = simulate(p,[0,4],x0);

theta2 = ytraj(2);

figure(1); clf;
fnplt(theta2);
hold on;
h = fnplt(1.5*xtraj(3)); set(h,'Color','r');  % draw the joint limit mode
return

v = p.constructVisualizer();
v.playback(ytraj);

for ts=0:.1:4;
  assert(theta2.eval(ts)<=1.5 && theta2.eval(ts)>=-1.5);
end



