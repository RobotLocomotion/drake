function testTSRB

% unit test to make sure that TimeSteppingRigidBodyManipulator works for
% the case when there are no contacts, limits, etc.

r = TimeSteppingRigidBodyManipulator('../Acrobot.urdf',.001);
r_orig = PlanarRigidBodyManipulator('../Acrobot.urdf');

x0 = randn(4,1);
xtraj = simulate(r,[0 4],x0);
xtraj_orig = simulate(r_orig,[0 4],x0);

v = r_orig.constructVisualizer();
v2 = v;
v2.fade_percent = .5;
mv = MultiVisualizer({v2,v});

mv.playback([setOutputFrame(xtraj,getOutputFrame(xtraj_orig));xtraj_orig]);

valuecheck(eval(xtraj(1:2),4),eval(xtraj_orig(1:2),4),.1);

