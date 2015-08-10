function multiple_pulleys
%r = PlanarRigidBodyManipulator('multiple_pulleys.urdf');
r = TimeSteppingRigidBodyManipulator('multiple_pulleys.urdf',.01,struct('twoD',true));


clf;
v = r.constructVisualizer();
v.xlim = [-5 5];
v.ylim = [-.2 6.2];

x0 = Point(getStateFrame(r));
x0.slider1 = 2;
x0.slider2 = 2.2;

%[l,dl]=r.position_constraints{1}.eval(x0(1:2))
%r.position_constraints{1}.checkGradient(.001,x0(1:2));

x0 = resolveConstraints(r,x0);
v.draw(0,x0(1:2));

%return;

ytraj = simulate(r,[0 4],x0);

ts = ytraj.getBreaks();
rbm = r.getManipulator();
length = rbm.position_constraints{1}.eval(x0(1:2));
for i=1:numel(ts)
  x = ytraj.eval(ts(i));
  valuecheck(rbm.position_constraints{1}.eval(x(1:2)),length,1e-2);
end

v.playback(ytraj,struct('slider',true));



qf = Point(getOutputFrame(r),ytraj.eval(1));
assert(qf.slider1>0);
assert(qf.slider2>0);
assert(qf.slider1>qf.slider2+.1);  % because mass2 weighs more
