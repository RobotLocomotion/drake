function simple_pulley_test

r = PlanarRigidBodyManipulator('simple_pulley.urdf');

clf;
v = r.constructVisualizer();
v.xlim = [-3 3];
v.ylim = [-.2 6.2];

x0 = Point(getStateFrame(r));
x0.slider1 = 2;
x0.slider2 = 2.2;

x0 = resolveConstraints(r,x0);
v.draw(0,x0(1:2));
%return;

ytraj = simulate(r,[0 4],x0);
if(0)
ts = ytraj.getBreaks();
for i=1:numel(ts)
  x = ytraj.eval(ts(i));
  length(i) = r.position_constraints{1}.eval(x(1:2));
end
figure(1); clf; plot(ts,length);
end

v.playback(ytraj,struct('slider',true));



qf = Point(getOutputFrame(r),ytraj.eval(1));
assert(qf.slider1>0);
assert(qf.slider2>0);
assert(qf.slider1>qf.slider2+.1);  % because mass2 weighs more
