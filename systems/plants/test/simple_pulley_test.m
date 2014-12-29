function simple_pulley_test

r = PlanarRigidBodyManipulator('simple_pulley.urdf');

clf;
v = r.constructVisualizer();
v.xlim = [-3 3];
v.ylim = [-.2 6.2];

x0 = Point(getStateFrame(r));
x0.slider1 = 2;
x0.slider2 = 2;

ytraj = simulate(r,[0 1],x0);
v.playback(ytraj);

qf = Point(getOutputFrame(r),ytraj.eval(1));
assert(qf.slider1>0);
assert(qf.slider2>0);
assert(qf.slider1>qf.slider2+.1);  % because mass2 weighs more
