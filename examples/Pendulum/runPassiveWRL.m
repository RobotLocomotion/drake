function runPassiveWRL

p = PlanarRigidBodyManipulator('Pendulum.urdf');
x = p.simulate([0 5],randn(2,1));

if (checkDependency('vrml'))
  v = RigidBodyWRLVisualizer(p);
  v.playback(x);
end
