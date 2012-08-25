function runPassiveWRL

p = PlanarRigidBodyManipulator('Pendulum.urdf');
x = p.simulate([0 5],randn(2,1));

if (checkDependency('vrml_enabled'))
  v = p.constructWRLVisualizer;
  v.playback(x);
end