function runPassive

w = warning('off','Drake:RigidBody:SimplifiedCollisionGeometry');
p = PlanarRigidBodyManipulator('FourBar.urdf');
warning(w);
v = p.constructVisualizer();
v.xlim = [-8 8]; v.ylim = [-4 10];

w = warning('off','Drake:DrakeSystem:ConstraintsNotEnforced');
xtraj = p.simulate([0 10]);
warning(w);

v.playback(xtraj);
