function runPassiveURDF
% runs the passive dynamics with the rigid body backend (from URDF)

p = PlanarRigidBodyManipulator('CompassGait.urdf');
xtraj = p.simulate([0 5]);

v=p.constructVisualizer();
v.axis = 2*[-1 1 -1 1];
v.playback_speed = .2;
v.playback(xtraj);

