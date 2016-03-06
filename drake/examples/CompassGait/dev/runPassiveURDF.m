% function runPassiveURDF
% runs the passive dynamics with the rigid body backend (from URDF)

options.twoD = true;
options.terrain = RigidBodyFlatTerrain();
p = TimeSteppingRigidBodyManipulator('CompassGait.urdf', 0.01, options);
% p = CompassGaitPlant()
% p.getInitialState()
% xtraj = p.simulate([0 2], p.getInitialState);
xtraj = p.simulate([0 2], p.getInitialState + [0 .1 0 pi/2 0 0 0 0]');

v=p.constructVisualizer();
v.axis = 2*[-1 1 -1 1];
v.playback_speed = 1;
v.playback(xtraj);

