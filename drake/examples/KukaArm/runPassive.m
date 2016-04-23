function runPassive

options.floating = false;
options.terrain = RigidBodyFlatTerrain();
r = TimeSteppingRigidBodyManipulator('urdf/lbr_iiwa.urdf',0.001,options);

v = r.constructVisualizer;
v.display_dt = .05;

xtraj = simulate(r,[0 2]);

v.playback(xtraj,struct('slider',true));
