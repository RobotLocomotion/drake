function runPassive

options.floating = false;
options.terrain = RigidBodyFlatTerrain();
w = warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits');
r = TimeSteppingRigidBodyManipulator('urdf/iiwa14.urdf',0.001,options);
warning(w);

v = r.constructVisualizer;
v.display_dt = .05;

xtraj = simulate(r,[0 2],zeros(r.getNumStates,1));

v.playback(xtraj,struct('slider',true));
