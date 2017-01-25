function runPassive

options.terrain = RigidBodyFlatTerrain();
r = TimeSteppingRigidBodyManipulator(KukaArm(options),0.01,options);

v = r.constructVisualizer;
v.display_dt = .02;

xtraj = simulate(r,[0 1],zeros(r.getNumStates,1));

v.playback(xtraj,struct('slider',true));
