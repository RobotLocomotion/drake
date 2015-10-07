function runPlanarAtlasSpringAnkleDynamics
% just runs it as a passive system

options.twoD = true;
options.view = 'right';
options.floating = true;
l = 0.4;
boxes = [0.0, 0.0, 2*l, 1, 0.12;
         l/2, 0.0, l, 1, 0.24];
options.terrain = RigidBodyStepTerrain(boxes);
%options.terrain = RigidBodyFlatTerrain();
s = '../urdf/atlas_simple_spring_ankle.urdf';
dt = 0.001;
w = warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits');
r = TimeSteppingRigidBodyManipulator(s,dt,options);
r = r.removeCollisionGroupsExcept({'heel','toe'});
r = compile(r);
warning(w);

v = r.constructVisualizer;
v.display_dt = 0.01;
x0 = zeros(r.getNumStates(),1);
x0(2) = 1.5;
% Run simulation, then play it back at realtime speed
xtraj = simulate(r,[0 1],x0);
v.playback(xtraj,struct('slider',true));
