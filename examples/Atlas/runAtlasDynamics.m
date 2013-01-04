function runAtlasDynamics
% NOTEST

% just runs it as a passive system for now
options.floating = true;
r = TimeSteppingRigidBodyManipulator('urdf/atlas_minimal_contact.urdf',.001,options);
v = r.constructVisualizer;
v.display_dt = .05;

x0 = Point(r.getStateFrame);
x0 = r.manip.resolveConstraints(double(x0));

% Run simulation, then play it back at realtime speed
xtraj = simulate(r,[0 3],x0);
v.playback(xtraj);
