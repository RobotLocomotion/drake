function runAtlasSagittalDynamics

% just runs it as a passive system for now

options.view = 'right';
options.floating = true;
s = warning('off','Drake:PlanarRigidBodyManipulator:RemovedJoint');
m = PlanarRigidBodyManipulator('urdf/atlas_minimal_contact.urdf',options);
warning(s);
r = TimeSteppingRigidBodyManipulator(m,.005);
v = r.constructVisualizer;
v.display_dt = .05;

% Run simulation, then play it back at realtime speed
xtraj = simulate(r,[0 3]);
v.playback(xtraj);
