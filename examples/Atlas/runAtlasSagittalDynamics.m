function runAtlasSagittalDynamics
% just runs it as a passive system

options.twoD = true;
options.view = 'right';
options.floating = true;
%s = 'urdf/simple_atlas_minimal_contact.urdf';
s = 'urdf/atlas_minimal_contact.urdf';
dt = 0.005;
w = warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits');
r = TimeSteppingRigidBodyManipulator(s,dt,options);
warning(w);

v = r.constructVisualizer;
v.display_dt = 0.02;

% Run simulation, then play it back at realtime speed
xtraj = simulate(r,[0 3]);
v.playback(xtraj);
