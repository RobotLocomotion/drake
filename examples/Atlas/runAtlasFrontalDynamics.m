function runAtlasFrontalDynamics
% just runs it as a passive system

options.view = 'front';
options.twoD = true;
options.floating = true;
options.terrain = RigidBodyFlatTerrain();
% s = 'urdf/simple_atlas_minimal_contact.urdf';
s = 'urdf/atlas_minimal_contact.urdf';
dt = 0.005;
w = warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits');
r = TimeSteppingRigidBodyManipulator(s,dt,options);
warning(w);

v = r.constructVisualizer;
v.display_dt = .02;

% Run simulation, then play it back at realtime speed
xtraj = simulate(r,[0 3]);
v.playback(xtraj);
