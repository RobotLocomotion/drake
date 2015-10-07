function runDynamics
% just runs it as a passive system

options.twoD = true;
options.view = 'right';
options.floating = true;
options.terrain = RigidBodyFlatTerrain();
s = 'urdf/spring_flamingo_passive_ankle.urdf';
dt = 0.005;
w = warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits');
r = TimeSteppingRigidBodyManipulator(s,dt,options);
warning(w);

v = r.constructVisualizer;
v.display_dt = 0.01;

% Run simulation, then play it back at realtime speed
xtraj = simulate(r,[0 3]);
v.playback(xtraj);
end

