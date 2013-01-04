function runAtlasFrontalDynamics

% just runs it as a passive system for now

options.view = 'front';
options.floating = true;
s = warning('off','Drake:PlanarRigidBodyManipulator:RemovedJoint');
m = PlanarRigidBodyManipulator('urdf/atlas_minimal_contact.urdf',options);
warning(s);
r = TimeSteppingRigidBodyManipulator(m,.001);
v = r.constructVisualizer;
v.display_dt = .05;

x0 = Point(r.getStateFrame);
x0 = resolveConstraints(r.manip,double(x0));

% Run simulation, then play it back at realtime speed
xtraj = simulate(r,[0 3],x0);
v.playback(xtraj);
