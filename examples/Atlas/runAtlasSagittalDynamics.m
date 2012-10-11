function runAtlasSagittalDynamics

% just runs it as a passive system for now

options.view = 'right';
options.floating = true;
s = warning('off','Drake:PlanarRigidBodyModel:RemovedJoint');
%m = PlanarRigidBodyModel('atlas_robot.urdf',options);
m = PlanarRigidBodyModel('atlas_robot_minimal_contact.urdf',options);
warning(s);
r = TimeSteppingRigidBodyManipulator(m,.01);
v = r.constructVisualizer;
v.display_dt = .05;

x0 = Point(r.getStateFrame);
x0.RElbowPitch = -.2;
x0 = resolveConstraints(r.manip,double(x0));

% Run simulation, then play it back at realtime speed
xtraj = simulate(r,[0 3],x0);
v.playback(xtraj);
