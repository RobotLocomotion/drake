function runAtlasFrontalDynamics

% just runs it as a passive system for now

options.view = 'front';
options.floating = true;
s = warning('off','Drake:PlanarRigidBodyModel:RemovedJoint');
m = PlanarRigidBodyModel('atlas_robot.urdf',options);
warning(s);
r = TimeSteppingRigidBodyManipulator(m,.005);
v = r.constructVisualizer;
v.display_dt = .05;

x0 = Point(r.getStateFrame);
x0.RShoulderRoll = -.2;
x0 = resolveConstraints(r.manip,double(x0));

% Run simulation, then play it back at realtime speed
xtraj = simulate(r,[0 5],x0);
v.playback(xtraj);
