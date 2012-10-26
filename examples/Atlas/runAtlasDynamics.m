function runAtlasDynamics
% NOTEST

% just runs it as a passive system for now
options.floating = true;
m = RigidBodyModel('atlas_robot.urdf',options);
%r = setSimulinkParam(r,'MinStep','0.001');
r = TimeSteppingRigidBodyManipulator(m,.005);
v = r.constructVisualizer;
v.display_dt = .05;

x0 = Point(r.getStateFrame);
x0.RElbowPitch = -.2;
x0 = r.manip.resolveConstraints(double(x0));

% Run simulation, then play it back at realtime speed
xtraj = simulate(r,[0 3],x0);
v.playback(xtraj);
