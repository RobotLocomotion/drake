function timeSteppingRigidBodyManipulatorFramesTest
% Check that various operations on TimeSteppingRigidBodyManipulator objects
% still work when the state frame contains more than simply the
% RigidBodyManipulator state.

S = warning('OFF','Drake:RigidBodyManipulator:WeldedLinkInd');
options.floating = true;
options.twoD = true;
p = TimeSteppingRigidBodyManipulator('FallingBrick.urdf',.01,options);

p = addSensor(p,FullStateFeedbackSensor());
body = findLinkInd(p,'brick');
frame = RigidBodyFrame(body,zeros(3,1),zeros(3,1),'FT_frame');
p = addFrame(p,frame);
p = addSensor(p,ContactForceTorqueSensor(p,frame));
p = compile(p);

% Check simulation
[ytraj,xtraj] = simulate(p,[0 5]);

% Check construction of visualizer
v = p.constructVisualizer();

% Check playback
v.playback(xtraj)

% Check constraint resolution
[xstar,success] = p.resolveConstraints(zeros(p.getNumStates(),1));
v.draw(0,xstar)

warning(S);