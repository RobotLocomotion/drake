function timeSteppingRigidBodyManipulatorFramesTest
% Check that various operations on TimeSteppingRigidBodyManipulator objects
% still work when the state frame contains more than simply the
% RigidBodyManipulator state.

S = warning('OFF','Drake:RigidBodyManipulator:WeldedLinkInd');
urdf = [getDrakePath() '/systems/plants/test/ActuatedPendulum.urdf'];
p = TimeSteppingRigidBodyManipulator(urdf,.01);

% Check simulation w/out sensors
[~,xtraj] = simulate(p,[0 5]);

p = addSensor(p,FullStateFeedbackSensor());

% Commented out pending re-write of ContactForceTorqueSensor
%frame = p.getFrame(p.findFrameId('tip'));
%p = addSensor(p,ContactForceTorqueSensor(p,frame));

p = compile(p);

% Check construction of visualizer
v = p.constructVisualizer();

% Check simulation
[~,xtraj] = simulate(p,[0 5]);

% Check playback
v.playback(xtraj)

% Check constraint resolution
[xstar,success] = p.resolveConstraints(zeros(p.getNumStates(),1));
v.draw(0,xstar)

% Check pd-controller creation
Kp = 10;
Kd = 1;
sys_pd = pdcontrol(p,Kp,Kd);

% Check pd-controller simulation
q_des = pi/4*(2*rand(1) - 1);
q_des_traj = ConstantTrajectory(Point(sys_pd.getInputFrame,q_des));
sys = cascade(q_des_traj,sys_pd);
[~,xtraj] = simulate(sys,[0,5],xstar);
v.playback(xtraj);

% Check fixed-point computation
x0 = rand(p.getNumStates(),1);
u0 = rand(p.getNumInputs(),1);
[xstar,ustar,success] = findFixedPoint(p,x0,u0,struct('active_collision_groups','','visualize',true));
v.draw(0,xstar)

warning(S);
