function runRobotiqTendons

%construct robot model
robot = TimeSteppingRigidBodyManipulator([], 0.01);
robot = robot.addRobotFromURDF('urdf/robotiq_tendons.urdf', [0;0;1], [-pi/2;0;0]);
robot = compile(robot);

%resolve initial condition
xstar_hand = robot.getInitialState();
xstar_hand = robot.resolveConstraints(xstar_hand);

%run simulation
ts = 0:0.01:12;

utraj = setOutputFrame(PPTrajectory(spline(ts,cos(2*ts))),getInputFrame(robot));
ytraj = simulate(cascade(utraj,robot),[0 ts(end)],xstar_hand);

%playback
v = robot.constructVisualizer;
v.playback(ytraj, struct('slider', true));
end

