function robotiq_sim(urdf, visualize, inspect)

if nargin < 2
  visualize = false;
end

if nargin < 3
  inspect = false;
end

%construct robot model
robot = TimeSteppingRigidBodyManipulator([], 0.01);
robot = robot.addRobotFromURDF(urdf, [0;0;1], [-pi/2;0;0]);
robot = compile(robot);

%resolve initial condition
xstar_hand = robot.getInitialState();
%xstar_hand = robot.resolveConstraints(xstar_hand);

if visualize && inspect
  v = robot.constructVisualizer;
  v.inspector;
  return;
end

%ts = 1:0.01:12.5;
%utraj = setOutputFrame(PPTrajectory(spline(ts,-60*( (ts<5 & ts>2.5) | (ts>7.5 & ts<10)))),getInputFrame(robot));
%utraj = setOutputFrame(PPTrajectory(spline(ts,-60-60*cos(2*ts))),getInputFrame(robot));
%ytraj = simulate(cascade(utraj,robot),[0 10],xstar_hand);


%run simulation
ytraj = simulate(robot, [0,10], xstar_hand);

%optionally visualize
if visualize
  v = robot.constructVisualizer;
  v.playback(ytraj, struct('slider', true));
end
end

