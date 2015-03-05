function robotiq_sim(urdf, visualize)

if nargin < 2
  visualize = false;
end

%construct robot model
robot = TimeSteppingRigidBodyManipulator([], 0.01);
robot = robot.addRobotFromURDF(urdf, [0;0;1], [-pi/2;0;0]);
robot = compile(robot);

%resolve initial condition
xstar_hands = robot.getInitialState();
xstar_hands = robot.resolveConstraints(xstar_hands);

%run simulation
ytraj = simulate(robot, [0,10], xstar_hands);

%optionally visualize
if visualize
  v = robot.constructVisualizer;
  v.playback(ytraj, struct('slider', true));
end

end

