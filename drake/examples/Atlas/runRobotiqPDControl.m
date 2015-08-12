function runRobotiqPDControl

%construct robot model
robot = TimeSteppingRigidBodyManipulator([], 0.01);
robot = robot.addRobotFromURDF('urdf/robotiq_simple.urdf', [0;0;1], [-pi/2;0;0]);
robot = robot.setJointLimits(-inf(robot.getNumPositions,1),inf(robot.getNumPositions,1));
robot = compile(robot);

%resolve initial condition
xstar_hand = robot.getInitialState();
xstar_hand = robot.resolveConstraints(xstar_hand);

%pd gains
kp = 3;
kd = 0.1;

%build reference trajectory
ts = 0:0.01:12;
q_closed = [1.2427, 1.0864, -0.1336, 1.2427, 1.0864,-0.1336, 1.2427, 0.9114, 0.0000]';
q_open = zeros(robot.getNumPositions, 1);
qdes_traj = PPTrajectory(spline(ts, spline([0 3 6 9 12],[q_open, q_closed, q_open, q_closed, q_open],ts)));

%wire up closed loop system and simulate
sys_pd = robot.pdcontrol(kp*eye(robot.getNumPositions), kd*eye(robot.getNumVelocities));
ytraj = simulate(cascade(setOutputFrame(qdes_traj, sys_pd.getInputFrame), sys_pd), [0, ts(end)], xstar_hand);

v = robot.constructVisualizer;
v.playback(ytraj, struct('slider', true));
end

