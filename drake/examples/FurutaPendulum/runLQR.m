function runLQR

% Construct robot and visualizer from urdf
plant = RigidBodyManipulator('FurutaPendulum.urdf');
visualizer = plant.constructVisualizer;

% Set the goal state for balancing
xGoal = Point(getStateFrame(plant));  % default is all zeros
xGoal.elbow = pi;
uGoal = Point(getInputFrame(plant));

% Set up an LQR problem (tilqr == Time Invariant Linear Quadratic Regulator)
Q = diag([10 10 1 1]);
R = .1;
controller = tilqr(plant,xGoal,uGoal,Q,R);

% Combine the feedback controller with the robot to get a new closed-loop
% system
sys = feedback(plant,controller);

% Simulate the closed loop system from a handful of random initial
% conditions, and visualize the results
for i=1:5
  x0 = double(xGoal)+.2*randn(4,1);
  xtraj = simulate(sys,[0 4],x0);
  visualizer.playback(xtraj);
end

