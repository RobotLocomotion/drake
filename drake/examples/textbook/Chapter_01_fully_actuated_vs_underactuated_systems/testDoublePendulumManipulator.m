function testDoublePendulumManipulator

% a platform independent way to cd into the example directory
cd(fullfile(getDrakePath,'examples','SimpleDoublePendulum'));

% first construct the "robot" or "plant" object
plant = DoublePendPlant;  % check out the code in DoublePendPlant.m

% a visualizer will draw the robot
visualizer = DoublePendVisualizer(plant);
visualizer.inspector();  % (optional) simple gui to examine the joints

% simulate the robot from time=0 to time=5 seconds from random
% initial conditions taken from a Gaussian distribution (using randn)
trajectory = simulate(plant, [0 5], randn(4,1));

% play back the simulation using the cpu clock to get the timing right
visualizer.playback(trajectory);

% if you want to access the manipulator equations programmatically,
% you can use, e.g.:
[H,C_times_v,G,B] = plant.manipulatorEquations()
% which outputs using the abbreviations v for qdot, s1 for sin(q1), etc

