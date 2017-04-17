function testDoublePendulumUrdf

cd(fullfile(getDrakePath,'examples','SimpleDoublePendulum'));

% construct the robot from a URDF high-level description file
plant = RigidBodyManipulator('SimpleDoublePendulum.urdf');
visualizer = plant.constructVisualizer();

% all of the other commands will work, e.g.:
trajectory = simulate(plant, [0 5], randn(4,1));
visualizer.playback(trajectory);
[H,C_times_v,G,B] = plant.manipulatorEquations()

