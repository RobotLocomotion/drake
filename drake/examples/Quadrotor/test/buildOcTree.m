function buildOcTree
% simple demonstration of the quadrotor flying through the forest building
% an octree as it flies.

addpath(fullfile(pwd,'..'));

r = Quadrotor('kinect');
r = addTrees(r, 25);
v = r.constructVisualizer();

x0 = Point(r.getStateFrame);
x0.base_x = -5;
x0.base_y = 5;
x0.base_z = .5;
x0.base_xdot = 3;  % 3 m/s
u0 = nominalThrust(r);

treesys = OcTreeSystem(r);
enableLCMGL(treesys,'octree');

% run with a constant input (the nominal input)
sys = cascade(ConstantTrajectory(u0),r);
sys = mimoCascade(sys,treesys);

options.capture_lcm_channels = 'LCMGL';
[~,xtraj,options.lcmlog] = simulate(sys,[0 5],x0,options);

options.slider = true;
v.playback(xtraj,options);

return;

% attach the octree system to the robot, forcing the newly constructed
% system to have the robot state as output (for the visualizer)
output_select(1).system = 1;
output_select(1).output = r.getStateFrame();
sys = mimoCascade(sys,treesys,[],[],output_select);

% feed the robot state to the visualizer
sys = cascade(sys,v);

