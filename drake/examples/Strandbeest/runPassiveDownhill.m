function runPassiveDownhill
% To learn more about Stranbeests, created by Theo Jansen, see http://www.strandbeest.com/
% Leg measurements taken from http://files.tested.com/photos/2015/04/05/74549-leg-measurement.jpg

% Load our URDF model of the Strandbeest
options.floating = true;
w = warning('off','Drake:RigidBody:SimplifiedCollisionGeometry');
p = TimeSteppingRigidBodyManipulator('Strandbeest.urdf',.01,options);
warning(w);

% Give it something to walk on
terrain = RigidBodyFlatTerrain();
% tilt the terrain
terrain.geom.T = makehgtform('yrotate', .15) * terrain.geom.T;
p = p.setTerrain(terrain).compile();

% Add a visualizer
v = p.constructVisualizer();
output_select(1).system=1;
output_select(1).output=1;
sys = mimoCascade(p,v,[],[],output_select);

% Simulate the system
x0 = p.getInitialState();
x0(1:6) = [0; 0; 1.25; 0; 0; 0];
nq = p.getNumPositions();
x0(nq+1:end) = 0;
% v.inspector(x0);
xtraj = sys.simulate([0 15], x0, struct('gui_control_interface', true));

v.playback(xtraj, struct('slider', true));
