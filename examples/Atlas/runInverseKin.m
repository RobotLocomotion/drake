function runInverseKin
% a simple example of how to use inverse kinematics for the Atlas model

% Load the model with a floating base
options.floating = true;
options.dt = 0.001;
r = Atlas('urdf/atlas_minimal_contact.urdf',options);

% Initialize the viewer
v = r.constructVisualizer;
v.display_dt = 0.01;

% Set up the cost structure for IK (some joint deflections are worse than
% others)
cost = Point(r.getStateFrame,1);
cost.base_x = 0;
cost.base_y = 0;
cost.base_z = 0;
cost.base_roll = 1000;
cost.base_pitch = 1000;
cost.base_yaw = 0;
cost.back_bky = 100;
cost.back_bkx = 100;
options = struct();
cost = double(cost);
options.Q = diag(cost(1:r.getNumDOF));

% set initial guess to a known comfortable fixed point
d = load('data/atlas_fp.mat');
q0 = d.xstar(1:r.getNumDOF);

% Get references to some key points on the body
com = 0;
r_foot = findLinkInd(r,'r_foot');
r_foot_pts = r.getBody(r_foot).getContactPoints();
l_foot = findLinkInd(r,'l_foot');
l_foot_pts = r.getBody(l_foot).getContactPoints();
r_hand = findLinkInd(r,'r_hand');
r_hand_pts = r.getBody(r_hand).getContactPoints();
l_hand = findLinkInd(r,'l_hand');
l_hand_pts = r.getBody(l_hand).getContactPoints();

%% Setup constraints
com_pos = [0;0;nan];  % com should be at x=0,y=0,z=don't care

% desired location for all rfoot contact points is:
%  on the ground in z, don't care in x,y
num_pts = size(r_foot_pts,2);
r_foot_pos = [nan(2,num_pts);zeros(1,num_pts)];

% same for lfoot
num_pts = size(l_foot_pts,2);
l_foot_pos = [nan(2,num_pts);zeros(1,num_pts)];


%% Call inverse kinematics
[q,info] = inverseKin(r,q0, ...
  com,com_pos,...
  r_foot,r_foot_pts,r_foot_pos,...
  l_foot,l_foot_pts,l_foot_pos,...
  options);

% Visualizer the result
v.draw(0,[q;0*q]);
