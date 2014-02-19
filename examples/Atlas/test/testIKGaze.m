function testIKGaze

addpath(fullfile(pwd,'..'));
options.floating = true;
options.dt = 0.001;
r = RigidBodyManipulator('../urdf/atlas_minimal_contact.urdf',options);
v = r.constructVisualizer();

cost = Point(r.getStateFrame,1);
cost.base_x = 0;
cost.base_y = 0;
cost.base_z = 0;
cost.base_roll = 1000;
cost.base_pitch = 1000;
cost.base_yaw = 1000;
cost.back_bky = 100;
cost.back_bkx = 100;
options = struct();
cost = double(cost);
options.Q = diag(cost(1:r.getNumDOF)) + 10*eye(34);

options.use_mex = false;

% set initial state to fixed point
load('../data/atlas_fp.mat');

q0 = xstar(1:r.getNumDOF);

% q = inverseKin(r,q0,options);
% qmex = inverseKin(r,q0,mexoptions);
% valuecheck(qmex,q,1e-5);
% v.draw(0,[q;0*q]); drawnow;

for i=1:1,
%DEFAULT CHECK

r_foot = r.findLinkInd('r_foot');
head = r.findLinkInd('head');
head_con.type = 'gaze';
head_con.gaze_axis = [1;0;0];
head_con.gaze_orientation = rpy2quat([randn/10;randn/10;randn/2]*0);
head_con.gaze_orientation = rpy2quat([0;0;pi/2]);
[q_seed,q_nom,constraint,ikoptions] = inverseKinWrapup(r,q0,0,[0;0;.95],r_foot,[0;0;0],[0;-.1;.2],head,[],head_con,options);
[q,info] = inverseKin(r,q_seed,q_nom,constraint{:},ikoptions);
kinsol = r.doKinematics(q);
xhead = r.forwardKin(kinsol,head,zeros(3,1),2);
Rhead = quat2rotmat(quatDiff(head_con.gaze_orientation,xhead(4:7)));

head_err = head_con.gaze_axis'*Rhead*head_con.gaze_axis;
valuecheck(head_err,1,1e-5);
v.draw(1,[q;0*q]); drawnow;

% CHECK GAZE AXIS
head_con5.type = 'gaze';
head_con5.gaze_axis = [1;0;0];
head_con5.gaze_dir = [randn;randn;0];
[q_seed,q_nom,constraint,ikoptions] = inverseKinWrapup(r,q0,0,[0;0;.95],r_foot,[0;0;0],[0;-.1;.2],head,[],head_con5,options);
[q,info] = inverseKin(r,q_seed,q_nom,constraint{:},ikoptions);
v.draw(1,[q;0*q]); drawnow;


% CHECK GAZING OFF IN SPACE
head_con4.type = 'gaze';
head_con4.gaze_axis = [1;0;0];
head_con4.gaze_target = [2*randn;2*randn;0]; %look at the ground
[q_seed,q_nom,constraint,ikoptions] = inverseKinWrapup(r,q0,0,[0;0;.95],r_foot,[0;0;0],[0;-.1;.2],head,[],head_con4,options);
[q,info] = inverseKin(r,q_seed,q_nom,constraint{:},ikoptions);
v.draw(1,[q;0*q]); drawnow;

kinsol = r.doKinematics(q);
xhead = r.forwardKin(kinsol,head,zeros(3,1),2);
% Rhead = quat2rotmat(quatDiff(head_con4.gaze_orientation,xhead(4:7)));

% CHECK WITH CONE CONSTRAINT
head_con2 = head_con;
head_con2.gaze_conethreshold = .2;
[q_seed,q_nom,constraint,ikoptions] = inverseKinWrapup(r,q0,0,[0;0;.95],r_foot,[0;0;0],[0;-.1;.2],head,[],head_con2,options);
[q,info] = inverseKin(r,q_seed,q_nom,constraint{:},ikoptions);

kinsol = r.doKinematics(q);
xhead = r.forwardKin(kinsol,head,zeros(3,1),2);
Rhead = quat2rotmat(quatDiff(head_con2.gaze_orientation,xhead(4:7)));

head_err = head_con2.gaze_axis'*Rhead*head_con2.gaze_axis;
rangecheck(head_err,cos(head_con2.gaze_conethreshold) - 1e5,1 + 1e5);
v.draw(1,[q;0*q]); drawnow;

% CHECK WITH THRESHOLD
head_con3 = head_con;
head_con3.gaze_orientation = rpy2quat([randn/10;randn/10;randn]);
head_con3.gaze_threshold = .5;
[q_seed,q_nom,constraint,ikoptions] = inverseKinWrapup(r,q0,0,[0;0;.95],r_foot,[0;0;0],[0;-.1;.2],head,[],head_con3,options);
[q,info] = inverseKin(r,q_seed,q_nom,constraint{:},ikoptions);

kinsol = r.doKinematics(q);
xhead = r.forwardKin(kinsol,head,zeros(3,1),2);
qdiff = quatDiff(head_con3.gaze_orientation,xhead(4:7));
Rhead = quat2rotmat(qdiff);

head_err = head_con3.gaze_axis'*Rhead*head_con3.gaze_axis;
valuecheck(head_err,1,1e-4);

rangecheck(qdiff(1),cos(head_con3.gaze_threshold/2) - 1e5, 1 + 1e5);
v.draw(1,[q;0*q]); drawnow;

end
