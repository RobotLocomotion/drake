function testIK
 
addpath(fullfile(pwd,'..'));
options.floating = true;
options.dt = 0.001;
r = Atlas('../urdf/atlas_minimal_contact.urdf',options);
v = r.constructVisualizer();
 
cost = Point(r.getStateFrame,1);
cost.base_x = 0;
cost.base_y = 0;
cost.base_z = 0;
cost.base_roll = 1000;
cost.base_pitch = 1000;
cost.base_yaw = 0;
cost.back_mby = 100;
cost.back_ubx = 100;
options = struct();
cost = double(cost);
options.Q = diag(cost(1:r.getNumDOF));
 
mexoptions = options;
approxmexoptions = options;
approxmexoptions.use_mex = true;
if (exist('inverseKinmex')==3)
  mexoptions.use_mex = true;
else
  warning('mex inverseKinematics is disabled... so I can''t test that'); 
end
options.use_mex = false;
 
% set initial state to fixed point
d = load('../data/atlas_fp.mat');
 
q0 = d.xstar(1:r.getNumDOF);
 
% q = inverseKin(r,q0,options);
% qmex = inverseKin(r,q0,mexoptions);
% valuecheck(qmex,q,1e-5);
% v.draw(0,[q;0*q]); drawnow;

 
%%%% Try to test wraparound bug for euler angles %%%%
% First, a case which should not exhibit the wraparound bug (since we're
% operating around pi/2 instead of +/- pi):
v.draw(1,[q0;0*q0]); drawnow;
q_rot0 = q0;
q_rot1 = q0;
q_rot0(6) = pi/2 - 0.01;
q_rot1(6) = pi/2 + 0.01;
l_foot = r.findLink('l_foot');
kinsol = doKinematics(r, q_rot1);
l_foot_pts = [0;0;0];
l_foot_pos = forwardKin(r, kinsol, l_foot, l_foot_pts, 1);
l_foot_pos_q = forwardKin(r, kinsol, l_foot, l_foot_pts, 2);
r_foot = r.findLink('r_foot');
kinsol = doKinematics(r, q_rot0);
r_foot_pts = [0;0;0];
r_foot_pos = forwardKin(r, kinsol, r_foot, r_foot_pts, 1);
tic
[q,info] = inverseKin(r,q_rot0,0,[0;0;nan],l_foot,l_foot_pts,l_foot_pos,r_foot,r_foot_pts,r_foot_pos,options);
toc
v.draw(1,[q;0*q]); drawnow;
kinsol = doKinematics(r, q);
l_foot_sol_q = forwardKin(r, kinsol, l_foot, l_foot_pts,2);
% low tolerances here
if ~valuecheck(l_foot_pos_q, l_foot_sol_q,1e-2) && ...
    ~valuecheck(-l_foot_pos_q, l_foot_sol_q,1e-2)
  error('testIK: values dont match');
end
 
% Next, a case which exhibits the wraparound bug:
v.draw(1,[q0;0*q0]); drawnow;
q_rot0 = q0;
q_rot1 = q0;
q_rot0(6) = pi - 0.01;
q_rot1(6) = -pi + 0.01;
l_foot = r.findLink('l_foot');
kinsol = doKinematics(r, q_rot1);
l_foot_pts = [0;0;0];
l_foot_pos = forwardKin(r, kinsol, l_foot, l_foot_pts, 1);
l_foot_pos_q = forwardKin(r, kinsol, l_foot, l_foot_pts, 2);
r_foot = r.findLink('r_foot');
kinsol = doKinematics(r, q_rot0);
r_foot_pts = [0;0;0];
r_foot_pos = forwardKin(r, kinsol, r_foot, r_foot_pts, 1);
tic
[q,info] = inverseKin(r,q_rot0,0,[0;0;nan],l_foot,l_foot_pts,l_foot_pos,r_foot,r_foot_pts,r_foot_pos,options);
toc
v.draw(1,[q;0*q]); drawnow;
kinsol = doKinematics(r, q);
l_foot_sol_q = forwardKin(r, kinsol, l_foot, l_foot_pts,2);
% low tolerances here
if ~valuecheck(l_foot_pos_q, l_foot_sol_q,1e-2) && ...
    ~valuecheck(-l_foot_pos_q, l_foot_sol_q,1e-2)
  error('testIK: values dont match');
end
%%%% End wraparound test %%%%

%test trivial approx IK (For 1/2 bug)


alt_options.Q = eye(34);
alt_options.use_mex = 0;
q = r.approximateIK(q0,1,zeros(3,1),struct('min',-inf(3,1),'max',inf(3,1)),alt_options);
valuecheck(q,q0)
alt_options.use_mex = 1;
q = r.approximateIK(q0,1,zeros(3,1),struct('min',-inf(3,1),'max',inf(3,1)),alt_options);
valuecheck(q,q0,1e-3);
 
function testmex(varargin)
  display('testing mex')
  tic
  q = approximateIK(r,q0,varargin{:},options);
  toc
  tic
  qmex = approximateIK(r,q0,varargin{:},approxmexoptions);
  toc
  valuecheck(qmex,q,1e-1);
  v.draw(1,[q;0*q]); drawnow;
end
 
testmex(0,[0;0;2]);
 
r_foot = r.findLink('r_foot');
testmex(0,[0;0;.95],r_foot,[0;0;0],[0;-.1;.2]);
testmex(0,[0;0;.95],r_foot,[0;0;0],[0;-.1;.2;0;0;0]);
testmex(0,[0;0;nan],r_foot,[0;0;0],[0;-.1;.2;0;0;0]);



q = inverseKin(r,q0,0,[0;0;2],options);
qmex = inverseKin(r,q0,0,[0;0;2],mexoptions);
valuecheck(qmex,q,1e-5);
v.draw(1,[q;0*q]); drawnow;

r_foot = r.findLink('r_foot');
q = inverseKin(r,q0,0,[0;0;.95],r_foot,[0;0;0],[0;-.1;.2],options);
tic
qmex = inverseKin(r,q0,0,[0;0;.95],r_foot,[0;0;0],[0;-.1;.2],mexoptions);
toc
valuecheck(qmex,q,1e-5);
v.draw(1,[q;0*q]); drawnow;

q = inverseKin(r,q0,0,[0;0;.95],r_foot,[0;0;0],[0;-.1;.2;0;0;0],options);
tic
qmex = inverseKin(r,q0,0,[0;0;.95],r_foot,[0;0;0],[0;-.1;.2;0;0;0],mexoptions);
toc
valuecheck(qmex,q,1e-5);
v.draw(1,[q;0*q]); drawnow;

q = inverseKin(r,q0,0,[0;0;nan],r_foot,[0;0;0],[0;-.1;.2;0;0;0],options);
tic
qmex = inverseKin(r,q0,0,[0;0;nan],r_foot,[0;0;0],[0;-.1;.2;0;0;0],mexoptions);
toc
valuecheck(qmex,q,1e-5);
v.draw(1,[q;0*q]); drawnow;

r_foot_pts = r_foot.getContactPoints();
kinsol = doKinematics(r,q+0.1*randn(size(q)));
r_foot_pos = forwardKin(r,kinsol,r_foot,r_foot_pts,2);
tic
[q,info] = inverseKin(r,q0,0,[0;0;nan],r_foot,r_foot_pts,r_foot_pos,options);
toc

l_foot = r.findLink('l_foot');
l_foot_pts = l_foot.getContactPoints();
kinsol = doKinematics(r,q+0.01*randn(size(q)));
l_foot_pos = forwardKin(r,kinsol,l_foot,l_foot_pts,1);
tic
[q,info] = inverseKin(r,q0,0,[0;0;nan],r_foot,r_foot_pts,r_foot_pos,l_foot,l_foot_pts,l_foot_pos,options);
toc

pelvis = r.findLink('pelvis');
kinsol = doKinematics(r,q+0.01*randn(size(q)));
pelvis_pos = forwardKin(r,kinsol,pelvis,[0;0;0],1);
tic
[q,info] = inverseKin(r,q0,0,[0;0;nan],r_foot,r_foot_pts,r_foot_pos,l_foot,l_foot_pts,l_foot_pos,pelvis,[0;0;0],pelvis_pos,options);
toc

l_hand = r.findLink('l_hand');
kinsol = doKinematics(r,q+0.01*randn(size(q)));
l_hand_pts = [[0;0;0] [1;0.1;0.2]];
l_hand_pos = forwardKin(r,kinsol,l_hand,l_hand_pts,2);
tic
[q,info] = inverseKin(r,q0,0,[0;0;nan],r_foot,r_foot_pts,r_foot_pos,l_foot,l_foot_pts,l_foot_pos,pelvis,[0;0;0],pelvis_pos,l_hand,l_hand_pts,l_hand_pos,options);
toc
% keyboard

% todo: uncomment this when my atlas model has collision groups defined
% q = inverseKin(r,q0,0,[0;0;nan],r_foot,1,[0;-.1;0;0;0;0],options);
% tic
% qmex = inverseKin(r,q0,0,[0;0;nan],r_foot,1,[0;-.1;0;0;0;0],mexoptions);
% toc
% valuecheck(qmex,q,1e-5);
% v.draw(1,[q;0*q]); drawnow;
 
end