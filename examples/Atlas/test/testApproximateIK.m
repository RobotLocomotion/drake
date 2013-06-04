function testApproximateIK

addpath('..');
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

% set initial state to fixed point
load('../data/atlas_fp.mat');

q0 = xstar(1:r.getNumDOF);

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
q = approximateIK(r,q_rot0,0,[0;0;nan],l_foot,l_foot_pts,l_foot_pos,r_foot,r_foot_pts,r_foot_pos,options);
toc
v.draw(1,[q;0*q]); drawnow;
kinsol = doKinematics(r, q);
l_foot_sol_q = forwardKin(r, kinsol, l_foot, l_foot_pts,2);
% low tolerances here
if ~valuecheck(l_foot_pos_q, l_foot_sol_q,1e-2) && ...
    ~valuecheck(-l_foot_pos_q, l_foot_sol_q,1e-2)
  error('testApproximateIK: values dont match');
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
q = approximateIK(r,q_rot0,0,[0;0;nan],l_foot,l_foot_pts,l_foot_pos,r_foot,r_foot_pts,r_foot_pos,options);
toc
v.draw(1,[q;0*q]); drawnow;
kinsol = doKinematics(r, q);
l_foot_sol_q = forwardKin(r, kinsol, l_foot, l_foot_pts,2);
% low tolerances here
if ~valuecheck(l_foot_pos_q, l_foot_sol_q,1e-2) && ...
    ~valuecheck(-l_foot_pos_q, l_foot_sol_q,1e-2)
  error('testApproximateIK: values dont match');
end
%%%% End wraparound test %%%%

kinsol = doKinematics(r,q0);
com = getCOM(r,kinsol);
for i=1:10
  % test that small COM shifts give approximately the same answer
  comdes = com+0.01*randn(3,1);
  q = approximateIK(r,q0,0,comdes,options);
  qik = inverseKin(r,q0,0,comdes,options);
  v.draw(1,[q;0*q]); drawnow;
  v.draw(1,[qik;0*q]); drawnow;
  valuecheck(qik,q,1e-2);
end

kinsol = doKinematics(r,q0);
l_foot_pos = forwardKin(r, kinsol, l_foot, l_foot_pts, 1);
r_foot_pos = forwardKin(r, kinsol, r_foot, r_foot_pts, 1);
q=q0;
for i=1:10
  l_foot_pos = l_foot_pos+0.005*randn(6,1);
  r_foot_pos = r_foot_pos+0.005*randn(6,1);

  q = approximateIK(r,q,l_foot,l_foot_pts,l_foot_pos,r_foot,r_foot_pts,r_foot_pos,options);
  qik = inverseKin(r,q,l_foot,l_foot_pts,l_foot_pos,r_foot,r_foot_pts,r_foot_pos,options);
  v.draw(1,[q;0*q]); drawnow;
  v.draw(1,[qik;0*q]); drawnow;
  valuecheck(qik,q,0.1); % pretty loose
end


% test inequalities
kinsol = doKinematics(r,q0);
l_foot_pos = forwardKin(r, kinsol, l_foot, l_foot_pts, 1);
r_foot_pos = forwardKin(r, kinsol, r_foot, r_foot_pts, 1);
q=q0;
l_foot_des.min = l_foot_pos;
l_foot_des.max = l_foot_pos;
r_foot_des.min = r_foot_pos;
r_foot_des.max = r_foot_pos;
for i=1:20 % NOTE: increase this number to reveal inequality constraint angle wrap bug when using rpy
  l_foot_des.min = l_foot_des.min+0.1*[.1 .1 .05 0 0 1]';
  l_foot_des.max = l_foot_des.max+0.2*[.1 .1 .05 0 0 1]';
  r_foot_des.min = r_foot_des.min+0.1*[.1 .1 .05 0 0 1]';
  r_foot_des.max = r_foot_des.max+0.2*[.1 .1 .05 0 0 1]';
  q = approximateIK(r,q,l_foot,l_foot_pts,l_foot_des,r_foot,r_foot_pts,r_foot_des,options);
  v.draw(1,[q;0*q]); drawnow;
end
