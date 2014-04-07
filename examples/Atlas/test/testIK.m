function testIK
 
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
cost.base_yaw = 0;
cost.back_bky = 100;
cost.back_bkx = 100;
options = struct();
cost = double(cost);
options.Q = diag(cost(1:r.getNumDOF));
 
% set initial state to fixed point
d = load('../data/atlas_fp.mat');
nq = r.getNumDOF();
q0 = d.xstar(1:nq);

r_foot = r.findLinkInd('r_foot');
l_foot = r.findLinkInd('l_foot');
pelvis = r.findLinkInd('pelvis');
l_hand = r.findLinkInd('l_hand');
r_hand = r.findLinkInd('r_hand');

d = load('../data/atlas_fp.mat');
 
q0 = d.xstar(1:r.getNumDOF);

l_hand_pts = [[0;0;0] [1;0.1;0.2]];
r_hand_pts = [[0;0;0] [1;0.1;0.2]];

tic
[q_seed,q_nom,constraint,ikoptions] = inverseKinWrapup(r,q0,0,[0;0;2],options);
toc
tic
[q,info] = inverseKin(r,q_seed,q_nom,constraint{:},ikoptions);
toc
kinsol = doKinematics(r,q);
com = getCOM(r,kinsol);
valuecheck(com,[0;0;2],1e-8);
display('COM constraint success');
 
%%%% Try to test wraparound bug for euler angles %%%%
% First, a case which should not exhibit the wraparound bug (since we're
% operating around pi/2 instead of +/- pi):
v.draw(1,[q0;0*q0]); drawnow;
q_rot0 = q0;
q_rot1 = q0;
q_rot0(6) = pi/2 - 0.01;
q_rot1(6) = pi/2 + 0.01;
l_foot = r.findLinkInd('l_foot');
kinsol = doKinematics(r, q_rot1);
l_foot_pts = [0;0;0];
l_foot_pos = forwardKin(r, kinsol, l_foot, l_foot_pts, 1);
l_foot_pos_q = forwardKin(r, kinsol, l_foot, l_foot_pts, 2);
r_foot = r.findLinkInd('r_foot');
kinsol = doKinematics(r, q_rot0);
r_foot_pts = [0;0;0];
r_foot_pos = forwardKin(r, kinsol, r_foot, r_foot_pts, 1);
tic
[q_seed,q_nom,constraint,ikoptions] = inverseKinWrapup(r,q_rot0,0,[0;0;nan],l_foot,l_foot_pts,l_foot_pos,r_foot,r_foot_pts,r_foot_pos,options);
[q,info] = inverseKin(r,q_seed,q_nom,constraint{:},ikoptions);
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
l_foot = r.findLinkInd('l_foot');
kinsol = doKinematics(r, q_rot1);
l_foot_pts = [0;0;0];
l_foot_pos = forwardKin(r, kinsol, l_foot, l_foot_pts, 1);
l_foot_pos_q = forwardKin(r, kinsol, l_foot, l_foot_pts, 2);
r_foot = r.findLinkInd('r_foot');
kinsol = doKinematics(r, q_rot0);
r_foot_pts = [0;0;0];
r_foot_pos = forwardKin(r, kinsol, r_foot, r_foot_pts, 1);
tic
[q_seed,q_nom,constraint,ikoptions] = inverseKinWrapup(r,q_rot0,0,[0;0;nan],l_foot,l_foot_pts,l_foot_pos,r_foot,r_foot_pts,r_foot_pos,options);
[q,info] = inverseKin(r,q_seed,q_nom,constraint{:},ikoptions);
toc
v.draw(1,[q;0*q]); drawnow;
kinsol = doKinematics(r, q);
l_foot_sol_q = forwardKin(r, kinsol, l_foot, l_foot_pts,2);
% low tolerances here
if ~valuecheck(l_foot_pos_q, l_foot_sol_q,1e-2) && ...
    ~valuecheck(-l_foot_pos_q, l_foot_sol_q,1e-2)
  error('testIK: values dont match');
end

display('Angle wrapup passed');
%%%% End wraparound test %%%%

%test trivial approx IK (For 1/2 bug)


% alt_options.Q = eye(34);
% alt_options.use_mex = 0;
% q = r.approximateIK(q0,1,zeros(3,1),struct('min',-inf(3,1),'max',inf(3,1)),alt_options);
% valuecheck(q,q0)
% alt_options.use_mex = 1;
% q = r.approximateIK(q0,1,zeros(3,1),struct('min',-inf(3,1),'max',inf(3,1)),alt_options);
% valuecheck(q,q0,1e-3);
% 
% display('Trivial approximate IK passed');
% function testmex(varargin)
%   display('testing mex')
%   tic
%   q = approximateIK(r,q0,varargin{:},options);
%   toc
%   tic
%   qmex = approximateIK(r,q0,varargin{:},approxmexoptions);
%   toc
%   valuecheck(qmex,q,1e-1);
%   v.draw(1,[q;0*q]); drawnow;
% end
%  
% testmex(0,[0;0;2]);
%  
% r_foot = r.findLinkInd('r_foot');
% testmex(0,[0;0;.95],r_foot,[0;0;0],[0;-.1;.2]);
% testmex(0,[0;0;.95],r_foot,[0;0;0],[0;-.1;.2;0;0;0]);
% testmex(0,[0;0;nan],r_foot,[0;0;0],[0;-.1;.2;0;0;0]);
% display('Approximate IK mex passed');


q = q0;
r_foot = r.findLinkInd('r_foot');
[q_seed,q_nom,constraint,ikoptions] = inverseKinWrapup(r,q0,0,[0;0;.95],r_foot,[0;0;0],[0;-.1;.2],options);
[q,info] = inverseKin(r,q_seed,q_nom,constraint{:},ikoptions);
kinsol = doKinematics(r,q);
com = getCOM(r,kinsol);
valuecheck(com,[0;0;0.95],1e-6);
r_foot_pos = forwardKin(r,kinsol,r_foot,[0;0;0],0);
valuecheck(r_foot_pos,[0;-0.1;0.2],1e-5);
display('IK for position only passed');

[q_seed,q_nom,constraint,ikoptions] = inverseKinWrapup(r,q0,0,[0;0;.95],r_foot,[0;0;0],[0;-.1;.2;0;0;0],options);
[q,info] = inverseKin(r,q_seed,q_nom,constraint{:},ikoptions);
kinsol = doKinematics(r,q);
com = getCOM(r,kinsol);
valuecheck(com,[0;0;0.95],1e-6);
r_foot_pos = forwardKin(r,kinsol,r_foot,[0;0;0],1);
valuecheck(r_foot_pos,[0;-0.1;0.2;0;0;0],1e-5);
display('IK for rpy passed');

v.draw(1,[q;0*q]); drawnow;

[q_seed,q_nom,constraint,ikoptions] = inverseKinWrapup(r,q0,0,[0;0;nan],r_foot,[0;0;0],[0;-.1;.2;0;0;0],options);
[q,info] = inverseKin(r,q_seed,q_nom,constraint{:},ikoptions);
kinsol = doKinematics(r,q);
com = getCOM(r,kinsol);
valuecheck(com(1:2),[0;0],1e-6);
r_foot_pos = forwardKin(r,kinsol,r_foot,[0;0;0],1);
valuecheck(r_foot_pos,[0;-0.1;0.2;0;0;0],1e-5);
v.draw(1,[q;0*q]); drawnow;
display('IK for nan passed');

kinsol = doKinematics(r,q);
r_foot_pos = forwardKin(r,kinsol,r_foot,r_foot_pts,2);
r_foot_pos(3,:) = zeros(1,size(r_foot_pts,2));
r_foot_pos(4:7,1) = r_foot_pos(4:7,1)+0.01*randn(4,1);
r_foot_pos(4:7,1) = r_foot_pos(4:7,1)/norm(r_foot_pos(4:7,1));
tic
[q_seed,q_nom,constraint,ikoptions] = inverseKinWrapup(r,q0,0,[0;0;nan],r_foot,r_foot_pts,r_foot_pos,options);
[q,info] = inverseKin(r,q_seed,q_nom,constraint{:},ikoptions);
toc
valuecheck(info,1);
kinsol = doKinematics(r,q);
com = getCOM(r,kinsol);
valuecheck(com(1:2),[0;0],1e-6);
r_foot_pos_sol = forwardKin(r,kinsol,r_foot,[0;0;0],2);
valuecheck(r_foot_pos_sol,r_foot_pos,1e-2);
display('IK for quaternion passed');

kinsol = doKinematics(r,q);
l_foot_pos = [nan(2,size(l_foot_pts,2));zeros(1,size(l_foot_pts,2))];
tic
[q_seed,q_nom,constraint,ikoptions] = inverseKinWrapup(r,q0,0,[0;0;nan],r_foot,r_foot_pts,r_foot_pos,l_foot,l_foot_pts,l_foot_pos,options);
% ikoptions = ikoptions.setMex(false);
[q,info] = inverseKin(r,q_seed,q_nom,constraint{:},ikoptions);
toc
kinsol = doKinematics(r,q);
com = getCOM(r,kinsol);
valuecheck(com(1:2),[0;0],1e-6);
r_foot_pos_sol = forwardKin(r,kinsol,r_foot,[0;0;0],2);
valuecheck(r_foot_pos_sol,r_foot_pos,1e-2);
l_foot_pos_sol = forwardKin(r,kinsol,l_foot,l_foot_pts,0);
valuecheck(l_foot_pos_sol(3,:),l_foot_pos(3,:),1e-5);
display('IK for both feet passed');

kinsol = doKinematics(r,q);
pelvis_pos = forwardKin(r,kinsol,pelvis,[0;0;0],1);
tic
[q_seed,q_nom,constraint,ikoptions] = inverseKinWrapup(r,q0,0,[0;0;nan],r_foot,r_foot_pts,r_foot_pos,l_foot,l_foot_pts,l_foot_pos,pelvis,[0;0;0],pelvis_pos,options);
[q,info] = inverseKin(r,q_seed,q_nom,constraint{:},ikoptions);
toc
valuecheck(info,1);
kinsol = doKinematics(r,q);
com = getCOM(r,kinsol);
valuecheck(com(1:2),[0;0],1e-6);
r_foot_pos_sol = forwardKin(r,kinsol,r_foot,[0;0;0],2);
valuecheck(r_foot_pos_sol,r_foot_pos,1e-2);
l_foot_pos_sol = forwardKin(r,kinsol,l_foot,l_foot_pts,0);
valuecheck(l_foot_pos_sol(3,:),l_foot_pos(3,:),1e-5);
pelvis_pos_sol = forwardKin(r,kinsol,pelvis,[0;0;0],1);
valuecheck(pelvis_pos_sol,pelvis_pos,1e-5);
display('IK for feet and pelvis passed');

kinsol = doKinematics(r,q);
l_hand_pos = forwardKin(r,kinsol,l_hand,l_hand_pts,2);
tic
[q_seed,q_nom,constraint,ikoptions] = inverseKinWrapup(r,q0,0,[0;0;nan],r_foot,r_foot_pts,r_foot_pos,l_foot,l_foot_pts,l_foot_pos,pelvis,[0;0;0],pelvis_pos,l_hand,l_hand_pts,l_hand_pos,options);
[q,info] = inverseKin(r,q_seed,q_nom,constraint{:},ikoptions);
valuecheck(info,1);
toc
kinsol = doKinematics(r,q);
com = getCOM(r,kinsol);
valuecheck(com(1:2),[0;0],1e-6);
r_foot_pos_sol = forwardKin(r,kinsol,r_foot,[0;0;0],2);
valuecheck(r_foot_pos_sol,r_foot_pos,1e-2);
l_foot_pos_sol = forwardKin(r,kinsol,l_foot,l_foot_pts,0);
valuecheck(l_foot_pos_sol(3,:),l_foot_pos(3,:),1e-5);
pelvis_pos_sol = forwardKin(r,kinsol,pelvis,[0;0;0],1);
valuecheck(pelvis_pos_sol,pelvis_pos,1e-5);
l_hand_pos_sol = forwardKin(r,kinsol,l_hand,l_hand_pts,2);
valuecheck(l_hand_pos_sol(1:3,:),l_hand_pos(1:3,:),1e-5);
valuecheck(l_hand_pos_sol(4:7,1)'*l_hand_pos(4:7,1),1,1e-5);
display('IK for feet, hands and pelvis passed');


% kinsol = doKinematics(r,q+0.01*randn(size(q)));

display('If you see warning that quternion is nan or zero for the following to test cases, don"t panic, it is by intention');
r_hand_pos = nan(7,2);
tic
[q_seed,q_nom,constraint,ikoptions] = inverseKinWrapup(r,q0,0,[0;0;nan],...
    r_foot,r_foot_pts,r_foot_pos,...
    l_foot,l_foot_pts,l_foot_pos,...
    pelvis,[0;0;0],pelvis_pos,...
    l_hand,l_hand_pts,l_hand_pos,...
    r_hand,r_hand_pts,r_hand_pos,options);
[q,info] = inverseKin(r,q_seed,q_nom,constraint{:},ikoptions);
toc
display('IK for nan passed');
tic
options.quasiStaticFlag = true;

nLPts = length(r.getBody(l_foot).getContactShapes);
lfoot_contact_pts = zeros(3,nLPts);
for i=1:nLPts,
  lfoot_contact_pts(:,i) = r.getBody(l_foot).getContactShapes{i}.getPoints;
end
nRPts = length(r.getBody(r_foot).getContactShapes);
rfoot_contact_pts = zeros(3,nRPts);
for i=1:nRPts,
  rfoot_contact_pts(:,i) = r.getBody(r_foot).getContactShapes{i}.getPoints;
end

kinsol = doKinematics(r,q0);
rfoot_contact_pos = forwardKin(r,kinsol,r_foot,rfoot_contact_pts,0);
lfoot_contact_pos = forwardKin(r,kinsol,r_foot,lfoot_contact_pts,0);
rfoot_contact_pos(3,:) = zeros(1,size(rfoot_contact_pts,2));
lfoot_contact_pos(3,:) = zeros(1,size(lfoot_contact_pts,2));
tic
[q_seed,q_nom,constraint,ikoptions] = inverseKinWrapup(r,q0,0,[0;nan;nan],...
    r_foot,rfoot_contact_pts,rfoot_contact_pos,...
    l_foot,lfoot_contact_pts,lfoot_contact_pos,...
    pelvis,[0;0;0],pelvis_pos,...
    l_hand,l_hand_pts,l_hand_pos,...
    r_hand,r_hand_pts,r_hand_pos,options);
toc
tic
[q,info] = inverseKin(r,q_seed,q_nom,constraint{:},ikoptions);
toc
valuecheck(info,1);
kinsol = doKinematics(r,q);
com = getCOM(r,kinsol);
valuecheck(com(1),0,1e-5);
rfoot_pos_sol = forwardKin(r,kinsol,r_foot,rfoot_contact_pts,0);
valuecheck(rfoot_pos_sol,rfoot_contact_pos,1e-5);
lfoot_pos_sol = forwardKin(r,kinsol,l_foot,lfoot_contact_pts,0);
valuecheck(lfoot_pos_sol,lfoot_contact_pos,1e-5);
pelvis_pos_sol = forwardKin(r,kinsol,pelvis,[0;0;0],1);
valuecheck(pelvis_pos_sol,pelvis_pos,1e-5);
l_hand_pos_sol = forwardKin(r,kinsol,l_hand,l_hand_pts,2);
valuecheck(l_hand_pos_sol(1:3,:),l_hand_pos(1:3,:),1e-5);
valuecheck(l_hand_pos_sol(4:7,1)'*l_hand_pos(4:7,1),1,1e-5);


for i = 1:length(constraint)
  if(isa(constraint{i},'QuasiStaticConstraint'))
    shrinkFactor = constraint{i}.shrinkFactor+1e-4; % The tolerance used by quadprog is different from that in SNOPT
  end
end
center_pos = mean([lfoot_pos_sol rfoot_pos_sol],2);
shrink_vertices = [lfoot_pos_sol rfoot_pos_sol]*shrinkFactor+repmat(center_pos*(1-shrinkFactor),1,size(lfoot_contact_pts,2)+size(rfoot_contact_pts,2));
num_vertices = size(shrink_vertices,2);
  quadoptions = optimset('Algorithm','interior-point-convex');
[weights,~,exit_flag] = quadprog(eye(num_vertices),rand(num_vertices,1),...
  [],[],...
  [shrink_vertices(1:2,:);ones(1,num_vertices)],[com(1:2);1],...
  zeros(num_vertices,1),ones(num_vertices,1),...
  1/num_vertices*ones(num_vertices,1),quadoptions);
valuecheck(exit_flag,1);
toc
display('IK for quasi static passed');

 
end
