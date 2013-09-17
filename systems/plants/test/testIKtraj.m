function testIKtraj()
options.floating = true;
options.dt = 0.001;
r = RigidBodyManipulator();
r = r.addRobotFromURDF('../../../examples/Atlas/urdf/atlas_minimal_contact.urdf',[],[],options);
nom_data = load('../../../examples/Atlas/data/atlas_fp.mat');
v = r.constructVisualizer();

r_foot = r.findLinkInd('r_foot');
l_foot = r.findLinkInd('l_foot');
r_hand = r.findLinkInd('r_hand');
l_hand = r.findLinkInd('l_hand');

r_foot_contact_pts = getContactPoints(getBody(r,r_foot));
l_foot_contact_pts = getContactPoints(getBody(r,l_foot));
r_hand_contact_pts = mean(getContactPoints(getBody(r,r_hand)),2);
l_hand_contact_pts = mean(getContactPoints(getBody(r,l_hand)),2);

nq = r.getNumDOF();
q0 = nom_data.xstar(1:nq);
qdot0 = zeros(nq,1);
kinsol0 = doKinematics(r,q0,false,false);
r_foot_contact_pos = forwardKin(r,kinsol0,r_foot,r_foot_contact_pts,0);
r_foot_contact_pos(3,:) = 0;
l_foot_contact_pos = forwardKin(r,kinsol0,l_foot,l_foot_contact_pts,0);
l_foot_contact_pos(3,:) = 0;
r_hand_contact_pos = forwardKin(r,kinsol0,r_hand,r_hand_contact_pts,0);
l_hand_contact_pos = forwardKin(r,kinsol0,l_hand,l_hand_contact_pts,0);
com_pos0 = getCOM(r,kinsol0);
com_height = com_pos0(3);
tspan = [0,1];
kc1 = WorldPositionConstraint(r,r_foot,r_foot_contact_pts,r_foot_contact_pos,r_foot_contact_pos,tspan);
kc2 = WorldPositionConstraint(r,l_foot,l_foot_contact_pts,l_foot_contact_pos,l_foot_contact_pos,tspan);
kc3 = WorldPositionConstraint(r,r_hand,r_hand_contact_pts,r_hand_contact_pos+[0.1;0.05;1],r_hand_contact_pos+[0.1;0.05;1],[tspan(end) tspan(end)]);
kc4 = WorldPositionConstraint(r,l_hand,l_hand_contact_pts,l_hand_contact_pos,l_hand_contact_pos,[tspan(end) tspan(end)]);
kc5 = WorldCoMConstraint(r,[-inf;-inf;com_height],[inf;inf;com_height+0.5],tspan);
ikoptions = IKoptions(r);
cost = Point(r.getStateFrame,1);
cost.base_x = 100;
cost.base_y = 100;
cost.base_z = 100;
cost.base_roll = 100;
cost.base_pitch = 100;
cost.base_yaw = 100;
cost = double(cost);
Q = diag(cost(1:nq));
ikoptions = ikoptions.setQ(Q);
ikoptions = ikoptions.setQa(10*Q);
ikoptions = ikoptions.setMajorIterationsLimit(2000);
ikoptions = ikoptions.setIterationsLimit(100000);
ikoptions = ikoptions.setSuperbasicsLimit(1000);
ikmexoptions = ikoptions;
ikmexoptions = ikmexoptions.setMex(true);
ikmexoptions = ikmexoptions.setDebug(true);
nT = 5;
% t = [tspan(1) tspan(1)+0.2*(tspan(end)-tspan(1)) tspan(1)+0.7*(tspan(end)-tspan(1)) tspan(end)];
t = linspace(tspan(1),tspan(end),nT);
q_nom = repmat(q0,1,nT-1);
q_seed = repmat(q0,1,nT-1);

display('Check IK traj');
xtraj = test_IKtraj_userfun(r,q0,qdot0,t,q_seed,q_nom,kc1,kc2,kc3,kc4,kc5,ikoptions);
v = r.constructVisualizer();
v.playback(xtraj,struct('slider',true));
display('Check IK traj with quasi static constraint');
qsc = QuasiStaticConstraint(r);
qsc = qsc.addContact(r_foot,r_foot_contact_pts);
qsc = qsc.addContact(l_foot,l_foot_contact_pts);
qsc = qsc.setActive(true);
qsc = qsc.setShrinkFactor(0.8);
xtraj = test_IKtraj_userfun(r,q0,qdot0,t,q_seed,q_nom,kc1,qsc,kc2,kc3,kc4,kc5,ikoptions);
v = r.constructVisualizer();
v.playback(xtraj,struct('slider',true));

display('Check IK traj with infeasibility')
kc_err = WorldCoMConstraint(r,[nan;nan;2],inf(3,1),tspan);
[xtraj,info,infeasible_constraint] = inverseKinTraj(r,q0,qdot0,t,q_seed,q_nom,kc_err,kc2,kc3,kc4,kc5,qsc,ikmexoptions);
if(info ~= 13)
  error('The problem should be infeasible');
end
display('The user should check that the infeasible constraints are the CoM z');
display('The infeasible constraints returned from IKtraj is');
display(infeasible_constraint);
end

function xtraj = test_IKtraj_userfun(r,q0,qdot0,t,q_seed,q_nom,varargin)
ikoptions = varargin{end};
ikoptions = ikoptions.setMex(false);
ikmexoptions = ikoptions;
ikmexoptions = ikmexoptions.setMex(true);
ikoptions = ikoptions.setMex(false);
display('IK mex start to solve the problem');
tic
[xtraj,info] = inverseKinTraj(r,q0,qdot0,t,q_seed,q_nom,varargin{1:end-1},ikmexoptions);
toc
if(info>10)
  error('SNOPT info is %d, IK mex fails to solve the problem',info);
end
% display('IK matlab start to solve the problem');
% tic
% [q,qdot,qddot,info] = inverseKinSequence(r,q0,qdot0,t,q_seed,q_nom,varargin{1:end-1},ikoptions);
% toc
% if(info>10)
%   error('SNOPT info is %d, IK fails to solve the problem',info);
% end
% valuecheck(q,qmex,1e-3);
% valuecheck(qdot,qdotmex,1e-3);
end