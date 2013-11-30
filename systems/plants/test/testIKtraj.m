function testIKtraj()
options.floating = true;
options.dt = 0.001;
r = RigidBodyManipulator();
w = warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits');
warning('off','Drake:RigidBody:SimplifiedCollisionGeometry');
warning('off','Drake:RigidBodyManipulator:UnsupportedJointLimits');
warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints');
r = r.addRobotFromURDF('../../../examples/Atlas/urdf/atlas_minimal_contact.urdf',[],[],options);
warning(w);
nom_data = load('../../../examples/Atlas/data/atlas_fp.mat');
v = r.constructVisualizer();

r_foot = r.findLinkInd('r_foot');
l_foot = r.findLinkInd('l_foot');
r_hand = r.findLinkInd('r_hand');
l_hand = r.findLinkInd('l_hand');
head = r.findLinkInd('head');
pelvis = r.findLinkInd('pelvis');

r_foot_contact_pts = getContactPoints(getBody(r,r_foot));
r_foot_pts = r_foot_contact_pts(:,1);
l_foot_contact_pts = getContactPoints(getBody(r,l_foot));
l_foot_pts = l_foot_contact_pts(:,1);
r_hand_pts = mean(getContactPoints(getBody(r,r_hand)),2);
l_hand_pts = mean(getContactPoints(getBody(r,l_hand)),2);



nq = r.getNumDOF();
coords = r.getStateFrame.coordinates(1:nq);
l_knee_idx = find(strcmp(coords,'l_leg_kny'));
r_knee_idx = find(strcmp(coords,'r_leg_kny'));

q0 = nom_data.xstar(1:nq);
qdot0 = zeros(nq,1);
kinsol0 = doKinematics(r,q0,false,false);
r_foot_pos = forwardKin(r,kinsol0,r_foot,r_foot_pts,2);
r_foot_pos(3,:) = 0;
l_foot_pos = forwardKin(r,kinsol0,l_foot,l_foot_pts,2);
l_foot_pos(3,:) = 0;
r_hand_pos = forwardKin(r,kinsol0,r_hand,r_hand_pts,0);
l_hand_pos = forwardKin(r,kinsol0,l_hand,l_hand_pts,0);
com_pos0 = getCOM(r,kinsol0,1);
com_height = com_pos0(3);
tspan = [0,1];
kc1 = {WorldPositionConstraint(r,r_foot,r_foot_pts,r_foot_pos(1:3),r_foot_pos(1:3),tspan),...
  WorldQuatConstraint(r,r_foot,r_foot_pos(4:7),0,tspan)};
kc2 = {WorldPositionConstraint(r,l_foot,l_foot_pts,l_foot_pos(1:3),l_foot_pos(1:3),tspan),...
  WorldQuatConstraint(r,l_foot,l_foot_pos(4:7),0,tspan)};
kc3 = WorldPositionConstraint(r,r_hand,r_hand_pts,r_hand_pos+[0.1;0.05;0.75],r_hand_pos+[0.1;0.05;1],[tspan(end) tspan(end)]);
kc4 = WorldPositionConstraint(r,l_hand,l_hand_pts,l_hand_pos,l_hand_pos,[tspan(end) tspan(end)]);
kc5 = WorldCoMConstraint(r,[-inf;-inf;com_height],[inf;inf;com_height+0.5],tspan,1);
pc_knee = PostureConstraint(r,tspan);

pc_knee = pc_knee.setJointLimits([l_knee_idx;r_knee_idx],[0.2;0.2],[inf;inf]);

qsc = QuasiStaticConstraint(r);
qsc = qsc.addContact(r_foot,r_foot_contact_pts);
qsc = qsc.addContact(l_foot,l_foot_contact_pts);
qsc = qsc.setActive(true);
qsc = qsc.setShrinkFactor(0.8);

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
ikoptions = ikoptions.setQa(0.001*Q);
ikoptions = ikoptions.setMajorIterationsLimit(10000);
ikoptions = ikoptions.setIterationsLimit(100000);
ikoptions = ikoptions.setSuperbasicsLimit(1000);
ikoptions = ikoptions.setDebug(true);
ikmexoptions = ikoptions;
ikmexoptions = ikmexoptions.setMex(true);
ikmexoptions = ikmexoptions.setDebug(true);
nT = 5;
% t = [tspan(1) tspan(1)+0.2*(tspan(end)-tspan(1)) tspan(1)+0.7*(tspan(end)-tspan(1)) tspan(end)];
t = linspace(tspan(1),tspan(end),nT);
q_nom_traj = PPTrajectory(foh(t,repmat(q0,1,nT)));
q_seed_traj = PPTrajectory(foh(t,repmat(q0,1,nT)+[zeros(nq,1) 1e-1*randn(nq,nT-1)]));

display('Check IK traj');
xtraj = test_IKtraj_userfun(r,t,q_seed_traj,q_nom_traj,kc1{:},kc2{:},kc3,kc4,kc5,pc_knee,ikoptions);
v = r.constructVisualizer();
v.playback(xtraj,struct('slider',true));
display('Check IK traj with quasi static constraint');
xtraj = test_IKtraj_userfun(r,t,q_seed_traj,q_nom_traj,kc1{:},qsc,kc2{:},kc3,kc4,kc5,pc_knee,ikoptions);
v = r.constructVisualizer();
v.playback(xtraj,struct('slider',true));

display('Check IK traj with infeasibility')
kc_err = WorldCoMConstraint(r,[nan;nan;2],inf(3,1),tspan);
[xtraj,info,infeasible_constraint] = inverseKinTraj(r,t,q_seed_traj,q_nom_traj,kc_err,kc2{:},kc3,kc4,kc5,pc_knee,qsc,ikmexoptions);
if(info ~= 13)
  error('The problem should be infeasible');
end
display('The user should check that the infeasible constraints are the CoM z');
display('The infeasible constraints returned from IKtraj is');
display(infeasible_constraint);

display('Check IK with WorldFixedPositionConstraint');
kc_fixedPosition = WorldFixedPositionConstraint(r,pelvis,[0;0;0],tspan);
xtraj = test_IKtraj_userfun(r,t,q_seed_traj,q_nom_traj,kc1{:},kc2{:},kc3,kc4,qsc,kc_fixedPosition,ikoptions);

display('Check IK with WorldFixedOrientConstraint');
kc_fixedOrient = WorldFixedOrientConstraint(r,pelvis,tspan);
xtraj = test_IKtraj_userfun(r,t,q_seed_traj,q_nom_traj,kc1{:},kc2{:},kc3,kc4,qsc,kc_fixedOrient,ikoptions);

display('Check IK with WorldFixedBodyPoseConstraint');
kc_fixedPose = WorldFixedBodyPoseConstraint(r,pelvis,tspan);
xtraj = test_IKtraj_userfun(r,t,q_seed_traj,q_nom_traj,kc1{:},kc2{:},kc3,kc4,qsc,pc_knee,kc_fixedPose,ikoptions);

display('Check IK with fixInitialState = false, the posture and velocity at the begining are also decision variables');
ikoptions = ikoptions.setFixInitialState(false);
xtraj = test_IKtraj_userfun(r,t,q_seed_traj,q_nom_traj,kc1{:},kc2{:},kc3,kc4,qsc,pc_knee,kc_fixedPose,ikoptions);

display('Check with inbetween samples')
t_inbetween = [0.1 0.15 0.3 0.4 0.6];
ikoptions = ikoptions.setAdditionaltSamples(t_inbetween);
xtraj = test_IKtraj_userfun(r,t,q_seed_traj,q_nom_traj,kc1{:},kc2{:},kc3,kc4,qsc,pc_knee,kc_fixedPose,ikoptions);
t_samples = sort([t t_inbetween]);
x_samples = xtraj.eval(t_samples);
q_samples = x_samples(1:nq,:);
c_fixedPose = kc_fixedPose.eval(t_samples,q_samples);
[lb_fixedPose,ub_fixedPose] = kc_fixedPose.bounds(t_samples);
if(any(c_fixedPose-ub_fixedPose>1e-3) || any(c_fixedPose-lb_fixedPose<-1e-3))
  error('The fixed pose constraint is not satisfied');
end

display('Check MultipleKinematicConstraint with tspan being only part of the t_breaks');
tspan2 = [0.2 0.7];
kc_fixedPose = WorldFixedBodyPoseConstraint(r,pelvis,tspan2);
xtraj = test_IKtraj_userfun(r,t,q_seed_traj,q_nom_traj,kc1{:},kc2{:},kc3,kc4,qsc,pc_knee,kc_fixedPose,ikoptions);

display('Check MultipleTimeLinearPostureConstraint');
pc_change = PostureChangeConstraint(r,[l_knee_idx;r_knee_idx],[-0.1;-0.03],[0.05;0.02],tspan);
ikoptions = ikoptions.setFixInitialState(true);
xtraj = test_IKtraj_userfun(r,t,q_seed_traj,q_nom_traj,kc1{:},kc2{:},kc3,kc4,qsc,pc_change,ikoptions);
xbreaks = xtraj.eval(t);
if(any(xbreaks(l_knee_idx,2:end)-xbreaks(l_knee_idx,1)>0.05+1e-10) || any(xbreaks(l_knee_idx,2:end)-xbreaks(l_knee_idx,1)<-0.1-1e-10)||...
   any(xbreaks(r_knee_idx,2:end)-xbreaks(r_knee_idx,1)>0.02+1e-10) || any(xbreaks(r_knee_idx,2:end)-xbreaks(r_knee_idx,1)<-0.03-1e-10))
 error('PostureChangeConstraint is not satisfied');
end

ikoptions = ikoptions.setFixInitialState(false);
xtraj = test_IKtraj_userfun(r,t,q_seed_traj,q_nom_traj,kc1{:},kc2{:},kc3,kc4,qsc,pc_change,ikoptions);
xbreaks = xtraj.eval(t);
if(any(xbreaks(l_knee_idx,2:end)-xbreaks(l_knee_idx,1)>0.05+1e-10) || any(xbreaks(l_knee_idx,2:end)-xbreaks(l_knee_idx,1)<-0.1-1e-10)||...
   any(xbreaks(r_knee_idx,2:end)-xbreaks(r_knee_idx,1)>0.02+1e-10) || any(xbreaks(r_knee_idx,2:end)-xbreaks(r_knee_idx,1)<-0.03-1e-10))
 error('PostureChangeConstraint is not satisfied');
end

display('Check MultipleTimeLinearPostureConstraint for time span being only part of the t_breaks');
pc_change2 = PostureChangeConstraint(r,[l_knee_idx;r_knee_idx],[-0.1;-0.03],[0.05;0.02],[t(2) t(end)]);
ikoptions = ikoptions.setFixInitialState(true);
xtraj = test_IKtraj_userfun(r,t,q_seed_traj,q_nom_traj,kc1{:},kc2{:},kc3,kc4,qsc,pc_change2,ikoptions);
xbreaks = xtraj.eval(t(2:end));
if(any(xbreaks(l_knee_idx,2:end)-xbreaks(l_knee_idx,1)>0.05+1e-10) || any(xbreaks(l_knee_idx,2:end)-xbreaks(l_knee_idx,1)<-0.1-1e-10)||...
   any(xbreaks(r_knee_idx,2:end)-xbreaks(r_knee_idx,1)>0.02+1e-10) || any(xbreaks(r_knee_idx,2:end)-xbreaks(r_knee_idx,1)<-0.03-1e-10))
 error('PostureChangeConstraint is not satisfied');
end
ikoptions = ikoptions.setFixInitialState(false);
xtraj = test_IKtraj_userfun(r,t,q_seed_traj,q_nom_traj,kc1{:},kc2{:},kc3,kc4,qsc,pc_change2,ikoptions);
xbreaks = xtraj.eval(t(2:end));
if(any(xbreaks(l_knee_idx,2:end)-xbreaks(l_knee_idx,1)>0.05+1e-10) || any(xbreaks(l_knee_idx,2:end)-xbreaks(l_knee_idx,1)<-0.1-1e-10)||...
   any(xbreaks(r_knee_idx,2:end)-xbreaks(r_knee_idx,1)>0.02+1e-10) || any(xbreaks(r_knee_idx,2:end)-xbreaks(r_knee_idx,1)<-0.03-1e-10))
 error('PostureChangeConstraint is not satisfied');
end

end

function xtraj = test_IKtraj_userfun(r,t,q_seed,q_nom,varargin)
ikoptions = varargin{end};
ikoptions = ikoptions.setMex(false);
ikmexoptions = ikoptions;
ikmexoptions = ikmexoptions.setMex(true);
ikoptions = ikoptions.setMex(false);
display('IK mex start to solve the problem');
tic
[xtraj,info] = inverseKinTraj(r,t,q_seed,q_nom,varargin{1:end-1},ikmexoptions);
toc
if(info>10)
  error('SNOPT info is %d, IK mex fails to solve the problem',info);
end
% display('IK matlab start to solve the problem');
% tic
% [xtraj,info,infeasible_constraint] = inverseKinTraj(r,t,q_seed,q_nom,varargin{1:end-1},ikoptions);
% toc
% if(info>10)
%   error('SNOPT info is %d, IK fails to solve the problem',info);
% end
end