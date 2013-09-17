function [qtraj,info] = testIKsequence(qtraj0)

options.floating = true;
options.dt = 0.001;
oldpath = addpath(fullfile(pwd,'..'));
p = Atlas('../urdf/atlas_minimal_contact.urdf',options);
load('../data/atlas_fp.mat');
p = p.setInitialState(xstar);
path(oldpath);

ks = ActionSequence();
r_foot = p.findLinkInd('r_foot');
l_foot = p.findLinkInd('l_foot');
r_hand = p.findLinkInd('r_hand');
l_hand = p.findLinkInd('l_hand');

r_foot_contact_pts = getContactPoints(getBody(p,r_foot));
l_foot_contact_pts = getContactPoints(getBody(p,l_foot));
r_hand_contact_pts = mean(getContactPoints(getBody(p,r_hand)),2);
l_hand_contact_pts = mean(getContactPoints(getBody(p,l_hand)),2);

nq = p.getNumDOF();
q0 = xstar(1:nq);
qdot0 = zeros(nq,1);
kinsol0 = doKinematics(p,q0);
r_foot_contact_pos = forwardKin(p,kinsol0,r_foot,r_foot_contact_pts,2);
r_foot_contact_pos(3,:) = 0;
l_foot_contact_pos = forwardKin(p,kinsol0,l_foot,l_foot_contact_pts,2);
l_foot_contact_pos(3,:) = 0;
r_hand_contact_pos = forwardKin(p,kinsol0,r_hand,r_hand_contact_pts,0);
l_hand_contact_pos = forwardKin(p,kinsol0,l_hand,l_hand_contact_pts,1);
com_pos0 = getCOM(p,kinsol0);
com_pos.min = [-inf;-inf;com_pos0(3)];
com_pos.max = [inf;inf;inf];
tspan = [0,1];
kc1 = ActionKinematicConstraint(p,r_foot,r_foot_contact_pts,r_foot_contact_pos,tspan,'rfoot');
ks = ks.addKinematicConstraint(kc1);
kc2 = ActionKinematicConstraint(p,l_foot,l_foot_contact_pts,l_foot_contact_pos,tspan,'lfoot');
ks = ks.addKinematicConstraint(kc2);
kc3 = ActionKinematicConstraint(p,r_hand,r_hand_contact_pts,r_hand_contact_pos+[0.1;0.05;1],[tspan(end) tspan(end)],'rhand_ee_goal');
ks = ks.addKinematicConstraint(kc3);
kc4 = ActionKinematicConstraint(p,l_hand,l_hand_contact_pts,l_hand_contact_pos,[tspan(end) tspan(end)],'lhand_ee_goal');
ks = ks.addKinematicConstraint(kc4);
kc5 = ActionKinematicConstraint(p,l_hand,l_hand_contact_pts,l_hand_contact_pos+[0.05;0.05;0.05;0;0;0],[0.7 0.7],'lhand_ee_goal2');
ks = ks.addKinematicConstraint(kc5);
kc6 = ActionKinematicConstraint(p,0,[0;0;0],com_pos,tspan,'com');
ks = ks.addKinematicConstraint(kc6);
% r_toe = r_foot.getContactPoints('toe');
% kc7 = ActionKinematicConstraint.groundConstraint(p,r_foot,r_toe,tspan,'toe_above_ground');
% ks = ks.addKinematicConstraint(kc7);
cost = Point(p.getStateFrame,1);
cost.base_x = 100;
cost.base_y = 100;
cost.base_z = 100;
cost.base_roll = 100;
cost.base_pitch = 100;
cost.base_yaw = 100;
cost = double(cost);
Q = diag(cost(1:nq));
options = struct('nSample',2);
options.MajorIterationsLimit = 300;
% options.q_nom = q0;
% options.Q = Q;
% options.Qv = 1*eye(nq);
options.Qa = 5*Q;
options.quasiStaticFlag = true;
% options.optimizeSparsity = true;
if(nargin>0)
    options.qtraj0 = qtraj0;
end
% profile on
tic
[t_breaks,q,qdot,qddot,info] = inverseKinSequence(p,q0,qdot0,ks,options);
toc
% profile off
% profile viewer
qtraj = PPTrajectory(pchipDeriv(t_breaks,q,qdot));
xtraj = PPTrajectory(foh(t_breaks,[q;qdot]));
xtraj = xtraj.setOutputFrame(p.getStateFrame());
v = p.constructVisualizer();
v.playback(xtraj,struct('slider',true));

end
