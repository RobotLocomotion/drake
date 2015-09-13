
load data/step_up_clearance.mat

warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints');
warning('off','Drake:RigidBodyManipulator:WeldedLinkInd');
warning('off','Drake:RigidBodyManipulator:UnsupportedJointLimits');
options.terrain = RigidBodyLinearStepTerrain(.3,.35,.02);
options.floating = true;
options.ignore_self_collisions = true;
options.use_bullet = false;
options.use_new_kinsol = true;
p = PlanarRigidBodyManipulator('../urdf/atlas_planar_one_arm_noback.urdf',options);

  Q = diag([100*ones(p.getNumPositions,1);1*ones(p.getNumVelocities,1)]);
  R = 0.01*eye(getNumInputs(p));
  Qf = 2*Q;
  
% R = cell(0);
% xtraj_=xtraj{1}.append(xtraj{2}).append(xtraj{3}).append(xtraj{4}).append(xtraj{5});
% utraj_=utraj{1}.append(utraj{2}).append(utraj{3}).append(utraj{4}).append(utraj{5});
% for i=1:length(xtraj),
%   R{i} = zeros(16,2*length(contact_seq{i}));
%   R{i}(contact_seq{i}*4-3,2:2:end) = eye(length(contact_seq{i}));
%   ltraj_full{i} = R{i}*ltraj{i};
% end
% ltraj_=ltraj_full{1}.append(ltraj_full{2}).append(ltraj_full{3}).append(ltraj_full{4});
% solveLQR(p,xtraj_,utraj_,ltraj_);

[c,Ktraj,Straj,Ptraj,Btraj,Straj_full,Ftraj,xtraj,utraj] = hybriddircolconstrainedtvlqr(p,xtraj,utraj,contact_seq,Q,R,Qf,options);
