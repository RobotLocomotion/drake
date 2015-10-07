load data/longer_step_smooth.mat

warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints');
warning('off','Drake:RigidBodyManipulator:WeldedLinkInd');
warning('off','Drake:RigidBodyManipulator:UnsupportedJointLimits');
options.terrain = RigidBodyFlatTerrain();
options.floating = true;
options.ignore_self_collisions = true;
options.use_new_kinsol = true;
options.use_bullet = false;

p = RigidBodyManipulator('../urdf/atlas_simple_contact_noback.urdf',options);

  Q = diag([100*ones(p.getNumPositions,1);1*ones(p.getNumVelocities,1)]);
  R = 0.01*eye(getNumInputs(p));
  Qf = 2*Q;
  
  nq = p.getNumPositions;
   R_periodic = zeros(p.getNumStates);
  R_periodic(1,1) = 1; %x 
  R_periodic(2,2) = -1; %y
  R_periodic(3,3) = 1; %z
  R_periodic(4,4) = -1; %roll
  R_periodic(5,5) = 1; %pitch
  R_periodic(6,6) = -1; %yaw
  R_periodic(7:12,13:18) = diag([-1;-1;1;1;1;-1]); %leg joints w/symmetry
  R_periodic(13:18,7:12) = diag([-1;-1;1;1;1;-1]); %leg joints w/symmetry
  R_periodic(nq+1:end,nq+1:end) = R_periodic(1:nq,1:nq);
  options.periodic_jump = R_periodic;
  options.periodic = true;
  
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
