load data/atlas_alt3mode_K10B2_passive_traj

warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints');
warning('off','Drake:RigidBodyManipulator:WeldedLinkInd');
warning('off','Drake:RigidBodyManipulator:UnsupportedJointLimits');
options.terrain = RigidBodyFlatTerrain();
options.floating = true;
options.ignore_self_collisions = true;
options.use_bullet = false;
options.use_new_kinsol = true;
p = PlanarRigidBodyManipulator('../urdf/atlas_simple_spring_ankle_planar_contact.urdf',options);

  Q = diag([100*ones(p.getNumPositions,1);1*ones(p.getNumVelocities,1)]);
  R = .01*eye(getNumInputs(p));
  Qf = 2*Q;
  
  R_periodic = zeros(p.getNumStates);
  R_periodic(1:3,1:3) = eye(3); %x,z,pitch
  R_periodic(4:6,8:10) = eye(3); %leg joints w/symmetry
  R_periodic(8:10,4:6) = eye(3); %leg joints w/symmetry
  R_periodic(7,7) = 1; % back joint
  R_periodic(11:13,11:13) = eye(3); %x,z,pitch velocities
  R_periodic(14:16,18:20) = eye(3); %leg joints w/symmetry
  R_periodic(18:20,14:16) = eye(3); %leg joints w/symmetry
  R_periodic(17,17) = 1; % back joint
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