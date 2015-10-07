function solveLQRDircol(p,xtraj,utraj,ltraj,Q,R,Qf)

if nargin <1
  load data/hopper_dircol_traj.mat
  warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints');
  warning('off','Drake:RigidBodyManipulator:WeldedLinkInd');
  warning('off','Drake:RigidBodyManipulator:UnsupportedJointLimits');
  options.terrain = RigidBodyFlatTerrain();
  options.floating = true;
  options.ignore_self_collisions = true;
  options.use_bullet = false;
  options.use_new_kinsol = true;
  p = PlanarRigidBodyManipulator('OneLegHopper.urdf',options);
end

if nargin < 5
  Q = diag([20*ones(p.getNumPositions,1);0.5*ones(p.getNumVelocities,1)]);
  R = 0.01*eye(getNumInputs(p));
  Qf = 2*Q;
end

[c,Ktraj,Straj,Ptraj,Btraj,Straj_full,Ftraj,xtraj,utraj] = hybriddircolconstrainedtvlqr(p,xtraj,utraj,contact_seq,Q,R,Qf);

keyboard;
save('data/hopper_hybrid_lqr.mat','xtraj','utraj','ltraj','c','Ktraj','Straj','Ptraj','Btraj','Straj_full','Ftraj','Q','R','Qf','contact_seq');

end

