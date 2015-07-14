function solveLQR(p,xtraj,utraj,ltraj,Q,R,Qf)

if nargin <1
  load data/hopper_traj_40_knots.mat
  warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints');
  warning('off','Drake:RigidBodyManipulator:WeldedLinkInd');
  warning('off','Drake:RigidBodyManipulator:UnsupportedJointLimits');
  options.terrain = RigidBodyFlatTerrain();
  options.floating = true;
  options.ignore_self_collisions = true;
  options.use_bullet = false;
  p = PlanarRigidBodyManipulator('OneLegHopper.urdf',options);
end

if nargin < 5
  Q = diag([10*ones(p.getNumPositions,1);1*ones(p.getNumVelocities,1)]);
  R = 0.001*eye(getNumInputs(p));
  Qf = 2*Q;
end

options.use_zoh_qd = true;
options.use_zoh_u = true;

[c,Ktraj,Straj,Ptraj,Btraj,tvec,Straj_full,Ftraj,xtraj,utraj,mode_data] = hybridconstrainedtvlqr(p,xtraj,utraj,ltraj,Q,R,Qf,options);

keyboard;
save('data/hopper_traj_lqr.mat','xtraj','utraj','ltraj','c','Ktraj','Straj','Ptraj','Btraj','tvec','Straj_full','Ftraj','Q','R','Qf','mode_data');

end

