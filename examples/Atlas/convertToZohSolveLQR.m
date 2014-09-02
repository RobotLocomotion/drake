function convertToZohSolveLQR()


warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints');
warning('off','Drake:RigidBodyManipulator:WeldedLinkInd');
warning('off','Drake:RigidBodyManipulator:UnsupportedJointLimits');
options.terrain = RigidBodyFlatTerrain();
options.floating = true;
options.ignore_self_collisions = true;
p = PlanarRigidBodyManipulator('urdf/atlas_simple_spring_ankle_planar_contact.urdf',options);

data_dir = fullfile(getDrakePath,'examples','Atlas','data');
traj_file = strcat(data_dir,'/atlas_passiveankle_traj_lqr_082914_2.mat');
load(traj_file);

Q = diag([100*ones(p.getNumPositions,1);10*ones(p.getNumVelocities,1)]);
R = 0.001*eye(getNumInputs(p));
Qf = Q;

t_t = xtraj.pp.breaks;
x = xtraj.eval(t_t);
qtraj = PPTrajectory(foh(t_t,x(1:p.getNumPositions,:)));
qdtraj = PPTrajectory(zoh(t_t,[x(1+p.getNumPositions:end,2:end) zeros(p.getNumVelocities,1)]));
xtraj = [qtraj;qdtraj];


[c,Ktraj,Straj,Ptraj,Btraj,tvec,Straj_full,Ftraj] = hybridconstrainedtvlqr(p,xtraj,utraj,ltraj,Q,R,Qf);

save(strcat(data_dir,'/atlas_passiveankle_traj_lqr_zoh.mat'),'xtraj','utraj','ltraj','c','Ktraj','Straj','Ptraj','Btraj','tvec','Straj_full','Ftraj','Q','R','Qf');

end

