function testCOM
% test COM computation with/without affordance
urdf = [getDrakePath,'/examples/Atlas/urdf/atlas_minimal_contact.urdf'];
options.floating = true;
w = warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits');
%warning('off','Drake:RigidBodyManipulator:ReplacedCylinder');
r = RigidBodyManipulator(urdf,options);
warning(w);
nq = r.getNumPositions();


nom_data = load('../../../examples/Atlas/data/atlas_fp.mat');
q_nom = nom_data.xstar(1:nq);
q_seed = q_nom+0.1*randn(nq,1);
qdot = randn(nq,1);

display('Check the CoM with robot only');
kinsol = doKinematics(r,q_seed,true,true,qdot);
[com_mex,J_mex,dJ_mex] = r.getCOM(kinsol);
[com_mex1,J_mex1,dJ_mex1] = r.getCOM(kinsol,1);
valuecheck(com_mex,com_mex1);
valuecheck(J_mex,J_mex1);
valuecheck(dJ_mex,dJ_mex1);
valuecheck(J_mex(1:3,1:3),eye(3),1e-6);

display('Check if MATLAB and mex are consistent for robot only');
kinsol = doKinematics(r,q_seed,true,false,qdot);
[com,J,dJ] = r.getCOM(kinsol);
[com1,J1,dJ1] = r.getCOM(kinsol,1);

valuecheck(com1,com);
valuecheck(J1,J);
valuecheck(dJ1,dJ);

valuecheck(com_mex,com,1e-10);
valuecheck(J_mex,J,1e-10);
valuecheck(dJ_mex,dJ,1e-10);

xyz = 0.1*randn(3,1)+[1;1;1];
rpy = 0.1*pi*randn(3,1)+[pi/2;0;0];
w = warning('off','Drake:RigidBodyManipulator:ReplacedCylinder');
r = r.addRobotFromURDF('valve_task_wall.urdf',xyz,rpy,struct('floating',false));
warning(w);
nq_aff = length(r.getStateFrame.frame{2}.coordinates)/2;
q_seed_aff = zeros(nq_aff,1);
nq = r.getNumPositions();
qdot = randn(nq,1);

display('Check if MATLAB and mex are consistent for robot and affordance together');
kinsol = doKinematics(r,[q_seed;q_seed_aff],true,true,qdot);
[com_mex,J_mex,dJ_mex] = r.getCOM(kinsol);
[com_mex1,J_mex1,dJ_mex1] = r.getCOM(kinsol,1);
[com_mex2,J_mex2,dJ_mex2] = r.getCOM(kinsol,2);
[com_mex12,J_mex12,dJ_mex12] = r.getCOM(kinsol,[1,2]);

valuecheck(com_mex,com_mex1);
valuecheck(J_mex,J_mex1);
valuecheck(dJ_mex,dJ_mex1);
valuecheck(J_mex(1:3,1:3),eye(3),1e-6);

kinsol = doKinematics(r,[q_seed;q_seed_aff],true,false,qdot);
[com,J,dJ] = r.getCOM(kinsol);
[com1,J1,dJ1] = r.getCOM(kinsol,1);
[com2,J2,dJ2] = r.getCOM(kinsol,2);
[com12,J12,dJ12] = r.getCOM(kinsol,[1,2]);

valuecheck(com_mex,com,1e-10);
valuecheck(com_mex1,com1,1e-10);
valuecheck(com_mex2,com2,1e-10);
valuecheck(com_mex12,com12,1e-10);
valuecheck(J_mex,J,1e-10);
valuecheck(J_mex1,J1,1e-10);
valuecheck(J_mex2,J2,1e-10);
valuecheck(J_mex12,J12,1e-10);
valuecheck(dJ_mex,dJ,1e-10);
valuecheck(dJ_mex1,dJ1,1e-10);
valuecheck(dJ_mex2,dJ2,1e-10);
valuecheck(dJ_mex12,dJ12,1e-10);

end
