function testComDynamicsFullKinematicsPlanner2
warning('off','Drake:RigidBody:SimplifiedCollisionGeometry');
warning('off','Drake:RigidBody:NonPositiveInertiaMatrix');
warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints');
warning('off','Drake:RigidBodyManipulator:UnsupportedJointLimits');
warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits');
urdf = [getDrakePath,'/examples/Atlas/urdf/atlas_minimal_contact.urdf'];
options.floating = true;
robot = RigidBodyManipulator(urdf,options);
nomdata = load([getDrakePath,'/examples/Atlas/data/atlas_fp.mat']);
nq = robot.getNumPositions();
qstar = nomdata.xstar(1:nq);
kinsol_star = robot.doKinematics(qstar);
nv = robot.getNumDOF();
vstar = zeros(nv,1);

l_foot = robot.findLinkInd('l_foot');
r_foot = robot.findLinkInd('r_foot');
l_foot_shapes_heel = robot.getBody(l_foot).getContactShapes('heel');
l_foot_shapes_toe = robot.getBody(l_foot).getContactShapes('toe');
r_foot_shapes_heel = robot.getBody(r_foot).getContactShapes('heel');
r_foot_shapes_toe = robot.getBody(r_foot).getContactShapes('toe');
l_foot_toe = [];
l_foot_heel = [];
r_foot_toe = [];
r_foot_heel = [];
for i = 1:length(l_foot_shapes_heel)
  l_foot_heel = [l_foot_heel l_foot_shapes_heel{i}.getPoints];
end
for i = 1:length(l_foot_shapes_toe)
  l_foot_toe = [l_foot_toe l_foot_shapes_toe{i}.getPoints];
end
for i = 1:length(r_foot_shapes_heel)
  r_foot_heel = [r_foot_heel r_foot_shapes_heel{i}.getPoints];
end
for i = 1:length(r_foot_shapes_toe)
  r_foot_toe = [r_foot_toe r_foot_shapes_toe{i}.getPoints];
end
l_foot_bottom = [l_foot_toe l_foot_heel];
r_foot_bottom = [r_foot_toe r_foot_heel];

lfoot_pos_star = robot.forwardKin(kinsol_star,l_foot,[0;0;0],2);
rfoot_pos_star = robot.forwardKin(kinsol_star,r_foot,[0;0;0],2);
lfoot_toe_pos_star = robot.forwardKin(kinsol_star,l_foot,l_foot_toe,0);
rfoot_toe_pos_star = robot.forwardKin(kinsol_star,r_foot,r_foot_toe,0);
lfoot_heel_pos_star = robot.forwardKin(kinsol_star,l_foot,l_foot_heel,0);
rfoot_heel_pos_star = robot.forwardKin(kinsol_star,r_foot,r_foot_heel,0);
com_star = robot.getCOM(kinsol_star);

mu = 1;

heel_takeoff_idx = 5;
toe_takeoff_idx = 7;
heel_land_idx = 14;
toe_land_idx = 12;
nT = 17;
l_foot_contact_wrench = struct('active_knot',[],'cw',[]);
l_foot_contact_wrench(1) = struct('active_knot',1:heel_takeoff_idx,'cw',FrictionConeWrench(robot,l_foot,l_foot_heel,mu,[0;0;1]));
l_foot_contact_wrench(2) = struct('active_knot',1:toe_takeoff_idx,'cw',FrictionConeWrench(robot,l_foot,l_foot_toe,mu,[0;0;1]));
l_foot_contact_wrench(3) = struct('active_knot',heel_land_idx:nT,'cw',FrictionConeWrench(robot,l_foot,l_foot_heel,mu,[0;0;1]));
l_foot_contact_wrench(4) = struct('active_knot',toe_land_idx:nT,'cw',FrictionConeWrench(robot,l_foot,l_foot_toe,mu,[0;0;1]));
r_foot_contact_wrench = struct('active_knot',[],'cw',[]);
r_foot_contact_wrench(1) = struct('active_knot',1:heel_takeoff_idx,'cw',FrictionConeWrench(robot,r_foot,r_foot_heel,mu,[0;0;1]));
r_foot_contact_wrench(2) = struct('active_knot',1:toe_takeoff_idx,'cw',FrictionConeWrench(robot,r_foot,r_foot_toe,mu,[0;0;1]));
r_foot_contact_wrench(3) = struct('active_knot',heel_land_idx:nT,'cw',FrictionConeWrench(robot,r_foot,r_foot_heel,mu,[0;0;1]));
r_foot_contact_wrench(4) = struct('active_knot',toe_land_idx:nT,'cw',FrictionConeWrench(robot,r_foot,r_foot_toe,mu,[0;0;1]));

bky_idx = robot.getBody(robot.findJointInd('back_bky')).dofnum;

tf_range = [1 2];
q_nom = bsxfun(@times,qstar,ones(1,nT));
Q_comddot = eye(3);
Q = eye(nq);
Q(1,1) = 0;
Q(2,2) = 0;
Q(6,6) = 0;
Q(bky_idx,bky_idx) = 100*Q(bky_idx,bky_idx);
Qv = 0.1*eye(nv);
cdfkp = ComDynamicsFullKinematicsPlanner(robot,nT,tf_range,Q_comddot,Qv,Q,q_nom,[l_foot_contact_wrench r_foot_contact_wrench]);

lfoot_toe_above_ground = WorldPositionConstraint(robot,l_foot,l_foot_toe,[nan(2,2);0.01*ones(1,2)],nan(3,2));
% lfoot_toe_above_ground = lfoot_toe_above_ground.generateConstraint([]);
cdfkp = cdfkp.addRigidBodyConstraint(lfoot_toe_above_ground,num2cell(toe_takeoff_idx+1:toe_land_idx-1));

rfoot_toe_above_ground = WorldPositionConstraint(robot,r_foot,r_foot_toe,[nan(2,2);0.01*ones(1,2)],nan(3,2));
% rfoot_toe_above_ground = rfoot_toe_above_ground.generateConstraint([]);
cdfkp = cdfkp.addRigidBodyConstraint(rfoot_toe_above_ground,num2cell(toe_takeoff_idx+1:toe_land_idx-1));

lfoot_heel_above_ground = WorldPositionConstraint(robot,l_foot,l_foot_heel,[nan(2,2);0.01*ones(1,2)],nan(3,2));
% lfoot_heel_above_ground = lfoot_heel_above_ground.generateConstraint([]);
cdfkp = cdfkp.addRigidBodyConstraint(lfoot_heel_above_ground,num2cell(heel_takeoff_idx+1:heel_land_idx-1));

rfoot_heel_above_ground = WorldPositionConstraint(robot,r_foot,r_foot_heel,[nan(2,2);0.01*ones(1,2)],nan(3,2));
% rfoot_heel_above_ground = rfoot_heel_above_ground.generateConstraint([]);
cdfkp = cdfkp.addRigidBodyConstraint(rfoot_heel_above_ground,num2cell(heel_takeoff_idx+1:heel_land_idx-1));

% lfoot_heel_on_ground = WorldPositionConstraint(robot,l_foot,l_foot_heel,lfoot_heel_pos_star,lfoot_heel_pos_star);
% % lfoot_heel_on_ground = lfoot_heel_on_ground.generateConstraint([]);
% sdfkp = sdfkp.addRigidBodyConstraint(lfoot_heel_on_ground,num2cell(heel_land_idx:toe_land_idx-1));
% 
% rfoot_heel_on_ground = WorldPositionConstraint(robot,r_foot,r_foot_heel,rfoot_heel_pos_star,rfoot_heel_pos_star);
% % rfoot_heel_on_ground = rfoot_heel_on_ground.generateConstraint([]);
% sdfkp = sdfkp.addRigidBodyConstraint(rfoot_heel_on_ground,num2cell(heel_land_idx:toe_land_idx-1));

lfoot_toe_on_ground = WorldPositionConstraint(robot,l_foot,l_foot_toe,lfoot_toe_pos_star,lfoot_toe_pos_star);
% lfoot_toe_on_ground = lfoot_toe_on_ground.generateConstraint([]);
cdfkp = cdfkp.addRigidBodyConstraint(lfoot_toe_on_ground,num2cell([(heel_takeoff_idx+1:toe_takeoff_idx) (toe_land_idx:heel_land_idx-1)]));

rfoot_toe_on_ground = WorldPositionConstraint(robot,r_foot,r_foot_toe,rfoot_toe_pos_star,rfoot_toe_pos_star);
% rfoot_toe_on_ground = rfoot_toe_on_ground.generateConstraint([]);
cdfkp = cdfkp.addRigidBodyConstraint(rfoot_toe_on_ground,num2cell([(heel_takeoff_idx+1:toe_takeoff_idx) (toe_land_idx:heel_land_idx-1)]));

lfoot_on_ground = {WorldPositionConstraint(robot,l_foot,[0;0;0],lfoot_pos_star(1:3),lfoot_pos_star(1:3)),...
  WorldQuatConstraint(robot,l_foot,lfoot_pos_star(4:7),0)};
% lfoot_on_ground = {lfoot_on_ground{1}.generateConstraint([]),lfoot_on_ground{2}.generateConstraint([])};
cdfkp = cdfkp.addRigidBodyConstraint(lfoot_on_ground{1},num2cell([(2:heel_takeoff_idx) (heel_land_idx:nT-1)]));
cdfkp = cdfkp.addRigidBodyConstraint(lfoot_on_ground{2},num2cell([(2:heel_takeoff_idx) (heel_land_idx:nT-1)]));

rfoot_on_ground = {WorldPositionConstraint(robot,r_foot,[0;0;0],rfoot_pos_star(1:3),rfoot_pos_star(1:3)),...
  WorldQuatConstraint(robot,r_foot,rfoot_pos_star(4:7),0)};
% rfoot_on_ground = {rfoot_on_ground{1}.generateConstraint([]),rfoot_on_ground{2}.generateConstraint([])};
cdfkp = cdfkp.addRigidBodyConstraint(rfoot_on_ground{1},num2cell([(2:heel_takeoff_idx) (heel_land_idx:nT-1)]));
cdfkp = cdfkp.addRigidBodyConstraint(rfoot_on_ground{2},num2cell([(2:heel_takeoff_idx) (heel_land_idx:nT-1)]));

% 
% sdfkp = sdfkp.addBoundingBoxConstraint(BoundingBoxConstraint(qstar(3)+0.3,inf),sdfkp.q_inds(3,apex_knot));

cdfkp = cdfkp.addBoundingBoxConstraint(BoundingBoxConstraint(qstar,qstar),cdfkp.q_inds(:,1));
cdfkp = cdfkp.addBoundingBoxConstraint(BoundingBoxConstraint(qstar,qstar),cdfkp.q_inds(:,end));
cdfkp = cdfkp.addBoundingBoxConstraint(BoundingBoxConstraint(vstar,vstar),cdfkp.v_inds(:,1));
cdfkp = cdfkp.addBoundingBoxConstraint(BoundingBoxConstraint(vstar,vstar),cdfkp.v_inds(:,end));

cdfkp = cdfkp.addBoundingBoxConstraint(BoundingBoxConstraint(0.02*ones(nT-1,1),0.2*ones(nT-1,1)),cdfkp.h_inds(:));

cdfkp = cdfkp.addCoMBounds(1:nT,bsxfun(@times,com_star-[0.5;0.5;0.5],ones(1,nT)),bsxfun(@times,com_star+[0.5;0.5;1],ones(1,nT)));

cdfkp = cdfkp.addBoundingBoxConstraint(BoundingBoxConstraint(...
  reshape(bsxfun(@times,[-9.81/2;-9.81/2;-9.81],ones(1,nT)),[],1),...
  reshape(bsxfun(@times,[9.81/2;9.81/2;9.81],ones(1,nT)),[],1)),cdfkp.comddot_inds(:));

x_seed = zeros(cdfkp.num_vars,1);
x_seed(cdfkp.h_inds) = 0.1;
x_seed(cdfkp.q_inds(:)) = reshape(bsxfun(@times,qstar,ones(1,nT)),[],1);
x_seed(cdfkp.com_inds(:)) = reshape(bsxfun(@times,com_star,ones(1,nT)),[],1);
x_seed(cdfkp.lambda_inds{1}(:)) = reshape(bsxfun(@times,[0;0;cdfkp.robot_mass*cdfkp.g/8],ones(1,4,nT)),[],1);
x_seed(cdfkp.lambda_inds{2}(:)) = reshape(bsxfun(@times,[0;0;cdfkp.robot_mass*cdfkp.g/8],ones(1,4,nT)),[],1);

% add a cost to maximize com apex_height
cdfkp = cdfkp.addCost(NonlinearConstraint(-inf,inf,2,@comApexHeightCost),{cdfkp.com_inds(3,toe_takeoff_idx);cdfkp.comdot_inds(3,toe_takeoff_idx)});
% add a constraint on the com apex height
apex_height_cnstr = NonlinearConstraint(1.4,inf,2,@comApexHeight);
apex_height_cnstr = apex_height_cnstr.setName({'com apex height'});
cdfkp = cdfkp.addNonlinearConstraint(apex_height_cnstr,{cdfkp.com_inds(3,toe_takeoff_idx);cdfkp.comdot_inds(3,toe_takeoff_idx)});

% add a symmetric constraint
symmetry_cnstr = symmetryConstraint(robot,2:nT-1);
cdfkp = cdfkp.addLinearConstraint(symmetry_cnstr,reshape(cdfkp.q_inds(:,2:nT-1),[],1));
% no yawing on the back
bkz_idx = robot.getBody(robot.findJointInd('back_bkz')).dofnum;
cdfkp = cdfkp.addBoundingBoxConstraint(BoundingBoxConstraint(zeros(nT-2,1),zeros(nT-2,1)),reshape(cdfkp.q_inds(bkz_idx,2:nT-1),[],1));
% sdfkp = sdfkp.addBoundingBoxConstraint(BoundingBoxConstraint(zeros(nT-2,1),zeros(nT-2,1)),reshape(sdfkp.q_inds(6,2:nT-1),[],1));
% sdfkp = sdfkp.addBoundingBoxConstraint(BoundingBoxConstraint(zeros(nT-2,1),zeros(nT-2,1)),reshape(sdfkp.q_inds(4,2:nT-1),[],1));

% no large pelvis pitch
cdfkp = cdfkp.addBoundingBoxConstraint(BoundingBoxConstraint((qstar(5)-0.2)*ones(nT-2,1),(qstar(5)+0.2)*ones(nT-2,1)),cdfkp.q_inds(5,2:nT-1));

cdfkp = cdfkp.setSolverOptions('snopt','iterationslimit',1e6);
cdfkp = cdfkp.setSolverOptions('snopt','majoriterationslimit',2000);
cdfkp = cdfkp.setSolverOptions('snopt','majorfeasibilitytolerance',1e-6);
cdfkp = cdfkp.setSolverOptions('snopt','majoroptimalitytolerance',2e-4);
cdfkp = cdfkp.setSolverOptions('snopt','superbasicslimit',2000);
cdfkp = cdfkp.setSolverOptions('snopt','print','test_cdfkp2.out');

seed_sol = load('test_cdfkp2','-mat','x_sol');
tic
[x_sol,F,info] = cdfkp.solve(seed_sol.x_sol);
toc
q_sol = reshape(x_sol(cdfkp.q_inds(:)),nq,nT);
v_sol = reshape(x_sol(cdfkp.v_inds(:)),nq,nT);
h_sol = reshape(x_sol(cdfkp.h_inds),1,[]);
t_sol = cumsum([0 h_sol]);
com_sol = reshape(x_sol(cdfkp.com_inds),3,[]);
comdot_sol = reshape(x_sol(cdfkp.comdot_inds),3,[]);
comddot_sol = reshape(x_sol(cdfkp.comddot_inds),3,[]);
H_sol = reshape(x_sol(cdfkp.H_inds),3,[]);
Hdot_sol = reshape(x_sol(cdfkp.Hdot_inds),3,[]);
lambda_sol = cell(2,1);
lambda_sol{1} = reshape(x_sol(cdfkp.lambda_inds{1}),size(cdfkp.lambda_inds{1},1),[],nT);
lambda_sol{2} = reshape(x_sol(cdfkp.lambda_inds{1}),size(cdfkp.lambda_inds{2},1),[],nT);
xtraj_sol = PPTrajectory(foh(cumsum([0 h_sol]),[q_sol;v_sol]));
xtraj_sol = xtraj_sol.setOutputFrame(robot.getStateFrame);
keyboard;
end

function [h,dh] = comApexHeight(takeoff_height,takeoff_vel)
g = 9.81;
h = takeoff_height+takeoff_vel^2/(2*g);
dh = [1 takeoff_vel/g];
end

function [h,dh] = comApexHeightCost(takeoff_height,takeoff_vel)
scaler = -200;
[h1,dh1] = comApexHeight(takeoff_height,takeoff_vel);
h = scaler*h1;
dh = scaler*dh1;
end

function symmetry_cnstr = symmetryConstraint(robot,t_idx)
N = numel(t_idx);
num_symmetry = 10;
symmetric_matrix = zeros(num_symmetry,robot.getNumPositions);
l_arm_usy_idx = robot.getBody(robot.findJointInd('l_arm_usy')).dofnum;
r_arm_usy_idx = robot.getBody(robot.findJointInd('r_arm_usy')).dofnum;
symmetric_matrix(1,[l_arm_usy_idx r_arm_usy_idx]) = [1 -1];
l_arm_shx_idx = robot.getBody(robot.findJointInd('l_arm_shx')).dofnum;
r_arm_shx_idx = robot.getBody(robot.findJointInd('r_arm_shx')).dofnum;
symmetric_matrix(2,[l_arm_shx_idx r_arm_shx_idx]) = [1 1];
l_arm_ely_idx = robot.getBody(robot.findJointInd('l_arm_ely')).dofnum;
r_arm_ely_idx = robot.getBody(robot.findJointInd('r_arm_ely')).dofnum;
symmetric_matrix(3,[l_arm_ely_idx r_arm_ely_idx]) = [1 -1];
l_arm_elx_idx = robot.getBody(robot.findJointInd('l_arm_elx')).dofnum;
r_arm_elx_idx = robot.getBody(robot.findJointInd('r_arm_elx')).dofnum;
symmetric_matrix(4,[l_arm_elx_idx r_arm_elx_idx]) = [1 1];
l_arm_uwy_idx = robot.getBody(robot.findJointInd('l_arm_uwy')).dofnum;
r_arm_uwy_idx = robot.getBody(robot.findJointInd('r_arm_uwy')).dofnum;
symmetric_matrix(5,[l_arm_uwy_idx r_arm_uwy_idx]) = [1 -1];
l_leg_hpz_idx = robot.getBody(robot.findJointInd('l_leg_hpz')).dofnum;
r_leg_hpz_idx = robot.getBody(robot.findJointInd('r_leg_hpz')).dofnum;
symmetric_matrix(6,[l_leg_hpz_idx r_leg_hpz_idx]) = [1 1];
l_leg_hpx_idx = robot.getBody(robot.findJointInd('l_leg_hpx')).dofnum;
r_leg_hpx_idx = robot.getBody(robot.findJointInd('r_leg_hpx')).dofnum;
symmetric_matrix(7,[l_leg_hpx_idx r_leg_hpx_idx]) = [1 1];
l_leg_hpy_idx = robot.getBody(robot.findJointInd('l_leg_hpy')).dofnum;
r_leg_hpy_idx = robot.getBody(robot.findJointInd('r_leg_hpy')).dofnum;
symmetric_matrix(8,[l_leg_hpy_idx r_leg_hpy_idx]) = [1 -1];
l_leg_aky_idx = robot.getBody(robot.findJointInd('l_leg_aky')).dofnum;
r_leg_aky_idx = robot.getBody(robot.findJointInd('r_leg_aky')).dofnum;
symmetric_matrix(9,[l_leg_aky_idx r_leg_aky_idx]) = [1 -1];
l_leg_kny_idx = robot.getBody(robot.findJointInd('l_leg_kny')).dofnum;
r_leg_kny_idx = robot.getBody(robot.findJointInd('r_leg_kny')).dofnum;
symmetric_matrix(10,[l_leg_kny_idx r_leg_kny_idx]) = [1 -1];

symmetry_cnstr = LinearConstraint(zeros(num_symmetry*N,1),zeros(num_symmetry*N,1),kron(speye(N),symmetric_matrix));
cnstr_name = cell(num_symmetry*N,1);
for i = 1:N
  cnstr_name{(i-1)*num_symmetry+1} = sprintf('arm_usy_symmetry[%d]',t_idx(i));
  cnstr_name{(i-1)*num_symmetry+2} = sprintf('arm_shx_symmetry[%d]',t_idx(i));
  cnstr_name{(i-1)*num_symmetry+3} = sprintf('arm_ely_symmetry[%d]',t_idx(i));
  cnstr_name{(i-1)*num_symmetry+4} = sprintf('arm_elx_symmetry[%d]',t_idx(i));
  cnstr_name{(i-1)*num_symmetry+5} = sprintf('arm_uwy_symmetry[%d]',t_idx(i));
  cnstr_name{(i-1)*num_symmetry+6} = sprintf('leg_hpz_symmetry[%d]',t_idx(i));
  cnstr_name{(i-1)*num_symmetry+7} = sprintf('leg_hpx_symmetry[%d]',t_idx(i));
  cnstr_name{(i-1)*num_symmetry+8} = sprintf('leg_hpy_symmetry[%d]',t_idx(i));
  cnstr_name{(i-1)*num_symmetry+9} = sprintf('leg_aky_symmetry[%d]',t_idx(i));
  cnstr_name{(i-1)*num_symmetry+10} = sprintf('leg_kyy_symmetry[%d]',t_idx(i));
end
symmetry_cnstr = symmetry_cnstr.setName(cnstr_name);
end