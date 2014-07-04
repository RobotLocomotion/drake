function testMonkeyBar()
checkDependency('lcmgl');
w = warning('off','Drake:RigidBody:SimplifiedCollisionGeometry');
warning('off','Drake:RigidBody:NonPositiveInertiaMatrix');
warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints');
warning('off','Drake:RigidBodyManipulator:UnsupportedJointLimits');
warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits');
robot = RigidBodyManipulator([getDrakePath,'/examples/Atlas/urdf/atlas_convex_hull.urdf'],struct('floating',true));
monkeybar_urdf = [getDrakePath,'/solvers/trajectoryOptimization/test/MonkeyBar.urdf'];
monkeybar_pos1 = [0;0;2.5;pi/2;0;-pi/2];
robot = robot.addRobotFromURDF(monkeybar_urdf,monkeybar_pos1(1:3),monkeybar_pos1(4:6));
% The monkey bar is a box with size bar_length x bar_width x bar_width
bar_length = 2;
bar_width = 0.05;
v = robot.constructVisualizer();

l_hand = robot.findLinkInd('l_hand');
r_hand = robot.findLinkInd('r_hand');

nomdata = load([getDrakePath,'/examples/Atlas/data/atlas_fp.mat']);
nq = robot.getNumPositions();
nv = robot.getNumVelocities();
qstar = nomdata.xstar(1:nq);
v.draw(0,nomdata.xstar);
l_hand_pt = [0;0.25;0];
r_hand_pt = [0;-0.25;0];
l_hand_grasp_axis = [0;0;1];
r_hand_grasp_axis = [0;0;-1];

l_arm_shx = robot.getBody(robot.findJointInd('l_arm_shx')).dofnum;
r_arm_shx = robot.getBody(robot.findJointInd('r_arm_shx')).dofnum;

%%
% Find the initial fix point first
nu = robot.getNumInputs();
robot_mass = robot.getMass();

fixed_pt_search = NonlinearProgramWConstraintObjects(nq+nu+6);
fixed_pt_search = fixed_pt_search.addSharedDataFunction(@(q) doKinematics(robot,q,true,true),(1:nq));
[lhand_grasp_cnstr1,lhand_grasp_cnstr2] = generateBarGraspConstraint(robot,l_hand,l_hand_pt,l_hand_grasp_axis,monkeybar_pos1,bar_length,bar_width);
lhand_grasp_cnstr1 = lhand_grasp_cnstr1.generateConstraint([]);
lhand_grasp_cnstr2 = lhand_grasp_cnstr2.generateConstraint([]);
lhand_grasp_cnstr1 = lhand_grasp_cnstr1{1};
lhand_grasp_cnstr2 = lhand_grasp_cnstr2{1};
[rhand_grasp_cnstr1,rhand_grasp_cnstr2] = generateBarGraspConstraint(robot,r_hand,r_hand_pt,r_hand_grasp_axis,monkeybar_pos1,bar_length,bar_width);
rhand_grasp_cnstr1 = rhand_grasp_cnstr1.generateConstraint([]);
rhand_grasp_cnstr2 = rhand_grasp_cnstr2.generateConstraint([]);
rhand_grasp_cnstr1 = rhand_grasp_cnstr1{1};
rhand_grasp_cnstr2 = rhand_grasp_cnstr2{1};
fixed_pt_search = fixed_pt_search.addDifferentiableConstraint(lhand_grasp_cnstr1,(1:nq),1);
fixed_pt_search = fixed_pt_search.addDifferentiableConstraint(lhand_grasp_cnstr2,(1:nq),1);
fixed_pt_search = fixed_pt_search.addDifferentiableConstraint(rhand_grasp_cnstr1,(1:nq),1);
fixed_pt_search = fixed_pt_search.addDifferentiableConstraint(rhand_grasp_cnstr2,(1:nq),1);
com_cnstr = generateComConstraint(robot,monkeybar_pos1);
com_cnstr = com_cnstr.generateConstraint([]);
com_cnstr = com_cnstr{1};
fixed_pt_search = fixed_pt_search.addDifferentiableConstraint(com_cnstr,(1:nq)',1);
fixed_pt_search = fixed_pt_search.addDifferentiableConstraint(FunctionHandleConstraint(zeros(nq,1),zeros(nq,1),nq+nu+6,@(q,u,l_hand_force,r_hand_force,kinsol) fixedPointDynamics(robot,q,u,l_hand_force,r_hand_force,kinsol,l_hand,l_hand_pt,r_hand,r_hand_pt)),{(1:nq)';nq+(1:nu)';nq+nu+(1:3)';nq+nu+3+(1:3)'},1);
input_coordinates = robot.getInputFrame.coordinates;
l_leg_kny_motor = (strcmp(input_coordinates,'l_leg_kny_motor'));
r_leg_kny_motor = (strcmp(input_coordinates,'r_leg_kny_motor'));
l_leg_aky_motor = (strcmp(input_coordinates,'l_leg_aky_motor'));
r_leg_aky_motor = (strcmp(input_coordinates,'r_leg_aky_motor'));
l_leg_akx_motor = (strcmp(input_coordinates,'l_leg_akx_motor'));
r_leg_akx_motor = (strcmp(input_coordinates,'r_leg_akx_motor'));
R = ones(nu,1);
R(l_leg_kny_motor) = 100;
R(r_leg_kny_motor) = 100;
R(l_leg_aky_motor) = 100;
R(r_leg_aky_motor) = 100;
R(l_leg_akx_motor) = 1000;
R(r_leg_akx_motor) = 1000;
R = diag(R);
fixed_pt_search = fixed_pt_search.addCost(QuadraticConstraint(-inf,inf,R,zeros(nu,1)),nq+(1:nu));
[q_lb,q_ub] = robot.getJointLimits();
fixed_pt_search = fixed_pt_search.addBoundingBoxConstraint(BoundingBoxConstraint(q_lb,q_ub),(1:nq)');
leg_symmetry = legSymmetricConstraint(robot);
fixed_pt_search = fixed_pt_search.addLinearConstraint(leg_symmetry,(1:nq)');
fixed_pt_search = fixed_pt_search.addBoundingBoxConstraint(ConstantConstraint(0),6);
q_fp_seed = qstar;
q_fp_seed(l_arm_shx) = pi/2;
q_fp_seed(r_arm_shx) = -pi/2;
fixed_pt_search = fixed_pt_search.setSolverOptions('snopt','majoriterationslimit',2000);
[x_fp,F,info] = fixed_pt_search.solve([q_fp_seed;zeros(nu,1);[0;0;robot_mass*9.81/2];[0;0;robot_mass*9.81/2]]);
q_fp = x_fp(1:nq);
q_fp = min([q_fp q_ub],[],2);
q_fp = max([q_fp q_lb],[],2);
u_fp = x_fp(nq+(1:nu));
l_hand_force_fp = x_fp(nq+nu+(1:3));
r_hand_force_fp = x_fp(nq+nu+3+(1:3));
kinsol_fp = robot.doKinematics(q_fp);
com_fp = robot.getCOM(kinsol_fp);
l_hand_pos_fp = robot.forwardKin(kinsol_fp,l_hand,l_hand_pt,2);
r_hand_pos_fp = robot.forwardKin(kinsol_fp,r_hand,r_hand_pt,2);
%%
% add monkeybar 
monkeybar_pos2 = monkeybar_pos1+[0.5;0;0;0;0;0];
robot = robot.addRobotFromURDF(monkeybar_urdf,monkeybar_pos2(1:3),monkeybar_pos2(4:6));
v = robot.constructVisualizer();
v.draw(0,[q_fp;zeros(nv,1)]);

l_hand_ungrasp_idx1 = 3;
l_hand_grasp_idx2 = 6;
r_hand_ungrasp_idx1 = 9;
r_hand_grasp_idx2 = 12;
nT = 15;

lhand_bar1_grasp_wrench = generateBarGraspingWrench(robot,l_hand,l_hand_pt,monkeybar_pos1);
rhand_bar1_grasp_wrench = generateBarGraspingWrench(robot,r_hand,r_hand_pt,monkeybar_pos1);
lhand_bar2_grasp_wrench = generateBarGraspingWrench(robot,l_hand,l_hand_pt,monkeybar_pos2);
rhand_bar2_grasp_wrench = generateBarGraspingWrench(robot,r_hand,r_hand_pt,monkeybar_pos2);

contact_wrench_struct = struct('active_knot',[],'cw',[]);
contact_wrench_struct(1) = struct('active_knot',1:l_hand_ungrasp_idx1,'cw',lhand_bar1_grasp_wrench);
contact_wrench_struct(2) = struct('active_knot',l_hand_grasp_idx2:nT,'cw',lhand_bar2_grasp_wrench);
contact_wrench_struct(3) = struct('active_knot',1:r_hand_ungrasp_idx1,'cw',rhand_bar1_grasp_wrench);
contact_wrench_struct(4) = struct('active_knot',r_hand_grasp_idx2:nT,'cw',rhand_bar2_grasp_wrench);


tf_range = [0.5 2];
q_nom = bsxfun(@times,q_fp,ones(1,nT));
Q_comddot = eye(3);
Q = eye(nq);
Q(1,1) = 0;
Q(2,2) = 0;
Qv = 0.1*eye(nv);
% Q_contact_force = 20/(robot.getMass*9.81)^2*eye(3);
Q_contact_force = zeros(3);
cdfkp = ComDynamicsFullKinematicsPlanner(robot,nT,tf_range,Q_comddot,Qv,Q,q_nom,Q_contact_force,contact_wrench_struct);


[~,lhand_bar1_grasp_dir_cnstr] = generateBarGraspConstraint(robot,l_hand,l_hand_pt,l_hand_grasp_axis,monkeybar_pos1,bar_length,bar_width);
lhand_bar1_grasp_pt_cnstr = WorldPositionConstraint(robot,l_hand,l_hand_pt,l_hand_pos_fp(1:3),l_hand_pos_fp(1:3));
cdfkp = cdfkp.addRigidBodyConstraint(lhand_bar1_grasp_pt_cnstr,num2cell(2:l_hand_ungrasp_idx1));
cdfkp = cdfkp.addRigidBodyConstraint(lhand_bar1_grasp_dir_cnstr,num2cell(2:l_hand_ungrasp_idx1));

[~,rhand_bar1_grasp_dir_cnstr] = generateBarGraspConstraint(robot,r_hand,r_hand_pt,r_hand_grasp_axis,monkeybar_pos1,bar_length,bar_width);
rhand_bar1_grasp_pt_cnstr = WorldPositionConstraint(robot,r_hand,r_hand_pt,r_hand_pos_fp(1:3),r_hand_pos_fp(1:3));
cdfkp = cdfkp.addRigidBodyConstraint(rhand_bar1_grasp_pt_cnstr,num2cell(1:r_hand_ungrasp_idx1));
cdfkp = cdfkp.addRigidBodyConstraint(rhand_bar1_grasp_dir_cnstr,num2cell(1:r_hand_ungrasp_idx1));

[lhand_bar2_grasp_pt_cnstr,lhand_bar2_grasp_dir_cnstr,lhand_bar2_fixed_pos_cnstr] = generateBarGraspConstraint(robot,l_hand,l_hand_pt,l_hand_grasp_axis,monkeybar_pos2,bar_length,bar_width);
cdfkp = cdfkp.addRigidBodyConstraint(lhand_bar2_grasp_pt_cnstr,{l_hand_grasp_idx2});
cdfkp = cdfkp.addRigidBodyConstraint(lhand_bar2_grasp_dir_cnstr,num2cell(l_hand_grasp_idx2:nT));
cdfkp = cdfkp.addRigidBodyConstraint(lhand_bar2_fixed_pos_cnstr,{l_hand_grasp_idx2:nT});

[rhand_bar2_grasp_pt_cnstr,rhand_bar2_grasp_dir_cnstr,rhand_bar2_fixed_pos_cnstr] = generateBarGraspConstraint(robot,r_hand,r_hand_pt,r_hand_grasp_axis,monkeybar_pos2,bar_length,bar_width);
cdfkp = cdfkp.addRigidBodyConstraint(rhand_bar2_grasp_pt_cnstr,{r_hand_grasp_idx2});
cdfkp = cdfkp.addRigidBodyConstraint(rhand_bar2_grasp_dir_cnstr,num2cell(r_hand_grasp_idx2:nT));
cdfkp = cdfkp.addRigidBodyConstraint(rhand_bar2_fixed_pos_cnstr,{r_hand_grasp_idx2:nT});

% fix initial and final state
cdfkp = cdfkp.addBoundingBoxConstraint(ConstantConstraint(q_fp),cdfkp.q_inds(:,1));
cdfkp = cdfkp.addBoundingBoxConstraint(ConstantConstraint(q_fp(7:end)),cdfkp.q_inds(7:end,nT));
cdfkp = cdfkp.addBoundingBoxConstraint(ConstantConstraint(zeros(nv,1)),cdfkp.v_inds(:,1));
cdfkp = cdfkp.addBoundingBoxConstraint(ConstantConstraint(zeros(nv,1)),cdfkp.v_inds(:,nT));

% add h bounds
cdfkp = cdfkp.addBoundingBoxConstraint(BoundingBoxConstraint(0.02*ones(nT-1,1),0.2*ones(nT-1,1)),cdfkp.h_inds(:));

% add com height upper bound
cdfkp = cdfkp.addCoMBounds(1:nT,-inf(3,nT),[inf(2,nT);(monkeybar_pos1(3)-0.3)*ones(1,nT)]);

cdfkp = cdfkp.addBoundingBoxConstraint(BoundingBoxConstraint(...
  reshape(bsxfun(@times,[-9.81/2;-9.81/2;-9.81],ones(1,nT)),[],1),...
  reshape(bsxfun(@times,[9.81/2;9.81/2;9.81],ones(1,nT)),[],1)),cdfkp.comddot_inds(:));

x_seed = zeros(cdfkp.num_vars,1);
x_seed(cdfkp.h_inds) = 0.1;
x_seed(cdfkp.q_inds(:)) = reshape(bsxfun(@times,q_fp,ones(1,nT)),[],1);
x_seed(cdfkp.com_inds(:)) = reshape(bsxfun(@times,com_fp,ones(1,nT)),[],1);
x_seed(cdfkp.lambda_inds{1}(:)) = reshape(bsxfun(@times,[l_hand_force_fp;zeros(3,1)],ones(1,nT)),[],1);
x_seed(cdfkp.lambda_inds{2}(:)) = reshape(bsxfun(@times,[r_hand_force_fp;zeros(3,1)],ones(1,nT)),[],1);

cdfkp = cdfkp.setSolverOptions('snopt','iterationslimit',1e6);
cdfkp = cdfkp.setSolverOptions('snopt','majoriterationslimit',2000);
cdfkp = cdfkp.setSolverOptions('snopt','majorfeasibilitytolerance',1e-6);
cdfkp = cdfkp.setSolverOptions('snopt','majoroptimalitytolerance',2e-4);
cdfkp = cdfkp.setSolverOptions('snopt','superbasicslimit',2000);
cdfkp = cdfkp.setSolverOptions('snopt','print','test_monkeybar.out');

seed_sol = load('test_monkeybar1','-mat','x_sol');
tic
[x_sol,F,info] = cdfkp.solve(seed_sol.x_sol);
toc;
q_sol = reshape(x_sol(cdfkp.q_inds(:)),nq,nT);
v_sol = reshape(x_sol(cdfkp.v_inds(:)),nq,nT);
h_sol = reshape(x_sol(cdfkp.h_inds),1,[]);
t_sol = cumsum([0 h_sol]);
com_sol = reshape(x_sol(cdfkp.com_inds),3,[]);
comdot_sol = reshape(x_sol(cdfkp.comdot_inds),3,[]);
comddot_sol = reshape(x_sol(cdfkp.comddot_inds),3,[]);
H_sol = reshape(x_sol(cdfkp.H_inds),3,[]);
Hdot_sol = reshape(x_sol(cdfkp.Hdot_inds),3,[])*cdfkp.torque_multiplier;
lambda_sol = cell(2,1);
lambda_sol{1} = reshape(x_sol(cdfkp.lambda_inds{1}),size(cdfkp.lambda_inds{1},1),[],nT);
lambda_sol{2} = reshape(x_sol(cdfkp.lambda_inds{2}),size(cdfkp.lambda_inds{2},1),[],nT);
xtraj_sol = PPTrajectory(foh(cumsum([0 h_sol]),[q_sol;v_sol]));
xtraj_sol = xtraj_sol.setOutputFrame(robot.getStateFrame);
wrench_sol = cdfkp.contactWrench(x_sol);
keyboard
end

function [grasp_pt_cnstr,grasp_dir_cnstr,grasp_fixed_pos_cnstr] = generateBarGraspConstraint(robot,hand_idx,hand_pt,hand_grasp_axis,bar_pos,bar_length,bar_width)
T_bar_to_world = [rpy2rotmat(bar_pos(4:6)) bar_pos(1:3);0 0 0 1];
bar_axis = [1;0;0];
world_bar_axis = [eye(3) zeros(3,1)]*T_bar_to_world*[bar_axis;0];
grasp_pt_cnstr = WorldPositionInFrameConstraint(robot,hand_idx,hand_pt,T_bar_to_world,[-bar_length*0.45;0;0],[bar_length*0.45;0;0]);
grasp_cone_threshold = pi/6;
grasp_dir_cnstr = WorldGazeDirConstraint(robot,hand_idx,hand_grasp_axis,world_bar_axis,grasp_cone_threshold);
grasp_fixed_pos_cnstr = WorldFixedPositionConstraint(robot,hand_idx,hand_pt);
end

function com_cnstr = generateComConstraint(robot,bar_pos)
com_ub = [-inf(2,1);bar_pos(3)-0.4];
com_cnstr = WorldCoMConstraint(robot,-inf(3,1),com_ub);
end

function [c,dc] = fixedPointDynamics(robot,q,u,l_hand_force,r_hand_force,kinsol,l_hand,l_hand_pt,r_hand,r_hand_pt)
nq = robot.getNumPositions();
v = zeros(robot.getNumVelocities,1);
[~,C,~,dC] = HandCmex(robot.getMexModelPtr,q,v);
[~,J_lhand,dJ_lhand] = forwardKin(robot,kinsol,l_hand,l_hand_pt,0);
dJ_lhand = reshape(dJ_lhand,numel(J_lhand),nq);
[~,J_rhand,dJ_rhand] = forwardKin(robot,kinsol,r_hand,r_hand_pt,0);
dJ_rhand = reshape(dJ_rhand,numel(J_rhand),nq);
c = C-robot.B*u-J_lhand'*l_hand_force-J_rhand'*r_hand_force;
dcdq = dC(:,1:nq)-matGradMult(dJ_lhand,l_hand_force,true)-matGradMult(dJ_rhand,r_hand_force,true);
dcdu = -robot.B;
dc = [dcdq dcdu -J_lhand' -J_rhand'];
end

function cnstr = legSymmetricConstraint(robot)
num_cnstr = 6;
nq = robot.getNumPositions();
A = zeros(num_cnstr,nq);
l_leg_hpz = robot.getBody(robot.findJointInd('l_leg_hpz')).dofnum;
r_leg_hpz = robot.getBody(robot.findJointInd('r_leg_hpz')).dofnum;
A(1,[l_leg_hpz r_leg_hpz]) = [1 1];
l_leg_hpx = robot.getBody(robot.findJointInd('l_leg_hpx')).dofnum;
r_leg_hpx = robot.getBody(robot.findJointInd('r_leg_hpx')).dofnum;
A(2,[l_leg_hpx r_leg_hpx]) = [1 1];
l_leg_hpy = robot.getBody(robot.findJointInd('l_leg_hpy')).dofnum;
r_leg_hpy = robot.getBody(robot.findJointInd('r_leg_hpy')).dofnum;
A(3,[l_leg_hpy r_leg_hpy]) = [1 -1];
l_leg_kny = robot.getBody(robot.findJointInd('l_leg_kny')).dofnum;
r_leg_kny = robot.getBody(robot.findJointInd('r_leg_kny')).dofnum;
A(4,[l_leg_kny r_leg_kny]) = [1 -1];
l_leg_aky = robot.getBody(robot.findJointInd('l_leg_aky')).dofnum;
r_leg_aky = robot.getBody(robot.findJointInd('r_leg_aky')).dofnum;
A(5,[l_leg_aky r_leg_aky]) = [1 -1];
l_leg_akx = robot.getBody(robot.findJointInd('l_leg_akx')).dofnum;
r_leg_akx = robot.getBody(robot.findJointInd('r_leg_akx')).dofnum;
A(6,[l_leg_akx r_leg_akx]) = [1 1];
cnstr = LinearConstraint(zeros(num_cnstr,1),zeros(num_cnstr,1),A);
cnstr_name = cell(num_cnstr,1);
cnstr_name{1} = 'l_leg_hpz symmetry';
cnstr_name{2} = 'l_leg_hpx symmetry';
cnstr_name{3} = 'l_leg_hpy symmetry';
cnstr_name{4} = 'l_leg_kny symmetry';
cnstr_name{5} = 'l_leg_aky symmetry';
cnstr_name{6} = 'l_leg_akx symmetry';
cnstr = cnstr.setName(cnstr_name);
end

function wrench = generateBarGraspingWrench(robot,hand,hand_pt,bar_pos)
robot_mass = robot.getMass;
g = 9.81;
force_max = robot_mass*g*1.5;
torque_axis_max = robot_mass*g;
torque_perp_max = robot_mass*g*0.1;
A_torque = eye(3);
b_torque_ub = [torque_axis_max;torque_perp_max;torque_perp_max];
b_torque_lb = -b_torque_ub;
T_bar_to_world = [rpy2rotmat(bar_pos(4:6)) bar_pos(1:3);0 0 0 1];
A_torque = A_torque*T_bar_to_world(1:3,1:3)';
wrench = GraspWrench(robot,hand,hand_pt,force_max,A_torque,b_torque_lb,b_torque_ub);
end