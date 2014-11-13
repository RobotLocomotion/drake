function testMonkeyBar(mode,ncp_tol)
% mode 1   jump off
% mode 2   swing1
% mode 3   landing
% mode 4   swing2
% mode 5   swing3
% mode 6   landing with complementarity constraint
% mode 7   swing4
% ncp_tol  The tolerance on nonlinear complementarity constraint
% mode 8   start swing
if(nargin<2)
  ncp_tol = 0;
end
checkDependency('lcmgl');
w = warning('off','Drake:RigidBody:SimplifiedCollisionGeometry');
warning('off','Drake:RigidBody:NonPositiveInertiaMatrix');
warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints');
warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits');
robot = RigidBodyManipulator([getDrakePath,'/examples/Atlas/urdf/atlas_convex_hull.urdf'],struct('floating',true));
monkeybar_urdf1 = [getDrakePath,'/solvers/trajectoryOptimization/test/MonkeyBar1.urdf'];
monkeybar_urdf2 = [getDrakePath,'/solvers/trajectoryOptimization/test/MonkeyBar2.urdf'];
monkeybar_urdf3 = [getDrakePath,'/solvers/trajectoryOptimization/test/MonkeyBar3.urdf'];
monkeybar_urdf4 = [getDrakePath,'/solvers/trajectoryOptimization/test/MonkeyBar4.urdf'];
monkeybar_urdf5 = [getDrakePath,'/solvers/trajectoryOptimization/test/MonkeyBar5.urdf'];
monkeybar_pos1 = [0;0;2.5;pi/2;0;-pi/2];
robot = robot.addRobotFromURDF(monkeybar_urdf1,monkeybar_pos1(1:3),monkeybar_pos1(4:6));
% The monkey bar is a box with size bar_length x bar_width x bar_width
bar_length = 2;
bar_width = 0.05;
v = robot.constructVisualizer();

l_hand = robot.findLinkInd('l_hand');
r_hand = robot.findLinkInd('r_hand');
l_foot = robot.findLinkInd('l_foot');
r_foot = robot.findLinkInd('r_foot');
utorso = robot.findLinkInd('utorso');
l_foot_geometry_heel = robot.getBody(l_foot).getCollisionGeometry('heel');
l_foot_geometry_toe = robot.getBody(l_foot).getCollisionGeometry('toe');
r_foot_geometry_heel = robot.getBody(r_foot).getCollisionGeometry('heel');
r_foot_geometry_toe = robot.getBody(r_foot).getCollisionGeometry('toe');
l_foot_toe = [];
l_foot_heel = [];
r_foot_toe = [];
r_foot_heel = [];
for i = 1:length(l_foot_geometry_heel)
  l_foot_heel = [l_foot_heel l_foot_geometry_heel{i}.getPoints];
end
for i = 1:length(l_foot_geometry_toe)
  l_foot_toe = [l_foot_toe l_foot_geometry_toe{i}.getPoints];
end
for i = 1:length(r_foot_geometry_heel)
  r_foot_heel = [r_foot_heel r_foot_geometry_heel{i}.getPoints];
end
for i = 1:length(r_foot_geometry_toe)
  r_foot_toe = [r_foot_toe r_foot_geometry_toe{i}.getPoints];
end
l_foot_bottom = [l_foot_toe l_foot_heel];
r_foot_bottom = [r_foot_toe r_foot_heel];

nomdata = load([getDrakePath,'/examples/Atlas/data/atlas_fp.mat']);
nq = robot.getNumPositions();
nv = robot.getNumVelocities();
qstar = nomdata.xstar(1:nq);
kinsol_star = robot.doKinematics(qstar);
lfoot_pos_star = robot.forwardKin(kinsol_star,l_foot,[0;0;0],2);
rfoot_pos_star = robot.forwardKin(kinsol_star,r_foot,[0;0;0],2);
v.draw(0,nomdata.xstar);
l_hand_pt = [0;0.25;0];
r_hand_pt = [0;-0.25;0];
l_hand_grasp_axis = [0;0;1];
r_hand_grasp_axis = [0;0;-1];

l_arm_shx = robot.getBody(robot.findJointInd('l_arm_shx')).position_num;
r_arm_shx = robot.getBody(robot.findJointInd('r_arm_shx')).position_num;
l_arm_elx = robot.getBody(robot.findJointInd('l_arm_elx')).position_num;
r_arm_elx = robot.getBody(robot.findJointInd('r_arm_elx')).position_num;
l_leg_hpz = robot.getBody(robot.findJointInd('l_leg_hpz')).position_num;
r_leg_hpz = robot.getBody(robot.findJointInd('r_leg_hpz')).position_num;
l_arm_uwy = robot.getBody(robot.findJointInd('l_arm_uwy')).position_num;
r_arm_uwy = robot.getBody(robot.findJointInd('r_arm_uwy')).position_num;
back_bkx = robot.getBody(robot.findJointInd('back_bkx')).position_num;
back_bky = robot.getBody(robot.findJointInd('back_bky')).position_num;
back_bkz = robot.getBody(robot.findJointInd('back_bkz')).position_num;
l_leg_kny = robot.getBody(robot.findJointInd('l_leg_kny')).position_num;
r_leg_kny = robot.getBody(robot.findJointInd('r_leg_kny')).position_num;
l_leg_hpx = robot.getBody(robot.findJointInd('l_leg_hpx')).position_num;
r_leg_hpx = robot.getBody(robot.findJointInd('r_leg_hpx')).position_num;
r_leg_aky = robot.getBody(robot.findJointInd('r_leg_aky')).position_num;
l_leg_aky = robot.getBody(robot.findJointInd('l_leg_aky')).position_num;
%%
% Find the initial fix point first
nu = robot.getNumInputs();
robot_mass = robot.getMass();

fixed_pt_search = NonlinearProgram(nq+nu+6);
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
R(l_leg_akx_motor) = 100;
R(r_leg_akx_motor) = 100;
R = diag(R);
fixed_pt_search = fixed_pt_search.addCost(QuadraticConstraint(-inf,inf,R,zeros(nu,1)),nq+(1:nu));
[q_lb,q_ub] = robot.getJointLimits();
fixed_pt_search = fixed_pt_search.addBoundingBoxConstraint(BoundingBoxConstraint(q_lb,q_ub),(1:nq)');
leg_symmetry = legSymmetricConstraint(robot,1);
fixed_pt_search = fixed_pt_search.addLinearConstraint(leg_symmetry,(1:nq)');
arm_symmetry = armSymmetricConstraint(robot,1);
fixed_pt_search = fixed_pt_search.addLinearConstraint(arm_symmetry,(1:nq)');
fixed_pt_search = fixed_pt_search.addBoundingBoxConstraint(ConstantConstraint(0),6);
fixed_pt_search = fixed_pt_search.addBoundingBoxConstraint(BoundingBoxConstraint([q_lb(l_arm_elx)+0.1;q_lb(r_arm_elx)],[q_ub(l_arm_elx);q_ub(r_arm_elx)-0.1]),[l_arm_elx;r_arm_elx]);
fixed_pt_search = fixed_pt_search.addDifferentiableConstraint(FunctionHandleConstraint(0.8,1.2,nq,@(~,kinsol) graspDistance(kinsol,robot,l_hand,r_hand,l_hand_pt,r_hand_pt)),{(1:nq)'},1);
fixed_pt_search = fixed_pt_search.addBoundingBoxConstraint(ConstantConstraint(monkeybar_pos1(2)),2);
fixed_pt_search = fixed_pt_search.addBoundingBoxConstraint(BoundingBoxConstraint([0;-0.2],[0.2;0]),[l_leg_hpz;r_leg_hpz]);
% fixed_pt_search = fixed_pt_search.addBoundingBoxConstraint(ConstantConstraint(zeros(2,1)),[l_arm_uwy;r_arm_uwy]);
% fixed_pt_search = fixed_pt_search.addDifferentiableConstraint(FunctionHandleConstraint(-inf(3,1),[inf(2,1);monkeybar_pos1(3)-1],nq,@(~,kinsol) comPosition(kinsol,robot)),{(1:nq)'},1);
fixed_pt_search = fixed_pt_search.addBoundingBoxConstraint(BoundingBoxConstraint(zeros(2,1),inf(2,1)),[l_leg_aky;r_leg_aky]);
q_fp_seed = qstar;
q_fp_seed(l_arm_shx) = pi/2;
q_fp_seed(r_arm_shx) = -pi/2;
fixed_pt_search = fixed_pt_search.setSolverOptions('snopt','majoriterationslimit',2000);
[x_fp,F,info] = fixed_pt_search.solve([q_fp_seed;zeros(nu,1);[0;0;robot_mass*9.81/2];[0;0;robot_mass*9.81/2]]);
active_collision_options.body_idx = [l_foot,r_foot];
active_collision_options.collision_groups = {'default'};
foot_nocollision_cnstr = MinDistanceConstraint(robot,0.02,[-inf,inf],active_collision_options);
foot_nocollision_cnstr = foot_nocollision_cnstr.generateConstraint();
foot_nocollision_cnstr = foot_nocollision_cnstr{1};
fixed_pt_search = fixed_pt_search.addDifferentiableConstraint(foot_nocollision_cnstr,(1:nq)',1);
[x_fp,F,info] = fixed_pt_search.solve(x_fp);
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
v.draw(0,[q_fp;zeros(nv,1)]);

stage_urdf = [getDrakePath,'/solvers/trajectoryOptimization/test/MonkeyBar_stage.urdf'];
jump_height = 2.6;
stage_pos1 = [monkeybar_pos1(1:3)-[1.2;0;jump_height+0.05];0;pi/2;0];
robot = robot.addRobotFromURDF(stage_urdf,stage_pos1(1:3),stage_pos1(4:6));
monkeybar_pos2 = monkeybar_pos1+[0.5;0;0;0;0;0];
monkeybar_pos3 = monkeybar_pos2+[0.4;0;0;0;0;0];
monkeybar_pos4 = monkeybar_pos3+[0.4;0;-0.1;0;0;0];
monkeybar_pos5 = monkeybar_pos4+[0.6;0;0;0;0;0];
robot = robot.addRobotFromURDF(monkeybar_urdf2,monkeybar_pos2(1:3),monkeybar_pos2(4:6));
robot = robot.addRobotFromURDF(monkeybar_urdf3,monkeybar_pos3(1:3),monkeybar_pos3(4:6));
robot = robot.addRobotFromURDF(monkeybar_urdf4,monkeybar_pos4(1:3),monkeybar_pos4(4:6));
robot = robot.addRobotFromURDF(monkeybar_urdf5,monkeybar_pos5(1:3),monkeybar_pos5(4:6));

land_height = 2.6;
stage_pos2 = [monkeybar_pos5(1:3)+[1.45;0;-land_height-0.05];0;-pi/2;0];
robot = robot.addRobotFromURDF(stage_urdf,stage_pos2(1:3),stage_pos2(4:6));
v = robot.constructVisualizer();
%%%%%%%%%%%%%%%%%%
%%
% add the takeoff stage
if(mode == 1)
  % find the starting position
  lfoot_pos_initial_cnstr = WorldPositionConstraint(robot,l_foot,[0;0;0],[nan(2,1);stage_pos1(3)+0.05+lfoot_pos_star(3)],[nan(2,1);stage_pos1(3)+0.05+lfoot_pos_star(3)]);
  rfoot_pos_initial_cnstr = WorldPositionConstraint(robot,r_foot,[0;0;0],[nan(2,1);stage_pos1(3)+0.05+rfoot_pos_star(3)],[nan(2,1);stage_pos1(3)+0.05+rfoot_pos_star(3)]);
  posture_cnstr = PostureConstraint(robot);
  posture_cnstr = posture_cnstr.setJointLimits((4:nq)',qstar(4:nq),qstar(4:nq));
  posture_cnstr = posture_cnstr.setJointLimits(1,monkeybar_pos1(1)-0.4,monkeybar_pos1(1)-0.4);
  ik_initial = InverseKin(robot,qstar,lfoot_pos_initial_cnstr,rfoot_pos_initial_cnstr,posture_cnstr);
  q_initial = ik_initial.solve(qstar);
  kinsol_initial = robot.doKinematics(q_initial);
  com_initial = robot.getCOM(kinsol_initial);
  lfoot_pos_initial = robot.forwardKin(kinsol_initial,l_foot,[0;0;0],2);
  rfoot_pos_initial = robot.forwardKin(kinsol_initial,r_foot,[0;0;0],2);
  lfoot_toe_pos_initial = robot.forwardKin(kinsol_initial,l_foot,l_foot_toe,0);
  rfoot_toe_pos_initial = robot.forwardKin(kinsol_initial,r_foot,r_foot_toe,0);
  lfoot_heel_pos_initial = robot.forwardKin(kinsol_initial,l_foot,l_foot_heel,0);
  rfoot_heel_pos_initial = robot.forwardKin(kinsol_initial,r_foot,r_foot_heel,0);
  v.draw(0,[q_initial;zeros(nv,1)]);
  % takeoff
  heel_takeoff_idx = 4;
  toe_takeoff_idx = 6;
  grasp_idx1 = 9;
  static_idx1 = 12;
  takeoff_contact_wrench_struct = struct('active_knot',[],'cw',[]);
  num_edges = 4;
  FC_theta = linspace(0,2*pi,num_edges+1);
  mu = 1;
  g = 9.81;
  FC_edge = robot_mass*g*[mu*[sin(FC_theta(1:num_edges));cos(FC_theta(1:num_edges))];ones(1,num_edges)];
  takeoff_contact_wrench_struct(1) = struct('active_knot',1:heel_takeoff_idx,'cw',LinearFrictionConeWrench(robot,l_foot,l_foot_bottom,FC_edge));
  takeoff_contact_wrench_struct(2) = struct('active_knot',1:heel_takeoff_idx,'cw',LinearFrictionConeWrench(robot,r_foot,r_foot_bottom,FC_edge));
  takeoff_contact_wrench_struct(3) = struct('active_knot',heel_takeoff_idx+1:toe_takeoff_idx,'cw',LinearFrictionConeWrench(robot,l_foot,l_foot_toe,FC_edge));
  takeoff_contact_wrench_struct(4) = struct('active_knot',heel_takeoff_idx+1:toe_takeoff_idx,'cw',LinearFrictionConeWrench(robot,r_foot,r_foot_toe,FC_edge));
  lhand_bar1_grasp_wrench = generateBarGraspingWrench(robot,l_hand,l_hand_pt,monkeybar_pos1);
  rhand_bar1_grasp_wrench = generateBarGraspingWrench(robot,r_hand,r_hand_pt,monkeybar_pos1);
  takeoff_contact_wrench_struct(5) = struct('active_knot',grasp_idx1:static_idx1,'cw',lhand_bar1_grasp_wrench);
  takeoff_contact_wrench_struct(6) = struct('active_knot',grasp_idx1:static_idx1,'cw',rhand_bar1_grasp_wrench);

  nT1 = static_idx1;
  tf_range = [1 3];
  q_nom = bsxfun(@times,qstar,ones(1,nT1));
  Q_comddot = eye(3);
  Q = zeros(nq);
  Qv = 1*eye(nv);
  Q_contact_force = 0/(robot_mass*g)^2*eye(3);
  cdfkp = ComDynamicsFullKinematicsPlanner(robot,nT1,tf_range,Q_comddot,Qv,Q,q_nom,Q_contact_force,takeoff_contact_wrench_struct);

  % add initial and final state constraint
  cdfkp = cdfkp.addBoundingBoxConstraint(ConstantConstraint(q_initial),cdfkp.q_inds(:,1));
  cdfkp = cdfkp.addBoundingBoxConstraint(ConstantConstraint(zeros(nv,1)),cdfkp.v_inds(:,1));
  cdfkp = cdfkp.addBoundingBoxConstraint(ConstantConstraint(q_fp),cdfkp.q_inds(:,static_idx1));
  cdfkp = cdfkp.addBoundingBoxConstraint(ConstantConstraint(zeros(nv,1)),cdfkp.v_inds(:,static_idx1));

  % add grasping kinematic constraint
  [~,lhand_bar1_grasp_dir_cnstr] = generateBarGraspConstraint(robot,l_hand,l_hand_pt,l_hand_grasp_axis,monkeybar_pos1,bar_length,bar_width);
  lhand_bar1_grasp_pt_cnstr = WorldPositionConstraint(robot,l_hand,l_hand_pt,l_hand_pos_fp(1:3),l_hand_pos_fp(1:3));
  [~,rhand_bar1_grasp_dir_cnstr] = generateBarGraspConstraint(robot,r_hand,r_hand_pt,r_hand_grasp_axis,monkeybar_pos1,bar_length,bar_width);
  rhand_bar1_grasp_pt_cnstr = WorldPositionConstraint(robot,r_hand,r_hand_pt,r_hand_pos_fp(1:3),r_hand_pos_fp(1:3));
  cdfkp = cdfkp.addRigidBodyConstraint(lhand_bar1_grasp_pt_cnstr,num2cell(grasp_idx1:static_idx1-1));
  cdfkp = cdfkp.addRigidBodyConstraint(rhand_bar1_grasp_pt_cnstr,num2cell(grasp_idx1:static_idx1-1));
  cdfkp = cdfkp.addRigidBodyConstraint(lhand_bar1_grasp_dir_cnstr,num2cell(grasp_idx1:static_idx1-1));
  cdfkp = cdfkp.addRigidBodyConstraint(rhand_bar1_grasp_dir_cnstr,num2cell(grasp_idx1:static_idx1-1));

  % add foot contact kinematic constraint
  cdfkp = cdfkp.addRigidBodyConstraint(WorldPositionConstraint(robot,l_foot,[0;0;0],lfoot_pos_initial(1:3),lfoot_pos_initial(1:3)),num2cell(2:heel_takeoff_idx));
  cdfkp = cdfkp.addRigidBodyConstraint(WorldPositionConstraint(robot,r_foot,[0;0;0],rfoot_pos_initial(1:3),rfoot_pos_initial(1:3)),num2cell(2:heel_takeoff_idx));
  cdfkp = cdfkp.addRigidBodyConstraint(WorldQuatConstraint(robot,l_foot,lfoot_pos_initial(4:7),0),num2cell(2:heel_takeoff_idx));
  cdfkp = cdfkp.addRigidBodyConstraint(WorldQuatConstraint(robot,r_foot,rfoot_pos_initial(4:7),0),num2cell(2:heel_takeoff_idx));
  cdfkp = cdfkp.addRigidBodyConstraint(WorldPositionConstraint(robot,l_foot,l_foot_toe,lfoot_toe_pos_initial(1:3,:),lfoot_toe_pos_initial(1:3,:)),num2cell(heel_takeoff_idx+1:toe_takeoff_idx));
  cdfkp = cdfkp.addRigidBodyConstraint(WorldPositionConstraint(robot,r_foot,r_foot_toe,rfoot_toe_pos_initial(1:3,:),rfoot_toe_pos_initial(1:3,:)),num2cell(heel_takeoff_idx+1:toe_takeoff_idx));

  cdfkp = cdfkp.addRigidBodyConstraint(WorldPositionConstraint(robot,l_foot,l_foot_heel,[nan(2,size(l_foot_heel,2));lfoot_heel_pos_initial(3,:)+0.01],nan(3,size(l_foot_heel,2))),num2cell(heel_takeoff_idx+1:grasp_idx1));
  cdfkp = cdfkp.addRigidBodyConstraint(WorldPositionConstraint(robot,r_foot,r_foot_heel,[nan(2,size(r_foot_heel,2));rfoot_heel_pos_initial(3,:)+0.01],nan(3,size(r_foot_heel,2))),num2cell(heel_takeoff_idx+1:grasp_idx1));
  cdfkp = cdfkp.addRigidBodyConstraint(WorldPositionConstraint(robot,l_foot,l_foot_toe,[nan(2,size(l_foot_toe,2));lfoot_toe_pos_initial(3,:)+0.01],nan(3,size(l_foot_toe,2))),num2cell(toe_takeoff_idx+1:grasp_idx1));
  cdfkp = cdfkp.addRigidBodyConstraint(WorldPositionConstraint(robot,r_foot,r_foot_toe,[nan(2,size(r_foot_toe,2));rfoot_toe_pos_initial(3,:)+0.01],nan(3,size(r_foot_toe,2))),num2cell(toe_takeoff_idx+1:grasp_idx1));
%   % arm symmetry
%   arm_symmetry = armSymmetricConstraint(robot,2:grasp_idx1);
%   cdfkp = cdfkp.addLinearConstraint(arm_symmetry,reshape(cdfkp.q_inds(:,2:grasp_idx1),[],1));
%
%   % leg symmetry
%   leg_symmetry = legSymmetricConstraint(robot,2:grasp_idx1);
%   % cdfkp = cdfkp.addLinearConstraint(leg_symmetry,reshape(cdfkp.q_inds(:,1:grasp_idx1),[],1));
%
  % time bounds
  cdfkp = cdfkp.addBoundingBoxConstraint(BoundingBoxConstraint(0.08*ones(nT1-1,1),0.2*ones(nT1-1,1)),cdfkp.h_inds(:));
  cdfkp = cdfkp.addBoundingBoxConstraint(BoundingBoxConstraint(0.1*ones(grasp_idx1-toe_takeoff_idx,1),0.2*ones(grasp_idx1-toe_takeoff_idx,1)),cdfkp.h_inds(toe_takeoff_idx:grasp_idx1-1)');

  % comddot bounds
  cdfkp = cdfkp.addBoundingBoxConstraint(BoundingBoxConstraint(...
    reshape(bsxfun(@times,[-9.81;-9.81;-1.5*9.81],ones(1,nT1)),[],1),...
    reshape(bsxfun(@times,[9.81;9.81;1.5*9.81],ones(1,nT1)),[],1)),cdfkp.comddot_inds(:));
%
%   % bound the pitch angle
%   cdfkp = cdfkp.addBoundingBoxConstraint(BoundingBoxConstraint(-0.15*ones(toe_takeoff_idx,1),0.3*ones(toe_takeoff_idx,1)),cdfkp.q_inds(5,1:toe_takeoff_idx)');
%
  % bound the position of the hand prior to grasping
  cdfkp = cdfkp.addRigidBodyConstraint(WorldPositionConstraint(robot,l_hand,l_hand_pt,nan(3,1),[monkeybar_pos1(1);nan(2,1)]),{grasp_idx1-1});
  cdfkp = cdfkp.addRigidBodyConstraint(WorldPositionConstraint(robot,r_hand,r_hand_pt,nan(3,1),[monkeybar_pos1(1);nan(2,1)]),{grasp_idx1-1});
%
%   % bound back_bkx (roll)
%   cdfkp = cdfkp.addBoundingBoxConstraint(BoundingBoxConstraint(-0.15*ones(grasp_idx1,1),0.2*ones(grasp_idx1,1)),cdfkp.q_inds(back_bkx,1:grasp_idx1)');
%
%   % bound back_bky (pitch)
%   cdfkp = cdfkp.addBoundingBoxConstraint(BoundingBoxConstraint(-0.2*ones(toe_takeoff_idx)
  % bend the knee in taking off
  knee_bend_idx = ceil((1+heel_takeoff_idx)/2);
  cdfkp = cdfkp.addBoundingBoxConstraint(BoundingBoxConstraint(q_initial(l_leg_kny)+0.4,inf),cdfkp.q_inds(l_leg_kny,knee_bend_idx));
  cdfkp = cdfkp.addBoundingBoxConstraint(BoundingBoxConstraint(q_initial(r_leg_kny)+0.4,inf),cdfkp.q_inds(r_leg_kny,knee_bend_idx));

  % bound the torso direction. To be as straight as possible
  torso_straight_cnstr = WorldGazeDirConstraint(robot,utorso,[0;0;1],[0;0;1],0.15);
  cdfkp = cdfkp.addRigidBodyConstraint(torso_straight_cnstr,num2cell(1:grasp_idx1));
%
  % bound back_bkz
  cdfkp = cdfkp.addBoundingBoxConstraint(BoundingBoxConstraint(-0.05*ones(nT1,1),0.05*ones(nT1,1)),cdfkp.q_inds(back_bkz,:)');

  % bound the velocity of back_bkz
  cdfkp = cdfkp.addBoundingBoxConstraint(BoundingBoxConstraint(-0.1*ones(nT1,1),0.1*ones(nT1,1)),cdfkp.v_inds(back_bkz,:)');

  % bound the yaw
  cdfkp = cdfkp.addBoundingBoxConstraint(BoundingBoxConstraint(-0.1*ones(nT1,1),0.1*ones(nT1,1)),cdfkp.q_inds(6,:)');

  % bound roll
  cdfkp = cdfkp.addBoundingBoxConstraint(BoundingBoxConstraint(-0.05*ones(nT1,1),0.05*ones(nT1,1)),cdfkp.q_inds(4,:)');
  cdfkp = cdfkp.addBoundingBoxConstraint(BoundingBoxConstraint(-0.1*ones(nT1,1),0.1*ones(nT1,1)),cdfkp.v_inds(4,:)');
%
  % bound leg_hpx
  cdfkp = cdfkp.addBoundingBoxConstraint(BoundingBoxConstraint(-0.07*ones(nT1,1),0.1*ones(nT1,1)),cdfkp.q_inds(l_leg_hpx,:)');
  cdfkp = cdfkp.addBoundingBoxConstraint(BoundingBoxConstraint(-0.07*ones(nT1,1),0.1*ones(nT1,1)),cdfkp.q_inds(r_leg_hpx,:)');

  x_seed = zeros(cdfkp.num_vars,1);
  x_seed(cdfkp.q_inds(:,1:toe_takeoff_idx)) = reshape(bsxfun(@times,q_initial,ones(1,toe_takeoff_idx)),[],1);
  x_seed(cdfkp.com_inds(:,1:toe_takeoff_idx)) = reshape(bsxfun(@times,com_initial,ones(1,toe_takeoff_idx)),[],1);
  x_seed(cdfkp.q_inds(:,toe_takeoff_idx+1:static_idx1)) = reshape(bsxfun(@times,q_fp,ones(1,static_idx1-toe_takeoff_idx)),[],1);
  x_seed(cdfkp.com_inds(:,toe_takeoff_idx+1:static_idx1)) = reshape(bsxfun(@times,com_fp,ones(1,static_idx1-toe_takeoff_idx)),[],1);
  x_seed(cdfkp.lambda_inds{1}(:,:,1:toe_takeoff_idx)) = reshape(1/(2*size(l_foot_bottom,2)*num_edges)*ones(num_edges,size(l_foot_bottom,2),toe_takeoff_idx),[],1);
  x_seed(cdfkp.lambda_inds{2}(:,:,1:toe_takeoff_idx)) = reshape(1/(2*size(r_foot_bottom,2)*num_edges)*ones(num_edges,size(r_foot_bottom,2),toe_takeoff_idx),[],1);
  x_seed(cdfkp.lambda_inds{3}(:,:,grasp_idx1:static_idx1)) = reshape(bsxfun(@times,[l_hand_force_fp;0;0;0],ones(1,static_idx1-grasp_idx1+1)),[],1);
  x_seed(cdfkp.lambda_inds{4}(:,:,grasp_idx1:static_idx1)) = reshape(bsxfun(@times,[r_hand_force_fp;0;0;0],ones(1,static_idx1-grasp_idx1+1)),[],1);

  cdfkp = cdfkp.setSolverOptions('snopt','iterationslimit',1e6);
  cdfkp = cdfkp.setSolverOptions('snopt','majoriterationslimit',3000);
  cdfkp = cdfkp.setSolverOptions('snopt','majorfeasibilitytolerance',1e-6);
  cdfkp = cdfkp.setSolverOptions('snopt','majoroptimalitytolerance',2e-4);
  cdfkp = cdfkp.setSolverOptions('snopt','superbasicslimit',2000);
  cdfkp = cdfkp.setSolverOptions('snopt','print','test_monkeybar_takeoff.out');

  seed_sol = load('test_monkeybar_takeoff','-mat','x_sol');
%   seed_sol.x_sol(cdfkp.lambda_inds{3}(:)) = seed_sol.x_sol(cdfkp.lambda_inds{3}(:))/(robot_mass*9.81);
%   seed_sol.x_sol(cdfkp.lambda_inds{4}(:)) = seed_sol.x_sol(cdfkp.lambda_inds{4}(:))/(robot_mass*9.81);
  tic
  [x_sol,F,info] = cdfkp.solve(seed_sol.x_sol);
  toc
  [q_sol,v_sol,h_sol,t_sol,com_sol,comdot_sol,comddot_sol,H_sol,Hdot_sol,lambda_sol,wrench_sol] = parseSolution(cdfkp,x_sol);
  xtraj_sol = PPTrajectory(foh(cumsum([0 h_sol]),[q_sol;v_sol]));
  xtraj_sol = xtraj_sol.setOutputFrame(robot.getStateFrame);
  keyboard;
end
%%
% add monkeybar
if(mode == 2)
% l_hand_ungrasp_idx1 = 3;
% l_hand_grasp_idx2 = 6;
% r_hand_ungrasp_idx1 = 9;
% r_hand_grasp_idx2 = 12;
% nT_swing1 = 15;
%
% lhand_bar1_grasp_wrench = generateBarGraspingWrench(robot,l_hand,l_hand_pt,monkeybar_pos1);
% rhand_bar1_grasp_wrench = generateBarGraspingWrench(robot,r_hand,r_hand_pt,monkeybar_pos1);
% lhand_bar2_grasp_wrench = generateBarGraspingWrench(robot,l_hand,l_hand_pt,monkeybar_pos2);
% rhand_bar2_grasp_wrench = generateBarGraspingWrench(robot,r_hand,r_hand_pt,monkeybar_pos2);
%
%
% bar_contact_wrench_struct = struct('active_knot',[],'cw',[]);
% bar_contact_wrench_struct(1) = struct('active_knot',1:l_hand_ungrasp_idx1,'cw',lhand_bar1_grasp_wrench);
% bar_contact_wrench_struct(2) = struct('active_knot',l_hand_grasp_idx2:nT_swing1,'cw',lhand_bar2_grasp_wrench);
% bar_contact_wrench_struct(3) = struct('active_knot',1:r_hand_ungrasp_idx1,'cw',rhand_bar1_grasp_wrench);
% bar_contact_wrench_struct(4) = struct('active_knot',r_hand_grasp_idx2:nT_swing1,'cw',rhand_bar2_grasp_wrench);
%
%
% tf_range = [0.5 2];
% q_nom = bsxfun(@times,q_fp,ones(1,nT_swing1));
% Q_comddot = eye(3);
% Q = eye(nq);
% Q(1,1) = 0;
% Q(2,2) = 0;
% Qv = 0.1*eye(nv);
% % Q_contact_force = 20/(robot.getMass*9.81)^2*eye(3);
% Q_contact_force = zeros(3);
% cdfkp = ComDynamicsFullKinematicsPlanner(robot,nT_swing1,tf_range,Q_comddot,Qv,Q,q_nom,Q_contact_force,bar_contact_wrench_struct);
%
%
% [~,lhand_bar1_grasp_dir_cnstr] = generateBarGraspConstraint(robot,l_hand,l_hand_pt,l_hand_grasp_axis,monkeybar_pos1,bar_length,bar_width);
% lhand_bar1_grasp_pt_cnstr = WorldPositionConstraint(robot,l_hand,l_hand_pt,l_hand_pos_fp(1:3),l_hand_pos_fp(1:3));
% cdfkp = cdfkp.addRigidBodyConstraint(lhand_bar1_grasp_pt_cnstr,num2cell(2:l_hand_ungrasp_idx1));
% cdfkp = cdfkp.addRigidBodyConstraint(lhand_bar1_grasp_dir_cnstr,num2cell(2:l_hand_ungrasp_idx1));
%
% [~,rhand_bar1_grasp_dir_cnstr] = generateBarGraspConstraint(robot,r_hand,r_hand_pt,r_hand_grasp_axis,monkeybar_pos1,bar_length,bar_width);
% rhand_bar1_grasp_pt_cnstr = WorldPositionConstraint(robot,r_hand,r_hand_pt,r_hand_pos_fp(1:3),r_hand_pos_fp(1:3));
% cdfkp = cdfkp.addRigidBodyConstraint(rhand_bar1_grasp_pt_cnstr,num2cell(2:r_hand_ungrasp_idx1));
% cdfkp = cdfkp.addRigidBodyConstraint(rhand_bar1_grasp_dir_cnstr,num2cell(2:r_hand_ungrasp_idx1));
%
% [lhand_bar2_grasp_pt_cnstr,lhand_bar2_grasp_dir_cnstr,lhand_bar2_fixed_pos_cnstr] = generateBarGraspConstraint(robot,l_hand,l_hand_pt,l_hand_grasp_axis,monkeybar_pos2,bar_length,bar_width);
% cdfkp = cdfkp.addRigidBodyConstraint(lhand_bar2_grasp_pt_cnstr,{l_hand_grasp_idx2});
% cdfkp = cdfkp.addRigidBodyConstraint(lhand_bar2_grasp_dir_cnstr,num2cell(l_hand_grasp_idx2:nT_swing1-1));
% cdfkp = cdfkp.addRigidBodyConstraint(lhand_bar2_fixed_pos_cnstr,{l_hand_grasp_idx2:nT_swing1-1});
%
% [rhand_bar2_grasp_pt_cnstr,rhand_bar2_grasp_dir_cnstr,rhand_bar2_fixed_pos_cnstr] = generateBarGraspConstraint(robot,r_hand,r_hand_pt,r_hand_grasp_axis,monkeybar_pos2,bar_length,bar_width);
% cdfkp = cdfkp.addRigidBodyConstraint(rhand_bar2_grasp_pt_cnstr,{r_hand_grasp_idx2});
% cdfkp = cdfkp.addRigidBodyConstraint(rhand_bar2_grasp_dir_cnstr,num2cell(r_hand_grasp_idx2:nT_swing1-1));
% cdfkp = cdfkp.addRigidBodyConstraint(rhand_bar2_fixed_pos_cnstr,{r_hand_grasp_idx2:nT_swing1-1});
%
% % fix initial and final state
% cdfkp = cdfkp.addBoundingBoxConstraint(ConstantConstraint(q_fp),cdfkp.q_inds(:,1));
% q_end_xyz = q_fp(1:3)+monkeybar_pos2(1:3)-monkeybar_pos1(1:3);
% q_end_rpy = q_fp(4:6);
% cdfkp = cdfkp.addBoundingBoxConstraint(ConstantConstraint([q_end_xyz;q_end_rpy;q_fp(7:end)]),cdfkp.q_inds(:,nT_swing1));
% cdfkp = cdfkp.addBoundingBoxConstraint(ConstantConstraint(zeros(nv,1)),cdfkp.v_inds(:,1));
% cdfkp = cdfkp.addBoundingBoxConstraint(ConstantConstraint(zeros(nv,1)),cdfkp.v_inds(:,nT_swing1));
%
% % add h bounds
% cdfkp = cdfkp.addBoundingBoxConstraint(BoundingBoxConstraint(0.02*ones(nT_swing1-1,1),0.2*ones(nT_swing1-1,1)),cdfkp.h_inds(:));
%
% % add com height upper bound
% cdfkp = cdfkp.addCoMBounds(1:nT_swing1,-inf(3,nT_swing1),[inf(2,nT_swing1);(monkeybar_pos1(3)-0.3)*ones(1,nT_swing1)]);
%
% cdfkp = cdfkp.addBoundingBoxConstraint(BoundingBoxConstraint(...
%   reshape(bsxfun(@times,[-9.81/2;-9.81/2;-9.81],ones(1,nT_swing1)),[],1),...
%   reshape(bsxfun(@times,[9.81/2;9.81/2;9.81],ones(1,nT_swing1)),[],1)),cdfkp.comddot_inds(:));

cdfkp = planBarSwing(robot,l_hand,r_hand,l_hand_pt,r_hand_pt,l_hand_grasp_axis,r_hand_grasp_axis,bar_length,bar_width,monkeybar_pos1,monkeybar_pos2,q_fp);

nT_swing1 = cdfkp.N;
x_seed = zeros(cdfkp.num_vars,1);
x_seed(cdfkp.h_inds) = 0.1;
x_seed(cdfkp.q_inds(:)) = reshape(bsxfun(@times,q_fp,ones(1,nT_swing1)),[],1);
x_seed(cdfkp.com_inds(:)) = reshape(bsxfun(@times,com_fp,ones(1,nT_swing1)),[],1);
x_seed(cdfkp.lambda_inds{1}(:)) = reshape(bsxfun(@times,[0;0;1/2;zeros(3,1)],ones(1,nT_swing1)),[],1);
x_seed(cdfkp.lambda_inds{2}(:)) = reshape(bsxfun(@times,[0;0;1/2;zeros(3,1)],ones(1,nT_swing1)),[],1);

cdfkp = cdfkp.setSolverOptions('snopt','iterationslimit',1e6);
cdfkp = cdfkp.setSolverOptions('snopt','majoriterationslimit',3000);
cdfkp = cdfkp.setSolverOptions('snopt','majorfeasibilitytolerance',1e-6);
cdfkp = cdfkp.setSolverOptions('snopt','majoroptimalitytolerance',1e-3);
cdfkp = cdfkp.setSolverOptions('snopt','superbasicslimit',2000);
cdfkp = cdfkp.setSolverOptions('snopt','print','test_monkeybar_swing.out');

seed_sol = load('test_monkeybar_swing','-mat','x_sol');
%   seed_sol.x_sol(cdfkp.lambda_inds{1}(1:3,:,:)) = seed_sol.x_sol(cdfkp.lambda_inds{1}(1:3,:,:))/(robot_mass*9.81);
%   seed_sol.x_sol(cdfkp.lambda_inds{1}(4:6,:,:)) = seed_sol.x_sol(cdfkp.lambda_inds{1}(4:6,:,:))/(robot_mass*9.81/100);
%   seed_sol.x_sol(cdfkp.lambda_inds{2}(1:3,:,:)) = seed_sol.x_sol(cdfkp.lambda_inds{2}(1:3,:,:))/(robot_mass*9.81);
%   seed_sol.x_sol(cdfkp.lambda_inds{2}(4:6,:,:)) = seed_sol.x_sol(cdfkp.lambda_inds{2}(4:6,:,:))/(robot_mass*9.81/100);
tic
[x_sol,F,info] = cdfkp.solve(seed_sol.x_sol);
toc;
[q_sol,v_sol,h_sol,t_sol,com_sol,comdot_sol,comddot_sol,H_sol,Hdot_sol,lambda_sol,wrench_sol] = parseSolution(cdfkp,x_sol);
xtraj_sol = PPTrajectory(foh(cumsum([0 h_sol]),[q_sol;v_sol]));
xtraj_sol = xtraj_sol.setOutputFrame(robot.getStateFrame);
keyboard
end
%%
if(mode == 3)
  % landing
  land_start_idx = 1;
  land_ungrasp_idx = 6;
  toe_land_idx = 9;
  heel_land_idx = 11;
  land_static_idx = 14;
  land_contact_wrench_struct = struct('active_knot',[],'cw',[]);
  lhand_bar_grasp_wrench = generateBarGraspingWrench(robot,l_hand,l_hand_pt,monkeybar_pos5);
  rhand_bar_grasp_wrench = generateBarGraspingWrench(robot,r_hand,r_hand_pt,monkeybar_pos5);
  land_contact_wrench_struct(1) = struct('active_knot',land_start_idx:land_ungrasp_idx,'cw',lhand_bar_grasp_wrench);
  land_contact_wrench_struct(2) = struct('active_knot',land_start_idx:land_ungrasp_idx,'cw',rhand_bar_grasp_wrench);
  num_edges = 4;
  FC_theta = linspace(0,2*pi,num_edges+1);
  mu = 1;
  g = 9.81;
  FC_edge = robot_mass*g*[mu*[sin(FC_theta(1:num_edges));cos(FC_theta(1:num_edges))];ones(1,num_edges)];
  land_contact_wrench_struct(3) = struct('active_knot',heel_land_idx:land_static_idx,'cw',LinearFrictionConeWrench(robot,l_foot,l_foot_heel,FC_edge));
  land_contact_wrench_struct(4) = struct('active_knot',heel_land_idx:land_static_idx,'cw',LinearFrictionConeWrench(robot,r_foot,r_foot_heel,FC_edge));
  land_contact_wrench_struct(5) = struct('active_knot',toe_land_idx:land_static_idx,'cw',LinearFrictionConeWrench(robot,l_foot,l_foot_toe,FC_edge));
  land_contact_wrench_struct(6) = struct('active_knot',toe_land_idx:land_static_idx,'cw',LinearFrictionConeWrench(robot,r_foot,r_foot_toe,FC_edge));

  nT3 = land_static_idx-land_start_idx+1;
  tf_range = [1 3];
  q_nom = bsxfun(@times,qstar,ones(1,nT3));
  Q_comddot = eye(3);
  Q = zeros(nq);
  Qv = 0.1*eye(nv);
  Q_contact_force = 0/(robot_mass*g)^2*eye(3);
  cdfkp = ComDynamicsFullKinematicsPlanner(robot,nT3,tf_range,Q_comddot,Qv,Q,q_nom,Q_contact_force,land_contact_wrench_struct);

  % add initial and final state constraint
  q_start = q_fp;
  q_start(1:3) = q_start(1:3)+monkeybar_pos5(1:3)-monkeybar_pos1(1:3);
  l_hand_pos_start = l_hand_pos_fp;
  l_hand_pos_start = l_hand_pos_start(1:3)+monkeybar_pos5(1:3)-monkeybar_pos1(1:3);
  r_hand_pos_start = r_hand_pos_fp;
  r_hand_pos_start = r_hand_pos_start(1:3)+monkeybar_pos5(1:3)-monkeybar_pos1(1:3);

  cdfkp = cdfkp.addBoundingBoxConstraint(ConstantConstraint(q_start),cdfkp.q_inds(:,land_start_idx));
  cdfkp = cdfkp.addBoundingBoxConstraint(ConstantConstraint(zeros(nv,1)),cdfkp.v_inds(:,land_start_idx));
  cdfkp = cdfkp.addBoundingBoxConstraint(ConstantConstraint(zeros(nv,1)),cdfkp.v_inds(:,land_static_idx));

  % add time constraint
  cdfkp = cdfkp.addBoundingBoxConstraint(BoundingBoxConstraint(0.05*ones(nT3-1,1),0.2*ones(nT3-1,1)),cdfkp.h_inds(:));

  % add grasping kinematic constraint
  [~,lhand_grasp_dir_cnstr1] = generateBarGraspConstraint(robot,l_hand,l_hand_pt,l_hand_grasp_axis,monkeybar_pos5,bar_length,bar_width);
  cdfkp = cdfkp.addRigidBodyConstraint(WorldPositionConstraint(robot,l_hand,l_hand_pt,l_hand_pos_start(1:3),l_hand_pos_start(1:3)),num2cell(land_start_idx+1:land_ungrasp_idx));
  cdfkp = cdfkp.addRigidBodyConstraint(lhand_grasp_dir_cnstr1,num2cell(land_start_idx+1:land_ungrasp_idx));
  [~,rhand_grasp_dir_cnstr] = generateBarGraspConstraint(robot,r_hand,r_hand_pt,r_hand_grasp_axis,monkeybar_pos5,bar_length,bar_width);
  cdfkp = cdfkp.addRigidBodyConstraint(WorldPositionConstraint(robot,r_hand,r_hand_pt,r_hand_pos_start(1:3),r_hand_pos_start(1:3)),num2cell(land_start_idx+1:land_ungrasp_idx));
  cdfkp = cdfkp.addRigidBodyConstraint(rhand_grasp_dir_cnstr,num2cell(land_start_idx+1:land_ungrasp_idx));

  % add landing kinematic constraint
%   T_stage2_to_world = [rpy2rotmat(stage_pos2(4:6)) stage_pos2(1:3);0 0 0 1];
  stage2_corner1 = stage_pos2(1:3)+[-0.95;-0.95;0.05];
  stage2_corner2 = stage_pos2(1:3)+[0.95;0.95;0.05];
  lfoot_toe_pos_cnstr = WorldPositionConstraint(robot,l_foot,l_foot_toe,repmat(stage2_corner1,1,2),repmat(stage2_corner2,1,2));
  cdfkp = cdfkp.addRigidBodyConstraint(lfoot_toe_pos_cnstr,{toe_land_idx});
  rfoot_toe_pos_cnstr = WorldPositionConstraint(robot,r_foot,r_foot_toe,repmat(stage2_corner1,1,2),repmat(stage2_corner2,1,2));
  cdfkp = cdfkp.addRigidBodyConstraint(rfoot_toe_pos_cnstr,{toe_land_idx});
  lfoot_toe_fixed_cnstr = WorldFixedPositionConstraint(robot,l_foot,l_foot_toe);
  cdfkp = cdfkp.addRigidBodyConstraint(lfoot_toe_fixed_cnstr,{toe_land_idx:heel_land_idx});
  rfoot_toe_fixed_cnstr = WorldFixedPositionConstraint(robot,r_foot,r_foot_toe);
  cdfkp = cdfkp.addRigidBodyConstraint(rfoot_toe_fixed_cnstr,{toe_land_idx:heel_land_idx});
  lfoot_toe_above_ground_cnstr = WorldPositionConstraint(robot,l_foot,l_foot_toe,repmat([-inf;-inf;stage_pos2(3)+0.06],1,2),inf(3,2));
  cdfkp = cdfkp.addRigidBodyConstraint(lfoot_toe_above_ground_cnstr,num2cell(land_start_idx+1:toe_land_idx-1));
  rfoot_toe_above_ground_cnstr = WorldPositionConstraint(robot,r_foot,r_foot_toe,repmat([-inf;-inf;stage_pos2(3)+0.06],1,2),inf(3,2));
  cdfkp = cdfkp.addRigidBodyConstraint(rfoot_toe_above_ground_cnstr,num2cell(land_start_idx+1:toe_land_idx-1));

  lfoot_heel_pos_cnstr1 = WorldPositionConstraint(robot,l_foot,l_foot_heel,repmat([stage2_corner1(1:2);nan],1,2),repmat([stage2_corner2(1:2);nan],1,2));
  cdfkp = cdfkp.addRigidBodyConstraint(lfoot_heel_pos_cnstr1,{heel_land_idx});
  lfoot_heel_pos_cnstr2 = WorldPositionConstraint(robot,l_foot,l_foot_heel(:,1),[nan;nan;stage_pos2(3)+0.05],[nan;nan;stage_pos2(3)+0.05]);
  cdfkp = cdfkp.addRigidBodyConstraint(lfoot_heel_pos_cnstr2,{heel_land_idx});
  rfoot_heel_pos_cnstr1 = WorldPositionConstraint(robot,r_foot,r_foot_heel,repmat([stage2_corner1(1:2);nan],1,2),repmat([stage2_corner2(1:2);nan],1,2));
  cdfkp = cdfkp.addRigidBodyConstraint(rfoot_heel_pos_cnstr1,{heel_land_idx});
  rfoot_heel_pos_cnstr2 = WorldPositionConstraint(robot,r_foot,r_foot_heel(:,1),[nan;nan;stage_pos2(3)+0.05],[nan;nan;stage_pos2(3)+0.05]);
  cdfkp = cdfkp.addRigidBodyConstraint(rfoot_heel_pos_cnstr2,{heel_land_idx});
  lfoot_heel_above_ground_cnstr = WorldPositionConstraint(robot,l_foot,l_foot_heel,repmat([-inf;-inf;stage_pos2(3)+0.06],1,2),inf(3,2));
  cdfkp = cdfkp.addRigidBodyConstraint(lfoot_heel_above_ground_cnstr,num2cell(land_start_idx+1:heel_land_idx-1));
  rfoot_heel_above_ground_cnstr = WorldPositionConstraint(robot,r_foot,r_foot_heel,repmat([-inf;-inf;stage_pos2(3)+0.06],1,2),inf(3,2));
  cdfkp = cdfkp.addRigidBodyConstraint(rfoot_heel_above_ground_cnstr,num2cell(land_start_idx+1:heel_land_idx-1));

  lfoot_fixed_cnstr = WorldFixedBodyPoseConstraint(robot,l_foot);
  cdfkp = cdfkp.addRigidBodyConstraint(lfoot_fixed_cnstr,{heel_land_idx:land_static_idx});
  rfoot_fixed_cnstr = WorldFixedBodyPoseConstraint(robot,r_foot);
  cdfkp = cdfkp.addRigidBodyConstraint(rfoot_fixed_cnstr,{heel_land_idx:land_static_idx});


  % add comddot bounds
  cdfkp = cdfkp.addBoundingBoxConstraint(BoundingBoxConstraint(...
    reshape(bsxfun(@times,[-9.81;-9.81;-1.5*9.81],ones(1,nT3)),[],1),...
    reshape(bsxfun(@times,[9.81;9.81;1.5*9.81],ones(1,nT3)),[],1)),cdfkp.comddot_inds(:));

  % arm and leg symmetry
  arm_symmetry = armSymmetricConstraint(robot,land_ungrasp_idx+1:land_static_idx);
  cdfkp = cdfkp.addLinearConstraint(arm_symmetry,reshape(cdfkp.q_inds(:,land_ungrasp_idx+1:land_static_idx),[],1));

  % leg symmetry
  leg_symmetry = legSymmetricConstraint(robot,land_ungrasp_idx+1:land_static_idx);
  cdfkp = cdfkp.addLinearConstraint(leg_symmetry,reshape(cdfkp.q_inds(:,land_ungrasp_idx+1:land_static_idx),[],1));

  % add a torso direction constraint
  torso_dir_cnstr = WorldGazeDirConstraint(robot,utorso,[0;0;1],[0;0;1],0.05);
  cdfkp = cdfkp.addRigidBodyConstraint(torso_dir_cnstr,{land_static_idx});

  % add a final posture cost
  Q = 10*eye(nq);
  Q(1,1) = 0;
  Q(2,2) = 0;
  Q(6,6) = 0;
  cdfkp = cdfkp.addCost(QuadraticConstraint(-inf,inf,Q,-Q*qstar),cdfkp.q_inds(:,land_static_idx));

%   % add a linear constraint on leg_hpx to prevent crossing the leg
%   hpx_diff_max = 0.12;
%   leg_nocrossing_cnstr = LinearConstraint(-inf(cdfkp.N-1,1),hpx_diff_max*ones(cdfkp.N-1,1),sparse(reshape(bsxfun(@times,ones(2,1),1:cdfkp.N-1),[],1),(1:2*(cdfkp.N-1))',reshape([ones(1,cdfkp.N-1);-ones(1,cdfkp.N-1)],[],1),cdfkp.N-1,2*(cdfkp.N-1)));
%   leg_nocrossing_name = cell(cdfkp.N-1,1);
%   for i = 1:cdfkp.N-1
%     leg_nocrossing_name{i} = sprintf('r_leg_hpx[%d]-l_leg_hpx[%d]<=%5.2f',i+1,i+1,hpx_diff_max);
%   end
%   leg_nocrossing_cnstr = leg_nocrossing_cnstr.setName(leg_nocrossing_name);
%   cdfkp = cdfkp.addLinearConstraint(leg_nocrossing_cnstr,reshape(cdfkp.q_inds([r_leg_hpx;l_leg_hpx],2:cdfkp.N),[],1));

%   % add a minimum distance between feet
%   active_collision_options.body_idx = [l_foot,r_foot];
%   active_collision_options.collision_groups = {'default'};
%   feet_min_distance_cnstr = MinDistanceConstraint(robot,0.001,[-inf,inf],active_collision_options);
%   cdfkp = cdfkp.addRigidBodyConstraint(feet_min_distance_cnstr,num2cell(land_start_idx:heel_land_idx-1));

  % This is a hack, but I try to prevent the two feet to collide
  lfoot_nocollide_cnstr = WorldPositionConstraint(robot,l_foot,l_foot_bottom,[nan(1,4);zeros(1,4);nan(1,4)],nan(3,4));
  cdfkp = cdfkp.addRigidBodyConstraint(lfoot_nocollide_cnstr,num2cell(land_start_idx+1:toe_land_idx-1));
  rfoot_nocollide_cnstr = WorldPositionConstraint(robot,r_foot,r_foot_bottom,nan(3,4),[nan(1,4);zeros(1,4);nan(1,4)]);
  cdfkp = cdfkp.addRigidBodyConstraint(rfoot_nocollide_cnstr,num2cell(land_start_idx+1:toe_land_idx-1));

  % bend the knee at landing
  knee_bend_idx = heel_land_idx+1;
%   cdfkp= cdfkp.addBoundingBoxConstraint(BoundingBoxConstraint([0.5;0.5],[inf;inf]),cdfkp.q_inds([l_leg_kny;r_leg_kny],knee_bend_idx));

  % Do not bend ankle too much
%   cdfkp = cdfkp.addBoundingBoxConstraint(BoundingBoxConstraint([-0.35;-0.35],[inf;inf]),cdfkp.q_inds([l_leg_aky;r_leg_aky],land_static_idx));

  x_seed = zeros(cdfkp.num_vars,1);
  q_land_start = q_fp;
  q_final_guess = qstar;
  q_final_guess(1:3) = q_final_guess(1:3)+stage_pos2(1:3);
  x_seed(cdfkp.q_inds(:,land_start_idx:land_ungrasp_idx)) = reshape(bsxfun(@times,q_land_start,ones(1,land_ungrasp_idx-land_start_idx+1)),[],1);
  x_seed(cdfkp.q_inds(:,land_ungrasp_idx+1:land_static_idx)) = reshape(bsxfun(@times,q_final_guess,ones(1,land_static_idx-land_ungrasp_idx)),[],1);
  x_seed(cdfkp.com_inds(:,land_start_idx:land_static_idx)) = reshape(bsxfun(@times,com_fp,ones(1,land_static_idx-land_start_idx+1)),[],1);
  x_seed(cdfkp.lambda_inds{1}(:,:,1:land_ungrasp_idx)) = reshape(bsxfun(@times,[0;0;1/2;0;0;0],ones(1,land_ungrasp_idx-land_start_idx+1)),[],1);
  x_seed(cdfkp.lambda_inds{2}(:,:,1:land_ungrasp_idx)) = reshape(bsxfun(@times,[0;0;1/2;0;0;0],ones(1,land_ungrasp_idx-land_start_idx+1)),[],1);
  x_seed(cdfkp.lambda_inds{3}(:,:,toe_land_idx:land_static_idx)) = reshape(bsxfun(@times,0.5/(num_edges*4)*ones(num_edges,4,1),ones(1,1,land_static_idx-toe_land_idx+1)),[],1);
  x_seed(cdfkp.lambda_inds{4}(:,:,toe_land_idx:land_static_idx)) = reshape(bsxfun(@times,0.5/(num_edges*4)*ones(num_edges,4,1),ones(1,1,land_static_idx-toe_land_idx+1)),[],1);

  cdfkp = cdfkp.setSolverOptions('snopt','iterationslimit',1e6);
  cdfkp = cdfkp.setSolverOptions('snopt','majoriterationslimit',3000);
  cdfkp = cdfkp.setSolverOptions('snopt','majorfeasibilitytolerance',1e-6);
  cdfkp = cdfkp.setSolverOptions('snopt','majoroptimalitytolerance',6e-4);
  cdfkp = cdfkp.setSolverOptions('snopt','superbasicslimit',2000);
  cdfkp = cdfkp.setSolverOptions('snopt','print','test_monkeybar_land.out');

  seed_sol = load('test_monkeybar_land','-mat','x_sol');
%   seed_sol.x_sol(cdfkp.lambda_inds{1}(1:3,:,:)) = seed_sol.x_sol(cdfkp.lambda_inds{1}(1:3,:,:))/(robot_mass*9.81);
%   seed_sol.x_sol(cdfkp.lambda_inds{2}(1:3,:,:)) = seed_sol.x_sol(cdfkp.lambda_inds{2}(1:3,:,:))/(robot_mass*9.81);
%   seed_sol.x_sol(cdfkp.lambda_inds{1}(4:6,:,:)) = seed_sol.x_sol(cdfkp.lambda_inds{1}(4:6,:,:))/(robot_mass*9.81/100);
%   seed_sol.x_sol(cdfkp.lambda_inds{2}(4:6,:,:)) = seed_sol.x_sol(cdfkp.lambda_inds{2}(4:6,:,:))/(robot_mass*9.81/100);
  tic
  [x_sol,F,info] = cdfkp.solve(seed_sol.x_sol);
  toc
  [q_sol,v_sol,h_sol,t_sol,com_sol,comdot_sol,comddot_sol,H_sol,Hdot_sol,lambda_sol,wrench_sol] = parseSolution(cdfkp,x_sol);
  xtraj_sol = PPTrajectory(foh(cumsum([0 h_sol]),[q_sol;v_sol]));
  xtraj_sol = xtraj_sol.setOutputFrame(robot.getStateFrame);
  keyboard;
end

if(mode == 4)
  q_start = q_fp;
  q_start(1:3) = q_start(1:3)+monkeybar_pos2(1:3)-monkeybar_pos1(1:3);
  kinsol_start = robot.doKinematics(q_start);
  com_start = robot.getCOM(kinsol_start);
  cdfkp = planBarSwing(robot,l_hand,r_hand,l_hand_pt,r_hand_pt,l_hand_grasp_axis,r_hand_grasp_axis,bar_length,bar_width,monkeybar_pos2,monkeybar_pos3,q_start);

  nT_swing1 = cdfkp.N;
  x_seed = zeros(cdfkp.num_vars,1);
  x_seed(cdfkp.h_inds) = 0.1;
  x_seed(cdfkp.q_inds(:)) = reshape(bsxfun(@times,q_start,ones(1,nT_swing1)),[],1);
  x_seed(cdfkp.com_inds(:)) = reshape(bsxfun(@times,com_start,ones(1,nT_swing1)),[],1);
  x_seed(cdfkp.lambda_inds{1}(:)) = reshape(bsxfun(@times,[0;0;1/2;zeros(3,1)],ones(1,nT_swing1)),[],1);
  x_seed(cdfkp.lambda_inds{2}(:)) = reshape(bsxfun(@times,[0;0;1/2;zeros(3,1)],ones(1,nT_swing1)),[],1);

  cdfkp = cdfkp.setSolverOptions('snopt','iterationslimit',1e6);
  cdfkp = cdfkp.setSolverOptions('snopt','majoriterationslimit',3000);
  cdfkp = cdfkp.setSolverOptions('snopt','majorfeasibilitytolerance',1e-6);
  cdfkp = cdfkp.setSolverOptions('snopt','majoroptimalitytolerance',2e-4);
  cdfkp = cdfkp.setSolverOptions('snopt','superbasicslimit',2000);
  cdfkp = cdfkp.setSolverOptions('snopt','print','test_monkeybar_swing2.out');
%   cdfkp = cdfkp.setSolverOptions('snopt','scaleoption',2);

%   seed_sol = load('test_monkeybar_swing','-mat','x_sol');
%   seed_sol.x_sol(cdfkp.q_inds(1:3,:)) = reshape(reshape(seed_sol.x_sol(cdfkp.q_inds(1:3,:)),3,[])+bsxfun(@times,monkeybar_pos2(1:3)-monkeybar_pos1(1:3),ones(1,nT_swing)),[],1);
%   seed_sol.x_sol(cdfkp.com_inds(:)) = reshape(reshape(seed_sol.x_sol(cdfkp.com_inds(:)),3,[])+bsxfun(@times,monkeybar_pos2(1:3)-monkeybar_pos1(1:3),ones(1,nT_swing)),[],1);
  seed_sol = load('test_monkeybar_swing2','-mat','x_sol');
%     seed_sol.x_sol(cdfkp.lambda_inds{1}(1:3,:,:)) = seed_sol.x_sol(cdfkp.lambda_inds{1}(1:3,:,:))/(robot_mass*9.81);
%   seed_sol.x_sol(cdfkp.lambda_inds{1}(4:6,:,:)) = seed_sol.x_sol(cdfkp.lambda_inds{1}(4:6,:,:))/(robot_mass*9.81/100);
%   seed_sol.x_sol(cdfkp.lambda_inds{2}(1:3,:,:)) = seed_sol.x_sol(cdfkp.lambda_inds{2}(1:3,:,:))/(robot_mass*9.81);
%   seed_sol.x_sol(cdfkp.lambda_inds{2}(4:6,:,:)) = seed_sol.x_sol(cdfkp.lambda_inds{2}(4:6,:,:))/(robot_mass*9.81/100);
  tic
  [x_sol,F,info] = cdfkp.solve(seed_sol.x_sol);
  toc;
  [q_sol,v_sol,h_sol,t_sol,com_sol,comdot_sol,comddot_sol,H_sol,Hdot_sol,lambda_sol,wrench_sol] = parseSolution(cdfkp,x_sol);
  xtraj_sol = PPTrajectory(foh(cumsum([0 h_sol]),[q_sol;v_sol]));
  xtraj_sol = xtraj_sol.setOutputFrame(robot.getStateFrame);
  keyboard
end

if(mode == 5)
  q_start = q_fp;
  q_start(1:3) = q_start(1:3)+monkeybar_pos3(1:3)-monkeybar_pos1(1:3);
  kinsol_start = robot.doKinematics(q_start);
  com_start = robot.getCOM(kinsol_start);
  cdfkp = planBarSwing(robot,l_hand,r_hand,l_hand_pt,r_hand_pt,l_hand_grasp_axis,r_hand_grasp_axis,bar_length,bar_width,monkeybar_pos3,monkeybar_pos4,q_start);

  nT_swing1 = cdfkp.N;
  x_seed = zeros(cdfkp.num_vars,1);
  x_seed(cdfkp.h_inds) = 0.1;
  x_seed(cdfkp.q_inds(:)) = reshape(bsxfun(@times,q_start,ones(1,nT_swing1)),[],1);
  x_seed(cdfkp.com_inds(:)) = reshape(bsxfun(@times,com_start,ones(1,nT_swing1)),[],1);
  x_seed(cdfkp.lambda_inds{1}(:)) = reshape(bsxfun(@times,[l_hand_force_fp;zeros(3,1)],ones(1,nT_swing1)),[],1);
  x_seed(cdfkp.lambda_inds{2}(:)) = reshape(bsxfun(@times,[r_hand_force_fp;zeros(3,1)],ones(1,nT_swing1)),[],1);

  cdfkp = cdfkp.setSolverOptions('snopt','iterationslimit',1e6);
  cdfkp = cdfkp.setSolverOptions('snopt','majoriterationslimit',3000);
  cdfkp = cdfkp.setSolverOptions('snopt','majorfeasibilitytolerance',1e-6);
  cdfkp = cdfkp.setSolverOptions('snopt','majoroptimalitytolerance',2e-4);
  cdfkp = cdfkp.setSolverOptions('snopt','superbasicslimit',2000);
  cdfkp = cdfkp.setSolverOptions('snopt','print','test_monkeybar_swing3.out');
%   cdfkp = cdfkp.setSolverOptions('snopt','scaleoption',2);

  seed_sol = load('test_monkeybar_swing32','-mat','x_sol');
%   seed_sol.x_sol(cdfkp.lambda_inds{1}(1:3,:,:)) = seed_sol.x_sol(cdfkp.lambda_inds{1}(1:3,:,:))/(robot_mass*9.81);
%   seed_sol.x_sol(cdfkp.lambda_inds{1}(4:6,:,:)) = seed_sol.x_sol(cdfkp.lambda_inds{1}(4:6,:,:))/(robot_mass*9.81/100);
%   seed_sol.x_sol(cdfkp.lambda_inds{2}(1:3,:,:)) = seed_sol.x_sol(cdfkp.lambda_inds{2}(1:3,:,:))/(robot_mass*9.81);
%   seed_sol.x_sol(cdfkp.lambda_inds{2}(4:6,:,:)) = seed_sol.x_sol(cdfkp.lambda_inds{2}(4:6,:,:))/(robot_mass*9.81/100);

  tic
  [x_sol,F,info] = cdfkp.solve(seed_sol.x_sol);
  toc
  [q_sol,v_sol,h_sol,t_sol,com_sol,comdot_sol,comddot_sol,H_sol,Hdot_sol,lambda_sol,wrench_sol] = parseSolution(cdfkp,x_sol);
  xtraj_sol = PPTrajectory(foh(cumsum([0 h_sol]),[q_sol;v_sol]));
  xtraj_sol = xtraj_sol.setOutputFrame(robot.getStateFrame);
  keyboard;
end

if(mode == 6)
  land_start_idx = 1;
  land_ungrasp_idx = 6;
  foot_contact_idx = 9;
  land_static_idx = 14;
  land_contact_wrench_struct = struct('active_knot',[],'cw',[]);
  lhand_bar_grasp_wrench = generateBarGraspingWrench(robot,l_hand,l_hand_pt,monkeybar_pos5);
  rhand_bar_grasp_wrench = generateBarGraspingWrench(robot,r_hand,r_hand_pt,monkeybar_pos5);
  land_contact_wrench_struct(1) = struct('active_knot',land_start_idx:land_ungrasp_idx,'cw',lhand_bar_grasp_wrench);
  land_contact_wrench_struct(2) = struct('active_knot',land_start_idx:land_ungrasp_idx,'cw',rhand_bar_grasp_wrench);
  num_edges = 4;
  FC_theta = linspace(0,2*pi,num_edges+1);
  mu = 1;
  g = 9.81;
  FC_edge = robot_mass*g*[mu*[sin(FC_theta(1:num_edges));cos(FC_theta(1:num_edges))];ones(1,num_edges)];
  land_contact_wrench_struct(3) = struct('active_knot',foot_contact_idx:land_static_idx-1,'cw',ComplementarityLinearFrictionConeWrench(robot,l_foot,l_foot_bottom,FC_edge,@(pt) stageDistance(pt,stage_pos2),ncp_tol));
  land_contact_wrench_struct(4) = struct('active_knot',foot_contact_idx:land_static_idx-1,'cw',ComplementarityLinearFrictionConeWrench(robot,r_foot,r_foot_bottom,FC_edge,@(pt) stageDistance(pt,stage_pos2),ncp_tol));
%   land_contact_wrench_struct(3) = struct('active_knot',foot_contact_idx:land_static_idx-1,'cw',LinearFrictionConeWrench(robot,l_foot,l_foot_bottom,FC_edge));
%   land_contact_wrench_struct(4) = struct('active_knot',foot_contact_idx:land_static_idx-1,'cw',LinearFrictionConeWrench(robot,r_foot,r_foot_bottom,FC_edge));
  land_contact_wrench_struct(5) = struct('active_knot',land_static_idx,'cw',LinearFrictionConeWrench(robot,l_foot,l_foot_bottom,FC_edge));
  land_contact_wrench_struct(6) = struct('active_knot',land_static_idx,'cw',LinearFrictionConeWrench(robot,r_foot,r_foot_bottom,FC_edge));

  nT3 = land_static_idx-land_start_idx+1;
  tf_range = [1 3];
  q_nom = bsxfun(@times,qstar,ones(1,nT3));
  Q_comddot = eye(3);
  Q = zeros(nq);
  Qv = 0.1*eye(nv);
  Q_contact_force = 0/(robot_mass*g)^2*eye(3);
  cdfkp = ComDynamicsFullKinematicsPlanner(robot,nT3,tf_range,Q_comddot,Qv,Q,q_nom,Q_contact_force,land_contact_wrench_struct,struct('ncp_tol',ncp_tol));

  q_start = q_fp;
  q_start(1:3) = q_start(1:3)+monkeybar_pos5(1:3)-monkeybar_pos1(1:3);
  l_hand_pos_start = l_hand_pos_fp;
  l_hand_pos_start = l_hand_pos_start(1:3)+monkeybar_pos5(1:3)-monkeybar_pos1(1:3);
  r_hand_pos_start = r_hand_pos_fp;
  r_hand_pos_start = r_hand_pos_start(1:3)+monkeybar_pos5(1:3)-monkeybar_pos1(1:3);

  % add initial and final state constraint
  cdfkp = cdfkp.addBoundingBoxConstraint(ConstantConstraint(q_start),cdfkp.q_inds(:,land_start_idx));
  cdfkp = cdfkp.addBoundingBoxConstraint(ConstantConstraint(zeros(nv,1)),cdfkp.v_inds(:,land_start_idx));
  cdfkp = cdfkp.addBoundingBoxConstraint(ConstantConstraint(zeros(nv,1)),cdfkp.v_inds(:,land_static_idx));

  % add time constraint
  cdfkp = cdfkp.addBoundingBoxConstraint(BoundingBoxConstraint(0.05*ones(foot_contact_idx-land_start_idx,1),0.2*ones(foot_contact_idx-land_start_idx,1)),cdfkp.h_inds(1:foot_contact_idx-land_start_idx));
  cdfkp = cdfkp.addBoundingBoxConstraint(BoundingBoxConstraint(0.0001*ones(land_static_idx-foot_contact_idx,1),0.2*ones(land_static_idx-foot_contact_idx,1)),cdfkp.h_inds(foot_contact_idx:land_static_idx-1));

  % add grasping kinematic constraint
  [~,lhand_grasp_dir_cnstr1] = generateBarGraspConstraint(robot,l_hand,l_hand_pt,l_hand_grasp_axis,monkeybar_pos5,bar_length,bar_width);
  cdfkp = cdfkp.addRigidBodyConstraint(WorldPositionConstraint(robot,l_hand,l_hand_pt,l_hand_pos_start(1:3),l_hand_pos_start(1:3)),num2cell(land_start_idx+1:land_ungrasp_idx));
  cdfkp = cdfkp.addRigidBodyConstraint(lhand_grasp_dir_cnstr1,num2cell(land_start_idx+1:land_ungrasp_idx));
  [~,rhand_grasp_dir_cnstr] = generateBarGraspConstraint(robot,r_hand,r_hand_pt,r_hand_grasp_axis,monkeybar_pos5,bar_length,bar_width);
  cdfkp = cdfkp.addRigidBodyConstraint(WorldPositionConstraint(robot,r_hand,r_hand_pt,r_hand_pos_start(1:3),r_hand_pos_start(1:3)),num2cell(land_start_idx+1:land_ungrasp_idx));
  cdfkp = cdfkp.addRigidBodyConstraint(rhand_grasp_dir_cnstr,num2cell(land_start_idx+1:land_ungrasp_idx));

  % add landing kinematic constraint
%   T_stage2_to_world = [rpy2rotmat(stage_pos2(4:6)) stage_pos2(1:3);0 0 0 1];
  stage2_corner1 = stage_pos2(1:3)+[-0.95;-0.95;0.05];
  stage2_corner2 = stage_pos2(1:3)+[0.95;0.95;0.05];
  lfoot_bottom_pos_cnstr = WorldPositionConstraint(robot,l_foot,l_foot_bottom,repmat([stage2_corner1(1:2);stage_pos2(3)+0.05],1,4),repmat([stage2_corner2(1:2);nan],1,4));
  cdfkp = cdfkp.addRigidBodyConstraint(lfoot_bottom_pos_cnstr,num2cell(foot_contact_idx:land_static_idx-1));
  rfoot_bottom_pos_cnstr = WorldPositionConstraint(robot,r_foot,r_foot_bottom,repmat([stage2_corner1(1:2);stage_pos2(3)+0.05],1,4),repmat([stage2_corner2(1:2);nan],1,4));
  cdfkp = cdfkp.addRigidBodyConstraint(rfoot_bottom_pos_cnstr,num2cell(foot_contact_idx:land_static_idx-1));

  lfoot_bottom_above_ground_cnstr = WorldPositionConstraint(robot,l_foot,l_foot_bottom,repmat([-inf;-inf;stage_pos2(3)+0.05],1,4),inf(3,4));
  cdfkp = cdfkp.addRigidBodyConstraint(lfoot_bottom_above_ground_cnstr,num2cell(land_start_idx+1:foot_contact_idx-1));
  rfoot_bottom_above_ground_cnstr = WorldPositionConstraint(robot,r_foot,r_foot_bottom,repmat([-inf;-inf;stage_pos2(3)+0.05],1,4),inf(3,4));
  cdfkp = cdfkp.addRigidBodyConstraint(rfoot_bottom_above_ground_cnstr,num2cell(land_start_idx+1:foot_contact_idx-1));

  lfoot_bottom_on_ground_cnstr1 = WorldPositionConstraint(robot,l_foot,l_foot_bottom,repmat([stage2_corner1(1:2);nan],1,4),repmat([stage2_corner2(1:2);nan],1,4));
  cdfkp = cdfkp.addRigidBodyConstraint(lfoot_bottom_on_ground_cnstr1,{land_static_idx});
  lfoot_bottom_on_ground_cnstr2 = WorldPositionConstraint(robot,l_foot,l_foot_bottom(:,1:3),repmat([nan;nan;stage_pos2(3)+0.05],1,3),repmat([nan;nan;stage_pos2(3)+0.05],1,3));
  cdfkp = cdfkp.addRigidBodyConstraint(lfoot_bottom_on_ground_cnstr2,{land_static_idx});
  rfoot_bottom_on_ground_cnstr1 = WorldPositionConstraint(robot,r_foot,r_foot_bottom,repmat([stage2_corner1(1:2);nan],1,4),repmat([stage2_corner2(1:2);nan],1,4));
  cdfkp = cdfkp.addRigidBodyConstraint(rfoot_bottom_on_ground_cnstr1,{land_static_idx});
  rfoot_bottom_on_ground_cnstr2 = WorldPositionConstraint(robot,r_foot,r_foot_bottom(:,1:3),repmat([nan;nan;stage_pos2(3)+0.05],1,3),repmat([nan;nan;stage_pos2(3)+0.05],1,3));
  cdfkp = cdfkp.addRigidBodyConstraint(rfoot_bottom_on_ground_cnstr2,{land_static_idx});

  % add comddot bounds
  cdfkp = cdfkp.addBoundingBoxConstraint(BoundingBoxConstraint(...
    reshape(bsxfun(@times,[-9.81;-9.81;-1.5*9.81],ones(1,nT3)),[],1),...
    reshape(bsxfun(@times,[9.81;9.81;1.5*9.81],ones(1,nT3)),[],1)),cdfkp.comddot_inds(:));

  % arm and leg symmetry
%   arm_symmetry = armSymmetricConstraint(robot,land_ungrasp_idx+1:land_static_idx);
%   cdfkp = cdfkp.addLinearConstraint(arm_symmetry,reshape(cdfkp.q_inds(:,land_ungrasp_idx+1:land_static_idx),[],1));

  % leg symmetry
%   leg_symmetry = legSymmetricConstraint(robot,land_ungrasp_idx+1:land_static_idx);
%   cdfkp = cdfkp.addLinearConstraint(leg_symmetry,reshape(cdfkp.q_inds(:,land_ungrasp_idx+1:land_static_idx),[],1));
  % add a torso direction constraint
   % add a final posture cost
  Q = 10*eye(nq);
  Q(1,1) = 0;
  Q(2,2) = 0;
  Q(6,6) = 0;
  cdfkp = cdfkp.addCost(QuadraticConstraint(-inf,inf,Q,-Q*qstar),cdfkp.q_inds(:,land_static_idx));

  % add a torso direction constraint
  torso_dir_cnstr = WorldGazeDirConstraint(robot,utorso,[0;0;1],[0;0;1],0.25);
  cdfkp = cdfkp.addRigidBodyConstraint(torso_dir_cnstr,{land_static_idx});

  x_seed = zeros(cdfkp.num_vars,1);
  q_land_start = q_fp;
  q_final_guess = qstar;
  q_final_guess(1:3) = q_final_guess(1:3)+stage_pos2(1:3);
  x_seed(cdfkp.q_inds(:,land_start_idx:land_ungrasp_idx)) = reshape(bsxfun(@times,q_land_start,ones(1,land_ungrasp_idx-land_start_idx+1)),[],1);
  x_seed(cdfkp.q_inds(:,land_ungrasp_idx+1:land_static_idx)) = reshape(bsxfun(@times,q_final_guess,ones(1,land_static_idx-land_ungrasp_idx)),[],1);
  x_seed(cdfkp.com_inds(:,land_start_idx:land_static_idx)) = reshape(bsxfun(@times,com_fp,ones(1,land_static_idx-land_start_idx+1)),[],1);
  x_seed(cdfkp.lambda_inds{1}(:,:,1:land_ungrasp_idx)) = reshape(bsxfun(@times,[l_hand_force_fp;0;0;0],ones(1,land_ungrasp_idx-land_start_idx+1)),[],1);
  x_seed(cdfkp.lambda_inds{2}(:,:,1:land_ungrasp_idx)) = reshape(bsxfun(@times,[r_hand_force_fp;0;0;0],ones(1,land_ungrasp_idx-land_start_idx+1)),[],1);
  x_seed(cdfkp.lambda_inds{3}(:,:,foot_contact_idx:land_static_idx)) = reshape(bsxfun(@times,0.5/(num_edges*4)*ones(num_edges,4,1),ones(1,1,land_static_idx-foot_contact_idx+1)),[],1);
  x_seed(cdfkp.lambda_inds{4}(:,:,foot_contact_idx:land_static_idx)) = reshape(bsxfun(@times,0.5/(num_edges*4)*ones(num_edges,4,1),ones(1,1,land_static_idx-foot_contact_idx+1)),[],1);

  cdfkp = cdfkp.setSolverOptions('snopt','iterationslimit',1e6);
  cdfkp = cdfkp.setSolverOptions('snopt','majoriterationslimit',3000);
  cdfkp = cdfkp.setSolverOptions('snopt','majorfeasibilitytolerance',1e-6);
  cdfkp = cdfkp.setSolverOptions('snopt','majoroptimalitytolerance',5e-4);
  cdfkp = cdfkp.setSolverOptions('snopt','superbasicslimit',2000);
  cdfkp = cdfkp.setSolverOptions('snopt','print','test_monkeybar_land_complementarity.out');

  seed_sol = load('test_monkeybar_land_complementarity2','-mat','x_sol');
%   seed_x_sol = seed_sol.x_sol;
%   seed_sol.x_sol(cdfkp.lambda_inds{1}(1:3,:,:)) = seed_sol.x_sol(cdfkp.lambda_inds{1}(1:3,:,:))/(robot_mass*9.81);
%   seed_sol.x_sol(cdfkp.lambda_inds{1}(4:6,:,:)) = seed_sol.x_sol(cdfkp.lambda_inds{1}(4:6,:,:))/(robot_mass*9.81/100);
%   seed_sol.x_sol(cdfkp.lambda_inds{2}(1:3,:,:)) = seed_sol.x_sol(cdfkp.lambda_inds{2}(1:3,:,:))/(robot_mass*9.81);
%   seed_sol.x_sol(cdfkp.lambda_inds{2}(4:6,:,:)) = seed_sol.x_sol(cdfkp.lambda_inds{2}(4:6,:,:))/(robot_mass*9.81/100);

%   seed_sol = load('test_monkeybar_land','-mat','q_sol','v_sol','h_sol','com_sol','comdot_sol','comddot_sol','H_sol','Hdot_sol','lambda_sol');
%   seed_x_sol = zeros(cdfkp.num_vars,1);
%   seed_x_sol(cdfkp.h_inds(:)) = seed_sol.h_sol;
%   seed_x_sol(cdfkp.q_inds(:)) = seed_sol.q_sol(:);
%   seed_x_sol(cdfkp.v_inds(:)) = seed_sol.v_sol(:);
%   seed_x_sol(cdfkp.com_inds(:)) = seed_sol.com_sol(:);
%   seed_x_sol(cdfkp.comdot_inds(:)) = seed_sol.comdot_sol(:);
%   seed_x_sol(cdfkp.comddot_inds(:)) = seed_sol.comddot_sol(:);
%   seed_x_sol(cdfkp.H_inds(:)) = seed_sol.H_sol(:);
%   seed_x_sol(cdfkp.Hdot_inds(:)) = seed_sol.Hdot_sol(:)/cdfkp.torque_multiplier;
%   seed_x_sol(cdfkp.lambda_inds{1}(:)) = seed_sol.lambda_sol{1}(:);
%   seed_x_sol(cdfkp.lambda_inds{2}(:)) = seed_sol.lambda_sol{2}(:);
%   seed_x_sol(cdfkp.lambda_inds{3}(:)) = seed_sol.lambda_sol{3}(:);
%   seed_x_sol(cdfkp.lambda_inds{4}(:)) = seed_sol.lambda_sol{4}(:);


%   cdfkp_initial_gamma = cdfkp;
%   cdfkp_initial_gamma = cdfkp_initial_gamma.addBoundingBoxConstraint(ConstantConstraint(seed_sol.h_sol),cdfkp_initial_gamma.h_inds(:));
%   cdfkp_initial_gamma = cdfkp_initial_gamma.addBoundingBoxConstraint(ConstantConstraint(seed_sol.q_sol(:)),cdfkp_initial_gamma.q_inds(:));
%   cdfkp_initial_gamma = cdfkp_initial_gamma.addBoundingBoxConstraint(ConstantConstraint(seed_sol.v_sol(:)),cdfkp_initial_gamma.v_inds(:));
%   cdfkp_initial_gamma = cdfkp_initial_gamma.addBoundingBoxConstraint(ConstantConstraint(seed_sol.com_sol(:)),cdfkp_initial_gamma.com_inds(:));
%   cdfkp_initial_gamma = cdfkp_initial_gamma.addBoundingBoxConstraint(ConstantConstraint(seed_sol.comdot_sol(:)),cdfkp_initial_gamma.comdot_inds(:));
%   cdfkp_initial_gamma = cdfkp_initial_gamma.addBoundingBoxConstraint(ConstantConstraint(seed_sol.comddot_sol(:)),cdfkp_initial_gamma.comddot_inds(:));
%   cdfkp_initial_gamma = cdfkp_initial_gamma.addBoundingBoxConstraint(ConstantConstraint(seed_sol.H_sol(:)),cdfkp_initial_gamma.H_inds(:));
%   cdfkp_initial_gamma = cdfkp_initial_gamma.addBoundingBoxConstraint(ConstantConstraint(seed_sol.Hdot_sol(:)),cdfkp_initial_gamma.Hdot_inds(:));
%   cdfkp_initial_gamma = cdfkp_initial_gamma.addBoundingBoxConstraint(ConstantConstraint(seed_sol.lambda_sol{1}(:)),cdfkp_initial_gamma.lambda_inds{1}(:));
%   cdfkp_initial_gamma = cdfkp_initial_gamma.addBoundingBoxConstraint(ConstantConstraint(seed_sol.lambda_sol{2}(:)),cdfkp_initial_gamma.lambda_inds{2}(:));
%   cdfkp_initial_gamma = cdfkp_initial_gamma.addBoundingBoxConstraint(ConstantConstraint(seed_sol.lambda_sol{3}(:)),cdfkp_initial_gamma.lambda_inds{3}(:));
%   cdfkp_initial_gamma = cdfkp_initial_gamma.addBoundingBoxConstraint(ConstantConstraint(seed_sol.lambda_sol{4}(:)),cdfkp_initial_gamma.lambda_inds{4}(:));
%
%   cdfkp_initial_gamma = cdfkp_initial_gamma.setSolverOptions('snopt','print','test_monkeybar_land_seed.out');
%   cdfkp_initial_gamma = cdfkp_initial_gamma.setSolverOptions('snopt','majorfeasibilitytolerance',1e-5);
%   tic
%   [x_seed,F,info] = cdfkp_initial_gamma.solve(seed_x_sol);
%   toc
%   keyboard
  tic
  [x_sol,F,info] = cdfkp.solve(seed_sol.x_sol);
  toc
  [q_sol,v_sol,h_sol,t_sol,com_sol,comdot_sol,comddot_sol,H_sol,Hdot_sol,lambda_sol,wrench_sol] = parseSolution(cdfkp,x_sol);
  xtraj_sol = PPTrajectory(foh(cumsum([0 h_sol]),[q_sol;v_sol]));
  xtraj_sol = xtraj_sol.setOutputFrame(robot.getStateFrame);
  keyboard;
end

if(mode == 7)
  q_start = q_fp;
  q_start(1:3) = q_start(1:3)+monkeybar_pos4(1:3)-monkeybar_pos1(1:3);
  kinsol_start = robot.doKinematics(q_start);
  com_start = robot.getCOM(kinsol_start);
  cdfkp = planBarSwing(robot,l_hand,r_hand,l_hand_pt,r_hand_pt,l_hand_grasp_axis,r_hand_grasp_axis,bar_length,bar_width,monkeybar_pos4,monkeybar_pos5,q_start);

  nT_swing1 = cdfkp.N;
  x_seed = zeros(cdfkp.num_vars,1);
  x_seed(cdfkp.h_inds) = 0.1;
  x_seed(cdfkp.q_inds(:)) = reshape(bsxfun(@times,q_start,ones(1,nT_swing1)),[],1);
  x_seed(cdfkp.com_inds(:)) = reshape(bsxfun(@times,com_start,ones(1,nT_swing1)),[],1);
  x_seed(cdfkp.lambda_inds{1}(:)) = reshape(bsxfun(@times,[0;0;1/2;zeros(3,1)],ones(1,nT_swing1)),[],1);
  x_seed(cdfkp.lambda_inds{2}(:)) = reshape(bsxfun(@times,[0;0;1/2;zeros(3,1)],ones(1,nT_swing1)),[],1);

  cdfkp = cdfkp.setSolverOptions('snopt','iterationslimit',1e6);
  cdfkp = cdfkp.setSolverOptions('snopt','majoriterationslimit',500);
  cdfkp = cdfkp.setSolverOptions('snopt','majorfeasibilitytolerance',1e-6);
  cdfkp = cdfkp.setSolverOptions('snopt','majoroptimalitytolerance',2e-4);
  cdfkp = cdfkp.setSolverOptions('snopt','superbasicslimit',2000);
  cdfkp = cdfkp.setSolverOptions('snopt','print','test_monkeybar_swing4.out');
%   cdfkp = cdfkp.setSolverOptions('snopt','scaleoption',2);

  seed_sol = load('test_monkeybar_swing4','-mat','x_sol');
% %   seed_sol.x_sol(cdfkp.q_inds(1:3,:)) = reshape(reshape(seed_sol.x_sol(cdfkp.q_inds(1:3,:)),3,[])+bsxfun(@times,monkeybar_pos4(1:3)-monkeybar_pos1(1:3),ones(1,nT_swing1)),[],1);
% %   seed_sol.x_sol(cdfkp.com_inds(:)) = reshape(reshape(seed_sol.x_sol(cdfkp.com_inds(:)),3,[])+bsxfun(@times,monkeybar_pos4(1:3)-monkeybar_pos1(1:3),ones(1,nT_swing1)),[],1);
% seed_sol.x_sol(cdfkp.lambda_inds{1}(1:3,:,:)) = seed_sol.x_sol(cdfkp.lambda_inds{1}(1:3,:,:))/(robot_mass*9.81);
%   seed_sol.x_sol(cdfkp.lambda_inds{1}(4:6,:,:)) = seed_sol.x_sol(cdfkp.lambda_inds{1}(4:6,:,:))/(robot_mass*9.81/100);
%   seed_sol.x_sol(cdfkp.lambda_inds{2}(1:3,:,:)) = seed_sol.x_sol(cdfkp.lambda_inds{2}(1:3,:,:))/(robot_mass*9.81);
%   seed_sol.x_sol(cdfkp.lambda_inds{2}(4:6,:,:)) = seed_sol.x_sol(cdfkp.lambda_inds{2}(4:6,:,:))/(robot_mass*9.81/100);

  tic
  [x_sol,F,info] = cdfkp.solve(seed_sol.x_sol);
  toc
  [q_sol,v_sol,h_sol,t_sol,com_sol,comdot_sol,comddot_sol,H_sol,Hdot_sol,lambda_sol,wrench_sol] = parseSolution(cdfkp,x_sol);
  xtraj_sol = PPTrajectory(foh(cumsum([0 h_sol]),[q_sol;v_sol]));
  xtraj_sol = xtraj_sol.setOutputFrame(robot.getStateFrame);
  keyboard;
end

if(mode == 8)
  % lead with left hand
  lhand_ungrasp_idx1 = 3;
  lhand_grasp_idx1 = 6;
  static_idx1 = 9;
  nT_swing1 = static_idx1;

  lhand_bar1_grasp_wrench = generateBarGraspingWrench(robot,l_hand,l_hand_pt,monkeybar_pos1);
  rhand_bar1_grasp_wrench = generateBarGraspingWrench(robot,r_hand,r_hand_pt,monkeybar_pos1);
  lhand_bar2_grasp_wrench = generateBarGraspingWrench(robot,l_hand,l_hand_pt,monkeybar_pos2);


  bar_contact_wrench_struct = struct('active_knot',[],'cw',[]);
  bar_contact_wrench_struct(1) = struct('active_knot',1:lhand_ungrasp_idx1,'cw',lhand_bar1_grasp_wrench);
  bar_contact_wrench_struct(2) = struct('active_knot',lhand_grasp_idx1:static_idx1,'cw',lhand_bar2_grasp_wrench);
  bar_contact_wrench_struct(3) = struct('active_knot',1:static_idx1,'cw',rhand_bar1_grasp_wrench);


  q_start = q_fp;
  com_start = com_fp;

  tf_range = [0.5 2];
  q_nom = bsxfun(@times,q_start,ones(1,static_idx1));
  Q_comddot = eye(3);
  Q = eye(nq);
  Q(1,1) = 0;
  Q(2,2) = 0;
  Qv = 0.1*eye(nv);
  % Q_contact_force = 20/(robot.getMass*9.81)^2*eye(3);
  Q_contact_force = zeros(3);
  cdfkp = ComDynamicsFullKinematicsPlanner(robot,nT_swing1,tf_range,Q_comddot,Qv,Q,q_nom,Q_contact_force,bar_contact_wrench_struct);

  % add initial and final state constraint

  cdfkp = cdfkp.addBoundingBoxConstraint(ConstantConstraint(q_start),cdfkp.q_inds(:,1));
  cdfkp = cdfkp.addBoundingBoxConstraint(ConstantConstraint(zeros(nv,1)),cdfkp.v_inds(:,1));
  cdfkp = cdfkp.addBoundingBoxConstraint(ConstantConstraint(zeros(nv,1)),cdfkp.v_inds(:,static_idx1));

  % add grasp kinematic constraint
  [~,rhand_grasp_dir_cnstr] = generateBarGraspConstraint(robot,r_hand,r_hand_pt,r_hand_grasp_axis,monkeybar_pos1,bar_length,bar_width);
  cdfkp = cdfkp.addRigidBodyConstraint(WorldPositionConstraint(robot,r_hand,r_hand_pt,r_hand_pos_fp(1:3),r_hand_pos_fp(1:3)),num2cell(2:static_idx1));
  cdfkp = cdfkp.addRigidBodyConstraint(rhand_grasp_dir_cnstr,num2cell(2:static_idx1));

  [~,lhand_grasp_dir_cnstr1] = generateBarGraspConstraint(robot,l_hand,l_hand_pt,l_hand_grasp_axis,monkeybar_pos1,bar_length,bar_width);
  cdfkp = cdfkp.addRigidBodyConstraint(WorldPositionConstraint(robot,l_hand,l_hand_pt,l_hand_pos_fp(1:3),l_hand_pos_fp(1:3)),num2cell(2:lhand_ungrasp_idx1));
  cdfkp = cdfkp.addRigidBodyConstraint(lhand_grasp_dir_cnstr1,num2cell(2:lhand_ungrasp_idx1));

  [lhand_grasp_pt_cnstr2,lhand_grasp_dir_cnstr2,lhand_grasp_fixed_pos_cnstr2] = generateBarGraspConstraint(robot,l_hand,l_hand_pt,l_hand_grasp_axis,monkeybar_pos2,bar_length,bar_width);
  cdfkp = cdfkp.addRigidBodyConstraint(lhand_grasp_pt_cnstr2,num2cell(lhand_grasp_idx1:static_idx1));
  cdfkp = cdfkp.addRigidBodyConstraint(lhand_grasp_dir_cnstr2,num2cell(lhand_grasp_idx1:static_idx1));
  cdfkp = cdfkp.addRigidBodyConstraint(lhand_grasp_fixed_pos_cnstr2,{lhand_grasp_idx1:static_idx1});

  % add h bounds
  cdfkp = cdfkp.addBoundingBoxConstraint(BoundingBoxConstraint(0.02*ones(nT_swing1-1,1),0.2*ones(nT_swing1-1,1)),cdfkp.h_inds(:));

  % add comddot bounds
  cdfkp = cdfkp.addBoundingBoxConstraint(BoundingBoxConstraint(...
    reshape(bsxfun(@times,[-9.81/2;-9.81/2;-1.5*9.81],ones(1,nT_swing1)),[],1),...
    reshape(bsxfun(@times,[9.81/2;9.81/2;1.5*9.81],ones(1,nT_swing1)),[],1)),cdfkp.comddot_inds(:));

  cdfkp = cdfkp.setSolverOptions('snopt','iterationslimit',1e6);
  cdfkp = cdfkp.setSolverOptions('snopt','majoriterationslimit',3000);
  cdfkp = cdfkp.setSolverOptions('snopt','majorfeasibilitytolerance',1e-6);
  cdfkp = cdfkp.setSolverOptions('snopt','majoroptimalitytolerance',1e-3);
  cdfkp = cdfkp.setSolverOptions('snopt','superbasicslimit',2000);
  cdfkp = cdfkp.setSolverOptions('snopt','print','test_monkeybar_swing_start.out');

%   seed_sol = load('test_monkeybar_swing','-mat','h_sol','q_sol','v_sol','com_sol','comdot_sol','comddot_sol','H_sol','Hdot_sol','lambda_sol');
%   seed_x_sol = zeros(cdfkp.num_vars,1);
%   seed_x_sol(cdfkp.h_inds(:)) = reshape(seed_sol.h_sol(:,1:static_idx1-1),[],1);
%   seed_x_sol(cdfkp.q_inds(:)) = reshape(seed_sol.q_sol(:,1:static_idx1),[],1);
%   seed_x_sol(cdfkp.v_inds(:)) = reshape(seed_sol.v_sol(:,1:static_idx1),[],1);
%   seed_x_sol(cdfkp.com_inds(:)) = reshape(seed_sol.com_sol(:,1:static_idx1),[],1);
%   seed_x_sol(cdfkp.comdot_inds(:)) = reshape(seed_sol.comdot_sol(:,1:static_idx1),[],1);
%   seed_x_sol(cdfkp.comddot_inds(:)) = reshape(seed_sol.comddot_sol(:,1:static_idx1),[],1);
%   seed_x_sol(cdfkp.lambda_inds{1}(:)) = reshape(seed_sol.lambda_sol{1}(:,:,1:static_idx1),[],1);
%   seed_x_sol(cdfkp.lambda_inds{2}(:)) = reshape(seed_sol.lambda_sol{2}(:,:,1:static_idx1),[],1);

  seed_x_sol = zeros(cdfkp.num_vars,1);
  seed_x_sol(cdfkp.h_inds) = 0.1*ones(static_idx1-1,1);
  seed_x_sol(cdfkp.q_inds(:)) = reshape(bsxfun(@times,ones(1,nT_swing1),q_start),[],1);
  seed_x_sol(cdfkp.com_inds(:)) = reshape(bsxfun(@times,ones(1,nT_swing1),com_start),[],1);
  seed_x_sol(cdfkp.lambda_inds{1}(:)) = reshape(bsxfun(@times,[0;0;1/2;0;0;0],ones(1,nT_swing1)),[],1);
  seed_x_sol(cdfkp.lambda_inds{2}(:)) = reshape(bsxfun(@times,[0;0;1/2;0;0;0],ones(1,nT_swing1)),[],1);
  seed_sol = load('test_monkeybar_swing_start','-mat','x_sol');
  tic
  [x_sol,F,info] = cdfkp.solve(seed_sol.x_sol);
  toc
  [q_sol,v_sol,h_sol,t_sol,com_sol,comdot_sol,comddot_sol,H_sol,Hdot_sol,lambda_sol,wrench_sol] = parseSolution(cdfkp,x_sol);
  xtraj_sol = PPTrajectory(foh(t_sol,[q_sol;v_sol]));
  xtraj_sol = xtraj_sol.setOutputFrame(robot.getStateFrame);
  keyboard
end

if(mode == 9)
  previous_swing = load('test_monkeybar_swing_start','-mat','q_sol','v_sol');
  cdfkp = planBarDoubleSwing(robot,l_hand,r_hand,l_hand_pt,r_hand_pt,l_hand_grasp_axis,r_hand_grasp_axis,bar_length,bar_width,monkeybar_pos1,monkeybar_pos2,monkeybar_pos3,previous_swing.q_sol(:,end),previous_swing.v_sol(:,end),'monkeybar2');

  nT = cdfkp.N;
  kinsol_start = robot.doKinematics(previous_swing.q_sol(:,end));
  com_start = robot.getCOM(kinsol_start);
  x_seed = zeros(cdfkp.num_vars,1);
  x_seed(cdfkp.h_inds(:)) = 0.1;
  x_seed(cdfkp.q_inds(:)) = reshape(bsxfun(@times,previous_swing.q_sol(:,end),ones(1,nT)),[],1);
  x_seed(cdfkp.com_inds(:)) = reshape(bsxfun(@times,com_start,ones(1,nT)),[],1);
  x_seed(cdfkp.lambda_inds{1}(:)) = reshape(bsxfun(@times,[0;0;1/2;0;0;0],ones(1,nT)),[],1);
  x_seed(cdfkp.lambda_inds{2}(:)) = reshape(bsxfun(@times,[0;0;1/2;0;0;0],ones(1,nT)),[],1);

  cdfkp = cdfkp.setSolverOptions('snopt','iterationslimit',1e6);
  cdfkp = cdfkp.setSolverOptions('snopt','majoriterationslimit',3000);
  cdfkp = cdfkp.setSolverOptions('snopt','majorfeasibilitytolerance',1e-6);
  cdfkp = cdfkp.setSolverOptions('snopt','majoroptimalitytolerance',1e-3);
  cdfkp = cdfkp.setSolverOptions('snopt','superbasicslimit',2000);
  cdfkp = cdfkp.setSolverOptions('snopt','print','test_monkeybar_doubleswing1.out');

  seed_sol = load('test_monkeybar_doubleswing1','-mat','x_sol');
  tic
  [x_sol,F,info] = cdfkp.solve(seed_sol.x_sol);
  toc
  [q_sol,v_sol,h_sol,t_sol,com_sol,comdot_sol,comddot_sol,H_sol,Hdot_sol,lambda_sol,wrench_sol] = parseSolution(cdfkp,x_sol);
  xtraj_sol = PPTrajectory(foh(t_sol,[q_sol;v_sol]));
  xtraj_sol = xtraj_sol.setOutputFrame(robot.getStateFrame);
  keyboard
end
end

function [grasp_pt_cnstr,grasp_dir_cnstr,grasp_fixed_pos_cnstr] = generateBarGraspConstraint(robot,hand_idx,hand_pt,hand_grasp_axis,bar_pos,bar_length,bar_width)
T_bar_to_world = [rpy2rotmat(bar_pos(4:6)) bar_pos(1:3);0 0 0 1];
bar_axis = [1;0;0];
world_bar_axis = [eye(3) zeros(3,1)]*T_bar_to_world*[bar_axis;0];
% grasp_pt_cnstr = WorldPositionInFrameConstraint(robot,hand_idx,hand_pt,T_bar_to_world,[-bar_length*0.45;0;0],[bar_length*0.45;0;0]);
grasp_pt_cnstr = WorldPositionConstraint(robot,hand_idx,hand_pt,[bar_pos(1);bar_pos(2)-bar_length*0.45;bar_pos(3)],[bar_pos(1);bar_pos(2)+bar_length*0.45;bar_pos(3)]);
grasp_cone_threshold = pi/12;
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

function cnstr = legSymmetricConstraint(robot,t_idx)
N = length(t_idx);
num_cnstr = 6;
nq = robot.getNumPositions();
A = zeros(num_cnstr,nq);
l_leg_hpz = robot.getBody(robot.findJointInd('l_leg_hpz')).position_num;
r_leg_hpz = robot.getBody(robot.findJointInd('r_leg_hpz')).position_num;
A(1,[l_leg_hpz r_leg_hpz]) = [1 1];
l_leg_hpx = robot.getBody(robot.findJointInd('l_leg_hpx')).position_num;
r_leg_hpx = robot.getBody(robot.findJointInd('r_leg_hpx')).position_num;
A(2,[l_leg_hpx r_leg_hpx]) = [1 1];
l_leg_hpy = robot.getBody(robot.findJointInd('l_leg_hpy')).position_num;
r_leg_hpy = robot.getBody(robot.findJointInd('r_leg_hpy')).position_num;
A(3,[l_leg_hpy r_leg_hpy]) = [1 -1];
l_leg_kny = robot.getBody(robot.findJointInd('l_leg_kny')).position_num;
r_leg_kny = robot.getBody(robot.findJointInd('r_leg_kny')).position_num;
A(4,[l_leg_kny r_leg_kny]) = [1 -1];
l_leg_aky = robot.getBody(robot.findJointInd('l_leg_aky')).position_num;
r_leg_aky = robot.getBody(robot.findJointInd('r_leg_aky')).position_num;
A(5,[l_leg_aky r_leg_aky]) = [1 -1];
l_leg_akx = robot.getBody(robot.findJointInd('l_leg_akx')).position_num;
r_leg_akx = robot.getBody(robot.findJointInd('r_leg_akx')).position_num;
A(6,[l_leg_akx r_leg_akx]) = [1 1];
cnstr = LinearConstraint(zeros(num_cnstr*N,1),zeros(num_cnstr*N,1),kron(speye(N),A));
cnstr_name = cell(num_cnstr*N,1);
for i = 1:N
  cnstr_name{(i-1)*num_cnstr+1} = sprintf('l_leg_hpz symmetry[%d]',t_idx(i));
  cnstr_name{(i-1)*num_cnstr+2} = sprintf('l_leg_hpx symmetry[%d]',t_idx(i));
  cnstr_name{(i-1)*num_cnstr+3} = sprintf('l_leg_hpy symmetry[%d]',t_idx(i));
  cnstr_name{(i-1)*num_cnstr+4} = sprintf('l_leg_kny symmetry[%d]',t_idx(i));
  cnstr_name{(i-1)*num_cnstr+5} = sprintf('l_leg_aky symmetry[%d]',t_idx(i));
  cnstr_name{(i-1)*num_cnstr+6} = sprintf('l_leg_akx symmetry[%d]',t_idx(i));
end
cnstr = cnstr.setName(cnstr_name);
end

function symmetry_cnstr = armSymmetricConstraint(robot,t_idx)
N = numel(t_idx);
num_symmetry = 5;
symmetric_matrix = zeros(num_symmetry,robot.getNumPositions);
l_arm_usy_idx = robot.getBody(robot.findJointInd('l_arm_usy')).position_num;
r_arm_usy_idx = robot.getBody(robot.findJointInd('r_arm_usy')).position_num;
symmetric_matrix(1,[l_arm_usy_idx r_arm_usy_idx]) = [1 -1];
l_arm_shx_idx = robot.getBody(robot.findJointInd('l_arm_shx')).position_num;
r_arm_shx_idx = robot.getBody(robot.findJointInd('r_arm_shx')).position_num;
symmetric_matrix(2,[l_arm_shx_idx r_arm_shx_idx]) = [1 1];
l_arm_ely_idx = robot.getBody(robot.findJointInd('l_arm_ely')).position_num;
r_arm_ely_idx = robot.getBody(robot.findJointInd('r_arm_ely')).position_num;
symmetric_matrix(3,[l_arm_ely_idx r_arm_ely_idx]) = [1 -1];
l_arm_elx_idx = robot.getBody(robot.findJointInd('l_arm_elx')).position_num;
r_arm_elx_idx = robot.getBody(robot.findJointInd('r_arm_elx')).position_num;
symmetric_matrix(4,[l_arm_elx_idx r_arm_elx_idx]) = [1 1];
l_arm_uwy_idx = robot.getBody(robot.findJointInd('l_arm_uwy')).position_num;
r_arm_uwy_idx = robot.getBody(robot.findJointInd('r_arm_uwy')).position_num;
symmetric_matrix(5,[l_arm_uwy_idx r_arm_uwy_idx]) = [1 -1];
symmetry_cnstr = LinearConstraint(zeros(num_symmetry*N,1),zeros(num_symmetry*N,1),kron(speye(N),symmetric_matrix));
cnstr_name = cell(num_symmetry*N,1);
for i = 1:N
  cnstr_name{(i-1)*num_symmetry+1} = sprintf('arm_usy_symmetry[%d]',t_idx(i));
  cnstr_name{(i-1)*num_symmetry+2} = sprintf('arm_shx_symmetry[%d]',t_idx(i));
  cnstr_name{(i-1)*num_symmetry+3} = sprintf('arm_ely_symmetry[%d]',t_idx(i));
  cnstr_name{(i-1)*num_symmetry+4} = sprintf('arm_elx_symmetry[%d]',t_idx(i));
  cnstr_name{(i-1)*num_symmetry+5} = sprintf('arm_uwy_symmetry[%d]',t_idx(i));
end
symmetry_cnstr = symmetry_cnstr.setName(cnstr_name);
end

function wrench = generateBarGraspingWrench(robot,hand,hand_pt,bar_pos)
robot_mass = robot.getMass;
g = 9.81;
force_max = robot_mass*g*1.5;
torque_axis_max = robot_mass*g*0.5;
torque_perp_max = robot_mass*g*0.05;
A_torque = eye(3);
b_torque_ub = [torque_axis_max;torque_perp_max;torque_perp_max];
b_torque_lb = -b_torque_ub;
T_bar_to_world = [rpy2rotmat(bar_pos(4:6)) bar_pos(1:3);0 0 0 1];
A_torque = A_torque*T_bar_to_world(1:3,1:3)';
wrench = GraspWrench(robot,hand,hand_pt,force_max,A_torque,b_torque_lb,b_torque_ub);
end

function [c,dc] = graspDistance(kinsol,robot,l_hand,r_hand,l_hand_pt,r_hand_pt)
[l_hand_pos,J_lhand] = robot.forwardKin(kinsol,l_hand,l_hand_pt,0);
[r_hand_pos,J_rhand] = robot.forwardKin(kinsol,r_hand,r_hand_pt,0);
displacement = l_hand_pos-r_hand_pos;
c = sum(displacement.^2);
dc = 2*displacement'*J_lhand-2*displacement'*J_rhand;
end


function cdfkp = planBarDoubleSwing(robot,fix_hand,swing_hand,fix_hand_pt,swing_hand_pt,fix_hand_grasp_axis,swing_hand_grasp_axis,bar_length,bar_width,bar1_pos,bar2_pos,bar3_pos,q_start,v_start,middle_bar_collision_group)
nq = robot.getNumPositions();
nv = robot.getNumVelocities();

ungrasp_idx = 4;
grasp_idx = 10;
final_idx = 13;
nT_swing = final_idx;

fix_hand_bar_grasp_wrench = generateBarGraspingWrench(robot,fix_hand,fix_hand_pt,bar2_pos);
swing_hand_bar_grasp_wrench1 = generateBarGraspingWrench(robot,swing_hand,swing_hand_pt,bar1_pos);
swing_hand_bar_grasp_wrench3 = generateBarGraspingWrench(robot,swing_hand,swing_hand_pt,bar3_pos);

bar_contact_wrench_struct = struct('active_knot',[],'cw',[]);
bar_contact_wrench_struct(1) = struct('active_knot',1:final_idx,'cw',fix_hand_bar_grasp_wrench);
bar_contact_wrench_struct(2) = struct('active_knot',1:ungrasp_idx,'cw',swing_hand_bar_grasp_wrench1);
bar_contact_wrench_struct(3) = struct('active_knot',grasp_idx:final_idx,'cw',swing_hand_bar_grasp_wrench3);

tf_range = [1 3];
q_nom = bsxfun(@times,q_start,ones(1,nT_swing));
Q_comddot = eye(3);
Q = eye(nq);
Q(1,1) = 0;
Q(2,2) = 0;
Qv = 0.1*eye(nv);
Q_contact_force = 20/(robot.getMass*9.81)^2*eye(3);
% Q_contact_force = zeros(3);
cdfkp = ComDynamicsFullKinematicsPlanner(robot,nT_swing,tf_range,Q_comddot,Qv,Q,q_nom,Q_contact_force,bar_contact_wrench_struct);

% h constraint
cdfkp = cdfkp.addBoundingBoxConstraint(BoundingBoxConstraint(0.02*ones(nT_swing-1,1),0.2*ones(nT_swing-1,1)),cdfkp.h_inds(:));

% initial and final state constraint
cdfkp = cdfkp.addBoundingBoxConstraint(ConstantConstraint(q_start),cdfkp.q_inds(:,1));
cdfkp = cdfkp.addBoundingBoxConstraint(ConstantConstraint(v_start),cdfkp.v_inds(:,1));
cdfkp = cdfkp.addBoundingBoxConstraint(ConstantConstraint(zeros(nv,1)),cdfkp.v_inds(:,final_idx));

% add grasp kinematic constraint
kinsol_start = robot.doKinematics(q_start);
fix_hand_pos_start = robot.forwardKin(kinsol_start,fix_hand,fix_hand_pt,2);
swing_hand_pos_start = robot.forwardKin(kinsol_start,swing_hand,swing_hand_pt,2);
cdfkp = cdfkp.addRigidBodyConstraint(WorldPositionConstraint(robot,fix_hand,fix_hand_pt,fix_hand_pos_start(1:3),fix_hand_pos_start(1:3)),num2cell(2:final_idx));
cdfkp = cdfkp.addRigidBodyConstraint(WorldPositionConstraint(robot,swing_hand,swing_hand_pt,swing_hand_pos_start(1:3),swing_hand_pos_start(1:3)),num2cell(2:ungrasp_idx));

[~,fix_hand_grasp_dir_cnstr] = generateBarGraspConstraint(robot,fix_hand,fix_hand_pt,fix_hand_grasp_axis,bar2_pos,bar_length,bar_width);
cdfkp = cdfkp.addRigidBodyConstraint(fix_hand_grasp_dir_cnstr,num2cell(2:final_idx));
[~,swing_hand_grasp_dir_cnstr1] = generateBarGraspConstraint(robot,swing_hand,swing_hand_pt,swing_hand_grasp_axis,bar1_pos,bar_length,bar_width);
cdfkp = cdfkp.addRigidBodyConstraint(swing_hand_grasp_dir_cnstr1,num2cell(2:ungrasp_idx));
[swing_hand_grasp_pt_cnstr3,swing_hand_grasp_dir_cnstr3,swing_hand_fixed_pos_cnstr3] = generateBarGraspConstraint(robot,swing_hand,swing_hand_pt,swing_hand_grasp_axis,bar3_pos,bar_length,bar_width);
cdfkp = cdfkp.addRigidBodyConstraint(swing_hand_grasp_pt_cnstr3,{grasp_idx});
cdfkp = cdfkp.addRigidBodyConstraint(swing_hand_grasp_dir_cnstr3,num2cell(grasp_idx:final_idx));
cdfkp = cdfkp.addRigidBodyConstraint(swing_hand_fixed_pos_cnstr3,{grasp_idx:final_idx});

% add com height upper bound
cdfkp = cdfkp.addBoundingBoxConstraint(BoundingBoxConstraint(-inf(nT_swing-1,1),(max([bar1_pos(3) bar2_pos(3) bar3_pos(3)])-0.3)*ones(nT_swing-1,1)),cdfkp.com_inds(3,2:final_idx)');

% add comddot bounds
cdfkp = cdfkp.addBoundingBoxConstraint(BoundingBoxConstraint(...
  reshape(bsxfun(@times,[-9.81;-9.81;-1.5*9.81],ones(1,nT_swing)),[],1),...
  reshape(bsxfun(@times,[9.81;9.81;1.5*9.81],ones(1,nT_swing)),[],1)),cdfkp.comddot_inds(:));

% add collision avoidance
% active_collision_options.body_idx = [swing_hand,1];
% active_collision_options.collision_group = {'default',middle_bar_collision_group};
% middle_bar_nocollision_cnstr = MinDistanceConstraint(robot,0.03,[-inf,inf],active_collision_options);
% cdfkp = cdfkp.addRigidBodyConstraint(middle_bar_nocollision_cnstr,num2cell(2:final_idx));
swing_middle_idx1 = floor((ungrasp_idx+grasp_idx)/2)+1;
swing_middle_idx2 = swing_middle_idx1+1;
cdfkp = cdfkp.addRigidBodyConstraint(WorldPositionConstraint(robot,swing_hand,swing_hand_pt,-inf(3,1),[inf;inf;bar2_pos(3)-0.2]),{swing_middle_idx1});
cdfkp = cdfkp.addRigidBodyConstraint(WorldPositionConstraint(robot,swing_hand,swing_hand_pt,-inf(3,1),[inf;inf;bar2_pos(3)-0.25]),{swing_middle_idx2});
end

function cdfkp = planBarSwing(robot,l_hand,r_hand,l_hand_pt,r_hand_pt,l_hand_grasp_axis,r_hand_grasp_axis,bar_length,bar_width,bar1_pos,bar2_pos,q_start)
nq = robot.getNumPositions();
nv = robot.getNumVelocities();

l_hand_ungrasp_idx1 = 3;
l_hand_grasp_idx2 = 6;
r_hand_ungrasp_idx1 = 9;
r_hand_grasp_idx2 = 12;
nT_swing = 15;

lhand_bar1_grasp_wrench = generateBarGraspingWrench(robot,l_hand,l_hand_pt,bar1_pos);
rhand_bar1_grasp_wrench = generateBarGraspingWrench(robot,r_hand,r_hand_pt,bar1_pos);
lhand_bar2_grasp_wrench = generateBarGraspingWrench(robot,l_hand,l_hand_pt,bar2_pos);
rhand_bar2_grasp_wrench = generateBarGraspingWrench(robot,r_hand,r_hand_pt,bar2_pos);


bar_contact_wrench_struct = struct('active_knot',[],'cw',[]);
bar_contact_wrench_struct(1) = struct('active_knot',1:l_hand_ungrasp_idx1,'cw',lhand_bar1_grasp_wrench);
bar_contact_wrench_struct(2) = struct('active_knot',l_hand_grasp_idx2:nT_swing,'cw',lhand_bar2_grasp_wrench);
bar_contact_wrench_struct(3) = struct('active_knot',1:r_hand_ungrasp_idx1,'cw',rhand_bar1_grasp_wrench);
bar_contact_wrench_struct(4) = struct('active_knot',r_hand_grasp_idx2:nT_swing,'cw',rhand_bar2_grasp_wrench);


tf_range = [0.5 2];
q_nom = bsxfun(@times,q_start,ones(1,nT_swing));
Q_comddot = eye(3);
Q = eye(nq);
Q(1,1) = 0;
Q(2,2) = 0;
Qv = 0.1*eye(nv);
% Q_contact_force = 20/(robot.getMass*9.81)^2*eye(3);
Q_contact_force = zeros(3);
cdfkp = ComDynamicsFullKinematicsPlanner(robot,nT_swing,tf_range,Q_comddot,Qv,Q,q_nom,Q_contact_force,bar_contact_wrench_struct);

kinsol_start = robot.doKinematics(q_start);
lhand_pos_start = robot.forwardKin(kinsol_start,l_hand,l_hand_pt,2);
rhand_pos_start = robot.forwardKin(kinsol_start,r_hand,r_hand_pt,2);

[~,lhand_bar1_grasp_dir_cnstr] = generateBarGraspConstraint(robot,l_hand,l_hand_pt,l_hand_grasp_axis,bar1_pos,bar_length,bar_width);
lhand_bar1_grasp_pt_cnstr = WorldPositionConstraint(robot,l_hand,l_hand_pt,lhand_pos_start(1:3),lhand_pos_start(1:3));
cdfkp = cdfkp.addRigidBodyConstraint(lhand_bar1_grasp_pt_cnstr,num2cell(2:l_hand_ungrasp_idx1));
cdfkp = cdfkp.addRigidBodyConstraint(lhand_bar1_grasp_dir_cnstr,num2cell(2:l_hand_ungrasp_idx1));

[~,rhand_bar1_grasp_dir_cnstr] = generateBarGraspConstraint(robot,r_hand,r_hand_pt,r_hand_grasp_axis,bar1_pos,bar_length,bar_width);
rhand_bar1_grasp_pt_cnstr = WorldPositionConstraint(robot,r_hand,r_hand_pt,rhand_pos_start(1:3),rhand_pos_start(1:3));
cdfkp = cdfkp.addRigidBodyConstraint(rhand_bar1_grasp_pt_cnstr,num2cell(2:r_hand_ungrasp_idx1));
cdfkp = cdfkp.addRigidBodyConstraint(rhand_bar1_grasp_dir_cnstr,num2cell(2:r_hand_ungrasp_idx1));

[lhand_bar2_grasp_pt_cnstr,lhand_bar2_grasp_dir_cnstr,lhand_bar2_fixed_pos_cnstr] = generateBarGraspConstraint(robot,l_hand,l_hand_pt,l_hand_grasp_axis,bar2_pos,bar_length,bar_width);
% cdfkp = cdfkp.addRigidBodyConstraint(lhand_bar2_grasp_pt_cnstr,{l_hand_grasp_idx2});
cdfkp = cdfkp.addRigidBodyConstraint(lhand_bar2_grasp_dir_cnstr,num2cell(l_hand_grasp_idx2:nT_swing-1));
cdfkp = cdfkp.addRigidBodyConstraint(lhand_bar2_fixed_pos_cnstr,{l_hand_grasp_idx2:nT_swing-1});

[rhand_bar2_grasp_pt_cnstr,rhand_bar2_grasp_dir_cnstr,rhand_bar2_fixed_pos_cnstr] = generateBarGraspConstraint(robot,r_hand,r_hand_pt,r_hand_grasp_axis,bar2_pos,bar_length,bar_width);
% cdfkp = cdfkp.addRigidBodyConstraint(rhand_bar2_grasp_pt_cnstr,{r_hand_grasp_idx2});
cdfkp = cdfkp.addRigidBodyConstraint(rhand_bar2_grasp_dir_cnstr,num2cell(r_hand_grasp_idx2:nT_swing-1));
cdfkp = cdfkp.addRigidBodyConstraint(rhand_bar2_fixed_pos_cnstr,{r_hand_grasp_idx2:nT_swing-1});

% fix initial and final state
cdfkp = cdfkp.addBoundingBoxConstraint(ConstantConstraint(q_start),cdfkp.q_inds(:,1));
q_end_xyz = q_start(1:3)+bar2_pos(1:3)-bar1_pos(1:3);
q_end_rpy = q_start(4:6);
q_end = [q_end_xyz;q_end_rpy;q_start(7:end)];
cdfkp = cdfkp.addBoundingBoxConstraint(ConstantConstraint(q_end),cdfkp.q_inds(:,nT_swing));
cdfkp = cdfkp.addBoundingBoxConstraint(ConstantConstraint(zeros(nv,1)),cdfkp.v_inds(:,1));
cdfkp = cdfkp.addBoundingBoxConstraint(ConstantConstraint(zeros(nv,1)),cdfkp.v_inds(:,nT_swing));

% add h bounds
cdfkp = cdfkp.addBoundingBoxConstraint(BoundingBoxConstraint(0.02*ones(nT_swing-1,1),0.2*ones(nT_swing-1,1)),cdfkp.h_inds(:));

% add com height upper bound
cdfkp = cdfkp.addCoMBounds(1:nT_swing,-inf(3,nT_swing),[inf(2,nT_swing);(max([bar1_pos(3) bar2_pos(3)])-0.3)*ones(1,nT_swing)]);

cdfkp = cdfkp.addBoundingBoxConstraint(BoundingBoxConstraint(...
  reshape(bsxfun(@times,[-9.81/2;-9.81/2;-1.5*9.81],ones(1,nT_swing)),[],1),...
  reshape(bsxfun(@times,[9.81/2;9.81/2;1.5*9.81],ones(1,nT_swing)),[],1)),cdfkp.comddot_inds(:));
end

function [phi,dphi] = stageDistance(pt,stage_pos)
n_pt = size(pt,2);
c = pt(3,:)-stage_pos(3)-0.05;
phi = c';
dphi = sparse((1:n_pt)',3*(1:n_pt)',ones(n_pt,1),n_pt,3*n_pt);
end
