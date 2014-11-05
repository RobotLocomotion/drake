function testSalmonBar(mode)
if(nargin<1)
  mode = 0;
end
checkDependency('lcmgl');
w = warning('off','Drake:RigidBody:SimplifiedCollisionGeometry');
warning('off','Drake:RigidBody:NonPositiveInertiaMatrix');
warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints');
warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits');
robot = RigidBodyManipulator([getDrakePath,'/examples/Atlas/urdf/atlas_convex_hull.urdf'],struct('floating',true));
nq = robot.getNumPositions();
nv = robot.getNumVelocities();
nu = robot.getNumInputs();
robot_mass = robot.getMass;
g = 9.81;
salmonbar_urdf = [getDrakePath,'/solvers/trajectoryOptimization/test/SalmonBar.urdf'];
salmonbar_pos = [0;0;0;0;0;pi];
robot = robot.addRobotFromURDF(salmonbar_urdf,salmonbar_pos(1:3),salmonbar_pos(4:6));

l_pole_pos = salmonbar_pos;
l_pole_pos(2) = salmonbar_pos(2)+1;
r_pole_pos = salmonbar_pos;
r_pole_pos(2) = salmonbar_pos(2)-1;

bar_radius = 0.025;
rung_urdf = [getDrakePath,'/solvers/trajectoryOptimization/test/SalmonBar_rung.urdf'];
l_rung_pos1 = zeros(6,1);
l_rung_pos1(1) = -0.07;
l_rung_pos1(2) = l_pole_pos(2);
l_rung_pos1(3) = 2.5;
l_rung_pos1(5) = pi/6;
r_rung_pos1 = zeros(6,1);
r_rung_pos1(1) = l_rung_pos1(1);
r_rung_pos1(2) = r_pole_pos(2);
r_rung_pos1(3) = l_rung_pos1(3);
r_rung_pos1(5) = pi/6;
robot = robot.addRobotFromURDF(rung_urdf,l_rung_pos1(1:3),l_rung_pos1(4:6));
robot = robot.addRobotFromURDF(rung_urdf,r_rung_pos1(1:3),r_rung_pos1(4:6));

nomdata = load([getDrakePath,'/examples/Atlas/data/atlas_fp.mat']);
qstar = nomdata.xstar(1:nq);
l_hand = robot.findLinkInd('l_hand');
r_hand = robot.findLinkInd('r_hand');
l_hand_pt = [0;0.25;0];
r_hand_pt = [0;-0.25;0];
l_hand_grasp_axis = [0;0;1];
r_hand_grasp_axis = [0;0;1];

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

fixed_pt_search = NonlinearProgram(nq+nu+6);
fixed_pt_search = fixed_pt_search.addSharedDataFunction(@(q) doKinematics(robot,q,true,true),(1:nq));
[hand_coaxial_cnstr] = generateBarGraspConstraint(robot,l_hand,r_hand,l_hand_pt,r_hand_pt,l_hand_grasp_axis,r_hand_grasp_axis,0);
hand_distance_cnstr = Point2PointDistanceConstraint(robot,l_hand,r_hand,l_hand_pt,r_hand_pt,1,2);
hand_distance_cnstr = hand_distance_cnstr.generateConstraint();
fixed_pt_search = fixed_pt_search.addDifferentiableConstraint(hand_distance_cnstr{1},(1:nq)',1);
fixed_pt_search = fixed_pt_search.addDifferentiableConstraint(hand_coaxial_cnstr,(1:nq)',1);
com_cnstr = WorldCoMConstraint(robot,[-inf;0;-inf],[-inf;0;l_rung_pos1(3)-0.5]);
com_cnstr = com_cnstr.generateConstraint([]);
com_cnstr = com_cnstr{1};
fixed_pt_search = fixed_pt_search.addDifferentiableConstraint(com_cnstr,(1:nq)',1);
[lhand_pos_cnstr,rhand_pos_cnstr] = generateBarOnRungConstraint(robot,l_hand,r_hand,l_hand_pt,r_hand_pt,l_hand_grasp_axis,r_hand_grasp_axis,l_rung_pos1,r_rung_pos1,l_pole_pos,r_pole_pos,bar_radius);
lhand_pos_cnstr = lhand_pos_cnstr.generateConstraint([]);
fixed_pt_search = fixed_pt_search.addDifferentiableConstraint(lhand_pos_cnstr{1},(1:nq)',1);
rhand_pos_cnstr = rhand_pos_cnstr.generateConstraint([]);
fixed_pt_search = fixed_pt_search.addDifferentiableConstraint(rhand_pos_cnstr{1},(1:nq)',1);
fixed_pt_search = fixed_pt_search.addDifferentiableConstraint(FunctionHandleConstraint(zeros(nq,1),zeros(nq,1),nq+nu+6,@(q,u,l_hand_force,r_hand_force,kinsol) fixedPointDynamics(robot,q,u,l_hand_force,r_hand_force,kinsol,l_hand,l_hand_pt,r_hand,r_hand_pt)),{(1:nq)';nq+(1:nu)';nq+nu+(1:3)';nq+nu+3+(1:3)'},1);
arm_symmetric_cnstr = armSymmetricConstraint(robot,1);
fixed_pt_search = fixed_pt_search.addLinearConstraint(arm_symmetric_cnstr,(1:nq)');
leg_symmetric_cnstr = legSymmetricConstraint(robot,1);
fixed_pt_search = fixed_pt_search.addLinearConstraint(leg_symmetric_cnstr,(1:nq)');
[q_lb,q_ub] = robot.getJointLimits();
fixed_pt_search = fixed_pt_search.addBoundingBoxConstraint(BoundingBoxConstraint(q_lb,q_ub),(1:nq)');
fixed_pt_search = fixed_pt_search.addBoundingBoxConstraint(ConstantConstraint(0),6);
fixed_pt_search = fixed_pt_search.addBoundingBoxConstraint(BoundingBoxConstraint([q_lb(l_arm_elx)+0.1;q_lb(r_arm_elx)],[q_ub(l_arm_elx);q_ub(r_arm_elx)-0.1]),[l_arm_elx;r_arm_elx]);
fixed_pt_search = fixed_pt_search.addBoundingBoxConstraint(BoundingBoxConstraint([0;-0.2],[0.2;0]),[l_leg_hpz;r_leg_hpz]);
fixed_pt_search = fixed_pt_search.addBoundingBoxConstraint(BoundingBoxConstraint(zeros(2,1),inf(2,1)),[l_leg_aky;r_leg_aky]);
input_coordinates = robot.getInputFrame.coordinates;
back_bkx_motor = (strcmp(input_coordinates,'back_bkx_motor'));
back_bky_motor = (strcmp(input_coordinates,'back_bky_motor'));
back_bkz_motor = (strcmp(input_coordinates,'back_bkz_motor'));
l_leg_kny_motor = (strcmp(input_coordinates,'l_leg_kny_motor'));
r_leg_kny_motor = (strcmp(input_coordinates,'r_leg_kny_motor'));
l_leg_aky_motor = (strcmp(input_coordinates,'l_leg_aky_motor'));
r_leg_aky_motor = (strcmp(input_coordinates,'r_leg_aky_motor'));
l_leg_akx_motor = (strcmp(input_coordinates,'l_leg_akx_motor'));
r_leg_akx_motor = (strcmp(input_coordinates,'r_leg_akx_motor'));
R = ones(nu,1);
R(back_bkx_motor) = 100;
R(back_bky_motor) = 100;
R(back_bkz_motor) = 100;
R(l_leg_kny_motor) = 100;
R(r_leg_kny_motor) = 100;
R(l_leg_aky_motor) = 100;
R(r_leg_aky_motor) = 100;
R(l_leg_akx_motor) = 100;
R(r_leg_akx_motor) = 100;
R = diag(R);
fixed_pt_search = fixed_pt_search.addCost(QuadraticConstraint(-inf,inf,R,zeros(nu,1)),nq+(1:nu));
q_fp_seed = qstar;
q_fp_seed(l_arm_shx) = pi/2;
q_fp_seed(r_arm_shx) = -pi/2;
fixed_pt_search = fixed_pt_search.setSolverOptions('snopt','majoriterationslimit',2000);
[x_fp,F,info] = fixed_pt_search.solve([q_fp_seed;zeros(nu,1);[0;0;robot_mass*9.81/2];[0;0;robot_mass*9.81/2]]);
q_fp = x_fp(1:nq);
kinsol_fp = robot.doKinematics(q_fp);
l_hand_pos_fp = robot.forwardKin(kinsol_fp,l_hand,l_hand_pt,2);
r_hand_pos_fp = robot.forwardKin(kinsol_fp,r_hand,r_hand_pt,2);
hand_distance = norm(l_hand_pos_fp(1:3)-r_hand_pos_fp(1:3));
% drawBar(l_hand_pos_fp(1:3),r_hand_pos_fp(1:3),bar_radius);

l_rung_pos2 = l_rung_pos1;
l_rung_pos2(3) = l_rung_pos2(3)+0.4;
r_rung_pos2 = r_rung_pos1;
r_rung_pos2(3) = r_rung_pos2(3)+0.4;
robot = robot.addRobotFromURDF(rung_urdf,l_rung_pos2(1:3),l_rung_pos2(4:6));
robot = robot.addRobotFromURDF(rung_urdf,r_rung_pos2(1:3),r_rung_pos2(4:6));
v = robot.constructVisualizer();
v.draw(0,[q_fp;zeros(nv,1)]);

if(mode == 0)
  sol = load('test_salmonbar_ladder1','-mat','t_sol','q_sol','v_sol','wrench_sol');
  xtraj = PPTrajectory(foh(sol.t_sol,[sol.q_sol;sol.v_sol]));
  xtraj = xtraj.setOutputFrame(robot.getStateFrame);
  v = SalmonBarVisualizer(robot,sol.t_sol,sol.wrench_sol,l_hand,r_hand,l_hand_pt,r_hand_pt);
  v.playback(xtraj,struct('slider',true));
end
if(mode == 1)
  q_start = q_fp;
  kinsol_start = robot.doKinematics(q_start);
  com_start = robot.getCOM(kinsol_start);
  cdfkp = climbOneStep(robot,l_hand,r_hand,l_hand_pt,r_hand_pt,l_hand_grasp_axis,r_hand_grasp_axis,l_rung_pos1,r_rung_pos1,l_rung_pos2,r_rung_pos2,l_pole_pos,r_pole_pos,bar_radius,q_start);

  cdfkp = cdfkp.setSolverOptions('snopt','iterationslimit',1e6);
  cdfkp = cdfkp.setSolverOptions('snopt','majoriterationslimit',1000);
  cdfkp = cdfkp.setSolverOptions('snopt','majorfeasibilitytolerance',2e-6);
  cdfkp = cdfkp.setSolverOptions('snopt','majoroptimalitytolerance',1e-3);
  cdfkp = cdfkp.setSolverOptions('snopt','superbasicslimit',2000);
  cdfkp = cdfkp.setSolverOptions('snopt','print','test_salmonbar_ladder1.out');

  nT = cdfkp.N;
  x_seed = zeros(cdfkp.num_vars,1);
  x_seed(cdfkp.h_inds(:)) = 0.1;
  x_seed(cdfkp.q_inds(:)) = reshape(bsxfun(@times,q_start,ones(1,nT)),[],1);
  x_seed(cdfkp.com_inds(:)) = reshape(bsxfun(@times,com_start,ones(1,nT)),[],1);
  x_seed(cdfkp.lambda_inds{1}(:)) = 1/16;

  seed_sol = load('test_salmonbar_ladder1','-mat','x_sol');
  tic
  [x_sol,F,info] = cdfkp.solve(seed_sol.x_sol);
  toc
  [q_sol,v_sol,h_sol,t_sol,com_sol,comdot_sol,comddot_sol,H_sol,Hdot_sol,lambda_sol,wrench_sol] = parseSolution(cdfkp,x_sol);
  xtraj_sol = PPTrajectory(foh(cumsum([0 h_sol]),[q_sol;v_sol]));
  xtraj_sol = xtraj_sol.setOutputFrame(robot.getStateFrame);
  keyboard;
end
end

function [hand_coaxial_cnstr,hand_distance_cnstr] = generateBarGraspConstraint(robot,l_hand,r_hand,l_hand_pt,r_hand_pt,l_hand_grasp_axis,r_hand_grasp_axis,hand_distance)
% hand_coaxial_cnstr1 = RelativeGazeTargetConstraint(robot,l_hand,r_hand,l_hand_grasp_axis,r_hand_pt,l_hand_pt,0);
% hand_coaxial_cnstr2 = RelativeGazeTargetConstraint(robot,r_hand,l_hand,r_hand_grasp_axis,l_hand_pt,r_hand_pt,0);
hand_coaxial_cnstr = FunctionHandleConstraint([0;-inf;zeros(6,1)],[inf;0;zeros(6,1)],robot.getNumPositions,@(~,kinsol) handCoaxialFun(robot,kinsol,l_hand,r_hand,l_hand_pt,r_hand_pt,l_hand_grasp_axis,r_hand_grasp_axis));
hand_distance_cnstr = Point2PointDistanceConstraint(robot,l_hand,r_hand,l_hand_pt,r_hand_pt,hand_distance,hand_distance);
end

function [lhand_pos_cnstr,rhand_pos_cnstr] = generateBarOnRungConstraint(robot,l_hand,r_hand,l_hand_pt,r_hand_pt,l_hand_grasp_axis,r_hand_grasp_axis,l_rung_pos,r_rung_pos,l_pole_pos,r_pole_pos,bar_radius)
lhand_x = l_pole_pos(1)-0.025-bar_radius;
rhand_x = r_pole_pos(1)-0.025-bar_radius;
lhand_z = sqrt(3)*bar_radius+l_rung_pos(3)-(-l_rung_pos(1)-0.025)/sqrt(3)+0.025/sqrt(3)*2;
rhand_z = sqrt(3)*bar_radius+r_rung_pos(3)-(-r_rung_pos(1)-0.025)/sqrt(3)+0.025/sqrt(3)*2;
lhand_pos_cnstr = WorldPositionConstraint(robot,l_hand,l_hand_pt,[lhand_x;0;lhand_z],[lhand_x;l_rung_pos(2)-0.1;lhand_z]);
rhand_pos_cnstr = WorldPositionConstraint(robot,r_hand,r_hand_pt,[rhand_x;r_rung_pos(2)+0.1;rhand_z],[rhand_x;0;rhand_z]);
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

function [c,dc] = handCoaxialFun(robot,kinsol,l_hand,r_hand,l_hand_pt,r_hand_pt,l_hand_axis,r_hand_axis)
[lhand_pos,dlhand_pos] = robot.forwardKin(kinsol,l_hand,l_hand_pt,2);
[rhand_pos,drhand_pos] = robot.forwardKin(kinsol,r_hand,r_hand_pt,2);
hand_pos_diff = lhand_pos(1:3)-rhand_pos(1:3);
dhand_pos_diff = dlhand_pos(1:3,:)-drhand_pos(1:3,:);
[lhand_axis_world,dlhand_axis_world] = quatRotateVec(lhand_pos(4:7),l_hand_axis);
dlhand_axis_world_dq = dlhand_axis_world(:,1:4)*dlhand_pos(4:7,:);
[rhand_axis_world,drhand_axis_world] = quatRotateVec(rhand_pos(4:7),r_hand_axis);
drhand_axis_world_dq = drhand_axis_world(:,1:4)*drhand_pos(4:7,:);
c1 = hand_pos_diff'*lhand_axis_world; % this should be positive
dc1 = lhand_axis_world'*dhand_pos_diff+hand_pos_diff'*dlhand_axis_world_dq;
c2 = hand_pos_diff'*rhand_axis_world; % this should be negative
dc2 = rhand_axis_world'*dhand_pos_diff+hand_pos_diff'*drhand_axis_world_dq;
c3 = cross(lhand_axis_world,hand_pos_diff);
c4 = cross(rhand_axis_world,hand_pos_diff);
hand_pos_diff_skew = vectorToSkewSymmetric(hand_pos_diff);
dc3 = -hand_pos_diff_skew*dlhand_axis_world_dq+vectorToSkewSymmetric(lhand_axis_world)*dhand_pos_diff;
dc4 = -hand_pos_diff_skew*drhand_axis_world_dq+vectorToSkewSymmetric(rhand_axis_world)*dhand_pos_diff;
c = [c1;c2;c3;c4];
dc = [dc1;dc2;dc3;dc4];
end

function cdfkp = climbOneStep(robot,l_hand,r_hand,l_hand_pt,r_hand_pt,l_hand_grasp_axis,r_hand_grasp_axis,l_rung_pos1,r_rung_pos1,l_rung_pos2,r_rung_pos2,l_pole_pos,r_pole_pos,bar_radius,q_start)
takeoff_idx = 6;
land_idx = 10;
static_idx = 14;

nq = robot.getNumPositions();
nv = robot.getNumVelocities;

l_pole_intersect_pt_z1 = l_rung_pos1(3)-(-l_rung_pos1(1)-0.025)/sqrt(3)+0.025/sqrt(3)*2;
r_pole_intersect_pt_z1 = r_rung_pos1(3)-(-r_rung_pos1(1)-0.025)/sqrt(3)+0.025/sqrt(3)*2;
l_pole_intersect_pt_z2 = l_rung_pos2(3)-(-l_rung_pos2(1)-0.025)/sqrt(3)+0.025/sqrt(3)*2;
r_pole_intersect_pt_z2 = r_rung_pos2(3)-(-r_rung_pos2(1)-0.025)/sqrt(3)+0.025/sqrt(3)*2;
l_pole_contact_pt1 = [l_pole_pos(1)-0.025;l_pole_pos(2);sqrt(3)*bar_radius+l_pole_intersect_pt_z1];
l_pole_contact_pt2 = [l_pole_pos(1)-0.025;l_pole_pos(2);sqrt(3)*bar_radius+l_pole_intersect_pt_z2];
r_pole_contact_pt1 = [r_pole_pos(1)-0.025;r_pole_pos(2);sqrt(3)*bar_radius+r_pole_intersect_pt_z1];
r_pole_contact_pt2 = [r_pole_pos(1)-0.025;r_pole_pos(2);sqrt(3)*bar_radius+r_pole_intersect_pt_z2];
l_rung_contact_pt1 = [l_pole_pos(1)-0.025-1.5*bar_radius;l_pole_pos(2);l_pole_intersect_pt_z1+sqrt(3)/2*bar_radius];
l_rung_contact_pt2 = [l_pole_pos(1)-0.025-1.5*bar_radius;l_pole_pos(2);l_pole_intersect_pt_z2+sqrt(3)/2*bar_radius];
r_rung_contact_pt1 = [r_pole_pos(1)-0.025-1.5*bar_radius;r_pole_pos(2);r_pole_intersect_pt_z1+sqrt(3)/2*bar_radius];
r_rung_contact_pt2 = [r_pole_pos(1)-0.025-1.5*bar_radius;r_pole_pos(2);r_pole_intersect_pt_z2+sqrt(3)/2*bar_radius];

robot_mass = robot.getMass();
g = 9.81;
num_edge = 4;
mu = 1.5;
FC_theta = linspace(0,2*pi,num_edge+1);
pole_FC_edge = [-ones(1,num_edge);mu*sin(FC_theta(1:num_edge));mu*cos(FC_theta(1:num_edge))];
pole_FC_edge = robot_mass*g*pole_FC_edge;
rung_FC_edge = [mu*sin(FC_theta(1:num_edge));mu*cos(FC_theta(1:num_edge));ones(1,num_edge)];
rung_FC_edge = roty(pi/6)*rung_FC_edge;
rung_FC_edge = robot_mass*g*rung_FC_edge;

wrench_struct = struct('active_knot',[],'cw',[]);
wrench_struct(1) = struct('active_knot',1:takeoff_idx,'cw',LinearFrictionConeWrench(robot,1,l_pole_contact_pt1,pole_FC_edge));
wrench_struct(2) = struct('active_knot',1:takeoff_idx,'cw',LinearFrictionConeWrench(robot,1,r_pole_contact_pt1,pole_FC_edge));
wrench_struct(3) = struct('active_knot',1:takeoff_idx,'cw',LinearFrictionConeWrench(robot,1,l_rung_contact_pt1,rung_FC_edge));
wrench_struct(4) = struct('active_knot',1:takeoff_idx,'cw',LinearFrictionConeWrench(robot,1,r_rung_contact_pt1,rung_FC_edge));
wrench_struct(5) = struct('active_knot',land_idx:static_idx,'cw',LinearFrictionConeWrench(robot,1,l_pole_contact_pt2,pole_FC_edge));
wrench_struct(6) = struct('active_knot',land_idx:static_idx,'cw',LinearFrictionConeWrench(robot,1,r_pole_contact_pt2,pole_FC_edge));
wrench_struct(7) = struct('active_knot',land_idx:static_idx,'cw',LinearFrictionConeWrench(robot,1,l_rung_contact_pt2,rung_FC_edge));
wrench_struct(8) = struct('active_knot',land_idx:static_idx,'cw',LinearFrictionConeWrench(robot,1,r_rung_contact_pt2,rung_FC_edge));

nT = static_idx;
tf_range = [0.5 2];
q_nom = bsxfun(@times,q_start,ones(1,nT));
Q = eye(nq);
Q(1,1) = 0;
Q(2,2) = 0;
Q_comddot = eye(3);

Qv = 0.1*eye(nv);
Q_contact_force = 10/(robot_mass*g)^2*eye(3);
cdfkp = ComDynamicsFullKinematicsPlanner(robot,nT,tf_range,Q_comddot,Qv,Q,q_nom,Q_contact_force,wrench_struct);

% add h bounds
cdfkp = cdfkp.addBoundingBoxConstraint(BoundingBoxConstraint(0.02*ones(nT-1,1),0.15*ones(nT-1,1)),cdfkp.h_inds(:));
cdfkp = cdfkp.addBoundingBoxConstraint(BoundingBoxConstraint(0.05*ones(land_idx-takeoff_idx,1),0.12*ones(land_idx-takeoff_idx,1)),cdfkp.h_inds(takeoff_idx:land_idx-1)');

% initial and final state constraint
cdfkp = cdfkp.addBoundingBoxConstraint(ConstantConstraint(q_start),cdfkp.q_inds(:,1));
cdfkp = cdfkp.addBoundingBoxConstraint(ConstantConstraint(zeros(nv,1)),cdfkp.v_inds(:,1));
cdfkp = cdfkp.addBoundingBoxConstraint(ConstantConstraint(zeros(nv,1)),cdfkp.v_inds(:,static_idx));

% add kinematic constraint on the hands before taking off
kinsol_start = robot.doKinematics(q_start);
lhand_pos_start = robot.forwardKin(kinsol_start,l_hand,l_hand_pt,2);
rhand_pos_start = robot.forwardKin(kinsol_start,r_hand,r_hand_pt,2);
lhand_axis_world = quatRotateVec(lhand_pos_start(4:7),l_hand_grasp_axis);
hand_distance = norm(lhand_pos_start(1:3)-rhand_pos_start(1:3));
cdfkp = cdfkp.addRigidBodyConstraint(WorldPositionConstraint(robot,l_hand,l_hand_pt,lhand_pos_start(1:3),lhand_pos_start(1:3)),num2cell(2:takeoff_idx));
cdfkp = cdfkp.addRigidBodyConstraint(WorldPositionConstraint(robot,r_hand,r_hand_pt,rhand_pos_start(1:3),rhand_pos_start(1:3)),num2cell(2:takeoff_idx));
cdfkp = cdfkp.addRigidBodyConstraint(WorldGazeDirConstraint(robot,l_hand,l_hand_grasp_axis,lhand_axis_world,0),num2cell(2:takeoff_idx));

% kinematic constraint for grasping the bar
[hand_coaxial_cnstr,hand_distance_cnstr] = generateBarGraspConstraint(robot,l_hand,r_hand,l_hand_pt,r_hand_pt,l_hand_grasp_axis,r_hand_grasp_axis,hand_distance);
for i = takeoff_idx+1:static_idx
cdfkp = cdfkp.addDifferentiableConstraint(hand_coaxial_cnstr,cdfkp.q_inds(:,i),cdfkp.kinsol_dataind(i));
end
cdfkp = cdfkp.addRigidBodyConstraint(hand_distance_cnstr,num2cell(takeoff_idx+1:static_idx));

% kinematic constraint on hands after landing
[lhand_land_pos_cnstr,rhand_land_pos_cnstr] = generateBarOnRungConstraint(robot,l_hand,r_hand,l_hand_pt,r_hand_pt,l_hand_grasp_axis,r_hand_grasp_axis,l_rung_pos2,r_rung_pos2,l_pole_pos,r_pole_pos,bar_radius);
cdfkp = cdfkp.addRigidBodyConstraint(lhand_land_pos_cnstr,{land_idx});
cdfkp = cdfkp.addRigidBodyConstraint(rhand_land_pos_cnstr,{land_idx});
cdfkp = cdfkp.addRigidBodyConstraint(WorldFixedPositionConstraint(robot,l_hand,l_hand_pt),{land_idx:static_idx});
cdfkp = cdfkp.addRigidBodyConstraint(WorldFixedPositionConstraint(robot,r_hand,r_hand_pt),{land_idx:static_idx});

% add the constraint that the bar should be away from the pole
lhand_flight_cnstr = WorldPositionConstraint(robot,l_hand,l_hand_pt,-inf(3,1),[l_pole_pos(1)-0.025-bar_radius-0.06;inf;inf]);
cdfkp = cdfkp.addRigidBodyConstraint(lhand_flight_cnstr,{takeoff_idx+1});
rhand_flight_cnstr = WorldPositionConstraint(robot,r_hand,r_hand_pt,-inf(3,1),[r_pole_pos(1)-0.025-bar_radius-0.06;inf;inf]);
cdfkp = cdfkp.addRigidBodyConstraint(rhand_flight_cnstr,{takeoff_idx+1});

lhand_flight_cnstr = WorldPositionConstraint(robot,l_hand,l_hand_pt,-inf(3,1),[l_pole_pos(1)-0.025-bar_radius-0.06;inf;inf]);
cdfkp = cdfkp.addRigidBodyConstraint(lhand_flight_cnstr,{takeoff_idx+2});
rhand_flight_cnstr = WorldPositionConstraint(robot,r_hand,r_hand_pt,-inf(3,1),[r_pole_pos(1)-0.025-bar_radius-0.06;inf;inf]);
cdfkp = cdfkp.addRigidBodyConstraint(rhand_flight_cnstr,{takeoff_idx+2});


lhand_flight_cnstr = WorldPositionConstraint(robot,l_hand,l_hand_pt,-inf(3,1),[l_pole_pos(1)-0.025-bar_radius-0.03;inf;inf]);
cdfkp = cdfkp.addRigidBodyConstraint(lhand_flight_cnstr,{takeoff_idx+3});
rhand_flight_cnstr = WorldPositionConstraint(robot,r_hand,r_hand_pt,-inf(3,1),[r_pole_pos(1)-0.025-bar_radius-0.03;inf;inf]);
cdfkp = cdfkp.addRigidBodyConstraint(rhand_flight_cnstr,{takeoff_idx+3});

% add a prelanding constraint
lhand_preland_cnstr = WorldPositionConstraint(robot,l_hand,l_hand_pt,[-inf;-inf;l_pole_contact_pt2(3)+0.01],[l_pole_contact_pt2(1)-bar_radius-0.01;inf;inf]);
cdfkp = cdfkp.addRigidBodyConstraint(lhand_preland_cnstr,{land_idx-1});
rhand_preland_cnstr = WorldPositionConstraint(robot,r_hand,r_hand_pt,[-inf;-inf;r_pole_contact_pt2(3)+0.01],[r_pole_contact_pt2(1)-bar_radius-0.01;inf;inf]);
cdfkp = cdfkp.addRigidBodyConstraint(rhand_preland_cnstr,{land_idx-1});

% add final comddot constraint
cdfkp = cdfkp.addBoundingBoxConstraint(ConstantConstraint(zeros(3,1)),cdfkp.comddot_inds(:,static_idx));

% add arm symmetric constraint
arm_symmetry = armSymmetricConstraint(robot,2:land_idx);
cdfkp = cdfkp.addLinearConstraint(arm_symmetry,reshape(cdfkp.q_inds(:,2:land_idx),[],1));

% add leg symmetric constraint
% leg_symmetry = legSymmetricConstraint(robot,2:land_idx);
% cdfkp = cdfkp.addLinearConstraint(leg_symmetry,reshape(cdfkp.q_inds(:,2:land_idx),[],1));

% add a roll constraint at the end
cdfkp = cdfkp.addBoundingBoxConstraint(ConstantConstraint(0),cdfkp.q_inds(4,static_idx));

% add a roll constraint for all
cdfkp = cdfkp.addBoundingBoxConstraint(BoundingBoxConstraint(-0.4*ones(static_idx-2,1),0.4*ones(static_idx-2,1)),cdfkp.q_inds(4,2:static_idx-1)');
% add a cost on the final posture
Q = 5*eye(nq);
Q(1,1) = 0;
Q(2,2) = 0;
cdfkp = cdfkp.addCost(QuadraticConstraint(-inf,inf,Q,zeros(nq,1)),cdfkp.q_inds(:,static_idx));
end
