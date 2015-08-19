function handMoveBlock()
block_size = [0.1;0.1;0.1];
block_origin_start = [0;0;block_size(3)/2;0;0;0];
robot = RigidBodyManipulator('block.urdf',struct('floating',true));
robot = robot.addRobotFromURDF('ground.urdf',[0;0;-0.025],[0;0;0]);
robot = robot.addRobotFromURDF([getDrakePath,'/examples/Atlas/urdf/robotiq.urdf'],[],[],struct('floating',true));
nq = robot.getNumPositions();
nv = robot.getNumVelocities();
q0 = zeros(nq,1);
q0(1:3) = [0;0;block_size(3)/2];
q0(9) = 1.5;
object_idx = 2;
finger1_link3 = robot.findLinkId('finger_1_link_3');
finger1_link3_contact_pts = robot.getBody(finger1_link3).getTerrainContactPoints;
finger1_tip = mean(finger1_link3_contact_pts(:,1:4),2);
finger1_tip_normal = cross(finger1_link3_contact_pts(:,2)-finger1_link3_contact_pts(:,1),finger1_link3_contact_pts(:,3)-finger1_link3_contact_pts(:,2));
finger1_tip_normal = finger1_tip_normal/norm(finger1_tip_normal);
finger1_tip_back = [0.01;-0.02;0];
finger2_link3 = robot.findLinkId('finger_2_link_3');
finger2_link3_contact_pts = robot.getBody(finger2_link3).getTerrainContactPoints;
finger2_tip = mean(finger2_link3_contact_pts(:,1:4),2);
finger2_tip_normal = cross(finger2_link3_contact_pts(:,2)-finger2_link3_contact_pts(:,1),finger2_link3_contact_pts(:,3)-finger2_link3_contact_pts(:,2));
finger2_tip_normal = finger2_tip_normal/norm(finger2_tip_normal);
finger2_tip_back = [0.01;-0.02;0];
finger3_link3 = robot.findLinkId('finger_middle_link_3');
finger3_link3_contact_pts = robot.getBody(finger3_link3).getTerrainContactPoints;
finger3_tip = mean(finger3_link3_contact_pts(:,1:4),2);
finger3_tip_normal = cross(finger3_link3_contact_pts(:,2)-finger3_link3_contact_pts(:,1),finger3_link3_contact_pts(:,3)-finger3_link3_contact_pts(:,2));
finger3_tip_normal = finger3_tip_normal/norm(finger3_tip_normal);
finger3_tip_back = [0.01;-0.02;0];
block_side1 = repmat(block_size/2,1,4).*[1 1 -1 -1;1 -1 1 -1;-1 -1 -1 -1];
block_side2 = repmat(block_size/2,1,4).*[1 1 1 1;1 1 -1 -1;1 -1 1 -1];
block_side3 = repmat(block_size/2,1,4).*[-1 -1 -1 -1;1 1 -1 -1;1 -1 1 -1];
block_corners = repmat(block_size/2,1,8).*[1 1 1 1 -1 -1 -1 -1;1 1 -1 -1 1 1 -1 -1;1 -1 1 -1 1 -1 1 -1];

pickup_finger1_cnstr1 = WorldPositionConstraint(robot,finger1_link3,finger1_tip,block_origin_start(1:3)+block_size/2.*[-1;-0.7;-0.7],block_origin_start(1:3)+block_size/2.*[-1;0.7;0.7]);
pickup_finger1_cnstr2 = WorldPositionConstraint(robot,finger1_link3,finger1_tip_back,block_origin_start(1:3)+block_size/2.*[-inf;-0.7;-0.7],block_origin_start(1:3)+block_size/2.*[-1.001;0.7;0.7]);
pickup_finger1_cnstr3 = WorldGazeDirConstraint(robot,finger1_link3,finger1_tip_normal,[1;0;0],0);
pickup_finger2_cnstr1 = WorldPositionConstraint(robot,finger2_link3,finger2_tip,block_origin_start(1:3)+block_size/2.*[-1;-0.7;-0.7],block_origin_start(1:3)+block_size/2.*[-1;0.7;0.7]);
pickup_finger2_cnstr2 = WorldPositionConstraint(robot,finger2_link3,finger2_tip_back,block_origin_start(1:3)+block_size/2.*[-inf;-0.7;-0.7],block_origin_start(1:3)+block_size/2.*[-1.001;0.7;0.7]);
pickup_finger2_cnstr3 = WorldGazeDirConstraint(robot,finger2_link3,finger2_tip_normal,[1;0;0],0);
pickup_finger3_cnstr1 = WorldPositionConstraint(robot,finger3_link3,finger3_tip,block_origin_start(1:3)+block_size/2.*[1;-0.4;-0.4],block_origin_start(1:3)+block_size/2.*[1;0.1;0.4]);
pickup_finger3_cnstr2 = WorldPositionConstraint(robot,finger3_link3,finger3_tip_back,block_origin_start(1:3)+block_size/2.*[1.001;-0.7;-0.7],block_origin_start(1:3)+block_size/2.*[inf;0.7;0.7]);
pickup_finger3_cnstr3 = WorldGazeDirConstraint(robot,finger3_link3,finger3_tip_normal,[-1;0;0],0);
block_base_idx = (1:6)';
pc_pickup = PostureConstraint(robot);
pc_pickup = pc_pickup.setJointLimits(block_base_idx,block_origin_start,block_origin_start);

% hand_above ground
palm = robot.findLinkId('palm');
palm_pts = robot.getBody(palm).getTerrainContactPoints();
hand_base_idx = robot.getBody(palm).position_num;
palm_above_ground = PostureConstraint(robot);
palm_above_ground = palm_above_ground.setJointLimits(hand_base_idx(3),0.05,inf);
palm_above_ground = {palm_above_ground,WorldPositionConstraint(robot,palm,palm_pts,[nan(2,1);0]*ones(1,size(palm_pts,2)),nan(size(palm_pts)))};


Q = ones(nq,1);
Q(1:12) = 0;
Q = diag(Q);
options = IKoptions(robot);
options = options.setQ(Q);
q_pickup = inverseKin(robot,q0,q0,pickup_finger1_cnstr1,pickup_finger1_cnstr3,pickup_finger2_cnstr1,pickup_finger2_cnstr3,pickup_finger3_cnstr1,pickup_finger3_cnstr3,pc_pickup,palm_above_ground{:},options);

grasp_idx = 3;
takeoff_idx = 4;
land_idx = 9;
nT = 10;

ground_contact = FrictionConeWrench(robot,object_idx,block_side1,1,[0;0;1]);
finger1_fc = GraspFrictionConeWrench(robot,object_idx, finger1_link3,finger1_tip,[-1;0;0],1);
finger2_fc = GraspFrictionConeWrench(robot,object_idx, finger2_link3,finger2_tip,[-1;0;0],1);
finger3_fc = GraspFrictionConeWrench(robot,object_idx, finger3_link3,finger3_tip,[1;0;0],1);
finger1_contact_wrench = struct('active_knot',grasp_idx:nT,'cw',finger1_fc);
finger2_contact_wrench = struct('active_knot',grasp_idx:nT,'cw',finger2_fc);
finger3_contact_wrench = struct('active_knot',grasp_idx:nT,'cw',finger3_fc);
ground_contact_wrench = struct('active_knot',[],'cw',[]);
ground_contact_wrench(1) = struct('active_knot',1:takeoff_idx,'cw',ground_contact);
ground_contact_wrench(2) = struct('active_knot',land_idx:nT,'cw',ground_contact);


tf_range = [0.4 2];
Q_comddot = eye(3);
Qv = zeros(nv);
Q = zeros(nq);
q_nom = bsxfun(@times,q_pickup,ones(1,nT));
Q_contact_force = 0.01*eye(3);
sbdfkp = SingleBodyDynamicsFullKinematicsPlanner(robot,object_idx,nT,tf_range,Q_comddot,Qv,Q,q_nom,Q_contact_force,[finger1_contact_wrench finger2_contact_wrench finger3_contact_wrench ground_contact_wrench]);

% bounds on time interval
sbdfkp = sbdfkp.addBoundingBoxConstraint(BoundingBoxConstraint(0.03*ones(nT-1,1),0.1*ones(nT-1,1)),sbdfkp.h_inds(:));


% bounds on initial state
hand_base_start = [0;0;0.5;0;0;0];
sbdfkp = sbdfkp.addBoundingBoxConstraint(ConstantConstraint(reshape(bsxfun(@times,block_origin_start,ones(1,takeoff_idx)),[],1)),reshape(sbdfkp.q_inds(block_base_idx,1:takeoff_idx),[],1));
sbdfkp = sbdfkp.addBoundingBoxConstraint(ConstantConstraint(hand_base_start),sbdfkp.q_inds(hand_base_idx,1));
sbdfkp = sbdfkp.addBoundingBoxConstraint(ConstantConstraint(zeros(nv,1)),sbdfkp.v_inds(:,1));

% bounds on final state
block_origin_final = block_origin_start+[0.3;0;0;0;0;pi/2];
sbdfkp = sbdfkp.addBoundingBoxConstraint(ConstantConstraint(reshape(bsxfun(@times,block_origin_final,ones(1,nT-land_idx+1)),[],1)),reshape(sbdfkp.q_inds(block_base_idx,land_idx:nT),[],1));

% grasp finger tips does not move
finger1_fixed_cnstr = RelativeFixedPositionConstraint(robot,finger1_link3,finger1_link3_contact_pts,object_idx,nT-grasp_idx+1);
finger2_fixed_cnstr = RelativeFixedPositionConstraint(robot,finger2_link3,finger2_link3_contact_pts,object_idx,nT-grasp_idx+1);
finger3_fixed_cnstr = RelativeFixedPositionConstraint(robot,finger3_link3,finger3_link3_contact_pts,object_idx,nT-grasp_idx+1);

sbdfkp = sbdfkp.addNonlinearConstraint(finger1_fixed_cnstr,sbdfkp.q_inds(:,grasp_idx:nT),sbdfkp.kinsol_dataind(grasp_idx:nT));
sbdfkp = sbdfkp.addNonlinearConstraint(finger2_fixed_cnstr,sbdfkp.q_inds(:,grasp_idx:nT),sbdfkp.kinsol_dataind(grasp_idx:nT));
sbdfkp = sbdfkp.addNonlinearConstraint(finger3_fixed_cnstr,sbdfkp.q_inds(:,grasp_idx:nT),sbdfkp.kinsol_dataind(grasp_idx:nT));

% fix finger posture
finger_joint_idx = robot.findPositionIndices('finger');
finger_pc = PostureConstraint(robot);
finger_pc = finger_pc.setJointLimits(finger_joint_idx,q_pickup(finger_joint_idx),q_pickup(finger_joint_idx));
sbdfkp = sbdfkp.addRigidBodyConstraint(finger_pc,num2cell(grasp_idx:nT));
% fix palm to q_pickup at grasping moment
hand_base_grasp_cnstr = PostureConstraint(robot);
hand_base_grasp_cnstr = hand_base_grasp_cnstr.setJointLimits(hand_base_idx,q_pickup(hand_base_idx),q_pickup(hand_base_idx));
sbdfkp = sbdfkp.addRigidBodyConstraint(hand_base_grasp_cnstr,{grasp_idx});
% sbdfkp = sbdfkp.addRigidBodyConstraint(pickup_finger1_cnstr1,{grasp_idx});
% sbdfkp = sbdfkp.addRigidBodyConstraint(pickup_finger2_cnstr1,{grasp_idx});
% sbdfkp = sbdfkp.addRigidBodyConstraint(pickup_finger3_cnstr1,{grasp_idx});
% sbdfkp = sbdfkp.addRigidBodyConstraint(pickup_finger1_cnstr2,{grasp_idx});
% sbdfkp = sbdfkp.addRigidBodyConstraint(pickup_finger2_cnstr2,{grasp_idx});
% sbdfkp = sbdfkp.addRigidBodyConstraint(pickup_finger3_cnstr2,{grasp_idx});
% sbdfkp = sbdfkp.addRigidBodyConstraint(pickup_finger1_cnstr3,{grasp_idx});
% sbdfkp = sbdfkp.addRigidBodyConstraint(pickup_finger2_cnstr3,{grasp_idx});
% sbdfkp = sbdfkp.addRigidBodyConstraint(pickup_finger3_cnstr3,{grasp_idx});


% object above ground
above_ground_cnstr = WorldPositionConstraint(robot,object_idx,block_corners,[nan;nan;0.01]*ones(1,8),nan(3,8));
sbdfkp = sbdfkp.addRigidBodyConstraint(above_ground_cnstr,num2cell(takeoff_idx+1:land_idx-1));

x_seed = zeros(sbdfkp.num_vars,1);

sbdfkp = sbdfkp.setSolverOptions('snopt','iterationslimit',1e6);
sbdfkp = sbdfkp.setSolverOptions('snopt','majoriterationslimit',2000);
sbdfkp = sbdfkp.setSolverOptions('snopt','majorfeasibilitytolerance',2e-6);
sbdfkp = sbdfkp.setSolverOptions('snopt','majoroptimalitytolerance',4e-3);
sbdfkp = sbdfkp.setSolverOptions('snopt','superbasicslimit',2000);
sbdfkp = sbdfkp.setSolverOptions('snopt','print','test_grasp_move.out');
tic
[x_sol,F,info] = sbdfkp.solve(x_seed);
toc
[q_sol,v_sol,h_sol,t_sol,com_sol,comdot_sol,comddot_sol,H_sol,Hdot_sol,lambda_sol,wrench_sol] = parseSolution(sbdfkp,x_sol);
v = robot.constructVisualizer();
xtraj_sol = PPTrajectory(foh(cumsum([0 h_sol]),[q_sol;v_sol]));
xtraj_sol = xtraj_sol.setOutputFrame(robot.getStateFrame);
qtraj_sol = PPTrajectory(foh(t_sol,q_sol));
qtraj_sol = qtraj_sol.setOutputFrame(v.getInputFrame);
v.playback(qtraj_sol,struct('slider',true));
% object above the ground
keyboard
end
