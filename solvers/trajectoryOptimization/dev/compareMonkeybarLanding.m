function compareMonkeybarLanding
checkDependency('lcmgl');
w = warning('off','Drake:RigidBody:SimplifiedCollisionGeometry');
warning('off','Drake:RigidBody:NonPositiveInertiaMatrix');
warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints');
warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits');
robot = RigidBodyManipulator([getDrakePath,'/examples/Atlas/urdf/atlas_minimal_contact.urdf'],struct('floating',true));
monkeybar_urdf = [getDrakePath,'/solvers/trajectoryOptimization/test/MonkeyBar.urdf'];
monkeybar_pos1 = [0;0;2.5;pi/2;0;-pi/2];
robot = robot.addRobotFromURDF(monkeybar_urdf,monkeybar_pos1(1:3),monkeybar_pos1(4:6));

stage_urdf = [getDrakePath,'/solvers/trajectoryOptimization/test/MonkeyBar_stage.urdf'];
jump_height = 2.6;
stage_pos1 = [monkeybar_pos1(1:3)-[1.2;0;jump_height+0.05];0;pi/2;0];
robot = robot.addRobotFromURDF(stage_urdf,stage_pos1(1:3),stage_pos1(4:6));
monkeybar_pos2 = monkeybar_pos1+[0.5;0;0;0;0;0];
monkeybar_pos3 = monkeybar_pos2+[0.4;0;0;0;0;0];
monkeybar_pos4 = monkeybar_pos3+[0.4;0;-0.1;0;0;0];
monkeybar_pos5 = monkeybar_pos4+[0.6;0;0;0;0;0];
robot = robot.addRobotFromURDF(monkeybar_urdf,monkeybar_pos2(1:3),monkeybar_pos2(4:6));
robot = robot.addRobotFromURDF(monkeybar_urdf,monkeybar_pos3(1:3),monkeybar_pos3(4:6));
robot = robot.addRobotFromURDF(monkeybar_urdf,monkeybar_pos4(1:3),monkeybar_pos4(4:6));
robot = robot.addRobotFromURDF(monkeybar_urdf,monkeybar_pos5(1:3),monkeybar_pos5(4:6));

land_height = 2.6;
stage_pos2 = [monkeybar_pos5(1:3)+[1.45;0;-land_height-0.05];0;-pi/2;0];
robot = robot.addRobotFromURDF(stage_urdf,stage_pos2(1:3),stage_pos2(4:6));

%%%%%%%%%%%%%
land = load('test_monkeybar_land.mat','t_sol','q_sol','v_sol','wrench_sol');
land_complementarity = load('test_monkeybar_land_complementarity2.mat','t_sol','q_sol','v_sol','wrench_sol');

xtraj = PPTrajectory(foh(land.t_sol,[land.q_sol;land.v_sol]));
xtraj = xtraj.setOutputFrame(robot.getStateFrame);
atlas = Atlas();
atlas.getStateFrame().publish(0,[land.q_sol(:,9);land.v_sol(:,9)],'EST_ROBOT_STATE');
atlas.getStateFrame().publish(0,[land_complementarity.q_sol(:,9);land_complementarity.v_sol(:,9)],'EST_ROBOT_STATE');
contact_pt_active = false(8,6);
contact_pt_active_complementarity = false(8,6);
for i = 9:14
  contact_pt_active(1:4,i-8) = (land.wrench_sol(3,i).pts_pos(3,:)<-0.2+0.001)';
  contact_pt_active(5:8,i-8) = (land.wrench_sol(4,i).pts_pos(3,:)<-0.2+0.001)';
  contact_pt_active_complementarity(1:4,i-8) = (land_complementarity.wrench_sol(3,i).pts_pos(3,:)<-0.2+0.001)';
  contact_pt_active_complementarity(5:8,i-8) = (land_complementarity.wrench_sol(4,i).pts_pos(3,:)<-0.2+0.001)';
end
pt_name = {'l_foot_l_heel  ';'l_foot_r_heel  ';'l_foot_l_toe  ';'l_foot_r_toe  ';'r_foot_l_heel  ';'r_foot_r_heel  ';'r_foot_l_toe  ';'r_foot_l_toe  '};
figure;
subplot(1,2,1)
hold on;
inactive_i = 8*ones(8,1);
inactive_j = (1:8)';
active_i = [];
active_j = [];
for i = 1:6
  active_idx_i = find(contact_pt_active(:,i));
  inactive_idx_i = find(~contact_pt_active(:,i));
  inactive_i = [inactive_i;(i+8)*ones(length(inactive_idx_i),1)];
  inactive_j = [inactive_j;inactive_idx_i];
  active_i = [active_i;(i+8)*ones(length(active_idx_i),1)];
  active_j = [active_j;active_idx_i];
end
plot(active_i,active_j,'.','Markersize',100,'Color','r');
plot(inactive_i,inactive_j,'.','Markersize',100,'Color','b');
legend('active contact','inactive contact','Fontsize',14);

set(gca,'YTickLabel',pt_name,'Fontsize',28)
set(gca,'XTick',8:14);
xlabel('knot')
title(sprintf('Contact sequence BEFORE optimizing \n with complementarity constraints'),'Fontsize',28);

subplot(1,2,2)
hold on;
inactive_i = 8*ones(8,1);
inactive_j = (1:8)';
active_i = [];
active_j = [];
for i = 1:6
  active_idx_i = find(contact_pt_active_complementarity(:,i));
  inactive_idx_i = find(~contact_pt_active_complementarity(:,i));
  inactive_i = [inactive_i;(i+8)*ones(length(inactive_idx_i),1)];
  inactive_j = [inactive_j;inactive_idx_i];
  active_i = [active_i;(i+8)*ones(length(active_idx_i),1)];
  active_j = [active_j;active_idx_i];
end
plot(active_i,active_j,'.','Markersize',100,'Color','r');
plot(inactive_i,inactive_j,'.','Markersize',100,'Color','b');
% legend('active contact','inactive contact');

set(gca,'YTickLabel',[],'Fontsize',28)
set(gca,'XTick',8:14);
xlabel('knot')
title(sprintf('Contact sequence AFTER optimizing \n with complementarity constraints'),'Fontsize',28);
end
