function [v,xtraj] = visualizeMonkeyBar(filename)
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
warning(w);
% v = robot.constructVisualizer();

%%%%%%%%%%%%%%%%%%
%%
% add the takeoff stage
S = load(filename,'xtraj_all','t_all','wrench_all');
v = ContactWrenchVisualizer(robot,S.t_all,S.wrench_all);
xtraj = S.xtraj_all.setOutputFrame(robot.getStateFrame());
end
