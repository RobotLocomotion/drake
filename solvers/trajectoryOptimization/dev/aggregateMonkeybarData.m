function aggregateMonkeybarData
w = warning('off','Drake:RigidBody:SimplifiedCollisionGeometry');
warning('off','Drake:RigidBody:NonPositiveInertiaMatrix');
warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints');
warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits');
robot = RigidBodyManipulator([getDrakePath,'/examples/Atlas/urdf/atlas_convex_hull.urdf'],struct('floating',true));

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

takeoff = load('test_monkeybar_takeoff.mat','t_sol','q_sol','v_sol','wrench_sol');
swing1 = load('test_monkeybar_swing.mat','t_sol','q_sol','v_sol','wrench_sol');
swing2 = load('test_monkeybar_swing2.mat','t_sol','q_sol','v_sol','wrench_sol');
swing3 = load('test_monkeybar_swing3.mat','t_sol','q_sol','v_sol','wrench_sol');
swing4 = load('test_monkeybar_swing4.mat','t_sol','q_sol','v_sol','wrench_sol');
land = load('test_monkeybar_land_complementarity2.mat','t_sol','q_sol','v_sol','wrench_sol');
% land = load('test_monkeybar_land.mat','t_sol','q_sol','v_sol','wrench_sol');

q_all = [takeoff.q_sol swing1.q_sol(:,2:end) swing2.q_sol(:,2:end) swing3.q_sol(:,2:end) swing4.q_sol(:,2:end) land.q_sol(:,2:end)];
v_all = [takeoff.v_sol swing1.v_sol(:,2:end) swing2.v_sol(:,2:end) swing3.v_sol(:,2:end) swing4.v_sol(:,2:end) land.v_sol(:,2:end)];
t_all = takeoff.t_sol;
t_all = [t_all t_all(end)+swing1.t_sol(2:end)];
t_all = [t_all t_all(end)+swing2.t_sol(2:end)];
t_all = [t_all t_all(end)+swing3.t_sol(2:end)];
t_all = [t_all t_all(end)+swing4.t_sol(2:end)];
t_all = [t_all t_all(end)+land.t_sol(2:end)];
takeoff_wrench = struct('body',[],'body_pts',[],'pts_pos',[],'force',[],'torque',[]);
for i = 1:size(takeoff.wrench_sol,2)
  takeoff_wrench(1,i) = takeoff.wrench_sol(3,i);
  takeoff_wrench(2,i) = takeoff.wrench_sol(4,i);
  takeoff_wrench(3,i) = takeoff.wrench_sol(1,i);
  takeoff_wrench(4,i) = takeoff.wrench_sol(2,i);
end
wrench_all = takeoff_wrench;
swing1_wrench = swing1.wrench_sol(:,2:end);
for i = 1:size(swing1_wrench,2)
  kinsol = robot.doKinematics(swing1.q_sol(:,i+1));
  l_foot_pos = robot.forwardKin(kinsol,l_foot,l_foot_bottom);
  r_foot_pos = robot.forwardKin(kinsol,r_foot,r_foot_bottom);
  swing1_wrench(3,i) = struct('body',l_foot,'body_pts',l_foot_bottom,'pts_pos',l_foot_pos,'force',zeros(3,4),'torque',zeros(3,4));
  swing1_wrench(4,i) = struct('body',r_foot,'body_pts',r_foot_bottom,'pts_pos',r_foot_pos,'force',zeros(3,4),'torque',zeros(3,4));
end
wrench_all = [wrench_all swing1_wrench];

swing2_wrench = swing2.wrench_sol(:,2:end);
for i = 1:size(swing2_wrench,2)
  kinsol = robot.doKinematics(swing2.q_sol(:,i+1));
  l_foot_pos = robot.forwardKin(kinsol,l_foot,l_foot_bottom);
  r_foot_pos = robot.forwardKin(kinsol,r_foot,r_foot_bottom);
  swing2_wrench(3,i) = struct('body',l_foot,'body_pts',l_foot_bottom,'pts_pos',l_foot_pos,'force',zeros(3,4),'torque',zeros(3,4));
  swing2_wrench(4,i) = struct('body',r_foot,'body_pts',r_foot_bottom,'pts_pos',r_foot_pos,'force',zeros(3,4),'torque',zeros(3,4));
end
wrench_all = [wrench_all swing2_wrench];

swing3_wrench = swing3.wrench_sol(:,2:end);
for i = 1:size(swing3_wrench,2)
  kinsol = robot.doKinematics(swing3.q_sol(:,i+1));
  l_foot_pos = robot.forwardKin(kinsol,l_foot,l_foot_bottom);
  r_foot_pos = robot.forwardKin(kinsol,r_foot,r_foot_bottom);
  swing3_wrench(3,i) = struct('body',l_foot,'body_pts',l_foot_bottom,'pts_pos',l_foot_pos,'force',zeros(3,4),'torque',zeros(3,4));
  swing3_wrench(4,i) = struct('body',r_foot,'body_pts',r_foot_bottom,'pts_pos',r_foot_pos,'force',zeros(3,4),'torque',zeros(3,4));
end
wrench_all = [wrench_all swing3_wrench];

swing4_wrench = swing4.wrench_sol(:,2:end);
for i = 1:size(swing4_wrench,2)
  kinsol = robot.doKinematics(swing1.q_sol(:,i+1));
  l_foot_pos = robot.forwardKin(kinsol,l_foot,l_foot_bottom);
  r_foot_pos = robot.forwardKin(kinsol,r_foot,r_foot_bottom);
  swing4_wrench(3,i) = struct('body',l_foot,'body_pts',l_foot_bottom,'pts_pos',l_foot_pos,'force',zeros(3,4),'torque',zeros(3,4));
  swing4_wrench(4,i) = struct('body',r_foot,'body_pts',r_foot_bottom,'pts_pos',r_foot_pos,'force',zeros(3,4),'torque',zeros(3,4));
end
wrench_all = [wrench_all swing4_wrench];

wrench_all = [wrench_all land.wrench_sol(:,2:end)];

xtraj_all = PPTrajectory(foh(t_all,[q_all;v_all;]));
save('test_monkeybar.mat','t_all','q_all','v_all','xtraj_all','wrench_all');
end
