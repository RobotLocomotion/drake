function testSimpleDynamicsFullKinematicsPlanner
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
heel_land_idx = 12;
toe_land_idx = 14;
nT = 17;
l_foot_contact_cnstr = struct('active_knot',[],'cwc',[]);
l_foot_contact_cnstr(1) = struct('active_knot',1:heel_takeoff_idx,'cwc',FrictionConeWrenchConstraint(robot,l_foot,l_foot_heel,mu,[0;0;1]));
l_foot_contact_cnstr(2) = struct('active_knot',1:toe_takeoff_idx,'cwc',FrictionConeWrenchConstraint(robot,l_foot,l_foot_toe,mu,[0;0;1]));
l_foot_contact_cnstr(3) = struct('active_knot',heel_land_idx:nT,'cwc',FrictionConeWrenchConstraint(robot,l_foot,l_foot_heel,mu,[0;0;1]));
l_foot_contact_cnstr(4) = struct('active_knot',toe_land_idx:nT,'cwc',FrictionConeWrenchConstraint(robot,l_foot,l_foot_toe,mu,[0;0;1]));
r_foot_contact_cnstr = struct('active_knot',[],'cwc',[]);
r_foot_contact_cnstr(1) = struct('active_knot',1:heel_takeoff_idx,'cwc',FrictionConeWrenchConstraint(robot,r_foot,r_foot_heel,mu,[0;0;1]));
r_foot_contact_cnstr(2) = struct('active_knot',1:toe_takeoff_idx,'cwc',FrictionConeWrenchConstraint(robot,r_foot,r_foot_toe,mu,[0;0;1]));
r_foot_contact_cnstr(3) = struct('active_knot',heel_land_idx:nT,'cwc',FrictionConeWrenchConstraint(robot,r_foot,r_foot_heel,mu,[0;0;1]));
r_foot_contact_cnstr(4) = struct('active_knot',toe_land_idx:nT,'cwc',FrictionConeWrenchConstraint(robot,r_foot,r_foot_toe,mu,[0;0;1]));

tf_range = [1 2];
q_nom = bsxfun(@times,qstar,ones(1,nT));
Q_comddot = eye(3);
Q = eye(nq);
Q(1,1) = 0;
Q(2,2) = 0;
Q(6,6) = 0;
sdfkp = ComDynamicsFullKinematicsPlanner(robot,nT,tf_range,Q_comddot,Q,q_nom,[l_foot_contact_cnstr;r_foot_contact_cnstr]);

lfoot_toe_above_ground = WorldPositionConstraint(robot,l_foot,l_foot_toe,[nan(2,2);0.0*ones(1,2)],nan(3,2));
lfoot_toe_above_ground = lfoot_toe_above_ground.generateConstraint([]);
for i = toe_takeoff_idx+1:toe_land_idx-1
  sdfkp = sdfkp.addManagedKinematicConstraint(lfoot_toe_above_ground,i);
end

rfoot_toe_above_ground = WorldPositionConstraint(robot,r_foot,r_foot_toe,[nan(2,2);0.0*ones(1,2)],nan(3,2));
rfoot_toe_above_ground = rfoot_toe_above_ground.generateConstraint([]);
for i = toe_takeoff_idx+1:toe_land_idx-1
  sdfkp = sdfkp.addManagedKinematicConstraint(rfoot_toe_above_ground,i);
end

lfoot_heel_above_ground = WorldPositionConstraint(robot,l_foot,l_foot_heel,[nan(2,2);0.0*ones(1,2)],nan(3,2));
lfoot_heel_above_ground = lfoot_heel_above_ground.generateConstraint([]);
for i = heel_takeoff_idx+1:heel_land_idx-1
  sdfkp = sdfkp.addManagedKinematicConstraint(lfoot_heel_above_ground,i);
end

rfoot_heel_above_ground = WorldPositionConstraint(robot,r_foot,r_foot_heel,[nan(2,2);0.0*ones(1,2)],nan(3,2));
rfoot_heel_above_ground = rfoot_heel_above_ground.generateConstraint([]);
for i = heel_takeoff_idx+1:heel_land_idx-1
  sdfkp = sdfkp.addManagedKinematicConstraint(rfoot_heel_above_ground,i);
end

lfoot_heel_on_ground = WorldPositionConstraint(robot,l_foot,l_foot_heel,lfoot_heel_pos_star,lfoot_heel_pos_star);
lfoot_heel_on_ground = lfoot_heel_on_ground.generateConstraint([]);
for i = heel_land_idx:toe_land_idx-1
  sdfkp = sdfkp.addManagedKinematicConstraint(lfoot_heel_on_ground,i);
end

rfoot_heel_on_ground = WorldPositionConstraint(robot,r_foot,r_foot_heel,rfoot_heel_pos_star,rfoot_heel_pos_star);
rfoot_heel_on_ground = rfoot_heel_on_ground.generateConstraint([]);
for i = heel_land_idx:toe_land_idx-1
  sdfkp = sdfkp.addManagedKinematicConstraint(rfoot_heel_on_ground,i);
end

lfoot_toe_on_ground = WorldPositionConstraint(robot,l_foot,l_foot_toe,lfoot_toe_pos_star,lfoot_toe_pos_star);
lfoot_toe_on_ground = lfoot_toe_on_ground.generateConstraint([]);
for i = heel_takeoff_idx+1:toe_takeoff_idx
  sdfkp = sdfkp.addManagedKinematicConstraint(lfoot_toe_on_ground,i);
end

rfoot_toe_on_ground = WorldPositionConstraint(robot,r_foot,r_foot_toe,rfoot_toe_pos_star,rfoot_toe_pos_star);
rfoot_toe_on_ground = rfoot_toe_on_ground.generateConstraint([]);
for i = heel_takeoff_idx+1:toe_takeoff_idx
  sdfkp = sdfkp.addManagedKinematicConstraint(rfoot_toe_on_ground,i);
end

lfoot_on_ground = {WorldPositionConstraint(robot,l_foot,[0;0;0],lfoot_pos_star(1:3),lfoot_pos_star(1:3)),...
  WorldQuatConstraint(robot,l_foot,lfoot_pos_star(4:7),0)};
lfoot_on_ground = {lfoot_on_ground{1}.generateConstraint([]),lfoot_on_ground{2}.generateConstraint([])};
for i = [(1:heel_takeoff_idx) (toe_land_idx:nT)]
  sdfkp = sdfkp.addManagedKinematicConstraint(lfoot_on_ground{1},i);
  sdfkp = sdfkp.addManagedKinematicConstraint(lfoot_on_ground{2},i);
end

rfoot_on_ground = {WorldPositionConstraint(robot,r_foot,[0;0;0],rfoot_pos_star(1:3),rfoot_pos_star(1:3)),...
  WorldQuatConstraint(robot,r_foot,rfoot_pos_star(4:7),0)};
rfoot_on_ground = {rfoot_on_ground{1}.generateConstraint([]),rfoot_on_ground{2}.generateConstraint([])};
for i = [(1:heel_takeoff_idx) (toe_land_idx:nT)]
  sdfkp = sdfkp.addManagedKinematicConstraint(rfoot_on_ground{1},i);
  sdfkp = sdfkp.addManagedKinematicConstraint(rfoot_on_ground{2},i);
end

apex_knot = floor((toe_takeoff_idx+heel_land_idx)/2);
sdfkp = sdfkp.addBoundingBoxConstraint(BoundingBoxConstraint(qstar(3)+0.2,inf),sdfkp.q_inds(3,apex_knot));

sdfkp = sdfkp.addBoundingBoxConstraint(BoundingBoxConstraint(qstar,qstar),sdfkp.q_inds(:,1));
sdfkp = sdfkp.addBoundingBoxConstraint(BoundingBoxConstraint(vstar,vstar),sdfkp.v_inds(:,1));
sdfkp = sdfkp.addBoundingBoxConstraint(BoundingBoxConstraint(vstar,vstar),sdfkp.v_inds(:,end));

sdfkp = sdfkp.addBoundingBoxConstraint(BoundingBoxConstraint(0.02*ones(nT-1,1),0.2*ones(nT-1,1)),sdfkp.h_inds(:));

sdfkp = sdfkp.addCoMBounds(1:nT,bsxfun(@times,com_star-[0.5;0.5;0.5],ones(1,nT)),bsxfun(@times,com_star+[0.5;0.5;1],ones(1,nT)));

sdfkp = sdfkp.addBoundingBoxConstraint(BoundingBoxConstraint(...
  reshape(bsxfun(@times,[-9.81/2;-9.81/2;-9.81],ones(1,nT)),[],1),...
  reshape(bsxfun(@times,[9.81/2;9.81/2;9.81],ones(1,nT)),[],1)),sdfkp.comddot_inds(:));

x_seed = zeros(sdfkp.num_vars,1);
x_seed(sdfkp.h_inds) = 0.1;
x_seed(sdfkp.q_inds(:)) = reshape(bsxfun(@times,qstar,ones(1,nT)),[],1);
x_seed(sdfkp.com_inds(:)) = reshape(bsxfun(@times,com_star,ones(1,nT)),[],1);
x_seed(sdfkp.lambda_inds{1}(:)) = reshape(bsxfun(@times,[0;0;sdfkp.robot_mass*sdfkp.g/8],ones(1,4,nT)),[],1);
x_seed(sdfkp.lambda_inds{2}(:)) = reshape(bsxfun(@times,[0;0;sdfkp.robot_mass*sdfkp.g/8],ones(1,4,nT)),[],1);

sdfkp = sdfkp.setSolverOptions('snopt','iterationslimit',1e6);
sdfkp = sdfkp.setSolverOptions('snopt','majoriterationslimit',1000);
sdfkp = sdfkp.setSolverOptions('snopt','majorfeasibilitytolerance',5e-5);
sdfkp = sdfkp.setSolverOptions('snopt','majoroptimalitytolerance',1e-3);
sdfkp = sdfkp.setSolverOptions('snopt','print','snopt.out');
tic
[x_sol,F,info] = sdfkp.solve(x_seed);
toc
q_sol = reshape(x_sol(sdfkp.q_inds(:)),nq,nT);
v_sol = reshape(x_sol(sdfkp.v_inds(:)),nq,nT);
h_sol = reshape(x_sol(sdfkp.h_inds),1,[]);
x_traj_sol = PPTrajectory(foh(h_sol,[q_sol;v_sol]));
x_traj_sol = x_traj_sol.setOutputFrame(robot.getStateFrame);
keyboard;
end