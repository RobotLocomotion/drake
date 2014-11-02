function testComDynamicsFullKinematicsPlanner3
% This is a trivial test, the robot should stand still on the flat ground. I intend to test the
% complementarity constraint constraint
warning('off','Drake:RigidBody:SimplifiedCollisionGeometry');
warning('off','Drake:RigidBody:NonPositiveInertiaMatrix');
warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints');
warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits');
urdf = [getDrakePath,'/examples/Atlas/urdf/atlas_minimal_contact.urdf'];
options.floating = true;
robot = RigidBodyManipulator(urdf,options);
nomdata = load([getDrakePath,'/examples/Atlas/data/atlas_fp.mat']);
nq = robot.getNumPositions();
qstar = nomdata.xstar(1:nq);
kinsol_star = robot.doKinematics(qstar,false,false);
nv = robot.getNumVelocities();
vstar = zeros(nv,1);

l_foot = robot.findLinkInd('l_foot');
r_foot = robot.findLinkInd('r_foot');
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

l_foot_pos_star = robot.forwardKin(kinsol_star,l_foot,[0;0;0],2);
r_foot_pos_star = robot.forwardKin(kinsol_star,r_foot,[0;0;0],2);
com_star = robot.getCOM(kinsol_star);

mu = 1;
plane_normal = [0;0;1];
plane_pt = [0;0;0];

l_foot_complementary_contact_wrench = ComplementarityFrictionConeWrench(robot,l_foot,l_foot_bottom,mu,plane_normal,@(pt_pos) phi_handle(pt_pos,plane_normal,plane_pt));
r_foot_complementary_contact_wrench = ComplementarityFrictionConeWrench(robot,r_foot,r_foot_bottom,mu,plane_normal,@(pt_pos) phi_handle(pt_pos,plane_normal,plane_pt));

l_foot_contact_wrench = FrictionConeWrench(robot,l_foot,l_foot_bottom,mu,plane_normal);
r_foot_contact_wrench = FrictionConeWrench(robot,r_foot,r_foot_bottom,mu,plane_normal);

nT = 4;
rb_wrench = struct('active_knot',[],'cw',[]);
rb_wrench(1) = struct('active_knot',[1 nT],'cw',l_foot_complementary_contact_wrench);
rb_wrench(2) = struct('active_knot',[1 nT],'cw',r_foot_complementary_contact_wrench);
rb_wrench(3) = struct('active_knot',2:nT-1,'cw',l_foot_contact_wrench);
rb_wrench(4) = struct('active_knot',2:nT-1,'cw',r_foot_contact_wrench);

tf_range = [0.5 1];
q_nom = bsxfun(@times,qstar,ones(1,nT));
Q_comddot = eye(3);
Q = eye(nq);
Q(1,1) = 0;
Q(2,2) = 0;
Q(6,6) = 0;
Qv = 0.1*eye(nv);
Q_contact_force = 0*eye(3);
cdfkp = ComDynamicsFullKinematicsPlanner(robot,nT,tf_range,Q_comddot,Qv,Q,q_nom,Q_contact_force,rb_wrench);
cdfkp = cdfkp.addRigidBodyConstraint(WorldFixedBodyPoseConstraint(robot,l_foot),{2:nT-1});
cdfkp = cdfkp.addRigidBodyConstraint(WorldFixedBodyPoseConstraint(robot,r_foot),{2:nT-1});
cdfkp = cdfkp.addRigidBodyConstraint(WorldPositionConstraint(robot,l_foot,l_foot_bottom,[nan(2,size(l_foot_bottom,2));zeros(1,size(l_foot_bottom,2))],[nan(2,size(l_foot_bottom,2));zeros(1,size(l_foot_bottom,2))]),{2});
cdfkp = cdfkp.addRigidBodyConstraint(WorldPositionConstraint(robot,r_foot,r_foot_bottom,[nan(2,size(r_foot_bottom,2));zeros(1,size(r_foot_bottom,2))],[nan(2,size(r_foot_bottom,2));zeros(1,size(r_foot_bottom,2))]),{2});

cdfkp = cdfkp.addBoundingBoxConstraint(BoundingBoxConstraint(com_star(3)-0.2,com_star(3)-0.1),cdfkp.com_inds(3,nT));

x_seed = zeros(cdfkp.num_vars,1);
x_seed(cdfkp.h_inds) = 0.2;
x_seed(cdfkp.q_inds(:)) = reshape(bsxfun(@times,qstar,ones(1,nT)),[],1);
x_seed(cdfkp.com_inds(:)) = reshape(bsxfun(@times,com_star,ones(1,nT)),[],1);
x_seed(cdfkp.lambda_inds{1}(:)) = reshape(bsxfun(@times,[0;0;cdfkp.robot_mass*cdfkp.g/8],ones(1,4,nT)),[],1);
x_seed(cdfkp.lambda_inds{2}(:)) = reshape(bsxfun(@times,[0;0;cdfkp.robot_mass*cdfkp.g/8],ones(1,4,nT)),[],1);

cdfkp = cdfkp.setSolverOptions('snopt','print','test_cdfkp3.out');
tic
[x_sol,F,info] = cdfkp.solve(x_seed);
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
wrench_sol = cdfkp.contactWrench(x_sol);
keyboard;
end

function [c,dc] = phi_handle(pt_pos,plane_normal,plane_pt)
num_pts = size(pt_pos,2);
pos2pt = pt_pos-bsxfun(@times,plane_pt,ones(1,num_pts));
c = sum(pos2pt.*bsxfun(@times,plane_normal,ones(1,num_pts)),1)';
dc = kron(speye(num_pts),plane_normal');
end
