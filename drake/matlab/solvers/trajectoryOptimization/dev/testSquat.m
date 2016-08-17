function testSquat
% The robot stands there, lowering down the com, and then bring the com back again.
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

l_foot = robot.findLinkId('l_foot');
r_foot = robot.findLinkId('r_foot');
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

lfoot_pos_star = robot.forwardKin(kinsol_star,l_foot,[0;0;0],2);
rfoot_pos_star = robot.forwardKin(kinsol_star,r_foot,[0;0;0],2);
com_star = robot.getCOM(kinsol_star);

mu = 1;
nT = 7;
num_edges = 3;
FC_angles = linspace(0,2*pi,num_edges+1);FC_angles(end) = [];
g = 9.81;
FC_axis = [0;0;1];
FC_perp1 = rotx(pi/2)*FC_axis;
FC_perp2 = cross(FC_axis,FC_perp1);
FC_edge = bsxfun(@plus,FC_axis,mu*(bsxfun(@times,cos(FC_angles),FC_perp1) + ...
                                   bsxfun(@times,sin(FC_angles),FC_perp2)));
FC_edge = bsxfun(@rdivide,FC_edge,sqrt(sum(FC_edge.^2,1)));
FC_edge = FC_edge*robot.getMass*g;
l_foot_contact_wrench = struct('active_knot',1:nT,'cw',LinearFrictionConeWrench(robot,l_foot,l_foot_bottom,FC_edge));
r_foot_contact_wrench = struct('active_knot',1:nT,'cw',LinearFrictionConeWrench(robot,r_foot,r_foot_bottom,FC_edge));

bky_idx = robot.getBody(robot.findJointId('back_bky')).position_num;

tf_range = [1 1.5];
q_nom = bsxfun(@times,qstar,ones(1,nT));
Q_comddot = eye(3);
Q = eye(nq);
Q(1,1) = 0;
Q(2,2) = 0;
Q(6,6) = 0;
Q(bky_idx,bky_idx) = 100*Q(bky_idx,bky_idx);
Qv = 0.1*eye(nv);
Q_contact_force = 1/(robot.getMass*g)^2*eye(3);
cdfkp = ComDynamicsFullKinematicsPlanner(robot,nT,tf_range,Q_comddot,Qv,Q,q_nom,Q_contact_force,[l_foot_contact_wrench r_foot_contact_wrench]);

lfoot_on_ground = {WorldPositionConstraint(robot,l_foot,[0;0;0],lfoot_pos_star(1:3),lfoot_pos_star(1:3)),...
  WorldQuatConstraint(robot,l_foot,lfoot_pos_star(4:7),0)};
% lfoot_on_ground = {lfoot_on_ground{1}.generateConstraint([]),lfoot_on_ground{2}.generateConstraint([])};
cdfkp = cdfkp.addRigidBodyConstraint(lfoot_on_ground{1},num2cell(2:nT));
cdfkp = cdfkp.addRigidBodyConstraint(lfoot_on_ground{2},num2cell(2:nT));

rfoot_on_ground = {WorldPositionConstraint(robot,r_foot,[0;0;0],rfoot_pos_star(1:3),rfoot_pos_star(1:3)),...
  WorldQuatConstraint(robot,r_foot,rfoot_pos_star(4:7),0)};
% rfoot_on_ground = {rfoot_on_ground{1}.generateConstraint([]),rfoot_on_ground{2}.generateConstraint([])};
cdfkp = cdfkp.addRigidBodyConstraint(rfoot_on_ground{1},num2cell(2:nT));
cdfkp = cdfkp.addRigidBodyConstraint(rfoot_on_ground{2},num2cell(2:nT));

cdfkp = cdfkp.addBoundingBoxConstraint(BoundingBoxConstraint([qstar;vstar;qstar;vstar],[qstar;vstar;qstar;vstar]),[cdfkp.q_inds(:,1);cdfkp.v_inds(:,1);cdfkp.q_inds(:,nT);cdfkp.v_inds(:,nT)]);

cdfkp = cdfkp.addBoundingBoxConstraint(BoundingBoxConstraint(0.02*ones(nT-1,1),0.2*ones(nT-1,1)),cdfkp.h_inds);

middle_knot = ceil(nT/2);

cdfkp = cdfkp.addBoundingBoxConstraint(BoundingBoxConstraint((qstar(5)-0.3)*ones(nT-2,1),(qstar(5)+0.3)*ones(nT-2,1)),cdfkp.q_inds(5,2:nT-1));

cdfkp = cdfkp.addBoundingBoxConstraint(BoundingBoxConstraint([com_star(1:2);com_star(3)-0.5],[com_star(1:2);com_star(3)-0.3]),cdfkp.com_inds(:,middle_knot));

x_seed = zeros(cdfkp.num_vars,1);
x_seed(cdfkp.h_inds) = 0.1;
x_seed(cdfkp.q_inds(:)) = reshape(bsxfun(@times,qstar,ones(1,nT)),[],1);
x_seed(cdfkp.com_inds(:)) = reshape(bsxfun(@times,com_star,ones(1,nT)),[],1);
x_seed(cdfkp.lambda_inds{1}(:)) = reshape(1/(8*num_edges)*ones(num_edges,4,7),[],1);
x_seed(cdfkp.lambda_inds{2}(:)) = reshape(1/(8*num_edges)*ones(num_edges,4,7),[],1);

cdfkp = cdfkp.setSolverOptions('snopt','iterationslimit',1e6);
cdfkp = cdfkp.setSolverOptions('snopt','majoriterationslimit',2000);
cdfkp = cdfkp.setSolverOptions('snopt','majorfeasibilitytolerance',1e-6);
cdfkp = cdfkp.setSolverOptions('snopt','majoroptimalitytolerance',2e-4);
cdfkp = cdfkp.setSolverOptions('snopt','superbasicslimit',2000);
cdfkp = cdfkp.setSolverOptions('snopt','print','test_cdfkp1.out');

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
Hdot_sol = reshape(x_sol(cdfkp.Hdot_inds),3,[])*cdfkp.torque_multiplier;
lambda_sol = cell(2,1);
lambda_sol{1} = reshape(x_sol(cdfkp.lambda_inds{1}),size(cdfkp.lambda_inds{1},1),[],nT);
lambda_sol{2} = reshape(x_sol(cdfkp.lambda_inds{2}),size(cdfkp.lambda_inds{2},1),[],nT);
xtraj_sol = PPTrajectory(foh(cumsum([0 h_sol]),[q_sol;v_sol]));
xtraj_sol = xtraj_sol.setOutputFrame(robot.getStateFrame);
wrench_sol = cdfkp.contactWrench(x_sol);
for i = 1:nT
  kinsol_tmp = robot.doKinematics(q_sol(:,i),false,false);
  valuecheck(robot.getCOM(kinsol_tmp),com_sol(:,i),1e-3);
  A = robot.centroidalMomentumMatrix(kinsol_tmp);
  valuecheck(A(1:3,:)*v_sol(:,i),H_sol(:,i),1e-3);
end
valuecheck(diff(com_sol,[],2)./bsxfun(@times,ones(3,1),h_sol),(comdot_sol(:,1:end-1)+comdot_sol(:,2:end))/2,1e-3);
valuecheck(diff(comdot_sol,[],2)./bsxfun(@times,ones(3,1),h_sol),comddot_sol(:,2:end),1e-3);
valuecheck(diff(H_sol,[],2)./bsxfun(@times,ones(3,1),h_sol),Hdot_sol(:,2:end),1e-3);
lfoot_force = reshape(l_foot_contact_wrench.cw.force*reshape(lambda_sol{1},[],nT),3,[],nT);
rfoot_force = reshape(r_foot_contact_wrench.cw.force*reshape(lambda_sol{2},[],nT),3,[],nT);
total_force = squeeze(sum(lfoot_force,2)+sum(rfoot_force,2))-bsxfun(@times,[0;0;cdfkp.robot_mass*cdfkp.g],ones(1,nT));
valuecheck(total_force/cdfkp.robot_mass,comddot_sol,1e-3);
lfoot_contact_pos = robot.forwardKin(kinsol_star,l_foot,l_foot_bottom,0);
rfoot_contact_pos = robot.forwardKin(kinsol_star,r_foot,r_foot_bottom,0);
for i = 1:nT
torque = crossSum(lfoot_contact_pos-bsxfun(@times,com_sol(:,i),ones(1,size(lfoot_contact_pos,2))),lfoot_force(:,:,i))...
  +crossSum(rfoot_contact_pos-bsxfun(@times,com_sol(:,i),ones(1,size(rfoot_contact_pos,2))),rfoot_force(:,:,i));
if(norm(torque-Hdot_sol(:,i))>1e-2*norm(torque))
  error('Centroidal angular momentum has large numerical error');
end
end
keyboard;
end
