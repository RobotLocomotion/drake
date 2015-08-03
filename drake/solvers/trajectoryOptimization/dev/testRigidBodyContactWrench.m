function testRigidBodyContactWrench
robot = RigidBodyManipulator([getDrakePath,'/examples/Atlas/urdf/atlas_minimal_contact.urdf'],struct('floating',true));
nomdata = load([getDrakePath,'/examples/Atlas/data/atlas_fp.mat']);
nq = robot.getNumPositions();
qstar = nomdata.xstar(1:robot.getNumPositions());
q = qstar+1e-2*randn(nq,1);
kinsol = robot.doKinematics(qstar);

display('Check FrictionConeWrench');
l_foot = robot.findLinkId('l_foot');
nLPts = length(robot.getBody(l_foot).getCollisionGeometry);
l_foot_pt = zeros(3,nLPts);
for i=1:nLPts,
  l_foot_pt(:,i) = robot.getBody(l_foot).getCollisionGeometry{i}.getPoints;
end
num_pts = size(l_foot_pt,2);
mu = 0.7;
FC_axis = rpy2rotmat(0.02*rand(3,1))*[0;0;2];

fc_wrench = FrictionConeWrench(robot,l_foot,l_foot_pt,mu,FC_axis);
valuecheck(num_pts,fc_wrench.num_pts);
valuecheck(sum(fc_wrench.FC_axis.^2,1),ones(1,num_pts));

[lincon,nlcon,bcon] = fc_wrench.generateWrenchConstraint;
alpha = atan(mu);
display('Check if the friction cone constraint is consistent with valid contact force');
if(any(~isinf(bcon.lb)) || any(~isinf(bcon.ub)))
  error('The BoundingBoxConstraint for friction cone is incorrect');
end
for i = 1:1000
F = 10*randn(3,num_pts);
cone_angle = acos(sum(F.*fc_wrench.FC_axis,1)./sqrt(sum(F.^2,1)));
[c,dc] = nlcon.eval(qstar,F,[],kinsol);
inCone = c>=nlcon.lb & c<= nlcon.ub & lincon.A*F(:)<=lincon.ub &lincon.A*F(:)>=lincon.lb;
valuecheck(inCone',cone_angle<=alpha);
end

display('Check torque');
F = 10*randn(3,num_pts);
A = fc_wrench.torque();
valuecheck(reshape(A*F(:),3,num_pts),zeros(3,num_pts),1e-5);

display('Check force');
F = 10*randn(3,num_pts);
A = fc_wrench.force();
valuecheck(reshape(A*F(:),3,num_pts),F*fc_wrench.force_normalize_factor,1e-5);

testRigidBodyContactWrench_userfun(fc_wrench,qstar+1e-2*randn(nq,1),F,[]);


%%%%%%%%
display('Check LinearFrictionConeWrench')
mu = 0.7;
FC_edge = rpy2rotmat(0.05*randn(3,1))*[mu*cos(linspace(0,2*pi,4));mu*sin(linspace(0,2*pi,4));ones(1,4)];
lfc_wrench = LinearFrictionConeWrench(robot,l_foot,l_foot_pt,FC_edge);
[lincon,nlcon,bcon] = lfc_wrench.generateWrenchConstraint();
valuecheck(bcon.lb,zeros(size(l_foot_pt,2)*size(FC_edge,2),1));
valuecheck(num_pts,lfc_wrench.num_pts);
F = rand(4,num_pts);
A = lfc_wrench.force();
valuecheck(reshape(A*F(:),3,[]),lfc_wrench.FC_edge*F,1e-5);
A = lfc_wrench.torque();
valuecheck(reshape(A*F(:),3,num_pts),zeros(3,num_pts),1e-5);
testRigidBodyContactWrench_userfun(lfc_wrench,qstar+1e-2*randn(nq,1),F,[]);

%%%%%%
display('Check GraspWrench');
l_hand = robot.findLinkId('l_hand');
force_max = 100;
A_torque = [eye(3);ones(1,3)];
b_torque_ub = [10;20;30;40];
b_torque_lb = [-10;-20;-30;-40];
grasp_wrench = GraspWrench(robot,l_hand,[0;0;0],force_max,A_torque,b_torque_lb,b_torque_ub);
[lincon,nlcon,bcon] = grasp_wrench.generateWrenchConstraint();
F = 10*randn(6,100);
valid_flag = all([sum(F(1:3,:).^2,1)<force_max^2;A_torque*F(4:6,:)<=bsxfun(@times,b_torque_ub,ones(1,100));A_torque*F(4:6,:)>=bsxfun(@times,b_torque_lb,ones(1,100))],1);
for i = 1:100
  nlcon_val = nlcon.eval(q,F(:,i),[],kinsol);
%   valuecheck(valid_flag(i),all(lincon.A*F(:,i)<=lincon.ub) & all(lincon.A*F(:,i)>=lincon.lb) & all(nlcon_val<=nlcon.ub) &all(nlcon_val>=nlcon.lb) & all(F(:,i)<=bcon.ub) &all(F(:,i)>=bcon.lb));
end
F = randn(6,1);
testRigidBodyContactWrench_userfun(grasp_wrench,qstar+1e-2*randn(nq,1),F,[]);

%%%%%
display('Check ComplementarityFrictionConeWrench');
plane_normal = randn(3,1);
plane_normal = plane_normal/norm(plane_normal);
plane_pt = randn(3,1);
cfc_wrench = ComplementarityFrictionConeWrench(robot,l_foot,l_foot_pt,mu,FC_axis,@(pt_pos) phi_distance(pt_pos,plane_normal,plane_pt));
[lincon,nlcon,bcon] = cfc_wrench.generateWrenchConstraint();
F = randn(3,size(l_foot_pt,2));
kinsol_star = robot.doKinematics(qstar,false,false);
pt_pos = randn(3,4);
[c,dc] = phi_distance(pt_pos,plane_normal,plane_pt);
[~,dc_numeric] = geval(@(pt_pos) phi_distance(pt_pos,plane_normal,plane_pt),pt_pos,struct('grad_method','numerical'));
valuecheck(dc,dc_numeric,1e-3);
valuecheck(bcon.lb(end-size(l_foot_pt,2)+1:end),zeros(size(l_foot_pt,2),1));
gamma = randn(size(l_foot_pt,2),1);
testRigidBodyContactWrench_userfun(cfc_wrench,qstar+1e-2*randn(nq,1),F,gamma);
q = qstar+1e-2*randn(nq,1);
kinsol = robot.doKinematics(q,false,false);
l_foot_pos = robot.forwardKin(kinsol,l_foot,l_foot_pt,0);
l_foot_pos_normal = cross(l_foot_pos(:,2)-l_foot_pos(:,1),l_foot_pos(:,3)-l_foot_pos(:,1));
l_foot_pos_normal = l_foot_pos_normal/norm(l_foot_pos_normal);
cfc_wrench = ComplementarityFrictionConeWrench(robot,l_foot,l_foot_pt,mu,l_foot_pos_normal,@(pt_pos) phi_distance(pt_pos,l_foot_pos_normal,l_foot_pos(:,1)));
[lincon,nlcon,bcon] = cfc_wrench.generateWrenchConstraint();
F_valid = false;
while(~F_valid)
F = randn(3,100);
valid_idx = sum(F.*bsxfun(@times,l_foot_pos_normal,ones(1,100)),1)./sqrt(sum(F.^2,1))>=1/sqrt(1+mu^2);
F = F(:,valid_idx);
if(size(F,2)>=size(l_foot_pt,2))
  F = F(:,1:size(l_foot_pt,2));
  F_valid = true;
end
end
gamma = zeros(size(l_foot_pt,2),1);
[c,dc] = nlcon.eval(q,F,gamma,kinsol);
if(any(c>nlcon.ub+1e-3) || any(c<nlcon.lb-1e-3))
  error('Nonlinear constraint in ComplementaryFrictionConeWrench is not correct');
end
if(any(lincon.A*[F(:);gamma]>lincon.ub+1e-3) || any(lincon.A*[F(:);gamma]<lincon.lb-1e-3))
  error('linear constraint in ComplementaryFrictionConeWrench is incorrect');
end

%%%%%
display('Check ComplementarityLinearFrictionConeWrench');
plane_normal = randn(3,1);
plane_normal = plane_normal/norm(plane_normal);
plane_pt = randn(3,1);
clfc_wrench = ComplementarityLinearFrictionConeWrench(robot,l_foot,l_foot_pt,FC_edge,@(pt_pos) phi_distance(pt_pos,plane_normal,plane_pt));
[lincon,nlcon,bcon] = clfc_wrench.generateWrenchConstraint();
F = 10*rand(size(FC_edge,2),size(l_foot_pt,2));
slack = randn(size(l_foot_pt,2),1);
valuecheck(bcon.lb,zeros(size(FC_edge,2)*size(l_foot_pt,2)+size(l_foot_pt,2),1));
testRigidBodyContactWrench_userfun(clfc_wrench,qstar+1e-2*randn(nq,1),F,slack);
gamma = zeros(size(l_foot_pt,2),1);
q = qstar+1e-2*randn(nq,1);
kinsol = robot.doKinematics(q);
l_foot_pos = robot.forwardKin(kinsol,l_foot,[l_foot_pt mean(l_foot_pt,2)],0);
l_foot_pos_normal = cross(l_foot_pos(:,2)-l_foot_pos(:,1),l_foot_pos(:,3)-l_foot_pos(:,1));
FC_edge = repmat(l_foot_pos_normal,1,size(l_foot_pt,2))+l_foot_pos(:,1:end-1)-repmat(l_foot_pos(:,end),1,size(l_foot_pt,2));
clfc_wrench = ComplementarityLinearFrictionConeWrench(robot,l_foot,l_foot_pt,FC_edge,@(pt_pos) phi_distance(pt_pos,l_foot_pos_normal,l_foot_pos(:,1)));
F = rand(size(l_foot_pt,2),size(l_foot_pt,2));
[lincon,nlcon,bcon] = clfc_wrench.generateWrenchConstraint();
c = nlcon.eval(q,F,gamma,kinsol);
if(any(c>nlcon.ub+1e-3) || any(c<nlcon.lb-1e-3))
  error('nonlinear constraint in ComplementarityLinearFrictionConeWrench is incorrect');
end
if(any(lincon.A*[F(:);gamma]>lincon.ub+1e-3) || any(lincon.A*[F(:);gamma]<lincon.lb-1e-3))
  error('linear constraint in ComplementarityLinearFrictionConeWrench is incorrect');
end
if(any([F(:);gamma]>bcon.ub+1e-3) || any([F(:);gamma]<bcon.lb-1e-3))
  error('bounding box constraint in ComplementarityLinearFrictionConeWrench is incorrect');
end

%%%%%%%%%%
display('check ComplementarityGraspWrench');
force_max = 100;
A_torque = [eye(3);ones(1,3)];
b_torque_ub = [10;20;30;40];
b_torque_lb = [-10;-20;-30;-40];
cg_wrench = ComplementarityGraspWrench(robot,l_hand,[0;0;0],force_max,A_torque,b_torque_lb,b_torque_ub,@(pt_pos) phi_distance(pt_pos,l_foot_pos_normal,l_foot_pos(:,1)));
slack = randn();
F = randn(6,1);
testRigidBodyContactWrench_userfun(cg_wrench,q,F,slack);

display('check GraspFrictionConeWrench');
robot = robot.addRobotFromURDF('block.urdf',[],[],struct('floating',true));
block = robot.findLinkId('block');
mu = 1;
normal = [0;0;1];
gfc_wrench = GraspFrictionConeWrench(robot,block,l_hand,[[0;0;0] [0;0;1]],normal,mu);
nq_new = robot.getNumPositions();
q_new = zeros(nq_new,1);
q_new(1:nq) = q;
F = randn(3,2);
testRigidBodyContactWrench_userfun(gfc_wrench,q_new,F,[]);
end

function testRigidBodyContactWrench_userfun(rb_wrench,q,F,slack)
% check constraint gradient
nq = rb_wrench.robot.getNumPositions();
kinsol = rb_wrench.robot.doKinematics(q);
[lincon,nlcon,bcon] = rb_wrench.generateWrenchConstraint();
valuecheck(rb_wrench.num_wrench_constraint,nlcon.num_cnstr);
[c,dc] = nlcon.eval(q,F,slack,kinsol);
[~,dc_numeric] = geval(@(x) evalConstraint(nlcon,rb_wrench.robot,x(1:nq),x(nq+(1:rb_wrench.num_pt_F*rb_wrench.num_pts)),x(end-rb_wrench.num_slack+1:end)),[q;F(:);slack],struct('grad_method','numerical'));
valuecheck(dc,dc_numeric,1e-5);
dc_sparse = zeros(nlcon.num_cnstr,nlcon.xdim);
dc_sparse(sub2ind([nlcon.num_cnstr,nlcon.xdim],nlcon.iCfun,nlcon.jCvar)) = dc(sub2ind([nlcon.num_cnstr,nlcon.xdim],nlcon.iCfun,nlcon.jCvar));
valuecheck(dc,dc_sparse,1e-10);
valuecheck(lincon.xdim,rb_wrench.num_pt_F*rb_wrench.num_pts+rb_wrench.num_slack);
valuecheck(bcon.xdim,rb_wrench.num_pt_F*rb_wrench.num_pts+rb_wrench.num_slack);
end

function c = evalConstraint(nlcon,robot,q,F,slack)
kinsol = robot.doKinematics(q);
c = nlcon.eval(q,F,slack,kinsol);
end

function [c,dc] = phi_distance(pt_pos,plane_normal,plane_pt)
num_pts = size(pt_pos,2);
pos2pt = pt_pos-bsxfun(@times,plane_pt,ones(1,num_pts));
c = sum(pos2pt.*bsxfun(@times,plane_normal,ones(1,num_pts)),1)';
dc = kron(speye(num_pts),plane_normal');
end
