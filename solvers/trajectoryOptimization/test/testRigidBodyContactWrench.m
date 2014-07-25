function testRigidBodyContactWrench
robot = RigidBodyManipulator([getDrakePath,'/examples/Atlas/urdf/atlas_minimal_contact.urdf'],struct('floating',true));
nomdata = load([getDrakePath,'/examples/Atlas/data/atlas_fp.mat']);
nq = robot.getNumDOF();
qstar = nomdata.xstar(1:robot.getNumDOF());
q = qstar+1e-2*randn(nq,1);
kinsol = robot.doKinematics(qstar);

display('Check FrictionConeWrench');
l_foot = robot.findLinkInd('l_foot');
nLPts = length(robot.getBody(l_foot).getContactShapes);
l_foot_pt = zeros(3,nLPts);
for i=1:nLPts,
  l_foot_pt(:,i) = robot.getBody(l_foot).getContactShapes{i}.getPoints;
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
[c,dc] = nlcon.eval(qstar,F,kinsol);
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
valuecheck(reshape(A*F(:),3,num_pts),F,1e-5);

testRigidBodyContactWrench_userfun(fc_wrench,qstar+1e-2*randn(nq,1),F);


%%%%%%%%
display('Check LinearFrictionConeWrench')
mu = 0.7;
FC_edge = rpy2rotmat(0.05*randn(3,1))*[mu*cos(linspace(0,2*pi,4));mu*sin(linspace(0,2*pi,4));ones(1,4)];
fc_wrench = LinearFrictionConeWrench(robot,l_foot,l_foot_pt,FC_edge);
valuecheck(num_pts,fc_wrench.num_pts);
F = rand(4,num_pts);
A = fc_wrench.force();
valuecheck(reshape(A*F(:),3,[]),fc_wrench.FC_edge*F,1e-5);
A = fc_wrench.torque();
valuecheck(reshape(A*F(:),3,num_pts),zeros(3,num_pts),1e-5);
testRigidBodyContactWrench_userfun(fc_wrench,qstar+1e-2*randn(nq,1),F);

%%%%%%
display('Check GraspWrench');
force_max = 100;
A_torque = [eye(3);ones(1,3)];
b_torque_ub = [10;20;30;40];
b_torque_lb = [-10;-20;-30;-40];
grasp_wrench = GraspWrench(robot,l_hand,[0;0;0],force_max,A_torque,b_torque_lb,b_torque_ub);
F = 10*randn(6,100);

testRigidBodyContactWrench_userfun(grasp_wrench,qstar+1e-2*randn(nq,1),F);
end

function testRigidBodyContactWrench_userfun(rb_wrench,q,F)
% check constraint gradient
nq = rb_wrench.robot.getNumDOF();
kinsol = rb_wrench.robot.doKinematics(q);
[lincon,nlcon,bcon] = rb_wrench.generateWrenchConstraint();
[c,dc] = nlcon.eval(q,F,kinsol);
[~,dc_numeric] = geval(@(x) evalConstraint(nlcon,rb_wrench.robot,x(1:nq),x(nq+1:end)),[q;F(:)],struct('grad_method','numerical'));
valuecheck(dc,dc_numeric,1e-5);
dc_sparse = zeros(nlcon.num_cnstr,nlcon.xdim);
dc_sparse(sub2ind([nlcon.num_cnstr,nlcon.xdim],nlcon.iCfun,nlcon.jCvar)) = dc(sub2ind([nlcon.num_cnstr,nlcon.xdim],nlcon.iCfun,nlcon.jCvar));
valuecheck(dc,dc_sparse,1e-10);
end

function c = evalConstraint(nlcon,robot,q,F)
kinsol = robot.doKinematics(q);
c = nlcon.eval(q,F,kinsol);
end