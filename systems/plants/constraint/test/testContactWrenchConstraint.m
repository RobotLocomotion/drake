function testContactWrenchConstraint
robot = RigidBodyManipulator([getDrakePath,'/examples/Atlas/urdf/atlas_minimal_contact.urdf'],struct('floating',true));
nomdata = load([getDrakePath,'/examples/Atlas/data/atlas_fp.mat']);
nq = robot.getNumDOF();
qstar = nomdata.xstar(1:robot.getNumDOF());
q = qstar+1e-2*randn(nq,1);
kinsol = robot.doKinematics(qstar);

display('Check FrictionConeWrenchConstraint');
l_foot = robot.findLinkInd('l_foot');
l_foot_pt = robot.getBody(l_foot).getContactPoints();
num_pts = size(l_foot_pt,2);
mu = 0.7;
FC_axis = rpy2rotmat(0.02*rand(3,1))*[0;0;2];
tspan = [0,1];

fc_cnst = FrictionConeWrenchConstraint(robot,l_foot,l_foot_pt,mu,FC_axis,tspan);
valuecheck(num_pts,fc_cnst.num_pts);
valuecheck(sum(fc_cnst.FC_axis.^2,1),ones(1,num_pts));

t = 0.5;
[lb,ub] = fc_cnst.bounds(t);
alpha = atan(mu);
display('Check if the friction cone constraint is consistent with valid contact force');
for i = 1:100
F = 10*randn(3,num_pts);
cone_angle = acos(sum(F.*fc_cnst.FC_axis,1)./sqrt(sum(F.^2,1)));
[c,dc] = fc_cnst.eval(t,kinsol,F);
inCone = c>=lb & c<= ub;
valuecheck(inCone',cone_angle<=alpha);
end

display('Check torque');
F = 10*randn(3,num_pts);
kinsol = robot.doKinematics(qstar);
l_foot_pos = forwardKin(robot,kinsol,l_foot,l_foot_pt,0);
[tau,dtau] = fc_cnst.torque(t,kinsol,F);
valuecheck(tau,sum(cross(l_foot_pos,F,1),2),1e-5);

display('Check force');
F = 10*randn(3,num_pts);
A = fc_cnst.force(t);
valuecheck(A*F(:),sum(F,2),1e-5);

display('Check wrench')
F = 10*randn(3,num_pts);
kinsol = robot.doKinematics(qstar);
[w,dw] = fc_cnst.wrench(t,kinsol,F);
valuecheck(w,evalWrench(fc_cnst,t,qstar,F),1e-10);

testContactWrenchConstraint_userfun(fc_cnst,t,qstar+1e-2*randn(nq,1),F);


%%%%%%%%
display('Check LinearFrictionConeWrenchConstraint')
mu = 0.7;
FC_edge = rpy2rotmat(0.05*randn(3,1))*[mu*cos(linspace(0,2*pi,4));mu*sin(linspace(0,2*pi,4));ones(1,4)];
fc_cnst = LinearFrictionConeWrenchConstraint(robot,l_foot,l_foot_pt,FC_edge,tspan);
valuecheck(num_pts,fc_cnst.num_pts);
F = rand(4,num_pts);
A = fc_cnst.force(t);
valuecheck(A*F(:),sum(fc_cnst.FC_edge*F,2));
kinsol = fc_cnst.robot.doKinematics(q);
[~,dtau] = fc_cnst.torque(t,kinsol,F);
[~,dtau_numeric] = geval(@(x) evalTorque(fc_cnst,t,x(1:nq),reshape(x(nq+1:end),fc_cnst.F_size(1),fc_cnst.F_size(2))),[q;F(:)],struct('grad_method','numerical'));
valuecheck(dtau,dtau_numeric,1e-4);
[~,dw] = fc_cnst.wrench(t,kinsol,F);
[~,dw_numeric] = geval(@(x) evalWrench(fc_cnst,t,x(1:nq),reshape(x(nq+1:end),fc_cnst.F_size(1),fc_cnst.F_size(2))),[q;F(:)],struct('grad_method','numerical'));
valuecheck(dw,dw_numeric,1e-4);

%%%%%%
display('Check RailGraspWrenchConstraint')
r_hand = robot.findLinkInd('r_hand');
rail_axis = randn(3,1);
rail_radius = 0.03;
rail_fc_mu = 0.7;
force_max = 50;
rg_cnst = RailGraspWrenchConstraint(robot,r_hand,[0;0;0],0.1,rail_axis,rail_radius,rail_fc_mu,force_max,tspan);
valuecheck(rg_cnst.num_pts,1);
valuecheck(rg_cnst.F_size,[6,1]);
F = rand(6,1);
kinsol = rg_cnst.robot.doKinematics(q);
A = rg_cnst.force(t);
valuecheck(A*F(:),F(1:3),1e-8);
tau = rg_cnst.torque(t,kinsol,F);
grasp_pos = robot.forwardKin(kinsol,r_hand,[0;0;0],0);
valuecheck(tau,cross(grasp_pos,F(1:3))+F(4:6),1e-8);
testContactWrenchConstraint_userfun(rg_cnst,t,q,F);
end

function testContactWrenchConstraint_userfun(fc_cnst,t,q,F)
% check constraint gradient
nq = fc_cnst.robot.getNumDOF();
kinsol = fc_cnst.robot.doKinematics(q);
[~,dc] = fc_cnst.eval(t,kinsol,F);
[~,dc_numeric] = geval(@(x) evalConstraint(fc_cnst,t,x(1:nq),reshape(x(nq+1:end),fc_cnst.F_size(1),fc_cnst.F_size(2))),[q;F(:)],struct('grad_method','numerical'));
valuecheck(dc,dc_numeric,1e-5);
kinsol = fc_cnst.robot.doKinematics(q);
[~,dtau] = fc_cnst.torque(t,kinsol,F);
[~,dtau_numeric] = geval(@(x) evalTorque(fc_cnst,t,x(1:nq),reshape(x(nq+1:end),fc_cnst.F_size(1),fc_cnst.F_size(2))),[q;F(:)],struct('grad_method','numerical'));
valuecheck(dtau,dtau_numeric,1e-4);
[~,dw] = fc_cnst.wrench(t,kinsol,F);
[~,dw_numeric] = geval(@(x) evalWrench(fc_cnst,t,x(1:nq),reshape(x(nq+1:end),fc_cnst.F_size(1),fc_cnst.F_size(2))),[q;F(:)],struct('grad_method','numerical'));
valuecheck(dw,dw_numeric,1e-4);
end

function c = evalConstraint(fc_cnst,t,q,F)
kinsol = fc_cnst.robot.doKinematics(q);
c = fc_cnst.eval(t,kinsol,F);
end

function tau = evalTorque(fc_cnst,t,q,F)
kinsol = fc_cnst.robot.doKinematics(q);
tau = fc_cnst.torque(t,kinsol,F);
end

function w = evalWrench(fc_cnst,t,q,F)
  tau = evalTorque(fc_cnst,t,q,F);
  A = fc_cnst.force(t);
  w = [A*F(:);tau];
end