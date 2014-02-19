function testFrictionConeWrenchConstraint
robot = RigidBodyManipulator([getDrakePath,'/examples/Atlas/urdf/atlas_minimal_contact.urdf'],struct('floating',true));
nomdata = load([getDrakePath,'/examples/Atlas/data/atlas_fp.mat']);
nq = robot.getNumDOF();
qstar = nomdata.xstar(1:robot.getNumDOF());
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
[c,dc] = fc_cnst.eval(t,F);
inCone = c>=lb & c<= ub;
valuecheck(inCone',cone_angle<=alpha);
end
display('Check eval gradient');
for i = 1:10
F = 10*randn(3,num_pts);
[~,dc] = fc_cnst.eval(t,F);
[~,dc_numeric] = geval(@(F) fc_cnst.eval(t,F),F,struct('grad_method','numerical'));
valuecheck(dc,dc_numeric,1e-5);
end

display('Check torque');
F = 10*randn(3,num_pts);
kinsol = robot.doKinematics(qstar);
l_foot_pos = forwardKin(robot,kinsol,l_foot,l_foot_pt,0);
[tau,dtau] = fc_cnst.torque(t,kinsol,F);
valuecheck(tau,sum(cross(l_foot_pos,F,1),2),1e-5);
[~,dtau_numeric] = geval(@(x) evalTorque(robot,l_foot,l_foot_pt,x(1:nq),reshape(x(nq+1:end),3,num_pts)),...
  [qstar;F(:)],struct('grad_method','numerical'));
valuecheck(dtau,dtau_numeric,1e-4);

display('Check force');
F = 10*randn(3,num_pts);
A = fc_cnst.force(t);
valuecheck(A*F(:),sum(F,2),1e-5);

display('Check wrench')
F = 10*randn(3,num_pts);
kinsol = robot.doKinematics(qstar);
[w,dw] = fc_cnst.wrench(t,kinsol,F);
valuecheck(w,evalWrench(robot,l_foot,l_foot_pt,qstar,F),1e-10);
[~,dw_numeric] = geval(@(x) evalWrench(robot,l_foot,l_foot_pt,x(1:nq),reshape(x(nq+1:end),3,num_pts)),...
  [qstar;F(:)],struct('grad_method','numerical'));
valuecheck(dw,dw_numeric,1e-4);
end

function tau = evalTorque(robot,body,body_pt,q,F)
kinsol = robot.doKinematics(q);
body_pos = forwardKin(robot,kinsol,body,body_pt,0);
tau = sum(cross(body_pos,F,1),2);
end

function w = evalWrench(robot,body,body_pt,q,F)
  tau = evalTorque(robot,body,body_pt,q,F);
  w = [sum(F,2);tau];
end