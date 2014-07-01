function testComplementarityStaticContactConstraint
robot = RigidBodyManipulator([getDrakePath,'/examples/Atlas/urdf/atlas_minimal_contact.urdf'],struct('floating',true));
nomdata = load([getDrakePath,'/examples/Atlas/data/atlas_fp.mat']);
nq = robot.getNumDOF();
qstar = nomdata.xstar(1:robot.getNumDOF());
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
kinsol_star = robot.doKinematics(qstar,false,false);

csc_cnstr = ComplementarityStaticContactConstraint(fc_wrench);

q1 = qstar+1e-2*randn(nq,1);
q2 = qstar+1e-2*randn(nq,1);
lambda1 = randn(fc_wrench.num_pt_F,fc_wrench.num_pts);
lambda2 = randn(fc_wrench.num_pt_F,fc_wrench.num_pts);
kinsol1 = robot.doKinematics(q1,false,false);
kinsol2 = robot.doKinematics(q2,false,false);
[c,dc] = csc_cnstr.eval(q1,q2,lambda1,lambda2,kinsol1,kinsol2);
[~,dc_numeric] = geval(@(q1,q2,lambda1,lambda2) evalComplementarityStaticConstraint(csc_cnstr,q1,q2,lambda1,lambda2),q1,q2,lambda1,lambda2,struct('grad_method','numerical'));
valuecheck(dc,dc_numeric,1e-3);
[c,dc] = csc_cnstr.eval(q1,q1,lambda1,lambda2,kinsol1,kinsol1);
valuecheck(c,zeros(csc_cnstr.num_cnstr,1),1e-3);
[c,dc] = csc_cnstr.eval(q1,q2,zeros(fc_wrench.num_pt_F,fc_wrench.num_pts),lambda2,kinsol1,kinsol2);
valuecheck(c,zeros(csc_cnstr.num_cnstr,1),1e-3);
[c,dc] = csc_cnstr.eval(q1,q2,lambda1,zeros(fc_wrench.num_pt_F,fc_wrench.num_pts),kinsol1,kinsol2);
valuecheck(c,zeros(csc_cnstr.num_cnstr,1),1e-3);
end

function [c,dc] = evalComplementarityStaticConstraint(csc_cnstr,q1,q2,lambda1,lambda2)
kinsol1 = csc_cnstr.rb_wrench.robot.doKinematics(q1,false,false);
kinsol2 = csc_cnstr.rb_wrench.robot.doKinematics(q2,false,false);
[c,dc] = csc_cnstr.eval(q1,q2,lambda1,lambda2,kinsol1,kinsol2);
end