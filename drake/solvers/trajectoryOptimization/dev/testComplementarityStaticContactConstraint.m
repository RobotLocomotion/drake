function testComplementarityStaticContactConstraint
robot = RigidBodyManipulator([getDrakePath,'/examples/Atlas/urdf/atlas_minimal_contact.urdf'],struct('floating',true));
nomdata = load([getDrakePath,'/examples/Atlas/data/atlas_fp.mat']);
nq = robot.getNumPositions();
qstar = nomdata.xstar(1:robot.getNumPositions());
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
kinsol_star = robot.doKinematics(qstar,false,false);

csc_cnstr = ComplementarityStaticContactConstraint(fc_wrench);
[nlcon,slack_bcon,num_slack,slack_name] = csc_cnstr.generateConstraint();
q1 = qstar+1e-2*randn(nq,1);
q2 = qstar+1e-2*randn(nq,1);
lambda = randn(fc_wrench.num_pt_F,fc_wrench.num_pts);
kinsol1 = robot.doKinematics(q1,false,false);
kinsol2 = robot.doKinematics(q2,false,false);
gamma = randn(2*fc_wrench.num_pts,1);
[c,dc] = nlcon.eval(q1,q2,lambda,gamma,kinsol1,kinsol2);
[~,dc_numeric] = geval(@(q1,q2,lambda,gamma) evalComplementarityStaticConstraint(nlcon,robot,q1,q2,lambda,gamma),q1,q2,lambda,gamma,struct('grad_method','numerical'));
valuecheck(dc,dc_numeric,1e-3);
dc_sparse = zeros(nlcon.num_cnstr,nlcon.xdim);
dc_sparse(sub2ind([nlcon.num_cnstr,nlcon.xdim],nlcon.iCfun,nlcon.jCvar)) = dc(sub2ind([nlcon.num_cnstr,nlcon.xdim],nlcon.iCfun,nlcon.jCvar)); 
valuecheck(dc,dc_sparse);

csssc_cnstr = ComplementaritySingleSideStaticContactConstraint(fc_wrench);
[nlcon,slack_bcon,num_slack,slack_name] = csssc_cnstr.generateConstraint();
[c,dc] = nlcon.eval(q1,q2,lambda1,[gamma1;gamma2],kinsol1,kinsol2);
[~,dc_numeric] = geval(@(q1,q2,lambda,gamma) evalComplementaritySingleSideStaticContactConstraint(nlcon,robot,q1,q2,lambda,gamma),q1,q2,lambda1,[gamma1;gamma2],struct('grad_method','numerical'));
valuecheck(dc,dc_numeric,1e-3);
dc_sparse = zeros(nlcon.num_cnstr,nlcon.xdim);
dc_sparse(sub2ind([nlcon.num_cnstr,nlcon.xdim],nlcon.iCfun,nlcon.jCvar)) = dc(sub2ind([nlcon.num_cnstr,nlcon.xdim],nlcon.iCfun,nlcon.jCvar)); 
valuecheck(dc,dc_sparse);
end

function [c,dc] = evalComplementarityStaticConstraint(nlcon,robot,q1,q2,lambda,gamma)
kinsol1 = robot.doKinematics(q1,false,false);
kinsol2 = robot.doKinematics(q2,false,false);
[c,dc] = nlcon.eval(q1,q2,lambda,gamma,kinsol1,kinsol2);
end

% function [c,dc] = evalComplementaritySingleSideStaticContactConstraint(nlcon,robot,q1,q2,lambda,gamma)
% kinsol1 = robot.doKinematics(q1,false,false);
% kinsol2 = robot.doKinematics(q2,false,false);
% [c,dc] = nlcon.eval(q1,q2,lambda,gamma,kinsol1,kinsol2);
% end
