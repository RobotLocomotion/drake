function testQuasiStaticConstraint
urdf = [getDrakePath,'/examples/Atlas/urdf/atlas_minimal_contact.urdf'];
options.floating = true;
r = RigidBodyManipulator(urdf,options);
nq = r.getNumDOF();

l_foot = r.findLinkInd('l_foot');
r_foot = r.findLinkInd('r_foot');
l_hand = r.findLinkInd('l_hand');
r_hand = r.findLinkInd('r_hand');
head = r.findLinkInd('head');
l_foot_pts = r.getBody(l_foot).contact_pts;
r_foot_pts = r.getBody(r_foot).contact_pts;
l_hand_pts = [0;0;0];
r_hand_pts = [0;0;0];

nom_data = load('../../../../examples/Atlas/data/atlas_fp.mat');
q = nom_data.xstar(1:nq);

shrinkFactor = 0.9;
tspan = [0,1];
t = 0.5;
qsc = QuasiStaticConstraint(r,tspan);
qsc = qsc.setActive(true);
qsc = qsc.setShrinkFactor(shrinkFactor);
qsc = qsc.addContact(l_foot,l_foot_pts,r_foot,r_foot_pts);

kinsol = doKinematics(r,q,false,false);
l_foot_pos = forwardKin(r,kinsol,l_foot,l_foot_pts,0);
r_foot_pos = forwardKin(r,kinsol,r_foot,r_foot_pts,0);
com = getCOM(r,kinsol);
center_pos = mean([l_foot_pos r_foot_pos],2);
shrink_vertices = [l_foot_pos r_foot_pos]*shrinkFactor+repmat(center_pos*(1-shrinkFactor),1,size(l_foot_pts,2)+size(r_foot_pts,2));
num_vertices = size(shrink_vertices,2);
exit_flag = -1;
while(exit_flag ~= 1)
  quadoptions = optimset('Algorithm','interior-point-convex');
[weights,~,exit_flag] = quadprog(eye(num_vertices),rand(num_vertices,1),...
  [],[],...
  [shrink_vertices(1:2,:);ones(1,num_vertices)],[com(1:2);1],...
  zeros(num_vertices,1),ones(num_vertices,1),...
  1/num_vertices*ones(num_vertices,1),quadoptions);
end
[c,dc] = qsc.eval(t,kinsol,weights);
valuecheck(c,[0;0]);
[active_mex,num_weights_mex,c_mex,dc_mex,lb_mex,ub_mex] = testQuasiStaticConstraintmex(qsc.mex_ptr,q,weights,t);
valuecheck(qsc.active,active_mex);
valuecheck(num_weights_mex,qsc.num_pts);
valuecheck(c_mex,c);
valuecheck(dc_mex,dc);
valuecheck(lb_mex,[0;0]);
valuecheck(ub_mex,[0;0]);

display('Check qsc with an affordance in the robot');
nq_cache = nq;
r = r.addRobotFromURDF('valve_task_wall.urdf',rand(3,1),pi*randn(3,1),struct('floating',false));
qsc = qsc.updateRobot(r);
nq = r.getNumDOF();
q = [q;randn(length(r.getStateFrame.frame{2}.coordinates)/2,1)];
kinsol = doKinematics(r,q);
[c2,dc2] = qsc.eval(t,kinsol,weights);
[~,~,c_mex2,dc_mex2,lb_mex2,ub_mex2] = testQuasiStaticConstraintmex(qsc.mex_ptr,q,weights,t);
valuecheck(c,c2,1e-10);
valuecheck(dc,dc2(:,[1:nq_cache nq+1:nq+num_weights_mex]),1e-10);
valuecheck(c_mex2,c2,1e-10);
valuecheck(dc_mex2,dc2,1e-10);
valuecheck(lb_mex2,[0;0]);
valuecheck(ub_mex2,[0;0]);
end