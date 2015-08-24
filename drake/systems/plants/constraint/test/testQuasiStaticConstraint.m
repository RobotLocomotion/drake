function testQuasiStaticConstraint

urdf = [getDrakePath,'/examples/Atlas/urdf/atlas_minimal_contact.urdf'];
options.floating = true;
r = RigidBodyManipulator(urdf,options);
nq = r.getNumPositions();

l_foot = r.findLinkId('l_foot');
r_foot = r.findLinkId('r_foot');
l_hand = r.findLinkId('l_hand');
r_hand = r.findLinkId('r_hand');
head = r.findLinkId('head');
nLPts = length(r.getBody(l_foot).getCollisionGeometry);
l_foot_pts = zeros(3,nLPts);
for i=1:nLPts,
  l_foot_pts(:,i) = r.getBody(l_foot).getCollisionGeometry{i}.getPoints;
end
nRPts = length(r.getBody(r_foot).getCollisionGeometry);
r_foot_pts = zeros(3,nRPts);
for i=1:nRPts,
  r_foot_pts(:,i) = r.getBody(r_foot).getCollisionGeometry{i}.getPoints;
end
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

% todo: move this farther down, so that more tests are included when this
% dependency is not satisfied...
checkDependency('rigidbodyconstraint_mex')

category_name_mex = constraintCategorymex(qsc.mex_ptr);
if(~strcmp(category_name_mex,qsc.categoryString()))
  error('category name string do not match')
end

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
nq = r.getNumPositions();
q = [q;randn(length(r.getStateFrame.frame{2}.getCoordinateNames())/2,1)];
kinsol = doKinematics(r,q);
[c2,dc2] = qsc.eval(t,kinsol,weights);
[~,~,c_mex2,dc_mex2,lb_mex2,ub_mex2] = testQuasiStaticConstraintmex(qsc.mex_ptr,q,weights,t);
valuecheck(c,c2,1e-10);
valuecheck(dc,dc2(:,[1:nq_cache nq+1:nq+num_weights_mex]),1e-10);
valuecheck(c_mex2,c2,1e-10);
valuecheck(dc_mex2,dc2,1e-10);
valuecheck(lb_mex2,[0;0]);
valuecheck(ub_mex2,[0;0]);

display('Check if the mex_ptr and MATLAB object are consistent after making changes to the MATLAB object');
[active_mex_cache,num_weights_mex_cache,c_mex_cache,dc_mex_cache,lb_mex_cache,ub_mex_cache] = testQuasiStaticConstraintmex(qsc.mex_ptr,q,weights,t);
qsc2 = qsc;
qsc2 = qsc2.setActive(~qsc.active);
qsc2 = qsc2.addContact(l_foot,mean(l_foot_pts,2));
qsc2 = qsc2.setShrinkFactor(0.7*qsc.shrinkFactor);
weights2 = [weights;0];
[active2_mex,num_weights2_mex,c2_mex,dc2_mex,lb2_mex,ub2_mex] = testQuasiStaticConstraintmex(qsc2.mex_ptr,q,weights2,t);
valuecheck(qsc2.active,active2_mex);
valuecheck(num_weights2_mex,qsc2.num_pts);
kinsol = r.doKinematics(q);
[c2,dc2] = qsc2.eval(t,kinsol,weights2);
valuecheck(c2,c2_mex,1e-10);
valuecheck(dc2,dc2_mex,1e-10);
valuecheck(lb2_mex,[0;0]);
valuecheck(ub2_mex,[0;0]);

[active_mex,num_weights_mex,c_mex,dc_mex,lb_mex,ub_mex] = testQuasiStaticConstraintmex(qsc.mex_ptr,q,weights,t);
valuecheck(active_mex_cache,active_mex);
valuecheck(num_weights_mex_cache,num_weights_mex);
valuecheck(c_mex_cache,c_mex);
valuecheck(dc_mex_cache,dc_mex);
valuecheck(lb_mex_cache,lb_mex);
valuecheck(ub_mex_cache,ub_mex);

display('Check generateConstraint');
qsc2 = qsc2.setActive(false);
qsc_cnstr = qsc2.generateConstraint(t);
if(~isempty(qsc_cnstr))
  error('QuasiStaticConstraint is not active, no constraint should be generated');
end
qsc2 = qsc2.setActive(true);
qsc_cnstr = qsc2.generateConstraint(t);
valuecheck(qsc_cnstr{1}.lb,[0;0]);
valuecheck(qsc_cnstr{1}.ub,[0;0]);
[c_cnstr,dc_cnstr] = qsc_cnstr{1}.eval(q,weights2,kinsol);
valuecheck(c_cnstr,c2);
valuecheck(dc_cnstr,dc2);
valuecheck(sparse(qsc_cnstr{1}.iCfun,qsc_cnstr{1}.jCvar,dc2(sub2ind([qsc_cnstr{1}.num_cnstr,qsc_cnstr{1}.xdim],qsc_cnstr{1}.iCfun,qsc_cnstr{1}.jCvar)),...
  qsc_cnstr{1}.num_cnstr,qsc_cnstr{1}.xdim),dc_cnstr);
valuecheck(qsc_cnstr{2}.lb,1);
valuecheck(qsc_cnstr{2}.lb,1);
valuecheck(qsc_cnstr{2}.A, ones(1,qsc2.num_pts));
valuecheck(qsc_cnstr{3}.lb,zeros(qsc2.num_pts,1));
valuecheck(qsc_cnstr{3}.ub,ones(qsc2.num_pts,1));
end
