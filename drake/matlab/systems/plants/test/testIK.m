function testIK

% Some rng seeds may cause the major iteration limit to be reached.
% See e.g. https://github.com/RobotLocomotion/drake/pull/1588 and https://github.com/RobotLocomotion/drake/pull/1383
% Setting rng to 1 here to circumvent this issue.
rng(1);

w = warning('off','Drake:RigidBody:SimplifiedCollisionGeometry');
warning('off','Drake:RigidBody:NonPositiveInertiaMatrix');
warning('off','Drake:RigidBodyManipulator:ReplacedCylinder');
warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints');
warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits');
urdf = [getDrakePath,'/examples/Atlas/urdf/atlas_minimal_contact.urdf'];
urdf_collision = [getDrakePath,'/examples/Atlas/urdf/atlas_convex_hull.urdf'];
options.floating = true;
robot = RigidBodyManipulator(urdf,options);
nq = robot.getNumPositions();

r_collision = RigidBodyManipulator(urdf_collision,options);
ignored_bodies = {'ltorso','mtorso','r_talus','l_talus'};
r_collision = addLinksToCollisionFilterGroup(r_collision,ignored_bodies,'no_collision',1);
r_collision = r_collision.compile();

l_foot = robot.findLinkId('l_foot');
r_foot = robot.findLinkId('r_foot');
l_hand = robot.findLinkId('l_hand');
r_hand = robot.findLinkId('r_hand');
head = robot.findLinkId('head');
world = robot.findLinkId('world');
l_foot_geometry = robot.getBody(l_foot).getCollisionGeometry;
r_foot_geometry = robot.getBody(r_foot).getCollisionGeometry;
l_foot_pts = [];
r_foot_pts = [];
for i=1:length(l_foot_geometry),
  l_foot_pts = [l_foot_pts robot.getBody(l_foot).getCollisionGeometry{i}.getPoints];
end
for i=1:length(r_foot_geometry),
  r_foot_pts = [r_foot_pts robot.getBody(r_foot).getCollisionGeometry{i}.getPoints];
end
n_l_foot_pts = size(l_foot_pts, 2);
n_r_foot_pts = size(r_foot_pts, 2);
l_hand_pts = [0;0;0];
r_hand_pts = [0;0;0];

coords = robot.getStateFrame.getCoordinateNames();
coords = coords(1:robot.getNumPositions);
l_leg_kny = find(strcmp(coords,'l_leg_kny'));
r_leg_kny = find(strcmp(coords,'r_leg_kny'));
l_leg_hpy = find(strcmp(coords,'l_leg_hpy'));
r_leg_hpy = find(strcmp(coords,'r_leg_hpy'));
l_leg_aky = find(strcmp(coords,'l_leg_aky'));
r_leg_aky = find(strcmp(coords,'r_leg_aky'));
l_leg_hpz = find(strcmp(coords,'l_leg_hpz'));
r_leg_hpz = find(strcmp(coords,'r_leg_hpz'));

tspan = [0,1];
nom_data = load('../../../../examples/Atlas/data/atlas_fp.mat');
q_nom = nom_data.xstar(1:nq);
q_seed = q_nom+1e-2*randn(nq,1);
ikoptions = IKoptions(robot);
ikoptions = ikoptions.setDebug(true);
ikoptions = ikoptions.setMex(false);
ikoptions = ikoptions.setMajorIterationsLimit(5000);
ikmexoptions = ikoptions;
ikmexoptions = ikmexoptions.setMex(true);

kc1 = WorldCoMConstraint(robot,[0;0;0.9],[0;0;1],tspan,1);
kc1prime = WorldCoMConstraint(robot,[nan;nan;0.9],[nan;nan;0.92],tspan/2,1);
display('Check a single CoM constraint')
q = test_IK_userfun(robot,q_seed,q_nom,kc1,ikoptions);
kinsol = doKinematics(robot,q);
com = getCOM(robot,kinsol);
valuecheck(com(1:2),[0;0],1e-5);
if(com(3)>1+1e-5 || com(3)<0.9-1e-5)
  error('CoM constraint is not satisfied')
end
display('Check IK pointwise with a single CoM constraint')
[q,info] = test_IKpointwise_userfun(robot,[0,1],[q_seed q_seed],[q_nom q_nom],kc1,ikoptions);
for i = 1:size(q,2)
  kinsol = doKinematics(robot,q(:,i));
  com = getCOM(robot,kinsol);
  valuecheck(com(1:2),[0;0],1e-5);
  if(com(3)>1+1e-5 ||com(3)<0.9-1e-5)
    error('CoM constraint is not satisfied');
  end
end
display('Check IK pointwise with multiple CoM constraints')
[q,info] = test_IKpointwise_userfun(robot,[0,1],[q_seed q_seed],[q_nom q_nom],kc1,kc1prime,ikoptions);
if(all(info == 1)&&~isempty(info))
kinsol = doKinematics(robot,q(:,1));
com = getCOM(robot,kinsol);
valuecheck(com(1:2),[0;0],1e-5);
if(com(3)>0.92+1e-5 ||com(3)<0.9-1e-5)
  error('CoM constraint is not satisfied');
end
kinsol = doKinematics(robot,q(:,2));
com = getCOM(robot,kinsol);
valuecheck(com(1:2),[0;0],1e-5);
if(com(3)>1+1e-5 ||com(3)<0.9-1e-5)
  error('CoM constraint is not satisfied');
end
end
pc_knee = PostureConstraint(robot,tspan);
l_knee_idx = find(strcmp(robot.getStateFrame.getCoordinateNames(),'l_leg_kny'));
r_knee_idx = find(strcmp(robot.getStateFrame.getCoordinateNames(),'r_leg_kny'));
pc_knee = pc_knee.setJointLimits([l_knee_idx;r_knee_idx],[0.2;0.2],[inf;inf]);
display('Check a single CoM constraint with a posture constraint')
q = test_IK_userfun(robot,q_seed,q_nom,kc1,pc_knee,ikoptions);
if(any(q([l_knee_idx;r_knee_idx])<0.2-1e-5))
  error('Posture constraint is not satisfied');
end
kinsol = doKinematics(robot,q);
com = getCOM(robot,kinsol);
valuecheck(com(1:2),[0;0],1e-5);
if(com(3)>1+1e-5 || com(3)<0.9-1e-5)
  error('CoM constraint is not satisfied')
end

display('Check IK pointwise with a single CoM constraint with a posture constraint')
[q,info] = test_IKpointwise_userfun(robot,[0,1],[q_seed q_seed+1e-3*randn(nq,1)],[q_nom q_nom],kc1,pc_knee,ikoptions);
if(all(info == 1)&&~isempty(info))
if(any(q([l_knee_idx;r_knee_idx],:)<0.2-1e-5))
  error('Posture constraint is not satisfied');
end
for i = 1:size(q,2)
  kinsol = doKinematics(robot,q(:,i));
  com = getCOM(robot,kinsol);
  valuecheck(com(1:2),[0;0],1e-5);
  if(com(3)>1+1e-5 ||com(3)<0.9-1e-5)
    error('CoM constraint is not satisfied');
  end
end
end

display('Check IK pointwise with multiple CoM constraints with a posture constraint')
[q,info] = test_IKpointwise_userfun(robot,[0,1],[q_seed q_seed+1e-3*randn(nq,1)],[q_nom q_nom],kc1,kc1prime,pc_knee,ikoptions);
if(all(info == 1)&&~isempty(info))
if(any(q([l_knee_idx;r_knee_idx],:)<0.2-1e-5))
  error('Posture constraint is not satisfied');
end
kinsol = doKinematics(robot,q(:,1));
com = getCOM(robot,kinsol);
valuecheck(com(1:2),[0;0],1e-5);
if(com(3)>0.92+1e-5 ||com(3)<0.9-1e-5)
  error('CoM constraint is not satisfied');
end
kinsol = doKinematics(robot,q(:,2));
com = getCOM(robot,kinsol);
valuecheck(com(1:2),[0;0],1e-5);
if(com(3)>1+1e-5 ||com(3)<0.9-1e-5)
  error('CoM constraint is not satisfied');
end
end


display('Check a body position constraint')
kc2l = WorldPositionConstraint(robot,l_foot,l_foot_pts,[nan(2,n_l_foot_pts);zeros(1,n_l_foot_pts)],[nan(2,n_l_foot_pts);zeros(1,n_l_foot_pts)],tspan);
kc2r = WorldPositionConstraint(robot,r_foot,r_foot_pts,[nan(2,n_r_foot_pts);zeros(1,n_r_foot_pts)],[nan(2,n_r_foot_pts);zeros(1,n_r_foot_pts)],tspan);
q = test_IK_userfun(robot,q_seed,q_nom,kc1,pc_knee,kc2l,kc2r,ikoptions);
kinsol = doKinematics(robot,q);
lfoot_pos = forwardKin(robot,kinsol,l_foot,l_foot_pts,0);
rfoot_pos = forwardKin(robot,kinsol,r_foot,r_foot_pts,0);
valuecheck(lfoot_pos(3,:),zeros(1,n_l_foot_pts),1e-5);
valuecheck(rfoot_pos(3,:),zeros(1,n_r_foot_pts),1e-5);

display('Check IK pointwise with body position constraint');
[q,info] = test_IKpointwise_userfun(robot,[0,1],[q_seed q_seed+1e-3*randn(nq,1)],[q_nom q_nom],kc1,pc_knee,kc2l,kc2r,ikoptions);
if(all(info == 1)&&~isempty(info))
for i = 1:size(q,2)
  kinsol = doKinematics(robot,q(:,i));
  lfoot_pos = forwardKin(robot,kinsol,l_foot,l_foot_pts,0);
  rfoot_pos = forwardKin(robot,kinsol,r_foot,r_foot_pts,0);
  valuecheck(lfoot_pos(3,:),zeros(1,n_l_foot_pts),1e-5);
  valuecheck(rfoot_pos(3,:),zeros(1,n_r_foot_pts),1e-5);
end
end

display('Check a body position (in random frame) constraint')
rpy = [pi/10,pi/20,pi/10];
xyz = [0.5;0.0;0.2];
T = [rpy2rotmat(rpy),xyz;zeros(1,3),1];
kc2l_frame = WorldPositionInFrameConstraint(robot,l_foot,l_foot_pts,T,[nan(2,n_l_foot_pts);zeros(1,n_l_foot_pts)],[nan(2,n_l_foot_pts);zeros(1,n_l_foot_pts)],tspan);
kc2r_frame = WorldPositionInFrameConstraint(robot,r_foot,r_foot_pts,T,[nan(2,n_r_foot_pts);zeros(1,n_r_foot_pts)],[nan(2,n_r_foot_pts);zeros(1,n_r_foot_pts)],tspan);
q = test_IK_userfun(robot,q_seed,q_nom,kc1,pc_knee,kc2l_frame,kc2r_frame,ikoptions);
kinsol = doKinematics(robot,q);
lfoot_pos = homogTransMult(invHT(T),forwardKin(robot,kinsol,l_foot,l_foot_pts,0));
rfoot_pos = homogTransMult(invHT(T),forwardKin(robot,kinsol,r_foot,r_foot_pts,0));
valuecheck(lfoot_pos(3,:),zeros(1,n_l_foot_pts),1e-5);
valuecheck(rfoot_pos(3,:),zeros(1,n_r_foot_pts),1e-5);

display('Check IK pointwise with body position (in random frame) constraint');
[q,info] = test_IKpointwise_userfun(robot,[0,1],[q_seed q_seed+1e-3*randn(nq,1)],[q_nom q_nom],kc1,pc_knee,kc2l_frame,kc2r_frame,ikoptions);
if(all(info == 1)&&~isempty(info))
for i = 1:size(q,2)
  kinsol = doKinematics(robot,q(:,i));
  lfoot_pos = homogTransMult(invHT(T),forwardKin(robot,kinsol,l_foot,l_foot_pts,0));
  rfoot_pos = homogTransMult(invHT(T),forwardKin(robot,kinsol,r_foot,r_foot_pts,0));
  valuecheck(lfoot_pos(3,:),zeros(1,n_l_foot_pts),1e-5);
  valuecheck(rfoot_pos(3,:),zeros(1,n_r_foot_pts),1e-5);
end
end

display('Check the infeasible case')
kc_err = WorldCoMConstraint(robot,[0;0;2],[0;0;inf],tspan);
if(checkDependency('snopt'))
% [q,info,infeasible_constraint] = inverseKin(robot,q_seed,q_nom,kc_err,kc2l,kc2r,ikoptions);
[qmex,info_mex,infeasible_constraint_mex] = inverseKin(robot,q_seed,q_nom,kc_err,kc2l,kc2r,ikmexoptions);
% valuecheck(info_mex,info);
if(info_mex ~= 13)
  error('This should be infeasible');
end
display('The infeasible constraints are');
disp(infeasible_constraint_mex);
end
ikproblem = InverseKinematics(robot,q_nom,kc_err,kc2l,kc2r);
[qik,F,info,infeasible_constraint_ik] = ikproblem.solve(q_seed);
if(checkDependency('snopt'))
[qmex,info_mex,infeasible_constraint_mex] = inverseKinPointwise(robot,[0,1],[q_seed q_seed+1e-3*randn(nq,1)],[q_nom q_nom],kc_err,kc2l,kc2r,ikmexoptions);
display('The infeasible constraints are');
disp(infeasible_constraint_mex);
end

display('Check a body quaternion constraint')
kc3 = WorldQuatConstraint(robot,r_foot,[1;0;0;0],0,tspan);
q = test_IK_userfun(robot,q_seed,q_nom,kc1,pc_knee,kc2l,kc2r,kc3,ikoptions);
kinsol = doKinematics(robot,q);
rfoot_pos = forwardKin(robot,kinsol,r_foot,[0;0;0],2);
valuecheck(rfoot_pos(4:7)'*[1;0;0;0],1,1e-5);
display('Check IK pointwise with body orientation constraint');
[q,info] = test_IKpointwise_userfun(robot,[0,1],[q_seed q_seed],[q_nom q_nom],kc1,pc_knee,kc2l,kc2r,kc3,ikoptions);
if(all(info == 1)&&~isempty(info))
for i = 1:size(q,2)
  kinsol = doKinematics(robot,q(:,i));
  rfoot_pos = forwardKin(robot,kinsol,r_foot,[0;0;0],2);
  valuecheck(rfoot_pos(4:7)'*[1;0;0;0],1,1e-5);
end
end

display('Check a gaze orientation constraint')
kc4 = WorldGazeOrientConstraint(robot,r_hand,[1;0;0],[0.5;0.5;-0.5;0.5],0.1*pi,0.8*pi,tspan);
q = test_IK_userfun(robot,q_seed,q_nom,kc1,pc_knee,kc2l,kc2r,kc3,kc4,ikoptions);
kinsol = doKinematics(robot,q);
rhand_pos = forwardKin(robot,kinsol,r_hand,[0 1;0 0;0 0],0);
rhand_gaze_vec = rhand_pos(:,2)-rhand_pos(:,1);
rhand_gaze_des = quat2rotmat([0.5;0.5;-0.5;0.5])*[1;0;0];
if(acos(rhand_gaze_vec'*rhand_gaze_des)>0.1*pi+1e-4)
  error('Gaze conethreshold does not satisfy');
end
display('Check IK pointwise with gaze orientation');
[q,info] = test_IKpointwise_userfun(robot,[0,1],[q_seed q_seed+1e-3*randn(nq,1)],[q_nom q_nom],kc1,pc_knee,kc2l,kc2r,kc3,kc4,ikoptions);
if(all(info == 1)&&~isempty(info))
for i = 1:size(q,2)
  kinsol = doKinematics(robot,q(:,i));
  rhand_pos = forwardKin(robot,kinsol,r_hand,[0 1;0 0;0 0],0);
  rhand_gaze_vec = rhand_pos(:,2)-rhand_pos(:,1);
  rhand_gaze_des = quat2rotmat([0.5;0.5;-0.5;0.5])*[1;0;0];
  if(acos(rhand_gaze_vec'*rhand_gaze_des)>0.1*pi+1e-4)
    error('Gaze conethreshold does not satisfy');
  end
end
end

display('Check a gaze direction constraint')
kc5 = WorldGazeDirConstraint(robot,l_hand,[1;0;0],[1;0;0],0.4*pi,tspan);
q = test_IK_userfun(robot,q_seed,q_nom,kc1,pc_knee,kc2l,kc2r,kc3,kc4,kc5,ikoptions);
kinsol = doKinematics(robot,q);
lhand_pos = forwardKin(robot,kinsol,l_hand,[0 1;0 0;0 0],0);
lhand_gaze_vec = lhand_pos(:,2)-lhand_pos(:,1);
if(acos(lhand_gaze_vec'*[1;0;0])>0.4*pi+1e-4)
  error('Gaze conethreshold does not satisfy');
end
display('Check IK pointwise with gaze direction Constraint');
[q,info] = test_IKpointwise_userfun(robot,[0,1],[q_seed q_seed+1e-3*randn(nq,1)],[q_nom q_nom],kc1,pc_knee,kc2l,kc2r,kc3,kc4,kc5,ikoptions);
if(all(info == 1)&&~isempty(info))
for i = 1:size(q,2)
  kinsol = doKinematics(robot,q(:,i));
  lhand_pos = forwardKin(robot,kinsol,l_hand,[0 1;0 0;0 0],0);
  lhand_gaze_vec = lhand_pos(:,2)-lhand_pos(:,1);
  if(acos(lhand_gaze_vec'*[1;0;0])>0.4*pi+1e-4)
    error('Gaze conethreshold does not satisfy');
  end
end
end

display('Check a gaze target constraint')
gaze_target = [1;0;1.5];
gaze_axis = [1;0;0];
gaze_origin = [0.1;0.2;0.3];
kc6 = WorldGazeTargetConstraint(robot,head,gaze_axis,gaze_target,gaze_origin,0.1*pi,tspan);
q = test_IK_userfun(robot,q_seed,q_nom,kc1,kc2l,kc2r,kc3,kc4,kc5,kc6,ikoptions);
kinsol = doKinematics(robot,q);
head_pos = forwardKin(robot,kinsol,head,[gaze_origin gaze_origin+gaze_axis],0);
head_gaze_vec = head_pos(:,2)-head_pos(:,1);
head_gaze_des = gaze_target-head_pos(:,1);
head_gaze_des = head_gaze_des/norm(head_gaze_des);
if(acos(head_gaze_vec'*head_gaze_des)>0.1*pi+1e-4)
  error('Does not satisfy conethreshold constraint');
end
display('Check IK pointwise with gaze target constraint');
[q,info] = test_IKpointwise_userfun(robot,[0,1],[q_seed q_seed+1e-3*randn(nq,1)],[q_nom q_nom],kc1,pc_knee,kc2l,kc2r,kc3,kc4,kc5,kc6,ikoptions);
if(all(info == 1)&&~isempty(info))
for i = 1:size(q,2)
  kinsol = doKinematics(robot,q(:,i));
  head_pos = forwardKin(robot,kinsol,head,[gaze_origin gaze_origin+gaze_axis],0);
  head_gaze_vec = head_pos(:,2)-head_pos(:,1);
  head_gaze_des = gaze_target-head_pos(:,1);
  head_gaze_des = head_gaze_des/norm(head_gaze_des);
  if(acos(head_gaze_vec'*head_gaze_des)>0.1*pi+1e-4)
    error('Does not satisfy conethreshold constraint');
  end
end
end

if(checkDependency('bullet'))
  display('Check all-to-all closest distance constraint')
  abcdc = AllBodiesClosestDistanceConstraint(robot,0.05,1e3,tspan);
  q = test_IK_userfun(robot,q_seed,q_nom,kc1,kc2l,kc2r,kc3,kc4,kc5,kc6,abcdc,ikoptions);
  display('Check IK pointwise with all-to-all closest distance constraint')
  q = test_IKpointwise_userfun(robot,[0,1],[q_seed,q_seed+1e-3*randn(nq,1)],[q_nom,q_nom],kc1,pc_knee,kc2l,kc2r,kc3,kc4,kc5,kc6,abcdc,ikoptions);

  display('Check minimum-distance constraint')
  min_dist_cnstr = MinDistanceConstraint(robot,0.05,[],tspan);
  q = test_IK_userfun(robot,q_seed,q_nom,kc1,kc2l,kc2r,kc3,kc4,kc5,kc6,min_dist_cnstr,ikoptions);
  display('Check IK pointwise with minimum-distance constraint')
  q = test_IKpointwise_userfun(robot,[0,1],[q_seed,q_seed+1e-3*randn(nq,1)],[q_nom,q_nom],kc1,pc_knee,kc2l,kc2r,kc3,kc4,kc5,kc6,min_dist_cnstr,ikoptions);
end
display('Check quasi static constraint')
qsc = QuasiStaticConstraint(robot);
qsc = qsc.addContact(r_foot,r_foot_pts);
qsc = qsc.addContact(l_foot,l_foot_pts);
qsc = qsc.setActive(true);
ikoptions = ikoptions.setMex(false);
q = test_IK_userfun(robot,q_seed,q_nom,kc1,qsc,kc2l,kc2r,kc3,kc4,kc5,kc6,ikoptions);
kinsol = doKinematics(robot,q);
valuecheck(qsc.checkConstraint(kinsol),true);

% check addCost in InverseKinematics, minimizing the largest QuasiStaticWeight
ikproblem = InverseKinematics(robot,q_nom,kc1,qsc,kc2l,kc2r,kc3,kc4,kc5,kc6);
ikproblem = ikproblem.setQ(ikoptions.Q);
ikproblem = ikproblem.addDecisionVariable(1,{'max_weight'});
ikproblem = ikproblem.addLinearConstraint(LinearConstraint(zeros(qsc.num_pts,1),inf(qsc.num_pts,1),[ones(qsc.num_pts,1) -eye(qsc.num_pts)]),...
  [ikproblem.num_vars;ikproblem.qsc_weight_idx]);
ikproblem = ikproblem.addCost(LinearConstraint(-inf,inf,1),ikproblem.num_vars);
[q,F,info,infeasible_constraint] = ikproblem.solve(q_seed);

display('Check quasi static constraint for pointwise');
[q,info] = test_IKpointwise_userfun(robot,[0,1],[q_seed q_seed+1e-3*randn(nq,1)],[q_nom q_nom],kc1,qsc,kc2l,kc2r,kc3,kc4,kc5,kc6,ikoptions);
if(all(info == 1)&&~isempty(info))
for i = 1:size(q,2)
  kinsol = doKinematics(robot,q(:,i));
  valuecheck(qsc.checkConstraint(kinsol),true);
end
end

display('Check SingleTimeLinearPostureConstraint');
iAfun = [1;1;2;2;3;3;4;4];
jAvar = [l_leg_kny;r_leg_kny;l_leg_hpy;r_leg_hpy;l_leg_aky;r_leg_aky;l_leg_hpz;r_leg_hpz];
A = [1;-1;1;-1;1;-1;1;-1];
lb = [0;0;0;-0.1*pi];
ub = [0;0;0;0.1*pi];
stlpc = SingleTimeLinearPostureConstraint(robot,iAfun,jAvar,A,lb,ub,tspan);
q = test_IK_userfun(robot,q_seed,q_nom,kc1,kc2l,kc2r,pc_knee,stlpc,qsc,ikoptions);
valuecheck(q([l_leg_kny;l_leg_hpy;l_leg_aky]),q([r_leg_kny;r_leg_hpy;r_leg_aky]),1e-10);
if(abs(q(l_leg_hpz)-q(r_leg_hpz))>0.1*pi+1e-8)
  error('SingleTimeLinearPostureConstraint is not satisfied');
end
display('Check SingleTimeLinearPostureConstraint for pointwise');
[q ,info]= test_IKpointwise_userfun(robot,[0,1],[q_seed q_seed+1e-3*randn(nq,1)],[q_nom q_nom],kc1,qsc,kc2l,kc2r,stlpc,ikoptions);
if(all(info == 1)&&~isempty(info))
valuecheck(q([l_leg_kny;l_leg_hpy;l_leg_aky],:),q([r_leg_kny;r_leg_hpy;r_leg_aky],:),1e-10);
if(any(abs(q(l_leg_hpz,:)-q(r_leg_hpz,:))>0.1*pi+1e-8))
  error('SingleTimeLinearPostureConstraint is not satisfied');
end
end

display('Check point to point distance constraint')
hands_distance_cnst = Point2PointDistanceConstraint(robot,l_hand,r_hand,[0;0;0],[0;0;0],0.5,1,tspan);
q = test_IK_userfun(robot,q_seed,q_nom,kc1,qsc,kc2l,kc2r,kc3,hands_distance_cnst,ikoptions);
kinsol = doKinematics(robot,q);
lhand_pos = forwardKin(robot,kinsol,l_hand,[0;0;0],0);
rhand_pos = forwardKin(robot,kinsol,r_hand,[0;0;0],0);
hand_dist = norm(lhand_pos-rhand_pos);
if(hand_dist>1+1e-5 || hand_dist<0.5-1e-5)
  error('point to point distance constraint is not satisfied');
end
[q,info] = test_IKpointwise_userfun(robot,[0 1],[q_seed q_seed+1e-3*randn(nq,1)],[q_nom q_nom],kc1,qsc,kc2l,kc2r,kc3,hands_distance_cnst,ikoptions);
if(all(info == 1)&&~isempty(info))
for i = 1:size(q,2)
  kinsol = doKinematics(robot,q(:,i));
  lhand_pos = forwardKin(robot,kinsol,l_hand,[0;0;0],0);
  rhand_pos = forwardKin(robot,kinsol,r_hand,[0;0;0],0);
  hand_dist = norm(lhand_pos-rhand_pos);
  if(hand_dist>1+1e-5 || hand_dist<0.5-1e-5)
    error('point to point distance constraint is not satisfied');
  end
end
end

head_pts = [[0;0;0] [0.1;0;0]];
world_pts = [[0;0;0] [0.1;0;0]];
head_world_dist_cnst = Point2PointDistanceConstraint(robot,head,world,head_pts,world_pts,[1.6 1.6],[1.9 1.9],tspan);
q = test_IK_userfun(robot,q_seed,q_nom,kc1,qsc,kc2l,kc2r,kc3,head_world_dist_cnst,ikoptions);
kinsol = doKinematics(robot,q);
head_pos = forwardKin(robot,kinsol,head,head_pts,0);
head_world_dist = head_pos-world_pts;
head_world_dist = sqrt(sum(head_world_dist.*head_world_dist,1));
if(any(head_world_dist>1.9+1e-5) || any(head_world_dist<1.6-1e-5))
  error('point to point distance constraint is not satisfied');
end

display('Check point to line segment distance constraint')
vertical_line = [[0;0;0] [0;0;1]];
hand_line_dist_cnst = Point2LineSegDistConstraint(robot,l_hand,[0;0;0],1,vertical_line,0.1,0.2,tspan);
q = test_IK_userfun(robot,q_seed,q_nom,qsc,kc2l,kc2r,hand_line_dist_cnst,ikoptions);
kinsol = doKinematics(robot,q);
l_hand_pos = forwardKin(robot,kinsol,l_hand,[0;0;0],0);
if(l_hand_pos(3)>1+1e-5 || l_hand_pos(3)<0-1e-5 || norm(l_hand_pos(1:2))>0.2+1e-5 || norm(l_hand_pos(1:2))<0.1-1e-5)
  error('point to line segment constraint is not satisfied')
end

display('Check with addRobotFromURDF');
xyz = 0.1*randn(3,1)+[1;1;1];
rpy = 0.1*pi*randn(3,1)+[pi/2;0;0];
robot = robot.addRobotFromURDF('valve_task_wall.urdf',xyz,rpy,struct('floating',false));
kc1 = kc1.updateRobot(robot);
qsc = qsc.updateRobot(robot);
kc2l = kc2l.updateRobot(robot);
kc2r = kc2r.updateRobot(robot);
kc3 = kc3.updateRobot(robot);
kc4 = kc4.updateRobot(robot);
kc5 = kc5.updateRobot(robot);
kc6 = kc6.updateRobot(robot);
ikoptions = ikoptions.updateRobot(robot);
ikoptions = ikoptions.setMajorIterationsLimit(2000);
ikmexoptions = ikmexoptions.updateRobot(robot);
nq_aff = length(robot.getStateFrame.frame{2}.getCoordinateNames())/2;
q_seed_aff = zeros(nq_aff,1);
q_nom_aff = zeros(nq_aff,1);
q = test_IK_userfun(robot,[q_seed;q_seed_aff],[q_nom;q_nom_aff],kc1,qsc,kc2l,kc2r,kc3,kc4,kc5,kc6,ikoptions);
kinsol = doKinematics(robot,q);
com = robot.getCOM(kinsol,1);
if(com(3)>1+1e-5 || com(3)<0.9-1e-5)
  error('CoM constraint not satisfied');
end
rfoot_pos = forwardKin(robot,kinsol,r_foot,r_foot_pts,0);
lfoot_pos = forwardKin(robot,kinsol,l_foot,l_foot_pts,0);
valuecheck(rfoot_pos(3,:),zeros(1,n_r_foot_pts),1e-5);
valuecheck(lfoot_pos(3,:),zeros(1,n_l_foot_pts),1e-5);
shrinkFactor = qsc.shrinkFactor+1e-4;
center_pos = mean([lfoot_pos(1:2,:) rfoot_pos(1:2,:)],2);
shrink_vertices = [lfoot_pos(1:2,:) rfoot_pos(1:2,:)]*shrinkFactor+repmat(center_pos*(1-shrinkFactor),1,size(l_foot_pts,2)+size(r_foot_pts,2));
num_vertices = size(shrink_vertices,2);
quadoptions = optimset('Algorithm','interior-point-convex');
[~,~,exit_flag] = quadprog(eye(num_vertices),rand(num_vertices,1),[],[],[shrink_vertices;ones(1,num_vertices)],...
  [com(1:2);1],zeros(num_vertices,1),ones(num_vertices,1),1/num_vertices*ones(num_vertices,1),quadoptions);
valuecheck(exit_flag,1);
q = test_IKpointwise_userfun(robot,[0,1],[q_seed q_seed+1e-3*randn(nq,1);q_seed_aff q_seed_aff+1e-3*randn(nq_aff,1)],...
  [q_nom q_nom;q_nom_aff q_nom_aff],kc1,qsc,kc2l,kc2r,kc3,kc4,kc5,kc6,ikoptions);
% display('Check sequentialSeedFlag')
% ikoptions = ikoptions.setSequentialSeedFlag(true);
% q = test_IK_userfun(r,q_seed,q_nom,kc1,qsc,kc2l,kc2r,kc3,kc4,kc5,kc6,ikoptions);
% display('Check sequentialSeedFlag for pointwise');
% q = test_IKpointwise_userfun(r,[0,1],[q_seed q_seed],[q_nom q_nom],kc1,qsc,kc2l,kc2r,kc3,kc4,kc5,kc6,ikoptions);

warning(w);
end

function qik = test_IK_userfun(r,q_seed,q_nom,varargin)
ikoptions = varargin{end};
ikoptions = ikoptions.setMex(false);
ikmexoptions = ikoptions;
ikmexoptions = ikmexoptions.setMex(true);
% [f,G,iGfun,jGvar,Fupp,Flow,xupp,xlow,iAfun,jAvar,A,nF] = inverseKin(r,q_seed,q_nom,varargin{1:end-1},ikmexoptions);
% keyboard;
% tic
% [q,info] = inverseKin(r,q_seed,q_nom,varargin{1:end-1},ikoptions);
% toc
% if(info>10)
%   error('SNOPT info is %d, IK fails to solve the problem',info);
% end
% testConstraint(r,[],q,varargin{1:end-1});
tic
ikproblem = InverseKinematics(r,q_nom,varargin{1:end-1});
ikproblem = ikproblem.setQ(ikoptions.Q);
ikproblem = ikproblem.setSolverOptions('fmincon','Algorithm','sqp');
ikproblem = ikproblem.setSolverOptions('fmincon','MaxIter',500);
ikproblem = ikproblem.setSolverOptions('snopt','MajorIterationsLimit',ikoptions.SNOPT_MajorIterationsLimit);
[qik,F,info,infeasible_cnstr_ik] = ikproblem.solve(q_seed);
toc
% valuecheck(qik,q,1e-6);
testConstraint(r,[],qik,varargin{1:end-1});
if(checkDependency('snopt'))
  tic
  [qmex,info_mex] = inverseKin(r,q_seed,q_nom,varargin{1:end-1},ikmexoptions);
  toc
  if(info_mex>10)
    error('SNOPT info is %d, IK mex fails to solve the problem',info_mex);
  end
  % valuecheck(q,qmex,5e-2);
	testConstraint(r,[],qmex,varargin{1:end-1});
end
end

function [qmex,info] = test_IKpointwise_userfun(r,t,q_seed,q_nom,varargin)
ikoptions = varargin{end};
ikoptions = ikoptions.setMex(false);
ikmexoptions = ikoptions;
ikmexoptions = ikmexoptions.setMex(true);
%tic
%[q,info] = inverseKinPointwise(r,t,q_seed,q_nom,varargin{1:end-1},ikoptions);
%toc
%if(info>10)
  %error('SNOPT info is %d, IK pointwisefails to solve the problem',info);
%end
%for i = 1:length(t)
  %testConstraint(r,t(i),q(:,i),varargin{1:end-1});
%end
if(checkDependency('snopt'))
  tic
  [qmex,info] = inverseKinPointwise(r,t,q_seed,q_nom,varargin{1:end-1},ikmexoptions);
  toc
  if(any(info>10))
    error('SNOPT info is %d, IK pointwise mex fails to solve the problem',info);
  end
  for i = 1:length(t)
    testConstraint(r,t(i),qmex(:,i),varargin{1:end-1});
  end
  % valuecheck(q,qmex,5e-2);
else
  qmex = [];
  info = [];
end
end

function testConstraint(robot,t,q_sol,varargin)
kinsol = robot.doKinematics(q_sol,false,true);
[q_lb,q_ub] = robot.getJointLimits();
if(any(q_sol>q_ub) || any(q_sol<q_lb))
  error('solution must be within the robot default joint limits');
end
for i = 1:nargin-3
  if(isa(varargin{i},'SingleTimeKinematicConstraint'))
    if(varargin{i}.isTimeValid(t))
      [lb,ub] = varargin{i}.bounds(t);
      c = varargin{i}.eval(t,kinsol);
      if(any(c-ub>1e-3) || any(c-lb<-1e-3))
        error('solution does not satisfy the SingleTimeKinematicConstraint');
      end
    end
  elseif(isa(varargin{i},'PostureConstraint'))
    if(varargin{i}.isTimeValid(t))
      [lb,ub] = varargin{i}.bounds(t);
      if(any(q_sol>ub) || any(q_sol<lb))
        error('solution does not satisfy the PostureConstraint');
      end
    end
  elseif(isa(varargin{i},'QuasiStaticConstraint'))
    if(varargin{i}.isTimeValid(t) && varargin{i}.active)
      if(~varargin{i}.checkConstraint(kinsol))
        error('solution does not satisfy the QuasiStaticConstraint');
      end
    end
  elseif(isa(varargin{i},'SingleTimeLinearPostureConstraint'))
    if(varargin{i}.isTimeValid(t))
      [lb,ub] = varargin{i}.bounds(t);
      c = varargin{i}.feval(t,q_sol);
      if(any(c-ub>1e-3) || any(c-lb < -1e-3))
        error('solution does not satisfy the SingleTimeLinearPostureConstraint');
      end
    end
  else
    error('The constraint is not supported');
  end
end
end

% TIMEOUT 1500
