function testIK
urdf = [getDrakePath,'/examples/Atlas/urdf/atlas_minimal_contact.urdf'];
urdf_collision = [getDrakePath,'/systems/plants/constraint/test/model_simple_visuals.urdf'];
options.floating = true;
robot = RigidBodyManipulator(urdf,options);
nq = robot.getNumDOF();

r_collision = RigidBodyManipulator(urdf_collision,options);
ignored_bodies = {'ltorso','mtorso','r_talus','l_talus'};
r_collision = addLinksToCollisionFilterGroup(r_collision,ignored_bodies,'no_collision',1);
r_collision = r_collision.compile();

l_foot = robot.findLinkInd('l_foot');
r_foot = robot.findLinkInd('r_foot');
l_hand = robot.findLinkInd('l_hand');
r_hand = robot.findLinkInd('r_hand');
head = robot.findLinkInd('head');
l_foot_pts = robot.getBody(l_foot).contact_pts;
r_foot_pts = robot.getBody(r_foot).contact_pts;
l_hand_pts = [0;0;0];
r_hand_pts = [0;0;0];
tspan = [0,1];
nom_data = load('../../../examples/Atlas/data/atlas_fp.mat');
q_nom = nom_data.xstar(1:nq);
q_seed = q_nom+0.1*randn(nq,1);
ikoptions = IKoptions(robot);
ikoptions = ikoptions.setDebug(true);
ikoptions = ikoptions.setMex(false);
ikmexoptions = ikoptions;
ikmexoptions = ikmexoptions.setMex(true);

kc1 = WorldCoMConstraint(robot,1,[0;0;0.9],[0;0;1],tspan);
kc1prime = WorldCoMConstraint(robot,1,[nan;nan;0.9],[nan;nan;0.92],tspan/2);
display('Check a single CoM constraint')
q = test_IK_userfun(robot,q_seed,q_nom,kc1,ikoptions);
kinsol = doKinematics(robot,q);
com = getCOM(robot,kinsol);
valuecheck(com(1:2),[0;0],1e-5);
if(com(3)>1+1e-5 || com(3)<0.9-1e-5)
  error('CoM constraint is not satisfied')
end
display('Check IK pointwise with a single CoM constraint')
q = test_IKpointwise_userfun(robot,[0,1],[q_seed q_seed],[q_nom q_nom],kc1,ikoptions);
for i = 1:size(q,2)
  kinsol = doKinematics(robot,q(:,i));
  com = getCOM(robot,kinsol);
  valuecheck(com(1:2),[0;0],1e-5);
  if(com(3)>1+1e-5 ||com(3)<0.9-1e-5)
    error('CoM constraint is not satisfied');
  end
end
display('Check IK pointwise with multiple CoM constraints')
q = test_IKpointwise_userfun(robot,[0,1],[q_seed q_seed],[q_nom q_nom],kc1,kc1prime,ikoptions);
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

pc_knee = PostureConstraint(robot,tspan);
l_knee_idx = find(strcmp(robot.getStateFrame.coordinates,'l_leg_kny'));
r_knee_idx = find(strcmp(robot.getStateFrame.coordinates,'r_leg_kny'));
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
q = test_IKpointwise_userfun(robot,[0,1],[q_seed q_seed+1e-3*randn(nq,1)],[q_nom q_nom],kc1,pc_knee,ikoptions);
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

display('Check IK pointwise with multiple CoM constraints with a posture constraint')
q = test_IKpointwise_userfun(robot,[0,1],[q_seed q_seed+1e-3*randn(nq,1)],[q_nom q_nom],kc1,kc1prime,pc_knee,ikoptions);
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

display('Check a body position constraint')
kc2l = WorldPositionConstraint(robot,l_foot,l_foot_pts,[nan(2,4);zeros(1,4)],[nan(2,4);zeros(1,4)],tspan);
kc2r = WorldPositionConstraint(robot,r_foot,r_foot_pts,[nan(2,4);zeros(1,4)],[nan(2,4);zeros(1,4)],tspan);
q = test_IK_userfun(robot,q_seed,q_nom,kc1,pc_knee,kc2l,kc2r,ikoptions);
kinsol = doKinematics(robot,q);
lfoot_pos = forwardKin(robot,kinsol,l_foot,l_foot_pts,0);
rfoot_pos = forwardKin(robot,kinsol,r_foot,r_foot_pts,0);
valuecheck(lfoot_pos(3,:),[0 0 0 0],1e-5);
valuecheck(rfoot_pos(3,:),[0 0 0 0],1e-5);

display('Check IK pointwise with body position constraint');
q = test_IKpointwise_userfun(robot,[0,1],[q_seed q_seed+1e-3*randn(nq,1)],[q_nom q_nom],kc1,pc_knee,kc2l,kc2r,ikoptions);
for i = 1:size(q,2)
  kinsol = doKinematics(robot,q(:,i));
  lfoot_pos = forwardKin(robot,kinsol,l_foot,l_foot_pts,0);
  rfoot_pos = forwardKin(robot,kinsol,r_foot,r_foot_pts,0);
  valuecheck(lfoot_pos(3,:),[0 0 0 0],1e-5);
  valuecheck(rfoot_pos(3,:),[0 0 0 0],1e-5);
end

display('Check a body position (in random frame) constraint')
rpy = [pi/10,pi/20,pi/10];
xyz = [0.5;0.0;0.2];
T = [rpy2rotmat(rpy),xyz;zeros(1,3),1];
kc2l_frame = WorldPositionInFrameConstraint(robot,l_foot,l_foot_pts,T,[nan(2,4);zeros(1,4)],[nan(2,4);zeros(1,4)],tspan);
kc2r_frame = WorldPositionInFrameConstraint(robot,r_foot,r_foot_pts,T,[nan(2,4);zeros(1,4)],[nan(2,4);zeros(1,4)],tspan);
q = test_IK_userfun(robot,q_seed,q_nom,kc1,pc_knee,kc2l_frame,kc2r_frame,ikoptions);
kinsol = doKinematics(robot,q);
lfoot_pos = homogTransMult(invHT(T),forwardKin(robot,kinsol,l_foot,l_foot_pts,0));
rfoot_pos = homogTransMult(invHT(T),forwardKin(robot,kinsol,r_foot,r_foot_pts,0));
valuecheck(lfoot_pos(3,:),[0 0 0 0],1e-5);
valuecheck(rfoot_pos(3,:),[0 0 0 0],1e-5);

display('Check IK pointwise with body position (in random frame) constraint');
q = test_IKpointwise_userfun(robot,[0,1],[q_seed q_seed+1e-3*randn(nq,1)],[q_nom q_nom],kc1,pc_knee,kc2l_frame,kc2r_frame,ikoptions);
for i = 1:size(q,2)
  kinsol = doKinematics(robot,q(:,i));
  lfoot_pos = homogTransMult(invHT(T),forwardKin(robot,kinsol,l_foot,l_foot_pts,0));
  rfoot_pos = homogTransMult(invHT(T),forwardKin(robot,kinsol,r_foot,r_foot_pts,0));
  valuecheck(lfoot_pos(3,:),[0 0 0 0],1e-5);
  valuecheck(rfoot_pos(3,:),[0 0 0 0],1e-5);
end


display('Check the infeasible case')
kc_err = WorldCoMConstraint(robot,1,[0;0;2],[0;0;inf],tspan);
[q,info,infeasible_constraint] = inverseKin(robot,q_seed,q_nom,kc_err,kc2l,kc2r,ikoptions);
[qmex,info_mex,infeasible_constraint_mex] = inverseKin(robot,q_seed,q_nom,kc_err,kc2l,kc2r,ikmexoptions);
valuecheck(info_mex,info);
if(info_mex ~= 13)
  error('This should be infeasible');
end
display('The infeasible constraints are');
disp(infeasible_constraint_mex);
[qmex,info_mex,infeasible_constraint_mex] = inverseKinPointwise(robot,[0,1],[q_seed q_seed+1e-3*randn(nq,1)],[q_nom q_nom],kc_err,kc2l,kc2r,ikmexoptions);
display('The infeasible constraints are');
disp(infeasible_constraint_mex);

display('Check a body quaternion constraint')
kc3 = WorldQuatConstraint(robot,r_foot,[1;0;0;0],0,tspan);
q = test_IK_userfun(robot,q_seed,q_nom,kc1,pc_knee,kc2l,kc2r,kc3,ikoptions);
kinsol = doKinematics(robot,q);
rfoot_pos = forwardKin(robot,kinsol,r_foot,[0;0;0],2);
valuecheck(rfoot_pos(4:7)'*[1;0;0;0],1,1e-5);
display('Check IK pointwise with body orientation constraint');
q = test_IKpointwise_userfun(robot,[0,1],[q_seed q_seed],[q_nom q_nom],kc1,pc_knee,kc2l,kc2r,kc3,ikoptions);
for i = 1:size(q,2)
  kinsol = doKinematics(robot,q(:,i));
  rfoot_pos = forwardKin(robot,kinsol,r_foot,[0;0;0],2);
  valuecheck(rfoot_pos(4:7)'*[1;0;0;0],1,1e-5);
end

display('Check a gaze orientation constraint')
kc4 = WorldGazeOrientConstraint(robot,r_hand,[1;0;0],[0.5;0.5;-0.5;0.5],0.1*pi,0.8*pi,tspan);
q = test_IK_userfun(robot,q_seed,q_nom,kc1,pc_knee,kc2l,kc2r,kc3,kc4,ikoptions);
kinsol = doKinematics(robot,q);
rhand_pos = forwardKin(robot,kinsol,r_hand,[0 1;0 0;0 0],0);
rhand_gaze_vec = rhand_pos(:,2)-rhand_pos(:,1);
rhand_gaze_des = quat2rotmat([0.5;0.5;-0.5;0.5])*[1;0;0];
if(acos(rhand_gaze_vec'*rhand_gaze_des)>0.1*pi+1e-5)
  error('Gaze conethreshold does not satisfy');
end
display('Check IK pointwise with gaze orientation');
q = test_IKpointwise_userfun(robot,[0,1],[q_seed q_seed+1e-3*randn(nq,1)],[q_nom q_nom],kc1,pc_knee,kc2l,kc2r,kc3,kc4,ikoptions);
for i = 1:size(q,2)
  kinsol = doKinematics(robot,q(:,i));
  rhand_pos = forwardKin(robot,kinsol,r_hand,[0 1;0 0;0 0],0);
  rhand_gaze_vec = rhand_pos(:,2)-rhand_pos(:,1);
  rhand_gaze_des = quat2rotmat([0.5;0.5;-0.5;0.5])*[1;0;0];
  if(acos(rhand_gaze_vec'*rhand_gaze_des)>0.1*pi+1e-5)
    error('Gaze conethreshold does not satisfy');
  end
end

display('Check a gaze direction constraint')
kc5 = WorldGazeDirConstraint(robot,l_hand,[1;0;0],[1;0;0],0.4*pi,tspan);
q = test_IK_userfun(robot,q_seed,q_nom,kc1,pc_knee,kc2l,kc2r,kc3,kc4,kc5,ikoptions);
kinsol = doKinematics(robot,q);
lhand_pos = forwardKin(robot,kinsol,l_hand,[0 1;0 0;0 0],0);
lhand_gaze_vec = lhand_pos(:,2)-lhand_pos(:,1);
if(acos(lhand_gaze_vec'*[1;0;0])>0.4*pi+1e-5)
  error('Gaze conethreshold does not satisfy');
end
display('Check IK pointwise with gaze direction Constraint');
q = test_IKpointwise_userfun(robot,[0,1],[q_seed q_seed+1e-3*randn(nq,1)],[q_nom q_nom],kc1,pc_knee,kc2l,kc2r,kc3,kc4,kc5,ikoptions);
for i = 1:size(q,2)
  kinsol = doKinematics(robot,q(:,i));
  lhand_pos = forwardKin(robot,kinsol,l_hand,[0 1;0 0;0 0],0);
  lhand_gaze_vec = lhand_pos(:,2)-lhand_pos(:,1);
  if(acos(lhand_gaze_vec'*[1;0;0])>0.4*pi+1e-5)
    error('Gaze conethreshold does not satisfy');
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
if(acos(head_gaze_vec'*head_gaze_des)>0.1*pi+1e-5)
  error('Does not satisfy conethreshold constraint');
end
display('Check IK pointwise with gaze target constraint');
q = test_IKpointwise_userfun(robot,[0,1],[q_seed q_seed+1e-3*randn(nq,1)],[q_nom q_nom],kc1,pc_knee,kc2l,kc2r,kc3,kc4,kc5,kc6,ikoptions);
for i = 1:size(q,2)
  kinsol = doKinematics(robot,q(:,i));
  head_pos = forwardKin(robot,kinsol,head,[gaze_origin gaze_origin+gaze_axis],0);
  head_gaze_vec = head_pos(:,2)-head_pos(:,1);
  head_gaze_des = gaze_target-head_pos(:,1);
  head_gaze_des = head_gaze_des/norm(head_gaze_des);
  if(acos(head_gaze_vec'*head_gaze_des)>0.1*pi+1e-5)
    error('Does not satisfy conethreshold constraint');
  end
end

display('Check all-to-all closest distance constraint')
abcdc = AllBodiesClosestDistanceConstraint(robot,0.05,1e3,tspan);
q = test_IK_userfun(robot,q_seed,q_nom,kc1,kc2l,kc2r,kc3,kc4,kc5,kc6,abcdc,ikoptions);
display('Check IK pointwise with all-to-all closest distance constraint')
q = test_IKpointwise_userfun(robot,[0,1],[q_seed,q_seed+1e-3*randn(nq,1)],[q_nom,q_nom],kc1,pc_knee,kc2l,kc2r,kc3,kc4,kc5,kc6,abcdc,ikoptions);

display('Check quasi static constraint')
qsc = QuasiStaticConstraint(robot,1);
qsc = qsc.addContact(r_foot,r_foot_pts);
qsc = qsc.addContact(l_foot,l_foot_pts);
qsc = qsc.setActive(true);
ikoptions = ikoptions.setMex(false);
q = test_IK_userfun(robot,q_seed,q_nom,kc1,qsc,kc2l,kc2r,kc3,kc4,kc5,kc6,ikoptions);
kinsol = doKinematics(robot,q);
com = getCOM(robot,kinsol);
rfoot_pos = forwardKin(robot,kinsol,r_foot,r_foot_pts,0);
lfoot_pos = forwardKin(robot,kinsol,l_foot,l_foot_pts,0);
shrinkFactor = qsc.shrinkFactor+1e-4;
center_pos = mean([lfoot_pos(1:2,:) rfoot_pos(1:2,:)],2);
shrink_vertices = [lfoot_pos(1:2,:) rfoot_pos(1:2,:)]*shrinkFactor+repmat(center_pos*(1-shrinkFactor),1,size(l_foot_pts,2)+size(r_foot_pts,2));
num_vertices = size(shrink_vertices,2);
quadoptions = optimset('Algorithm','interior-point-convex');
[~,~,exit_flag] = quadprog(eye(num_vertices),rand(num_vertices,1),[],[],[shrink_vertices;ones(1,num_vertices)],...
  [com(1:2);1],zeros(num_vertices,1),ones(num_vertices,1),1/num_vertices*ones(num_vertices,1),quadoptions);
valuecheck(exit_flag,1);

display('Check quasi static constraint for pointwise');
q = test_IKpointwise_userfun(robot,[0,1],[q_seed q_seed+1e-3*randn(nq,1)],[q_nom q_nom],kc1,qsc,kc2l,kc2r,kc3,kc4,kc5,kc6,ikoptions);
for i = 1:size(q,2)
  kinsol = doKinematics(robot,q(:,i));
  com = getCOM(robot,kinsol);
  rfoot_pos = forwardKin(robot,kinsol,r_foot,r_foot_pts,0);
  lfoot_pos = forwardKin(robot,kinsol,l_foot,l_foot_pts,0);
  shrinkFactor = qsc.shrinkFactor+1e-4;
  center_pos = mean([lfoot_pos(1:2,:) rfoot_pos(1:2,:)],2);
  shrink_vertices = [lfoot_pos(1:2,:) rfoot_pos(1:2,:)]*shrinkFactor+repmat(center_pos*(1-shrinkFactor),1,size(l_foot_pts,2)+size(r_foot_pts,2));
  num_vertices = size(shrink_vertices,2);
  quadoptions = optimset('Algorithm','interior-point-convex');
  [~,~,exit_flag] = quadprog(eye(num_vertices),rand(num_vertices,1),[],[],[shrink_vertices;ones(1,num_vertices)],...
    [com(1:2);1],zeros(num_vertices,1),ones(num_vertices,1),1/num_vertices*ones(num_vertices,1),quadoptions);
  valuecheck(exit_flag,1);
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
q = test_IKpointwise_userfun(robot,[0 1],[q_seed q_seed+1e-3*randn(nq,1)],[q_nom q_nom],kc1,qsc,kc2l,kc2r,kc3,hands_distance_cnst,ikoptions);
for i = 1:size(q,2)
  kinsol = doKinematics(robot,q(:,i));
  lhand_pos = forwardKin(robot,kinsol,l_hand,[0;0;0],0);
  rhand_pos = forwardKin(robot,kinsol,r_hand,[0;0;0],0);
  hand_dist = norm(lhand_pos-rhand_pos);
  if(hand_dist>1+1e-5 || hand_dist<0.5-1e-5)
    error('point to point distance constraint is not satisfied');
  end
end

head_pts = [[0;0;0] [0.1;0;0]];
world_pts = [[0;0;0] [0.1;0;0]];
head_world_dist_cnst = Point2PointDistanceConstraint(robot,head,0,head_pts,world_pts,[1.6 1.6],[1.9 1.9],tspan);
q = test_IK_userfun(robot,q_seed,q_nom,kc1,qsc,kc2l,kc2r,kc3,head_world_dist_cnst,ikoptions);
kinsol = doKinematics(robot,q);
head_pos = forwardKin(robot,kinsol,head,head_pts,0);
head_world_dist = head_pos-world_pts;
head_world_dist = sqrt(sum(head_world_dist.*head_world_dist,1));
if(any(head_world_dist>1.9+1e-5) || any(head_world_dist<1.6-1e-5))
  error('point to point distance constraint is not satisfied');
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
ikmexoptions = ikmexoptions.updateRobot(robot);
nq_aff = length(robot.getStateFrame.frame{2}.coordinates)/2;
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
valuecheck(rfoot_pos(3,:),[0 0 0 0],1e-5);
valuecheck(lfoot_pos(3,:),[0 0 0 0],1e-5);
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
end

function qmex = test_IK_userfun(r,q_seed,q_nom,varargin)
ikoptions = varargin{end};
ikoptions = ikoptions.setMex(false);
ikmexoptions = ikoptions;
ikmexoptions = ikmexoptions.setMex(true);
% [f,G,iGfun,jGvar,Fupp,Flow,xupp,xlow,iAfun,jAvar,A,nF] = inverseKin(r,q_seed,q_nom,varargin{1:end-1},ikmexoptions);
% keyboard;
tic
[q,info] = inverseKin(r,q_seed,q_nom,varargin{1:end-1},ikoptions);
toc
if(info>10)
  error('SNOPT info is %d, IK fails to solve the problem',info);
end
tic
[qmex,info_mex] = inverseKin(r,q_seed,q_nom,varargin{1:end-1},ikmexoptions);
toc
if(info_mex>10)
  error('SNOPT info is %d, IK mex fails to solve the problem',info_mex);
end
valuecheck(q,qmex,1e-2);
end

function qmex = test_IKpointwise_userfun(r,t,q_seed,q_nom,varargin)
ikoptions = varargin{end};
ikoptions = ikoptions.setMex(false);
ikmexoptions = ikoptions;
ikmexoptions = ikmexoptions.setMex(true);
tic
[q,info] = inverseKinPointwise(r,t,q_seed,q_nom,varargin{1:end-1},ikoptions);
toc
if(info>10)
  error('SNOPT info is %d, IK pointwisefails to solve the problem',info);
end
tic
[qmex,info] = inverseKinPointwise(r,t,q_seed,q_nom,varargin{1:end-1},ikmexoptions);
toc
if(info>10)
  error('SNOPT info is %d, IK pointwise mex fails to solve the problem',info);
end
valuecheck(q,qmex,1e-2);
end
