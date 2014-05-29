function testKinCnst

checkDependency('rigidbodyconstraint_mex');

warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints');
warning('off','Drake:RigidBodyManipulator:UnsupportedJointLimits');
urdf = [getDrakePath,'/examples/Atlas/urdf/atlas_minimal_contact.urdf'];
urdf_collision = [getDrakePath,'/examples/Atlas/urdf/atlas_chull.urdf'];
aff_urdf = [getDrakePath,'/systems/plants/constraint/test/valve_task_wall.urdf'];

options.floating = true;
w = warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits');
warning('off','Drake:RigidBody:SimplifiedCollisionGeometry');
warning('off','Drake:RigidBodyManipulator:NonPSDInertia');
robot = RigidBodyManipulator(urdf,options);
nq = robot.getNumDOF();

robot_aff = robot.addRobotFromURDF(aff_urdf,[0;0;0],[pi/2;0;0],struct('floating',false));
nq_aff = robot_aff.getNumDOF();

robot_collision = RigidBodyManipulator(urdf_collision,options);
warning(w);
ignored_bodies = {'ltorso','mtorso','r_talus','l_talus'};
robot_collision = addLinksToCollisionFilterGroup(robot_collision,ignored_bodies,'no_collision',1);
robot_collision = robot_collision.compile();
robot_aff = robot.addRobotFromURDF(aff_urdf,[0;0;0],[pi/2;0;0],struct('floating',false));
robot_collision_aff = robot_collision.addRobotFromURDF(aff_urdf,[0;0;0],[pi/2;0;0],struct('floating',false));
robot_collision_aff.collision_filter_groups('aff') = CollisionFilterGroup();
%aff_link_indices = find([robot_collision_aff.body.robotnum]==2);
aff_link_indices = [robot_collision_aff.body.robotnum]==2;
linknames = {robot_collision_aff.body.linkname};
robot_collision_aff = robot_collision_aff.addLinksToCollisionFilterGroup(linknames(aff_link_indices),'aff',2);
robot_collision_aff = robot_collision_aff.addToIgnoredListOfCollisionFilterGroup('aff','aff');
robot_collision_aff = robot_collision_aff.compile();

nq_aff = robot_aff.getNumDOF();

l_foot = robot.findLinkInd('l_foot');
r_foot = robot.findLinkInd('r_foot');
l_hand = robot.findLinkInd('l_hand');
r_hand = robot.findLinkInd('r_hand');
head = robot.findLinkInd('head');
nLPts = length(robot.getBody(l_foot).getContactShapes);
l_foot_pts = zeros(3,nLPts);
for i=1:nLPts,
  l_foot_pts(:,i) = robot.getBody(l_foot).getContactShapes{i}.getPoints;
end
nRPts = length(robot.getBody(r_foot).getContactShapes);
r_foot_pts = zeros(3,nRPts);
for i=1:nRPts,
  r_foot_pts(:,i) = robot.getBody(r_foot).getContactShapes{i}.getPoints;
end
l_hand_pts = [0;0;0];
r_hand_pts = [0;0;0];

nom_data = load([getDrakePath,'/examples/Atlas/data/atlas_fp.mat']);
q = randn(nq,1);
% q_aff = randn(nq_aff,1);
q_aff = [q;randn(length(robot_aff.getStateFrame.frame{2}.coordinates)/2,1)];
tspan0 = [0,1];
tspan1 = [];
t = 0.5;

q_rot1 = q;
q_rot1(6) = pi/2 + 0.01;
kinsol = doKinematics(robot, q_rot1);
l_foot_pos = forwardKin(robot, kinsol, l_foot, l_foot_pts, 2);

display('Check world CoM constraint');
testKinCnst_userfun(true,false,t,q,q_aff,RigidBodyConstraint.WorldCoMConstraintType,robot,robot_aff,[0;0;0.9],[0;0;1],tspan0,1);
display('Check world CoM constraint with empty robot number');
testKinCnst_userfun(true,false,t,q,q_aff,RigidBodyConstraint.WorldCoMConstraintType,robot,robot_aff,[0;0;0.9],[0;0;1],tspan0);
display('Check world CoM constraint with empty tspan');
testKinCnst_userfun(true,false,t,q,q_aff,RigidBodyConstraint.WorldCoMConstraintType,robot,robot_aff,[0;0;0.9],[0;0;1]);
display('Check world CoM constraint with nan');
testKinCnst_userfun(true,false,t,q,q_aff,RigidBodyConstraint.WorldCoMConstraintType,robot,robot_aff,[-inf;nan;0.9],[nan;inf;0.92],tspan0/2);
display('Check world CoM constraint with multiple robots');
testKinCnst_userfun(true,false,t,q_aff,q_aff,RigidBodyConstraint.WorldCoMConstraintType,robot_aff,robot_aff,[0;0;0.9],[0;0;1],tspan0,[1,2]);

display('Check world position constraint');
testKinCnst_userfun(true,false,t,q,q_aff,RigidBodyConstraint.WorldPositionConstraintType,robot,robot_aff,l_foot,l_foot_pts,[-100*ones(2,4);zeros(1,4)],[100*ones(2,4);zeros(1,4)],tspan0);

display('Check world position constraint with nan');
testKinCnst_userfun(true,false,t,q,q_aff,RigidBodyConstraint.WorldPositionConstraintType,robot,robot_aff,l_foot,l_foot_pts,[nan(2,4);zeros(1,4)],[nan(2,4);zeros(1,4)],tspan0);

display('Check world quaternion constraint')
testKinCnst_userfun(true,false,t,q,q_aff,RigidBodyConstraint.WorldQuatConstraintType,robot,robot_aff,r_foot,[1;0;0;0],0.01,tspan0);

display('Check world quaternion constraint, a subtle case')
testKinCnst_userfun(true,false,t,q,q_aff,RigidBodyConstraint.WorldQuatConstraintType,robot,robot_aff,l_foot,l_foot_pos(4:7,1),0,tspan0);

display('Check world euler constraint')
testKinCnst_userfun(true,false,t,q,q_aff,RigidBodyConstraint.WorldEulerConstraintType,robot,robot_aff,l_foot,[0;nan;0.9*pi],[0;nan;1.1*pi],tspan0);

display('Check world gaze orientation constraint')
testKinCnst_userfun(true,false,t,q,q_aff,RigidBodyConstraint.WorldGazeOrientConstraintType,robot,robot_aff,head,[1;0;0],[0.5;0.5;-0.5;0.5],0.3*pi,0.1*pi,tspan0);

display('Check world gaze orientation constraint with empty threshold')
testKinCnst_userfun(true,false,t,q,q_aff,RigidBodyConstraint.WorldGazeOrientConstraintType,robot,robot_aff,head,[1;0;0],[0.5;0.5;-0.5;0.5],[],[],tspan0);

display('Check world gaze direction constraint')
testKinCnst_userfun(true,false,t,q,q_aff,RigidBodyConstraint.WorldGazeDirConstraintType,robot,robot_aff,l_hand,[1;0;0],[0.5;0.3;0.4],0.3*pi,tspan0);

display('Check world gaze direction constraint with empty threshold')
testKinCnst_userfun(true,false,t,q,q_aff,RigidBodyConstraint.WorldGazeDirConstraintType,robot,robot_aff,head,[1;0;0],[0.5;0.3;0.4],[],tspan0);

display('Check world gaze target constraint');
testKinCnst_userfun(true,false,t,q,q_aff,RigidBodyConstraint.WorldGazeTargetConstraintType,robot,robot_aff,head,[1;0;0],[20;10;1],[0.3;0.1;0.2],0.3*pi,tspan0);

display('Check world gaze target constraint with empty threshold');
testKinCnst_userfun(true,false,t,q,q_aff,RigidBodyConstraint.WorldGazeTargetConstraintType,robot,robot_aff,head,[1;0;0],[20;10;1],[0.3;0.1;0.2],[],tspan0);

t2 = [0 0.5 1];
q21 = randn(nq,length(t2));
q22 = repmat(randn(nq,1),1,length(t2));
tspan2 = [0.3 0.7];
t3 = [0 0.1 0.4 0.5 0.6 0.8];
q31 = repmat(randn(nq,1),1,length(t3));
% q21_aff = randn(nq_aff,length(t2));
% q22_aff = repmat(randn(nq_aff,1),1,length(t2));
% q31_aff = repmat(randn(nq_aff,1),1,length(t3));
q21_aff = [q21;randn(length(robot_aff.getStateFrame.frame{2}.coordinates)/2,length(t2))];
q22_aff = [q22;repmat(randn(length(robot_aff.getStateFrame.frame{2}.coordinates)/2,1),1,length(t2))];
q31_aff = [q31;repmat(randn(length(robot_aff.getStateFrame.frame{2}.coordinates)/2,1),1,length(t3))];
display('Check world fixed position constraint')
testKinCnst_userfun(false,false,t2,q21,q21_aff,RigidBodyConstraint.WorldFixedPositionConstraintType,robot,robot_aff,l_hand,[[0;0;1] [1;0;1]],tspan0);
kc = WorldFixedPositionConstraint(robot,l_hand,[[0;0;1] [1;0;1]],tspan0);
kinsol_cell = cell(1,size(q22,2));
for i = 1:size(q22,2)
  kinsol_cell{i} = kc.robot.doKinematics(q22(:,i),false,false);
end
[c,dc] = kc.eval(t2,kinsol_cell);
valuecheck(c,[0;0]);
testKinCnst_userfun(false,false,t2,q22,q22_aff,RigidBodyConstraint.WorldFixedPositionConstraintType,robot,robot_aff,l_hand,[[0;0;1] [1;0;1]],tspan0);

kc = WorldFixedPositionConstraint(robot,l_hand,[[0;0;1] [1;0;1]],tspan2);
kinsol_cell = cell(1,size(q31,2));
for i = 1:size(q31,2)
  kinsol_cell{i} = kc.robot.doKinematics(q31(:,i),false,false);
end
[c,dc] = kc.eval(t3,kinsol_cell);
valuecheck(c,[0;0]);
testKinCnst_userfun(false,false,t3,q31,q31_aff,RigidBodyConstraint.WorldFixedPositionConstraintType,robot,robot_aff,l_hand,[[0;0;1] [1;0;1]],tspan2);

display('Check world fixed orientation constraint')
testKinCnst_userfun(false,false,t2,q21,q21_aff,RigidBodyConstraint.WorldFixedOrientConstraintType,robot,robot_aff,l_hand,tspan0);
kc = WorldFixedOrientConstraint(robot,l_hand,tspan0);
kinsol_cell = cell(1,size(q22,2));
for i = 1:size(q22,2)
  kinsol_cell{i} = kc.robot.doKinematics(q22(:,i),false,false);
end
[c,dc] = kc.eval(t2,kinsol_cell);
valuecheck(c,length(t2));
testKinCnst_userfun(false,false,t2,q22,q22_aff,RigidBodyConstraint.WorldFixedOrientConstraintType,robot,robot_aff,l_hand,tspan0);

kc = WorldFixedOrientConstraint(robot,l_hand,tspan2);
kinsol_cell = cell(1,size(q31,2));
for i = 1:size(q31,2)
  kinsol_cell{i} = kc.robot.doKinematics(q31(:,i),false,false);
end
[c,dc] = kc.eval(t3,kinsol_cell);
valuecheck(c,sum(kc.isTimeValid(t3)));
testKinCnst_userfun(false,false,t3,q31,q31_aff,RigidBodyConstraint.WorldFixedOrientConstraintType,robot,robot_aff,l_hand,tspan2);

display('Check world fixed body pose constraint')
testKinCnst_userfun(false,false,t2,q21,q21_aff,RigidBodyConstraint.WorldFixedBodyPoseConstraintType,robot,robot_aff,l_hand,tspan0);
kc = WorldFixedBodyPoseConstraint(robot,l_hand,tspan0);
kinsol_cell = cell(1,size(q22,2));
for i = 1:size(q22,2)
  kinsol_cell{i} = kc.robot.doKinematics(q22(:,i),false,false);
end
[c,dc] = kc.eval(t2,kinsol_cell);
valuecheck(c,[0;length(t2)]);
testKinCnst_userfun(false,false,t2,q22,q22_aff,RigidBodyConstraint.WorldFixedBodyPoseConstraintType,robot,robot_aff,l_hand,tspan0);

kc = WorldFixedBodyPoseConstraint(robot,l_hand,tspan2);
kinsol_cell = cell(1,size(q31,2));
for i = 1:size(q31,2)
  kinsol_cell{i} = kc.robot.doKinematics(q31(:,i),false,false);
end
[c,dc] = kc.eval(t3,kinsol_cell);
valuecheck(c,[0;sum(kc.isTimeValid(t3))]);
testKinCnst_userfun(false,false,t3,q31,q31_aff,RigidBodyConstraint.WorldFixedBodyPoseConstraintType,robot,robot_aff,l_hand,tspan2);


 display('Check all-to-all closest-distance constraint');
 testKinCnst_userfun(true,true,t,q,q_aff,RigidBodyConstraint.AllBodiesClosestDistanceConstraintType,robot_collision,robot_collision_aff,0.05,1e1,tspan0);

display('Check point to point distance constraint');
lhand_pts = rand(3,2);
rhand_pts = rand(3,2);
kc = Point2PointDistanceConstraint(robot,l_hand,r_hand,lhand_pts,rhand_pts,[0 0],[1 1],tspan0);
kinsol = doKinematics(robot,q);
lhand_pos = forwardKin(robot,kinsol,l_hand,lhand_pts,0);
rhand_pos = forwardKin(robot,kinsol,r_hand,rhand_pts,0);
[c,dc] = kc.eval(t,kinsol);
valuecheck(c',sum((lhand_pos-rhand_pos).*(lhand_pos-rhand_pos),1));
testKinCnst_userfun(true,false,t,q,q_aff,RigidBodyConstraint.Point2PointDistanceConstraintType,robot,robot_aff,l_hand,r_hand,lhand_pts,rhand_pts,[0 0],[1 1],tspan0);
testKinCnst_userfun(true,false,t,q,q_aff,RigidBodyConstraint.Point2PointDistanceConstraintType,robot,robot_aff,0,r_hand,rand(3,2),rhand_pts,[0 0],[1 1],tspan0);

display('Check world position in frame constraint');
rpy = 2*pi*rand(3,1) - pi;
xyz = [0.2;0.2;0.2].*rand(3,1) + [0.5;0.0;0.5];
T = [rpy2rotmat(rpy),xyz;zeros(1,3),1];
testKinCnst_userfun(true,false,t,q,q_aff,RigidBodyConstraint.WorldPositionInFrameConstraintType,robot,robot_aff,l_foot,l_foot_pts,T,[-100*ones(2,4);zeros(1,4)],[100*ones(2,4);zeros(1,4)],tspan0);

display('Check point to line segment distance constraint');
testKinCnst_userfun(true,false,t,q,q_aff,RigidBodyConstraint.Point2LineSegDistConstraintType,robot,robot_aff,l_hand,[0;0;0],r_foot,r_foot_pts(:,1:2),0.1,1,tspan0);

display('Check Relative gaze target constraint');
testKinCnst_userfun(true,false,t,q,q_aff,RigidBodyConstraint.RelativeGazeTargetConstraintType,robot,robot_aff,head,r_hand,[1;0;0],[0;0;0],[0;0;0],pi/30,tspan0);

display('Check relative gaze direction constraint')
testKinCnst_userfun(true,false,t,q,q_aff,RigidBodyConstraint.RelativeGazeDirConstraintType,robot,robot_aff,head,l_hand,[1;0;0],[0.5;0.3;0.4],0.3*pi,tspan0);

display('Check Relative position constraint');
testKinCnst_userfun(true,false,t,q,q_aff,RigidBodyConstraint.RelativePositionConstraintType,robot,robot_aff,...
  [[0;0;0],[0;1;0.2]],[[nan;-0.1;-0.2],[-0.2;nan;-0.4]],[[nan;0.1;0.2],[0.3;nan;0]],...
  r_hand,head,[[0;0.1;0.2];rpy2quat(randn(3,1))],tspan0);

display('Check Relative quaternion constraint');
testKinCnst_userfun(true,false,t,q,q_aff,RigidBodyConstraint.RelativeQuatConstraintType,robot,robot_aff,...
  r_hand,l_hand,rpy2quat(randn(3,1)),5/180*pi,tspan0);
end

function testKinCnst_userfun(singleTimeFlag,always_use_mex_dynamics,t,q,q_aff,cnst_type,robot,robot_aff,varargin)
robot_ptr = robot.getMexModelPtr;
if(singleTimeFlag)
  kc_mex = constructRigidBodyConstraint(cnst_type,true,robot_ptr,varargin{:});
  kc = constructRigidBodyConstraint(cnst_type,false,robot,varargin{:});
  [type_mex,num_cnst_mex,c_mex,dc_mex,cnst_name_mex,lb_mex,ub_mex] = testSingleTimeKinCnstmex(kc_mex,q,t);
  [~,~,c_mex_ptr,~,~,~,~] = testSingleTimeKinCnstmex(kc.mex_ptr,q,t);
  valuecheck(c_mex,c_mex_ptr,1e-5);
else
  kc_mex = constructRigidBodyConstraint(cnst_type,true,robot_ptr,varargin{:});
  kc = constructRigidBodyConstraint(cnst_type,false,robot,varargin{:});
  [type_mex,num_cnst_mex,c_mex,dc_mex,cnst_name_mex,lb_mex,ub_mex] = testMultipleTimeKinCnstmex(kc_mex,q,t);
  [~,~,c_mex_ptr,~,~,~,~] = testMultipleTimeKinCnstmex(kc.mex_ptr,q,t);
  valuecheck(c_mex,c_mex_ptr,1e-5);
end
[~,dc_mex_numeric] = geval(@(q) eval_numerical(singleTimeFlag,kc,t,q),q,struct('grad_method','numerical'));
valuecheck(dc_mex,dc_mex_numeric,1e-3);
valuecheck(type_mex,kc.type);
type_name = constraintTypemex(kc_mex);
if ~strcmp(type_name,kc_mex.name)
  error('mex type name do not match');
end
num_cnst = kc.getNumConstraint(t);
if(singleTimeFlag)
  kinsol = kc.robot.doKinematics(q,false,always_use_mex_dynamics);
  [c,dc] = kc.eval(t,kinsol);
  cnstr = kc.generateConstraint(t);
  kinsol = kc.robot.doKinematics(q,false,always_use_mex_dynamics);
  [c_cnstr,dc_cnstr] = cnstr{1}.eval(kinsol);
else
  kinsol_cell = cell(1,length(t));
  for i = 1:length(t)
    kinsol_cell{i} = doKinematics(robot,q(:,i),false,always_use_mex_dynamics);
  end
  [c,dc] = kc.eval(t,kinsol_cell);
  cnstr = kc.generateConstraint(t);
  [c_cnstr,dc_cnstr] = cnstr{1}.eval(kinsol_cell);
end
[lb,ub] = kc.bounds(t);
cnst_name = kc.name(t);
valuecheck(strcmp(cnst_name,cnstr{1}.name),1);
% strcmp(cnst_name_mex,cnst_name);
valuecheck(num_cnst_mex,num_cnst);
valuecheck(c_mex,c,1e-8);
valuecheck(dc_mex,dc,1e-8);
valuecheck(lb_mex,lb,1e-8);
valuecheck(ub_mex,ub,1e-8);
valuecheck(c,c_cnstr);
valuecheck(dc,dc_cnstr);
valuecheck(sparse(cnstr{1}.iCfun,cnstr{1}.jCvar,dc_cnstr(sub2ind([cnstr{1}.num_cnstr,cnstr{1}.xdim],cnstr{1}.iCfun,cnstr{1}.jCvar)),...
cnstr{1}.num_cnstr,cnstr{1}.xdim),dc_cnstr);
valuecheck(lb,cnstr{1}.lb);
valuecheck(ub,cnstr{1}.ub);


%%%% Check Kinematics Constraint after adding robot
kc = kc.updateRobot(robot_aff);
kc_mex = kc.mex_ptr;
num_cnst = kc.getNumConstraint(t);
if(singleTimeFlag)
  [type_mex,num_cnst_mex,c_mex,dc_mex,cnst_name_mex,lb_mex,ub_mex] = testSingleTimeKinCnstmex(kc_mex,q_aff,t);
else
  [type_mex,num_cnst_mex,c_mex,dc_mex,cnst_name_mex,lb_mex,ub_mex] = testMultipleTimeKinCnstmex(kc_mex,q_aff,t);
end
if(singleTimeFlag)
  kinsol = kc.robot.doKinematics(q_aff,false,always_use_mex_dynamics);
  [c,dc] = kc.eval(t,kinsol);
else
  kinsol_cell = cell(1,length(t));
  for i = 1:length(t)
    kinsol_cell{i} = kc.robot.doKinematics(q_aff(:,i),false,always_use_mex_dynamics);
  end
  [c,dc] = kc.eval(t,kinsol_cell);
end
[~,dc_numerical] = geval(@(q) eval_numerical(singleTimeFlag,kc,t,q),q_aff,struct('grad_method','numerical'));
valuecheck(dc_mex,dc_numerical,1e-3);
valuecheck(type_mex,kc.type);
type_name = constraintTypemex(kc_mex);
if ~strcmp(type_name,kc_mex.name)
  error('mex type name do not match');
end
[lb,ub] = kc.bounds(t);
cnst_name = kc.name(t);
strcmp(cnst_name_mex,cnst_name);
valuecheck(num_cnst_mex,num_cnst);
valuecheck(c_mex,c,1e-8);
valuecheck(dc_mex,dc,1e-8);
valuecheck(lb_mex,lb,1e-8);
valuecheck(ub_mex,ub,1e-8);
end

function c = eval_numerical(singleTimeFlag,constraint,t,q)
if(singleTimeFlag)
  [~,~,c,~,~,~,~] = testSingleTimeKinCnstmex(constraint.mex_ptr,q,t);
else
  [~,~,c,~,~,~,~] = testMultipleTimeKinCnstmex(constraint.mex_ptr,q,t);
end
end

function c = eval_numerical_matlab(constraint,t,q)
kinsol = doKinematics(constraint.robot,q,false,false);
c = constraint.eval(t,kinsol);
end
