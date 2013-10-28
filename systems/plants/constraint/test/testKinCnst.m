function testKinCnst
urdf = [getDrakePath,'/examples/Atlas/urdf/atlas_minimal_contact.urdf'];
urdf_collision = [getDrakePath,'/systems/plants/constraint/test/model_simple_visuals.urdf'];
aff_urdf = [getDrakePath,'/systems/plants/constraint/test/valve_task_wall.urdf'];

options.floating = true;
robot = RigidBodyManipulator(urdf,options);
nq = robot.getNumDOF();

r_collision = RigidBodyManipulator(urdf_collision,options);
ignored_bodies = {'ltorso','mtorso','r_talus','l_talus'};
r_collision = addLinksToCollisionFilterGroup(r_collision,ignored_bodies,'no_collision',1);
r_collision = r_collision.compile();
robot_aff = robot.addRobotFromURDF(aff_urdf,[0;0;0],[pi/2;0;0],struct('floating',false));
nq_aff = robot_aff.getNumDOF();

l_foot = robot.findLinkInd('l_foot');
r_foot = robot.findLinkInd('r_foot');
l_hand = robot.findLinkInd('l_hand');
r_hand = robot.findLinkInd('r_hand');
head = robot.findLinkInd('head');
l_foot_pts = robot.getBody(l_foot).contact_pts;
r_foot_pts = robot.getBody(r_foot).contact_pts;
l_hand_pts = [0;0;0];
r_hand_pts = [0;0;0];

nom_data = load([getDrakePath,'/examples/Atlas/data/atlas_fp.mat']);
q = nom_data.xstar(1:nq)+0.1*randn(nq,1);
q_aff = randn(nq_aff,1);
tspan0 = [0,1];
tspan1 = [];
t = 0.5;

q_rot1 = q;
q_rot1(6) = pi/2 + 0.01;
kinsol = doKinematics(robot, q_rot1);
l_foot_pos = forwardKin(robot, kinsol, l_foot, l_foot_pts, 2);

display('Check world CoM constraint');
testKinCnst_userfun(true,t,q,SingleTimeKinematicConstraint.WorldCoMConstraint,robot,[0;0;0.9],[0;0;1],tspan0,1);
display('Check world CoM constraint with empty robot number');
testKinCnst_userfun(true,t,q,SingleTimeKinematicConstraint.WorldCoMConstraint,robot,[0;0;0.9],[0;0;1],tspan0);
display('Check world CoM constraint with empty tspan');
testKinCnst_userfun(true,t,q,SingleTimeKinematicConstraint.WorldCoMConstraint,robot,[0;0;0.9],[0;0;1]);
display('Check world CoM constraint with nan');
testKinCnst_userfun(true,t,q,SingleTimeKinematicConstraint.WorldCoMConstraint,robot,[nan;nan;0.9],[nan;nan;0.92],tspan0/2);
display('Check world CoM constraint with multiple robots');
testKinCnst_userfun(true,t,q_aff,SingleTimeKinematicConstraint.WorldCoMConstraint,robot_aff,[0;0;0.9],[0;0;1],tspan0,[1,2]);

display('Check world position constraint');
testKinCnst_userfun(true,t,q,SingleTimeKinematicConstraint.WorldPositionConstraint,robot,l_foot,l_foot_pts,[-100*ones(2,4);zeros(1,4)],[100*ones(2,4);zeros(1,4)],tspan0);

display('Check world position constraint with nan');
testKinCnst_userfun(true,t,q,SingleTimeKinematicConstraint.WorldPositionConstraint,robot,l_foot,l_foot_pts,[nan(2,4);zeros(1,4)],[nan(2,4);zeros(1,4)],tspan0);

display('Check world quaternion constraint')
testKinCnst_userfun(true,t,q,SingleTimeKinematicConstraint.WorldQuatConstraint,robot,r_foot,[1;0;0;0],0.01,tspan0);

display('Check world quaternion constraint, a subtle case')
testKinCnst_userfun(true,t,q,SingleTimeKinematicConstraint.WorldQuatConstraint,robot,l_foot,l_foot_pos(4:7,1),0,tspan0);

display('Check world euler constraint')
testKinCnst_userfun(true,t,q,SingleTimeKinematicConstraint.WorldEulerConstraint,robot,l_foot,[0;nan;0.9*pi],[0;nan;1.1*pi],tspan0);

display('Check world gaze orientation constraint')
testKinCnst_userfun(true,t,q,SingleTimeKinematicConstraint.WorldGazeOrientConstraint,robot,head,[1;0;0],[0.5;0.5;-0.5;0.5],0.3*pi,0.1*pi,tspan0);

display('Check world gaze orientation constraint with empty threshold')
testKinCnst_userfun(true,t,q,SingleTimeKinematicConstraint.WorldGazeOrientConstraint,robot,head,[1;0;0],[0.5;0.5;-0.5;0.5],[],[],tspan0);

display('Check world gaze direction constraint')
testKinCnst_userfun(true,t,q,SingleTimeKinematicConstraint.WorldGazeDirConstraint,robot,l_hand,[1;0;0],[0.5;0.3;0.4],0.3*pi,tspan0);

display('Check world gaze direction constraint with empty threshold')
testKinCnst_userfun(true,t,q,SingleTimeKinematicConstraint.WorldGazeDirConstraint,robot,head,[1;0;0],[0.5;0.3;0.4],[],tspan0);

display('Check world gaze target constraint');
testKinCnst_userfun(true,t,q,SingleTimeKinematicConstraint.WorldGazeTargetConstraint,robot,head,[1;0;0],[20;10;1],[0.3;0.1;0.2],0.3*pi,tspan0);

display('Check world gaze target constraint with empty threshold');
testKinCnst_userfun(true,t,q,SingleTimeKinematicConstraint.WorldGazeTargetConstraint,robot,head,[1;0;0],[20;10;1],[0.3;0.1;0.2],[],tspan0);

t2 = [0 0.5 1];
q21 = randn(nq,3);
q22 = repmat(randn(nq,1),1,length(t2));
tspan2 = [0.3 0.7];
t3 = [0 0.1 0.4 0.5 0.6 0.8];
q31 = repmat(randn(nq,1),1,length(t3));
display('Check world fixed position constraint')
testKinCnst_userfun(false,t2,q21,MultipleTimeKinematicConstraint.WorldFixedPositionConstraint,robot,l_hand,[[0;0;1] [1;0;1]],tspan0);
kc = WorldFixedPositionConstraint(robot,l_hand,[[0;0;1] [1;0;1]],tspan0);
[c,dc] = kc.eval(t2,q22);
valuecheck(c,[0;0]);
testKinCnst_userfun(false,t2,q22,MultipleTimeKinematicConstraint.WorldFixedPositionConstraint,robot,l_hand,[[0;0;1] [1;0;1]],tspan0);

kc = WorldFixedPositionConstraint(robot,l_hand,[[0;0;1] [1;0;1]],tspan2);
[c,dc] = kc.eval(t3,q31);
valuecheck(c,[0;0]);
testKinCnst_userfun(false,t3,q31,MultipleTimeKinematicConstraint.WorldFixedPositionConstraint,robot,l_hand,[[0;0;1] [1;0;1]],tspan2);

display('Check world fixed orientation constraint')
testKinCnst_userfun(false,t2,q21,MultipleTimeKinematicConstraint.WorldFixedOrientConstraint,robot,l_hand,tspan0);
kc = WorldFixedOrientConstraint(robot,l_hand,tspan0);
[c,dc] = kc.eval(t2,q22);
valuecheck(c,length(t2));
testKinCnst_userfun(false,t2,q22,MultipleTimeKinematicConstraint.WorldFixedOrientConstraint,robot,l_hand,tspan0);

kc = WorldFixedOrientConstraint(robot,l_hand,tspan2);
[c,dc] = kc.eval(t3,q31);
valuecheck(c,sum(kc.isTimeValid(t3)));
testKinCnst_userfun(false,t3,q31,MultipleTimeKinematicConstraint.WorldFixedOrientConstraint,robot,l_hand,tspan2);

display('Check world fixed body pose constraint')
testKinCnst_userfun(false,t2,q21,MultipleTimeKinematicConstraint.WorldFixedBodyPoseConstraint,robot,l_hand,tspan0);
kc = WorldFixedBodyPoseConstraint(robot,l_hand,tspan0);
[c,dc] = kc.eval(t2,q22);
valuecheck(c,[0;length(t2)]);
testKinCnst_userfun(false,t2,q22,MultipleTimeKinematicConstraint.WorldFixedBodyPoseConstraint,robot,l_hand,tspan0);

kc = WorldFixedBodyPoseConstraint(robot,l_hand,tspan2);
[c,dc] = kc.eval(t3,q31);
valuecheck(c,[0;sum(kc.isTimeValid(t3))]);
testKinCnst_userfun(false,t3,q31,MultipleTimeKinematicConstraint.WorldFixedBodyPoseConstraint,robot,l_hand,tspan2);


% display('Check all-to-all closest-distance constraint');
% testKinCnst_userfun(true,t,q,SingleTimeKinematicConstraint.AllBodiesClosestDistanceConstraint,r_collision,0.05,1e1,tspan0);

display('Check point to point distance constraint');
lhand_pts = rand(3,2);
rhand_pts = rand(3,2);
kc = Point2PointDistanceConstraint(robot,l_hand,r_hand,lhand_pts,rhand_pts,[0 0],[1 1],tspan0);
kinsol = doKinematics(robot,q);
lhand_pos = forwardKin(robot,kinsol,l_hand,lhand_pts,0);
rhand_pos = forwardKin(robot,kinsol,r_hand,rhand_pts,0);
[c,dc] = kc.eval(t,kinsol);
valuecheck(c',sum((lhand_pos-rhand_pos).*(lhand_pos-rhand_pos),1));
testKinCnst_userfun(true,t,q,SingleTimeKinematicConstraint.Point2PointDistanceConstraint,robot,l_hand,r_hand,lhand_pts,rhand_pts,[0 0],[1 1],tspan0);
testKinCnst_userfun(true,t,q,SingleTimeKinematicConstraint.Point2PointDistanceConstraint,robot,0,r_hand,rand(3,2),rhand_pts,[0 0],[1 1],tspan0);

display('Check world position in frame constraint');
rpy = 2*pi*rand(3,1) - pi;
xyz = [0.2;0.2;0.2].*rand(3,1) + [0.5;0.0;0.5];
T = [rpy2rotmat(rpy),xyz;zeros(1,3),1];
testKinCnst_userfun(true,t,q,SingleTimeKinematicConstraint.WorldPositionInFrameConstraint,robot,l_foot,l_foot_pts,T,[-100*ones(2,4);zeros(1,4)],[100*ones(2,4);zeros(1,4)],tspan0);
end

function testKinCnst_userfun(singleTimeFlag,t,q,kc_type,varargin)
robot = varargin{1};
robot_ptr = robot.getMexModelPtr;
if(singleTimeFlag)
  kc_mex = constructSingleTimeKinematicConstraint(kc_type,true,robot_ptr,varargin{2:end});
  kc = constructSingleTimeKinematicConstraint(kc_type,false,varargin{:});
  [num_cnst_mex,c_mex,dc_mex,cnst_name_mex,lb_mex,ub_mex] = testSingleTimeKinCnstmex(kc_mex,q,t);
  [~,c_mex_ptr,~,~,~,~] = testSingleTimeKinCnstmex(kc.mex_ptr,q,t);
  valuecheck(c_mex,c_mex_ptr,1e-5);
else
  kc_mex = constructMultipleTimeKinematicConstraint(kc_type,true,robot_ptr,varargin{2:end});
  kc = constructMultipleTimeKinematicConstraint(kc_type,false,varargin{:});
  [num_cnst_mex,c_mex,dc_mex,cnst_name_mex,lb_mex,ub_mex] = testMultipleTimeKinCnstmex(kc_mex,q,t);
end
[~,dc_mex_numeric] = geval(@(q) eval_numerical(singleTimeFlag,kc,t,q),q,struct('grad_method','numerical'));
valuecheck(dc_mex,dc_mex_numeric,1e-3);
num_cnst = kc.getNumConstraint(t);
if(singleTimeFlag)
  kinsol = kc.robot.doKinematics(q,false,false);
  [c,dc] = kc.eval(t,kinsol);
else
  [c,dc] = kc.eval(t,q);
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
  [~,c,~,~,~,~] = testSingleTimeKinCnstmex(constraint.mex_ptr,q,t);
else
  [~,c,~,~,~,~] = testMultipleTimeKinCnstmex(constraint.mex_ptr,q,t);
end
end

function c = eval_numerical_matlab(constraint,t,q)
kinsol = doKinematics(constraint.robot,q,false,false);
c = constraint.eval(t,kinsol);
end
