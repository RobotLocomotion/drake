function testKinCnstWaff
% test Single/MultipleTimeKinematicConstraints with affordance
urdf = [getDrakePath,'/examples/Atlas/urdf/atlas_minimal_contact.urdf'];
aff_urdf = [getDrakePath,'/systems/plants/constraint/test/valve_task_wall.urdf'];

options.floating = true;
robot = RigidBodyManipulator(urdf,options);
nq = robot.getNumDOF();
robot_aff = robot.addRobotFromURDF(aff_urdf,[0;0;0],[pi/2;0;0],struct('floating',false));
nq_aff = robot_aff.getNumDOF();

l_foot = robot_aff.findLinkInd('l_foot');
r_foot = robot_aff.findLinkInd('r_foot');
l_hand = robot_aff.findLinkInd('l_hand');
r_hand = robot_aff.findLinkInd('r_hand');
head = robot_aff.findLinkInd('head');
l_foot_pts = robot_aff.getBody(l_foot).contact_pts;
r_foot_pts = robot_aff.getBody(r_foot).contact_pts;
l_hand_pts = [0;0;0];
r_hand_pts = [0;0;0];


nom_data = load([getDrakePath,'/examples/Atlas/data/atlas_fp.mat']);
q = nom_data.xstar(1:nq)+0.1*randn(nq,1);
tspan0 = [0,1];
tspan1 = [];
t = 0.5;

q_rot1 = q;
q_rot1(6) = pi/2 + 0.01;
kinsol = doKinematics(robot, q_rot1);
l_foot_pos = forwardKin(robot, kinsol, l_foot, l_foot_pts, 2);
q_aff = [q;randn(length(robot_aff.getStateFrame.frame{2}.coordinates)/2,1)];

display('Check world CoM constraint');
testKinCnst_userfun(true,t,q_aff,SingleTimeKinematicConstraint.WorldCoMConstraint,robot,robot_aff,[0;0;0.9],[0;0;1],tspan0,1);
display('Check world CoM constraint with empty tspan');
testKinCnst_userfun(true,t,q_aff,SingleTimeKinematicConstraint.WorldCoMConstraint,robot,robot_aff,[0;0;0.9],[0;0;1]);
display('Check world CoM constraint with nan');
testKinCnst_userfun(true,t,q_aff,SingleTimeKinematicConstraint.WorldCoMConstraint,robot,robot_aff,[nan;nan;0.9],[nan;nan;0.92],tspan0/2,1);

display('Check world position constraint');
testKinCnst_userfun(true,t,q_aff,SingleTimeKinematicConstraint.WorldPositionConstraint,robot,robot_aff,l_foot,l_foot_pts,[-100*ones(2,4);zeros(1,4)],[100*ones(2,4);zeros(1,4)],tspan0);

display('Check world position constraint with nan');
testKinCnst_userfun(true,t,q_aff,SingleTimeKinematicConstraint.WorldPositionConstraint,robot,robot_aff,l_foot,l_foot_pts,[nan(2,4);zeros(1,4)],[nan(2,4);zeros(1,4)],tspan0);

display('Check world position in frame constraint');
rpy = [pi/10,pi/20,pi/10];
xyz = [0.5;0.0;0.2];
T = [rpy2rotmat(rpy),xyz;zeros(1,3),1];
testKinCnst_userfun(true,t,q_aff,SingleTimeKinematicConstraint.WorldPositionInFrameConstraint,robot,robot_aff,l_foot,l_foot_pts,T,[NaN(2,4);zeros(1,4)],[NaN(2,4);zeros(1,4)],tspan0);

display('Check world quaternion constraint')
testKinCnst_userfun(true,t,q_aff,SingleTimeKinematicConstraint.WorldQuatConstraint,robot,robot_aff,r_foot,[1;0;0;0],0.01,tspan0);

display('Check world quaternion constraint, a subtle case')
testKinCnst_userfun(true,t,q_aff,SingleTimeKinematicConstraint.WorldQuatConstraint,robot,robot_aff,l_foot,l_foot_pos(4:7,1),0,tspan0);

display('Check world euler constraint')
testKinCnst_userfun(true,t,q_aff,SingleTimeKinematicConstraint.WorldEulerConstraint,robot,robot_aff,l_foot,[0;nan;0.9*pi],[0;nan;1.1*pi],tspan0);

display('Check world gaze orientation constraint')
testKinCnst_userfun(true,t,q_aff,SingleTimeKinematicConstraint.WorldGazeOrientConstraint,robot,robot_aff,head,[1;0;0],[0.5;0.5;-0.5;0.5],0.3*pi,0.1*pi,tspan0);

display('Check world gaze orientation constraint with empty threshold')
testKinCnst_userfun(true,t,q_aff,SingleTimeKinematicConstraint.WorldGazeOrientConstraint,robot,robot_aff,head,[1;0;0],[0.5;0.5;-0.5;0.5],[],[],tspan0);

display('Check world gaze direction constraint')
testKinCnst_userfun(true,t,q_aff,SingleTimeKinematicConstraint.WorldGazeDirConstraint,robot,robot_aff,l_hand,[1;0;0],[0.5;0.3;0.4],0.3*pi,tspan0);

display('Check world gaze direction constraint with empty threshold')
testKinCnst_userfun(true,t,q_aff,SingleTimeKinematicConstraint.WorldGazeDirConstraint,robot,robot_aff,head,[1;0;0],[0.5;0.3;0.4],[],tspan0);

display('Check world gaze target constraint');
testKinCnst_userfun(true,t,q_aff,SingleTimeKinematicConstraint.WorldGazeTargetConstraint,robot,robot_aff,head,[1;0;0],[20;10;1],[0.3;0.1;0.2],0.3*pi,tspan0);

display('Check world gaze target constraint with empty threshold');
testKinCnst_userfun(true,t,q_aff,SingleTimeKinematicConstraint.WorldGazeTargetConstraint,robot,robot_aff,head,[1;0;0],[20;10;1],[0.3;0.1;0.2],[],tspan0);

t2 = [0 0.5 1];
q2 = randn(nq_aff,3);
display('Check world fixed position constraint')
testKinCnst_userfun(false,t2,q2,MultipleTimeKinematicConstraint.WorldFixedPositionConstraint,robot,robot_aff,l_hand,[[0;0;1] [1;0;1]],tspan0);
q3 = repmat(randn(nq_aff,1),1,3);
kc = WorldFixedPositionConstraint(robot_aff,l_hand,[[0;0;1] [1;0;1]],tspan0);
[c,dc] = kc.eval(t2,q3);
valuecheck(c,[0;0]);
testKinCnst_userfun(false,t2,q3,MultipleTimeKinematicConstraint.WorldFixedPositionConstraint,robot,robot_aff,l_hand,[[0;0;1] [1;0;1]],tspan0);

display('Check world fixed orientation constraint')
testKinCnst_userfun(false,t2,q2,MultipleTimeKinematicConstraint.WorldFixedOrientConstraint,robot,robot_aff,l_hand,tspan0);
kc = WorldFixedOrientConstraint(robot_aff,l_hand,tspan0);
[c,dc] = kc.eval(t2,q3);
valuecheck(c,3);
testKinCnst_userfun(false,t2,q3,MultipleTimeKinematicConstraint.WorldFixedOrientConstraint,robot,robot_aff,l_hand,tspan0);

display('Check world fixed body pose constraint')
testKinCnst_userfun(false,t2,q2,MultipleTimeKinematicConstraint.WorldFixedBodyPoseConstraint,robot,robot_aff,l_hand,tspan0);
kc = WorldFixedBodyPoseConstraint(robot_aff,l_hand,tspan0);
[c,dc] = kc.eval(t2,q3);
valuecheck(c,[0;3]);
testKinCnst_userfun(false,t2,q3,MultipleTimeKinematicConstraint.WorldFixedBodyPoseConstraint,robot,robot_aff,l_hand,tspan0);

end

function testKinCnst_userfun(singleTimeFlag,t,q,kc_type,robot,robot_aff,varargin)
robot_ptr = robot.getMexModelPtr;

if(singleTimeFlag)
  kc_mex = constructSingleTimeKinematicConstraint(kc_type,true,robot_ptr,varargin{:});
  kc = constructSingleTimeKinematicConstraint(kc_type,false,robot,varargin{:});
  
  kc = kc.updateRobot(robot_aff);
  
  if(kc_type == SingleTimeKinematicConstraint.WorldCoMConstraint)
    updatePtrWorldCoMConstraintmex(kc_mex,'robot',robot_aff.getMexModelPtr);
  elseif(kc_type == SingleTimeKinematicConstraint.WorldPositionConstraint)
    updatePtrWorldPositionConstraintmex(kc_mex,'robot',robot_aff.getMexModelPtr);
  elseif(kc_type == SingleTimeKinematicConstraint.WorldPositionInFrameConstraint)
    updatePtrWorldPositionInFrameConstraintmex(kc_mex,'robot',robot_aff.getMexModelPtr);
  elseif(kc_type == SingleTimeKinematicConstraint.WorldEulerConstraint)
    updatePtrWorldEulerConstraintmex(kc_mex,'robot',robot_aff.getMexModelPtr);
  elseif(kc_type == SingleTimeKinematicConstraint.WorldQuatConstraint)
    updatePtrWorldQuatConstraintmex(kc_mex,'robot',robot_aff.getMexModelPtr);
  elseif(kc_type == SingleTimeKinematicConstraint.WorldGazeOrientConstraint)
    updatePtrWorldGazeOrientConstraintmex(kc_mex,'robot',robot_aff.getMexModelPtr);
  elseif(kc_type == SingleTimeKinematicConstraint.WorldGazeDirConstraint)
    updatePtrWorldGazeDirConstraintmex(kc_mex,'robot',robot_aff.getMexModelPtr);
  elseif(kc_type == SingleTimeKinematicConstraint.WorldGazeTargetConstraint)
    updatePtrWorldGazeTargetConstraintmex(kc_mex,'robot',robot_aff.getMexModelPtr);
  end
  [num_cnst_mex,c_mex,dc_mex,cnst_name_mex,lb_mex,ub_mex] = testSingleTimeKinCnstmex(kc_mex,q,t);
else
  kc_mex = constructMultipleTimeKinematicConstraint(kc_type,true,robot_ptr,varargin{:});
  kc = constructMultipleTimeKinematicConstraint(kc_type,false,robot,varargin{:});
  kc = kc.updateRobot(robot_aff);
  if(kc_type == MultipleTimeKinematicConstraint.WorldFixedPositionConstraint)
    updatePtrWorldFixedPositionConstraintmex(kc_mex,'robot',robot_aff.getMexModelPtr);
  elseif(kc_type == MultipleTimeKinematicConstraint.WorldFixedOrientConstraint)
    updatePtrWorldFixedOrientConstraintmex(kc_mex,'robot',robot_aff.getMexModelPtr);
  elseif(kc_type == MultipleTimeKinematicConstraint.WorldFixedBodyPoseConstraint)
    updatePtrWorldFixedBodyPoseConstraintmex(kc_mex,'robot',robot_aff.getMexModelPtr);  
  end
  [num_cnst_mex,c_mex,dc_mex,cnst_name_mex,lb_mex,ub_mex] = testMultipleTimeKinCnstmex(kc_mex,q,t);
end
num_cnst = kc.getNumConstraint(t);
if(singleTimeFlag)
  kinsol = kc.robot.doKinematics(q,false,false);
  [c,dc] = kc.eval(t,kinsol);
else
  [c,dc] = kc.eval(t,q);
end
[~,dc_numerical] = geval(@(q) eval_numerical(singleTimeFlag,kc,t,q),q,struct('grad_method','numerical'));
valuecheck(dc_mex,dc_numerical,1e-5);
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
