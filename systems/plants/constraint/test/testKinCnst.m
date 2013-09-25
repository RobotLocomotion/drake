function testKinCnst
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

nom_data = load([getDrakePath,'/examples/Atlas/data/atlas_fp.mat']);
q = nom_data.xstar(1:nq)+0.1*randn(nq,1);
tspan0 = [0,1];
tspan1 = [];
t = 0.5;

q_rot1 = q;
q_rot1(6) = pi/2 + 0.01;
kinsol = doKinematics(r, q_rot1);
l_foot_pos = forwardKin(r, kinsol, l_foot, l_foot_pts, 2);

display('Check world CoM constraint');
testKinCnst_userfun(true,t,q,SingleTimeKinematicConstraint.WorldCoMConstraint,r,[0;0;0.9],[0;0;1],tspan0);
display('Check world CoM constraint with empty tspan');
testKinCnst_userfun(true,t,q,SingleTimeKinematicConstraint.WorldCoMConstraint,r,[0;0;0.9],[0;0;1]);
display('Check world CoM constraint with nan');
testKinCnst_userfun(true,t,q,SingleTimeKinematicConstraint.WorldCoMConstraint,r,[nan;nan;0.9],[nan;nan;0.92],tspan0/2);

display('Check world position constraint');
testKinCnst_userfun(true,t,q,SingleTimeKinematicConstraint.WorldPositionConstraint,r,l_foot,l_foot_pts,[-100*ones(2,4);zeros(1,4)],[100*ones(2,4);zeros(1,4)],tspan0);

display('Check world position constraint with nan');
testKinCnst_userfun(true,t,q,SingleTimeKinematicConstraint.WorldPositionConstraint,r,l_foot,l_foot_pts,[nan(2,4);zeros(1,4)],[nan(2,4);zeros(1,4)],tspan0);

display('Check world quaternion constraint')
testKinCnst_userfun(true,t,q,SingleTimeKinematicConstraint.WorldQuatConstraint,r,r_foot,[1;0;0;0],0.01,tspan0);

display('Check world quaternion constraint, a subtle case')
testKinCnst_userfun(true,t,q,SingleTimeKinematicConstraint.WorldQuatConstraint,r,l_foot,l_foot_pos(4:7,1),0,tspan0);

display('Check world euler constraint')
testKinCnst_userfun(true,t,q,SingleTimeKinematicConstraint.WorldEulerConstraint,r,l_foot,[0;nan;0.9*pi],[0;nan;1.1*pi],tspan0);

display('Check world gaze orientation constraint')
testKinCnst_userfun(true,t,q,SingleTimeKinematicConstraint.WorldGazeOrientConstraint,r,head,[1;0;0],[0.5;0.5;-0.5;0.5],0.3*pi,0.1*pi,tspan0);

display('Check world gaze orientation constraint with empty threshold')
testKinCnst_userfun(true,t,q,SingleTimeKinematicConstraint.WorldGazeOrientConstraint,r,head,[1;0;0],[0.5;0.5;-0.5;0.5],[],[],tspan0);

display('Check world gaze direction constraint')
testKinCnst_userfun(true,t,q,SingleTimeKinematicConstraint.WorldGazeDirConstraint,r,l_hand,[1;0;0],[0.5;0.3;0.4],0.3*pi,tspan0);

display('Check world gaze direction constraint with empty threshold')
testKinCnst_userfun(true,t,q,SingleTimeKinematicConstraint.WorldGazeDirConstraint,r,head,[1;0;0],[0.5;0.3;0.4],[],tspan0);

display('Check world gaze target constraint');
testKinCnst_userfun(true,t,q,SingleTimeKinematicConstraint.WorldGazeTargetConstraint,r,head,[1;0;0],[20;10;1],[0.3;0.1;0.2],0.3*pi,tspan0);

display('Check world gaze target constraint with empty threshold');
testKinCnst_userfun(true,t,q,SingleTimeKinematicConstraint.WorldGazeTargetConstraint,r,head,[1;0;0],[20;10;1],[0.3;0.1;0.2],[],tspan0);

t2 = [0 0.5 1];
q2 = randn(nq,3);
display('Check world fixed position constraint')
testKinCnst_userfun(false,t2,q2,MultipleTimeKinematicConstraint.WorldFixedPositionConstraint,r,l_hand,[[0;0;1] [1;0;1]],tspan0);
q3 = repmat(randn(nq,1),1,3);
kc = WorldFixedPositionConstraint(r,l_hand,[[0;0;1] [1;0;1]],tspan0);
[c,dc] = kc.eval(t2,q3);
valuecheck(c,[0;0]);
testKinCnst_userfun(false,t2,q3,MultipleTimeKinematicConstraint.WorldFixedPositionConstraint,r,l_hand,[[0;0;1] [1;0;1]],tspan0);

display('Check world fixed orientation constraint')
testKinCnst_userfun(false,t2,q2,MultipleTimeKinematicConstraint.WorldFixedOrientConstraint,r,l_hand,tspan0);
kc = WorldFixedOrientConstraint(r,l_hand,tspan0);
[c,dc] = kc.eval(t2,q3);
valuecheck(c,3);
testKinCnst_userfun(false,t2,q3,MultipleTimeKinematicConstraint.WorldFixedOrientConstraint,r,l_hand,tspan0);

display('Check world fixed body posture constraint')
testKinCnst_userfun(false,t2,q2,MultipleTimeKinematicConstraint.WorldFixedBodyPostureConstraint,r,l_hand,tspan0);
kc = WorldFixedBodyPostureConstraint(r,l_hand,tspan0);
[c,dc] = kc.eval(t2,q3);
valuecheck(c,[0;3]);
testKinCnst_userfun(false,t2,q3,MultipleTimeKinematicConstraint.WorldFixedBodyPostureConstraint,r,l_hand,tspan0);

end

function testKinCnst_userfun(singleTimeFlag,t,q,kc_type,varargin)
robot = varargin{1};
robot_ptr = robot.getMexModelPtr;
if(singleTimeFlag)
  kc_mex = constructSingleTimeKinematicConstraint(kc_type,true,robot_ptr,varargin{2:end});
  kc = constructSingleTimeKinematicConstraint(kc_type,false,varargin{:});
  [num_cnst_mex,c_mex,dc_mex,cnst_name_mex,lb_mex,ub_mex] = testSingleTimeKinCnstmex(kc_mex,q,t);
else
  kc_mex = constructMultipleTimeKinematicConstraint(kc_type,true,robot_ptr,varargin{2:end});
  kc = constructMultipleTimeKinematicConstraint(kc_type,false,varargin{:});
  [num_cnst_mex,c_mex,dc_mex,cnst_name_mex,lb_mex,ub_mex] = testMultipleTimeKinCnstmex(kc_mex,q,t);
end
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