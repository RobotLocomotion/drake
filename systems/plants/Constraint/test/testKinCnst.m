function testKinCnst
urdf = [getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact.urdf'];
options.floating = true;
r = Atlas(urdf,options);
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
q = nom_data.xstar(1:nq)+0.1*randn(nq,1);
tspan0 = [0,1];
tspan1 = [];
t = 0.5;

q_rot1 = q;
q_rot1(6) = pi/2 + 0.01;
kinsol = doKinematics(r, q_rot1);
l_foot_pos = forwardKin(r, kinsol, l_foot, l_foot_pts, 2);

display('Check world CoM constraint');
testKinCnst_userfun(t,q,KinematicConstraint.WorldCoMConstraint,r,[0;0;0.9],[0;0;1],tspan0);
display('Check world CoM constraint with empty tspan');
testKinCnst_userfun(t,q,KinematicConstraint.WorldCoMConstraint,r,[0;0;0.9],[0;0;1]);
display('Check world CoM constraint with nan');
testKinCnst_userfun(t,q,KinematicConstraint.WorldCoMConstraint,r,[nan;nan;0.9],[nan;nan;0.92],tspan0/2);

display('Check world position constraint');
testKinCnst_userfun(t,q,KinematicConstraint.WorldPositionConstraint,r,l_foot,l_foot_pts,[-100*ones(2,4);zeros(1,4)],[100*ones(2,4);zeros(1,4)],tspan0);

display('Check world position constraint with nan');
testKinCnst_userfun(t,q,KinematicConstraint.WorldPositionConstraint,r,l_foot,l_foot_pts,[nan(2,4);zeros(1,4)],[nan(2,4);zeros(1,4)],tspan0);

display('Check world quaternion constraint')
testKinCnst_userfun(t,q,KinematicConstraint.WorldQuatConstraint,r,r_foot,[1;0;0;0],0.01,tspan0);

display('Check world quaternion constraint, a subtle case')
testKinCnst_userfun(t,q,KinematicConstraint.WorldQuatConstraint,r,l_foot,l_foot_pos(4:7,1),0,tspan0);

display('Check world euler constraint')
testKinCnst_userfun(t,q,KinematicConstraint.WorldEulerConstraint,r,l_foot,[0;nan;0.9*pi],[0;nan;1.1*pi],tspan0);

display('Check world gaze orientation constraint')
testKinCnst_userfun(t,q,KinematicConstraint.WorldGazeOrientConstraint,r,head,[1;0;0],[0.5;0.5;-0.5;0.5],0.3*pi,0.1*pi,tspan0);

display('Check world gaze orientation constraint with empty threshold')
testKinCnst_userfun(t,q,KinematicConstraint.WorldGazeOrientConstraint,r,head,[1;0;0],[0.5;0.5;-0.5;0.5],[],[],tspan0);

display('Check world gaze direction constraint')
testKinCnst_userfun(t,q,KinematicConstraint.WorldGazeDirConstraint,r,l_hand,[1;0;0],[0.5;0.3;0.4],0.3*pi,tspan0);

display('Check world gaze direction constraint with empty threshold')
testKinCnst_userfun(t,q,KinematicConstraint.WorldGazeDirConstraint,r,head,[1;0;0],[0.5;0.3;0.4],[],tspan0);

display('Check world gaze target constraint');
testKinCnst_userfun(t,q,KinematicConstraint.WorldGazeTargetConstraint,r,head,[1;0;0],[20;10;1],[0.3;0.1;0.2],0.3*pi,tspan0);

display('Check world gaze target constraint with empty threshold');
testKinCnst_userfun(t,q,KinematicConstraint.WorldGazeTargetConstraint,r,head,[1;0;0],[20;10;1],[0.3;0.1;0.2],[],tspan0);
end

function testKinCnst_userfun(t,q,kc_type,varargin)
robot = varargin{1};
robot_ptr = robot.getMexModelPtr;
kc_mex = constructKinematicConstraint(kc_type,true,robot_ptr,varargin{2:end});
kc = constructKinematicConstraint(kc_type,false,varargin{:});
[num_cnst_mex,c_mex,dc_mex,cnst_name_mex,lb_mex,ub_mex] = testKinCnstmex(kc_mex,q,t);
num_cnst = kc.getNumConstraint(t);
kinsol = kc.robot.doKinematics(q,false,false);
[c,dc] = kc.eval(t,kinsol);
[lb,ub] = kc.bounds(t);
cnst_name = kc.name(t);
valuecheck(num_cnst_mex,num_cnst);
valuecheck(c_mex,c,1e-8);
valuecheck(dc_mex,dc,1e-8);
valuecheck(lb_mex,lb,1e-8);
valuecheck(ub_mex,ub,1e-8);
end