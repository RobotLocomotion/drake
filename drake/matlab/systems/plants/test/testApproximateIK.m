function testApproximateIK()
options.floating = true;
options.dt = 0.001;
urdf = fullfile(getDrakePath,'examples','Atlas','urdf','atlas_minimal_contact.urdf');
w = warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits');
warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints');
r = RigidBodyManipulator(urdf,options);
warning(w);
v = r.constructVisualizer();
nq = r.getNumPositions();
cost = Point(r.getStateFrame,1);
cost.base_x = 0;
cost.base_y = 0;
cost.base_z = 0;
cost.base_roll = 1000;
cost.base_pitch = 1000;
cost.base_yaw = 0;
cost.back_bky = 100;
cost.back_bkx = 100;
cost = double(cost);
ikoption = IKoptions(r);
ikoption = ikoption.setQ(diag(cost(1:r.getNumPositions)));

nom_data = load('../../../../examples/Atlas/data/atlas_fp.mat');
q0 = nom_data.xstar(1:r.getNumPositions);
l_foot = r.findLinkId('l_foot');
r_foot = r.findLinkId('r_foot');
r_hand = r.findLinkId('r_hand');
l_hand = r.findLinkId('l_hand');
l_foot_pts = [0;0;0];
r_foot_pts = [0;0;0];
r_hand_pts = [0;0;0];
l_hand_pts = [0;0;0];
kinsol = doKinematics(r,q0);
com = getCOM(r,kinsol);
l_foot_pos0 = forwardKin(r, kinsol, l_foot, l_foot_pts, 0);
r_foot_pos0 = forwardKin(r, kinsol, r_foot, r_foot_pts, 0);
r_hand_pos0 = forwardKin(r, kinsol, r_hand, r_hand_pts, 0);
l_hand_pos0 = forwardKin(r, kinsol, l_hand, l_hand_pts, 0);
comdes = com+2e-3*rand(3,1);
q_seed = q0+1e-3*randn(nq,1);
kc_com = WorldCoMConstraint(r,comdes,comdes);
display('Check equality constraint only');
q = test_approximateIK_userfun(r,q_seed,q0,kc_com,ikoption);

kc_rfoot = WorldPositionConstraint(r,r_foot,r_foot_pts,r_foot_pos0,inf(3,1));
display('Check equality and lb constraint')
q = test_approximateIK_userfun(r,q_seed,q0,kc_com,kc_rfoot,ikoption);

display('Check equality,lb and ub constraint');
kc_lfoot = WorldPositionConstraint(r,l_foot,l_foot_pts,-inf(3,1),l_foot_pos0);
q = test_approximateIK_userfun(r,q_seed,q0,kc_com,kc_rfoot,kc_lfoot,ikoption);

display('Check equality, lb, ub and two-side constraints');
kc_rhand = WorldPositionConstraint(r,r_hand,r_hand_pts,r_hand_pos0-1e-3*rand(3,1),r_hand_pos0+1e-3*rand(3,1));
q = test_approximateIK_userfun(r,q_seed,q0,kc_com,kc_rfoot,kc_lfoot,kc_rhand,ikoption);
% v.draw(1,[q0;0*q0]); drawnow;

display('Check with posture constraint');
pc = PostureConstraint(r);
l_leg_kny = find(strcmp(r.getStateFrame.getCoordinateNames(),'l_leg_kny'));
r_leg_kny = find(strcmp(r.getStateFrame.getCoordinateNames(),'r_leg_kny'));
pc = pc.setJointLimits([l_leg_kny;r_leg_kny],[0.2;0.2],[inf;inf]);
q = test_approximateIK_userfun(r,q_seed,q0,kc_com,kc_rfoot,kc_lfoot,kc_rhand,ikoption);

display('Check with wrapup');
v.draw(1,[q0;0*q0]); drawnow;
q_rot0 = q0;
q_rot1 = q0;
q_rot0(6) = pi/2 - 0.01;
q_rot1(6) = pi/2 + 0.01;
l_foot = r.findLinkId('l_foot');
kinsol = doKinematics(r, q_rot1);
l_foot_pts = [0;0;0];
l_foot_pos = forwardKin(r, kinsol, l_foot, l_foot_pts, 1);
l_foot_pos_q = forwardKin(r, kinsol, l_foot, l_foot_pts, 2);
r_foot = r.findLinkId('r_foot');
kinsol = doKinematics(r, q_rot0);
r_foot_pts = [0;0;0];
r_foot_pos = forwardKin(r, kinsol, r_foot, r_foot_pts, 1);
kc1 = constructRigidBodyConstraint(RigidBodyConstraint.WorldCoMConstraintType,true,r,[0;0;nan],[0;0;nan]);
kc2 = constructRigidBodyConstraint(RigidBodyConstraint.WorldPositionConstraintType,true,r,l_foot,l_foot_pts,l_foot_pos(1:3,:),l_foot_pos(1:3,:));
kc3 = constructRigidBodyConstraint(RigidBodyConstraint.WorldEulerConstraintType,true,r,l_foot,l_foot_pos(4:6,1),l_foot_pos(4:6,1));
kc4 = constructRigidBodyConstraint(RigidBodyConstraint.WorldPositionConstraintType,true,r,r_foot,r_foot_pts,r_foot_pos(1:3,:),r_foot_pos(1:3,:));
kc5 = constructRigidBodyConstraint(RigidBodyConstraint.WorldEulerConstraintType,true,r,r_foot,r_foot_pos(4:6,1),r_foot_pos(4:6,1));
q = test_approximateIK_userfun(r,q_rot1,q_rot1,kc1,kc2,kc3,kc4,kc5,ikoption);
v.draw(1,[q;0*q]); drawnow;
kinsol = doKinematics(r, q);
l_foot_sol_q = forwardKin(r, kinsol, l_foot, l_foot_pts,2);
% low tolerances here
if ~valuecheck(l_foot_sol_q, l_foot_pos_q,1e-2) && ...
    ~valuecheck(-l_foot_pos_q, l_foot_pos_q,1e-2)
  info
  valuecheck(l_foot_sol_q, l_foot_pos_q,1e-2);  % note, could be either sign?
end

% Next, a case which exhibits the wraparound bug:
display('A case of wraparound bug');
v.draw(1,[q0;0*q0]); drawnow;
q_rot0 = q0;
q_rot1 = q0;
q_rot0(6) = pi - 0.01;
q_rot1(6) = -pi + 0.01;
l_foot = r.findLinkId('l_foot');
kinsol = doKinematics(r, q_rot1);
l_foot_pts = [0;0;0];
l_foot_pos = forwardKin(r, kinsol, l_foot, l_foot_pts, 1);
l_foot_pos_q = forwardKin(r, kinsol, l_foot, l_foot_pts, 2);
r_foot = r.findLinkId('r_foot');
kinsol = doKinematics(r, q_rot0);
r_foot_pts = [0;0;0];
r_foot_pos = forwardKin(r, kinsol, r_foot, r_foot_pts, 1);
kc1 = constructRigidBodyConstraint(RigidBodyConstraint.WorldCoMConstraintType,true,r,[0;0;nan],[0;0;nan]);
kc2 = constructRigidBodyConstraint(RigidBodyConstraint.WorldPositionConstraintType,true,r,l_foot,l_foot_pts,l_foot_pos(1:3,:),l_foot_pos(1:3,:));
kc3 = constructRigidBodyConstraint(RigidBodyConstraint.WorldEulerConstraintType,true,r,l_foot,l_foot_pos(4:6,1),l_foot_pos(4:6,1));
kc4 = constructRigidBodyConstraint(RigidBodyConstraint.WorldPositionConstraintType,true,r,r_foot,r_foot_pts,r_foot_pos(1:3,:),r_foot_pos(1:3,:));
kc5 = constructRigidBodyConstraint(RigidBodyConstraint.WorldEulerConstraintType,true,r,r_foot,r_foot_pos(4:6,1),r_foot_pos(4:6,1));
q = test_approximateIK_userfun(r,q_rot1,q_rot1,kc1,kc2,kc3,kc4,kc5,ikoption);
v.draw(1,[q;0*q]); drawnow;
kinsol = doKinematics(r, q);
l_foot_sol_q = forwardKin(r, kinsol, l_foot, l_foot_pts,2);
% low tolerances here
if ~valuecheck(l_foot_pos_q, l_foot_sol_q,1e-2) && ...
    ~valuecheck(-l_foot_pos_q, l_foot_sol_q,1e-2)
  info
  valuecheck(l_foot_sol_q, l_foot_pos_q,1e-2);
end
end

function q_app_mex = test_approximateIK_userfun(r,q_seed,q_nom,varargin)
ikoption = varargin{end};
ikoption = ikoption.setMex(true);
ikmexoption = ikoption;
ikmexoption = ikmexoption.setMex(true);
ikmatlaboption = ikoption;
ikmatlaboption = ikmatlaboption.setMex(false);
ikappoption = ikoption;
ikappoption = ikappoption.setMajorIterationsLimit(3);
ikappoption = ikappoption.setMajorOptimalityTolerance(1);
ikappoption = ikappoption.setMajorFeasibilityTolerance(1);
ikappoption = ikappoption.setMex(true);
display('IK mex')
tic
[q,info] = inverseKin(r,q_seed,q_nom,varargin{1:end-1},ikoption);
toc
if(info>10)
  error('IK fails to solve the problem');
end
display('Approximate IK mex');
tic
[q_app_mex,info] = approximateIK(r,q_seed,q_nom,varargin{1:end-1},ikmexoption);
toc
if(info~=0)
  error('approximateIKmex fails');
end
valuecheck(q_app_mex,q,3e-1);

% display('Approximate IK');
% tic
% [q_app,info] = approximateIK(r,q_seed,q_nom,varargin{1:end-1},ikmatlaboption);
% toc
% if(info~=0)
%   error('approximateIK fails');
% end
% valuecheck(q_app,q_app_mex,1e-4);
end
