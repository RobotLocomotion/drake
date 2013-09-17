function testIK
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
tspan = [0,1];
nom_data = load('../../../examples/Atlas/data/atlas_fp.mat');
q_nom = nom_data.xstar(1:nq);
q_seed = q_nom+0.1*randn(nq,1);
ikoptions = IKoptions(r);
ikoptions = ikoptions.setDebug(true);
ikoptions = ikoptions.setMex(false);
ikmexoptions = ikoptions;
ikmexoptions = ikmexoptions.setMex(true);

kc1 = WorldCoMConstraint(r,[0;0;0.9],[0;0;1],tspan);
kc1prime = WorldCoMConstraint(r,[nan;nan;0.9],[nan;nan;0.92],tspan/2);
display('Check a single CoM constraint')
test_IK_userfun(r,q_seed,q_nom,kc1,ikoptions);
display('Check IK fine sample with a single CoM constraint')
test_IKpointwise_userfun(r,[0,1],[q_seed q_seed],[q_nom q_nom],kc1,ikoptions);
display('Check IK fine sample with multiple CoM constraints')
test_IKpointwise_userfun(r,[0,1],[q_seed q_seed],[q_nom q_nom],kc1,kc1prime,ikoptions);

pc_knee = PostureConstraint(r,tspan);
l_knee_idx = find(~cellfun(@isempty,strfind(r.getJointNames,'l_leg_kny')));
r_knee_idx = find(~cellfun(@isempty,strfind(r.getJointNames,'r_leg_kny')));
pc_knee = pc_knee.setJointLimits([l_knee_idx;r_knee_idx],[0.2;0.2],[inf;inf]);
display('Check a single CoM constraint with a posture constraint')
test_IK_userfun(r,q_seed,q_nom,kc1,pc_knee,ikoptions);
display('Check IK fine sample with a single CoM constraint with a posture constraint')
test_IKpointwise_userfun(r,[0,1],[q_seed q_seed],[q_nom q_nom],kc1,pc_knee,ikoptions);
display('Check IK fine sample with multiple CoM constraints with a posture constraint')
test_IKpointwise_userfun(r,[0,1],[q_seed q_seed],[q_nom q_nom],kc1,kc1prime,pc_knee,ikoptions);

display('Check a body position constraint')
kc2l = WorldPositionConstraint(r,l_foot,l_foot_pts,[nan(2,4);zeros(1,4)],[nan(2,4);zeros(1,4)],tspan);
kc2r = WorldPositionConstraint(r,r_foot,r_foot_pts,[nan(2,4);zeros(1,4)],[nan(2,4);zeros(1,4)],tspan);
q = test_IK_userfun(r,q_seed,q_nom,kc1,pc_knee,kc2l,kc2r,ikoptions);
display('Check IK fine sample with body position constraint');
q = test_IKpointwise_userfun(r,[0,1],[q_seed q_seed],[q_nom q_nom],kc1,pc_knee,kc2l,kc2r,ikoptions);

display('Check the infeasible case')
kc_err = WorldCoMConstraint(r,[0;0;2],[0;0;inf],tspan);
[q,info,infeasible_constraint] = inverseKin(r,q_seed,q_nom,kc_err,kc2l,kc2r,ikoptions);
[qmex,info_mex,infeasible_constraint_mex] = inverseKin(r,q_seed,q_nom,kc_err,kc2l,kc2r,ikmexoptions);
valuecheck(info_mex,info);
if(info_mex ~= 13)
  error('This should be infeasible');
end
display('The infeasible constraints are');
disp(infeasible_constraint_mex);

display('Check a body quaternion constraint')
kc3 = WorldQuatConstraint(r,r_foot,[1;0;0;0],0,tspan);
q = test_IK_userfun(r,q_seed,q_nom,kc1,pc_knee,kc2l,kc2r,kc3,ikoptions);
display('Check IK fine sample with body orientation constraint');
q = test_IKpointwise_userfun(r,[0,1],[q_seed q_seed],[q_nom q_nom],kc1,pc_knee,kc2l,kc2r,kc3,ikoptions);

display('Check a gaze orientation constraint')
kc4 = WorldGazeOrientConstraint(r,r_hand,[1;0;0],[0.5;0.5;-0.5;0.5],0.1*pi,0.8*pi,tspan);
q = test_IK_userfun(r,q_seed,q_nom,kc1,pc_knee,kc2l,kc2r,kc3,kc4,ikoptions);
display('Check IK fine sample with gaze orientation');
q = test_IKpointwise_userfun(r,[0,1],[q_seed q_seed],[q_nom q_nom],kc1,pc_knee,kc2l,kc2r,kc3,kc4,ikoptions);

display('Check a gaze direction constraint')
kc5 = WorldGazeDirConstraint(r,l_hand,[1;0;0],[1;0;0],0.2*pi,tspan);
q = test_IK_userfun(r,q_seed,q_nom,kc1,pc_knee,kc2l,kc2r,kc3,kc4,kc5,ikoptions);
display('Check IK fine sample with gaze direction Constraint');
q = test_IKpointwise_userfun(r,[0,1],[q_seed q_seed],[q_nom q_nom],kc1,pc_knee,kc2l,kc2r,kc3,kc4,kc5,ikoptions);

display('Check a gaze target constraint')
kc6 = WorldGazeTargetConstraint(r,head,[1;0;0],[1;0;1.5],[0.1;0.2;0.3],0.1*pi,tspan);
q = test_IK_userfun(r,q_seed,q_nom,kc1,kc2l,kc2r,kc3,kc4,kc5,kc6,ikoptions);
display('Check IK fine sample with gaze target constraint');
q = test_IKpointwise_userfun(r,[0,1],[q_seed q_seed],[q_nom q_nom],kc1,pc_knee,kc2l,kc2r,kc3,kc4,kc5,kc6,ikoptions);

display('Check quasi static constraint')
qsc = QuasiStaticConstraint(r);
qsc = qsc.addContact(r_foot,r_foot_pts);
qsc = qsc.addContact(l_foot,l_foot_pts);
qsc = qsc.setActive(true);
ikoptions = ikoptions.setMex(false);
q = test_IK_userfun(r,q_seed,q_nom,kc1,qsc,kc2l,kc2r,kc3,kc4,kc5,kc6,ikoptions);
display('Check quasi static constraint for fine sample');
q = test_IKpointwise_userfun(r,[0,1],[q_seed q_seed],[q_nom q_nom],kc1,qsc,kc2l,kc2r,kc3,kc4,kc5,kc6,ikoptions);

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
[qmex,info] = inverseKin(r,q_seed,q_nom,varargin{1:end-1},ikmexoptions);
toc
if(info>10)
  error('SNOPT info is %d, IK mex fails to solve the problem',info);
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
  error('SNOPT info is %d, IK fine samplefails to solve the problem',info);
end
tic
[qmex,info] = inverseKinPointwise(r,t,q_seed,q_nom,varargin{1:end-1},ikmexoptions);
toc
if(info>10)
  error('SNOPT info is %d, IK fine sample mex fails to solve the problem',info);
end
valuecheck(q,qmex,1e-2);
end
