function testIKoptions()
% Check if the MATLAB and C++ IKoptions are consistent
warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints');
urdf = [getDrakePath,'/examples/Atlas/urdf/atlas_minimal_contact.urdf'];
w = warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits');
warning('off','Drake:RigidBody:SimplifiedCollisionGeometry');
robot = RigidBodyManipulator(urdf,struct('floating',true));
warning(w);

nq = robot.getNumPositions();

% Check if the default properties are the same.
ikoptions = IKoptions(robot);
checkIKoptions(ikoptions);
display('Default properties match');

% check Q
Q = randn(nq,nq);
Q = Q'*diag(abs(randn(1,nq)))*Q;
ikoptions = ikoptions.setQ(Q);
checkIKoptions(ikoptions);
valuecheck(ikoptions.Q,Q,1e-10);
display('setQ pass');

% Check setQv
Qv = randn(nq,nq);
Qv = Qv'*diag(abs(randn(1,nq)))*Qv;
ikoptions = ikoptions.setQv(Qv);
checkIKoptions(ikoptions);
valuecheck(ikoptions.Qv,Qv,1e-10);
display('setQv pass');

% Check setQa
Qa = randn(nq,nq);
Qa = Qa'*diag(abs(randn(1,nq)))*Qa;
ikoptions = ikoptions.setQa(Qa);
valuecheck(ikoptions.Qa,Qa,1e-10);
checkIKoptions(ikoptions);
display('setQa pass');

% Check setDebug
ikoptions = ikoptions.setDebug(~ikoptions.debug_mode);
checkIKoptions(ikoptions);
display('setDebug pass');

% Check sequentialSeedFlag
ikoptions = ikoptions.setSequentialSeedFlag(~ikoptions.sequentialSeedFlag);
checkIKoptions(ikoptions);
display('setSequentialSeedFlag pass');

% Check setMajorOptimalityTolerance
ikoptions = ikoptions.setMajorOptimalityTolerance(0.1*ikoptions.SNOPT_MajorOptimalityTolerance);
checkIKoptions(ikoptions);
display('setMajorOptimalityTolerance pass');

% Check setSuperbasicsLimit
ikoptions = ikoptions.setSuperbasicsLimit(2*ikoptions.SNOPT_SuperbasicsLimit);
checkIKoptions(ikoptions);
display('setSuperbasicsLimit pass');

% Check setMajorIterationsLimit
ikoptions = ikoptions.setMajorIterationsLimit(100*ikoptions.SNOPT_MajorIterationsLimit);
checkIKoptions(ikoptions);
display('setMajorIterationsLimit pass');

% Check setIterationsLimit
ikoptions = ikoptions.setIterationsLimit(100*ikoptions.SNOPT_IterationsLimit);
checkIKoptions(ikoptions);
display('setIterationsLimit pass');

% Check setFixInitialState
ikoptions = ikoptions.setFixInitialState(~ikoptions.fixInitialState);
checkIKoptions(ikoptions);
display('setFixInitialState pass');

% Check setq0
ikoptions = ikoptions.setq0(-rand(nq,1),rand(nq,1));
checkIKoptions(ikoptions);
display('setq0 pass');

% Check setqd0
ikoptions = ikoptions.setqd0(-rand(nq,1),rand(nq,1));
checkIKoptions(ikoptions);
display('setqd0 pass');

% Check setqdf
ikoptions = ikoptions.setqdf(-rand(nq,1),rand(nq,1));
checkIKoptions(ikoptions);
display('setqdf pass');

% Check setAdditionaltSamples
ikoptions = ikoptions.setAdditionaltSamples([]);
checkIKoptions(ikoptions);
valuecheck(ikoptions.additional_tSamples,[]);
ikoptions = ikoptions.setAdditionaltSamples([-1 -1 -2 0 1]);
checkIKoptions(ikoptions);
valuecheck(ikoptions.additional_tSamples,[-2 -1 0 1]);
display('setAdditionaltSamples pass');

% check use_rbm_joint_bnd
% Check updateRobot
urdf_new = [getDrakePath,'/examples/PR2/pr2.urdf'];
w = warning('off','Drake:RigidBodyManipulator:BodyHasZeroInertia');
robot_new = RigidBodyManipulator(urdf_new,struct('floating',true));
warning(w);
ikoptions = ikoptions.updateRobot(robot_new);
checkIKoptions(ikoptions);
display('updateRobot pass');
end

function checkIKoptions(ikoptions)

if ikoptions.mex_ptr==0, return; end  % can't test without mex

ikoptions_mex = ikoptions.mex_ptr;
[robot_address_mex,Q_mex,Qa_mex,Qv_mex,debug_mode_mex,sequentialSeedFlag_mex,...
  majorFeasibilityTolerance_mex,majorIterationsLimit_mex,...
  iterationsLimit_mex,superbasicsLimit_mex,majorOptimalityTolerance_mex,...
  additional_tSamples_mex,...
  fixInitialState_mex,q0_lb_mex,q0_ub_mex,qd0_lb_mex,qd0_ub_mex,qdf_lb_mex,qdf_ub_mex]...
  = testIKoptionsmex(ikoptions_mex);
valuecheck(ikoptions.robot.getMexModelPtr.ptr,robot_address_mex);
valuecheck(ikoptions.Q,Q_mex,1e-10);
valuecheck(ikoptions.Qa,Qa_mex,1e-10);
valuecheck(ikoptions.Qv,Qv_mex,1e-10);
valuecheck(ikoptions.debug_mode,debug_mode_mex);
valuecheck(ikoptions.sequentialSeedFlag,sequentialSeedFlag_mex);
valuecheck(ikoptions.SNOPT_MajorFeasibilityTolerance, majorFeasibilityTolerance_mex,1e-10);
valuecheck(ikoptions.SNOPT_MajorIterationsLimit, majorIterationsLimit_mex);
valuecheck(ikoptions.SNOPT_IterationsLimit,iterationsLimit_mex);
valuecheck(ikoptions.SNOPT_SuperbasicsLimit,superbasicsLimit_mex);
valuecheck(ikoptions.SNOPT_MajorOptimalityTolerance,majorOptimalityTolerance_mex);
valuecheck(ikoptions.additional_tSamples,additional_tSamples_mex,1e-10);
valuecheck(ikoptions.fixInitialState,fixInitialState_mex);
valuecheck(ikoptions.q0_lb,q0_lb_mex);
valuecheck(ikoptions.q0_ub,q0_ub_mex);
valuecheck(ikoptions.qd0_lb,qd0_lb_mex);
valuecheck(ikoptions.qd0_ub,qd0_ub_mex);
valuecheck(ikoptions.qdf_lb,qdf_lb_mex);
valuecheck(ikoptions.qdf_ub,qdf_ub_mex);
end
