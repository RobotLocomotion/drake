function testIKoptions()
% Check if the MATLAB and C++ IKoptions are consistent
warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints');
warning('off','Drake:RigidBodyManipulator:UnsupportedJointLimits');
urdf = [getDrakePath,'/examples/Atlas/urdf/atlas_minimal_contact.urdf'];
w = warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits');
warning('off','Drake:RigidBody:SimplifiedCollisionGeometry');
robot = RigidBodyManipulator(urdf,struct('floating',true));
warning(w);

nq = robot.getNumDOF();

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
end

function checkIKoptions(ikoptions)
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