function relativeEulerTest(visualize)
  import drakeFunction.*
  import drakeFunction.euclidean.*
  import drakeFunction.kinematic.*
  
  if nargin <1, visualize = false; end
  
  %% Initial Setup
  % Create the robot
  w = warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints');
  warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits');
  warning('off','Drake:RigidBodyManipulator:UnsupportedJointLimits');
  options.floating = true;
  urdf = fullfile(getDrakePath(),'examples','Atlas','urdf','atlas_minimal_contact.urdf');
  rbm = RigidBodyManipulator(urdf,options);
  warning(w);
  nq = rbm.getNumPositions();

  % Initialize visualization (if needed)
  if visualize
    lcmgl = LCMGLClient('relativePositionTest');
    v = rbm.constructVisualizer();
  end

  % Create short name for R^3
  R3 = drakeFunction.frames.realCoordinateSpace(3);

  % Load nominal posture
  S = load(fullfile(getDrakePath(),'examples','Atlas','data','atlas_fp.mat'));
  q_nom = S.xstar(1:nq);
  q0 = q_nom;
  
  %% Test a basic RelativeEuler object
  hand_orient_fcn = RelativeEuler(rbm,'l_hand','world');
  kinsol_matlab = rbm.doKinematics(q0,false,false);
  
  % Evaluate that DrakeFunction
  [rpy,J] = hand_orient_fcn.eval(kinsol_matlab);
  kinsol_mex = rbm.doKinematics(q0,false,true);
  [rpy,J] = hand_orient_fcn.eval(kinsol_mex);
  [~,J_geval] = geval(@(q) hand_orient_fcn.eval(doKinematics(rbm,q,false,true)),q0,struct('grad_method','numerical'));
  valuecheck(J,J_geval,1e-4);

  %% Test a nontrivial RelativeEuler object
  hand_orient_fcn = RelativeEuler(rbm,'l_hand','r_hand');
  kinsol_matlab = rbm.doKinematics(q0,false,false);
  [rpy,J] = hand_orient_fcn.eval(kinsol_matlab);
  kinsol_mex = rbm.doKinematics(q0,false,true);
  [~,J_geval] = geval(@(q) hand_orient_fcn.eval(doKinematics(rbm,q,false,true)),q0,struct('grad_method','numerical'));
  valuecheck(J,J_geval,1e-4);
end