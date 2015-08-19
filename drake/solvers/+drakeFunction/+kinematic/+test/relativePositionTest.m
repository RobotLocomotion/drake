function relativePositionTest(visualize)
  import drakeFunction.*
  import drakeFunction.euclidean.*
  import drakeFunction.kinematic.*

  if nargin < 1, visualize = false; end

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

  % Load nominal posture
  S = load(fullfile(getDrakePath(),'examples','Atlas','data','atlas_fp.mat'));
  q_nom = S.xstar(1:nq);
  q0 = q_nom;

  %% Test a basic RelativePosition object
  % Create a DrakeFunction that computes the world position of the hand points
  hand_pts_in_body = [0;0.2;0];
  hand_pts_fcn = RelativePosition(rbm,'l_hand','world',hand_pts_in_body);

  % Evaluate that DrakeFunction
  [pos,J] = hand_pts_fcn(q0);

  if visualize
    lcmgl.glColor3f(1,0,0);
    for pt = reshape(pos,3,[])
      lcmgl.sphere(pt,0.05,20,20);
    end
    rbm.drawLCMGLAxes(lcmgl,q0,rbm.findLinkId('l_hand'));
    lcmgl.switchBuffers();
    v.draw(0,q0);
  end

  % Check the gradients of the DrakeFunction
  [f,df] = geval(@(q) eval(hand_pts_fcn,q),q0,struct('grad_method',{{'user','taylorvar'}},'tol',1e-6));
end
