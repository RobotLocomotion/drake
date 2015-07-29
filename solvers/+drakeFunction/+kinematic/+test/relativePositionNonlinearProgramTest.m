function relativePositionNonlinearProgramTest(visualize)
  % fmincon doesn't solve this problem in a timely manner, so check that
  % we can use snopt here.
  checkDependency('snopt');

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

  %% Solve an inverse kinematics problem
  
  % Create a DrakeFunction that computes the world position of the hand points
  hand_pts_in_body = [0;0.2;0];
  hand_pts_fcn = RelativePosition(rbm,'l_hand','world',hand_pts_in_body);

  %% Constraints on body points
  % Create a DrakeFunction that computes the signed distance from a point to
  % the xy-plane
  single_pt_dist_to_ground = SignedDistanceToHyperplane([0;0;0], [0;0;1]);

  %% Enforce that the corners of the left foot be on the ground
  % Create a DrakeFunction that computes the world positions of foot points
  foot_pts_in_body = rbm.getBody(rbm.findLinkId('l_foot')).getTerrainContactPoints();
  foot_pts_fcn = RelativePosition(rbm,'l_foot','world',foot_pts_in_body);

  % Create a DrakeFunction that computes the signed distances from m points to
  % the xy-plane, where m = foot_pts_fcn.n_pts
  dist_to_ground_fcn = duplicate(single_pt_dist_to_ground,foot_pts_fcn.n_pts);

  % Create an constraint mandating that the signed distances between the foot
  % points and the ground be zero
  lb = zeros(foot_pts_fcn.n_pts,1);
  ub = lb;
  foot_on_ground_cnstr = DrakeFunctionConstraint(lb,ub,dist_to_ground_fcn(foot_pts_fcn));

  %% Enforce that the hand points be on the ground
  % Create a DrakeFunction that computes the signed distances from m points to
  % the xy-plane, where m = hand_pts_fcn.n_pts
  dist_to_ground_fcn = duplicate(single_pt_dist_to_ground,hand_pts_fcn.n_pts);

  % Create an constraint mandating that the signed distances between the foot
  % points and the ground be zero
  lb = zeros(hand_pts_fcn.n_pts,1);
  ub = lb;
  hand_on_ground_cnstr = DrakeFunctionConstraint(lb,ub,dist_to_ground_fcn(hand_pts_fcn));

  %% Enforce that the projection of the COM onto the xy-plane be within the
  %% convex hull of the foot points

  % Create a DrakeFunction that computes the COM position
  com_fcn = RelativePosition(rbm,0,'world');  % warning: this is not supported... findKinematicPathmex will (quietly) complain and get the sparsity pattern wrong
  % Add unused inputs for the weights
  com_fcn = addInputs(com_fcn, foot_pts_fcn.n_pts);

  % Create a DrakeFunction that computes a linear combination of points in R3
  % given the points and the weights as inputs
  lin_comb_of_pts = LinearCombination(foot_pts_fcn.n_pts, 3);
  % Create a DrakeFUnction that computes a linear combination of the foot
  % points given joint-angles and weights as inputs
  lin_comb_of_foot_pts = lin_comb_of_pts([foot_pts_fcn;Identity(foot_pts_fcn.n_pts)]);

  % Create a DrakeFunction that computes the difference between the COM
  % position computed from the joint-angles and the linear combination of the
  % foot points.
  support_polygon{1} = minus(com_fcn,lin_comb_of_foot_pts,true);
  support_polygon{2} = Identity(foot_pts_fcn.n_pts);

  % Create a DrakeFunction that computes the sum of the weights
  support_polygon{3} = Linear(ones(1,foot_pts_fcn.n_pts));

  % Create quasi-static constraints
  % xy-coordinates of COM must match a linear combination of the foot points
  % for some set of weights 
  qsc_cnstr{1} = DrakeFunctionConstraint([0;0;-Inf],[0;0;Inf],support_polygon{1});

  % The weights must be between 0 and 1
  lb = zeros(foot_pts_fcn.n_pts,1);
  ub = ones(foot_pts_fcn.n_pts,1);
  qsc_cnstr{2} = DrakeFunctionConstraint(lb,ub,support_polygon{2});

  % The weights must sum to 1
  qsc_cnstr{3} = DrakeFunctionConstraint(1,1,support_polygon{3});

  Q = eye(numel(q_nom));
  prog = InverseKinematics(rbm,q_nom);
  prog = prog.setQ(Q);
  q_inds = prog.q_idx;
  w_inds = reshape(prog.num_vars+(1:foot_pts_fcn.n_pts),[],1);
  prog = prog.addDecisionVariable(numel(w_inds));

  prog = prog.addConstraint(foot_on_ground_cnstr,q_inds);
  prog = prog.addConstraint(hand_on_ground_cnstr,q_inds);

  prog = prog.addConstraint(qsc_cnstr{1},[q_inds;w_inds]);
  prog = prog.addConstraint(qsc_cnstr{2},w_inds);
  prog = prog.addConstraint(qsc_cnstr{3},w_inds);

  time_prog = tic;
  [q,F,info] = solve(prog,q0);
  display(snoptInfo(info))
  toc(time_prog);

  if visualize
    kinsol = doKinematics(rbm,q);
    com = getCOM(rbm,kinsol);
    lcmgl.glColor3f(1,0,0);
    lcmgl.sphere([com(1:2);0],0.02,20,20);
    lcmgl.switchBuffers();
    v.draw(0,q);
  end
