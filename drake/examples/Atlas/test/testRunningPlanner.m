function [sol,robot_vis,v,cdfkp] = testRunningPlanner(seed,stride_length,major_iteration_limit, suffix,options)
  checkDependency('lcmgl');
  if nargin < 1, seed = []; end
  if (nargin < 2 || isempty(stride_length)), stride_length = 2; end
  if (nargin < 3 || isempty(major_iteration_limit)), major_iteration_limit = 200; end
  if (nargin < 4 || isempty(suffix)), suffix = 'testRunning'; end
  if (nargin < 5 || isempty(options))
    options = defaultOptionsStruct();
  else
    options = parseOptionsStruct(options);
  end

  % Construct RigidBodyManipulator
  w = warning('off','Drake:RigidBody:SimplifiedCollisionGeometry');
  warning('off','Drake:RigidBody:NonPositiveInertiaMatrix');
  warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints');
  warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits');
  options.floating = true;
  options.terrain = RigidBodyFlatTerrain;
  atlas_urdf = [getDrakePath,'/examples/Atlas/urdf/atlas_convex_hull.urdf'];
  atlas_vis_urdf = [getDrakePath,'/examples/Atlas/urdf/atlas_minimal_contact.urdf'];
  robot = RigidBodyManipulator(atlas_urdf,options);
  robot = robot.replaceCollisionGeometryWithConvexHull(robot.findLinkId('l_hand'),1);
  robot = robot.replaceCollisionGeometryWithConvexHull(robot.findLinkId('r_hand'),1);
  robot = compile(robot);
  robot_vis = RigidBodyManipulator(atlas_vis_urdf,options);
  if options.add_obstacle
    %robot = addObstacle(robot,0.7*(stride_length/2));
    %robot_vis = addObstacle(robot_vis,0.7*(stride_length/2));
    robot = addObstacle(robot,0);
    robot_vis = addObstacle(robot_vis,0);
  end
  %mu = 1.16; % rubber on rubber
  mu = 1; % rubber on rubber
  warning(w);


  % Create convenience variables
  nq = robot.getNumPositions();
  %nv = robot.getNumVelocities();
  %world = robot.findLinkId('world');
  l_foot = robot.findLinkId('l_foot');
  r_foot = robot.findLinkId('r_foot');
  l_uleg = robot.findLinkId('l_uleg');
  r_uleg = robot.findLinkId('r_uleg');
  head = robot.findLinkId('head');
  neck_idx = robot.getBody(robot.findJointId('neck_ay')).position_num;
  freeze_idx = neck_idx;
  r_foot_bottom = robot.getBody(r_foot).getTerrainContactPoints();
  l_foot_toe    = robot.getBody(l_foot).getTerrainContactPoints('toe');
  l_foot_heel   = robot.getBody(l_foot).getTerrainContactPoints('heel');
  %arm_idx = findPositionIndices(robot,'arm');
  %leg_idx = findPositionIndices(robot,'leg');
  %back_idx = findPositionIndices(robot,'back');
  %wrist_idx = [findPositionIndices(robot,'uwy');findPositionIndices(robot,'mwx');findPositionIndices(robot,'ely');findPositionIndices(robot,'shx')];

  % Construct visualization tools
  v = constructVisualizer(robot_vis);

  % Load nominal data
  nomdata = load([getDrakePath(), '/examples/Atlas/data/atlas_fp.mat']);
  qstar = nomdata.xstar(1:nq);
  v.draw(0,qstar);
  q0 = qstar;
  q0(neck_idx) = 0;
  if options.start_from_standing
    assert(~isempty(options.stride_filename));
    S = load(options.stride_filename);
    options.q_apex = S.sol.q(:,end);
    options.comdot_apex = S.sol.comdot(:,end);
    qf = options.q_apex;
    qf(1) = qf(1)/2;
  elseif options.add_obstacle
    assert(~isempty(options.stride_filename));
    S = load(options.stride_filename);
    options.q_apex = S.sol.q(:,end);
    options.comdot_0 = S.sol.comdot(:,1);
    options.comdot_apex = S.sol.comdot(:,end);
    options.com_0 = S.sol.com(:,1);
    options.com_apex = S.sol.com(:,end);
    q0 = S.sol.q(:,1);
    %q0(1) = -stride_length/4;
    q0(1) = -1.2;
    qf = options.q_apex;
    qf(1) = 0.8;
    H0 = S.sol.H(:,1);
    Hf = S.sol.H(:,end);
    Hdot0 = S.sol.Hdot(:,1);
    Hdotf = S.sol.Hdot(:,end);
  else
    qf = q0;
    qf(1) = stride_length/2;
  end
  com_0 = robot.getCOM(q0);
  com_f = robot.getCOM(qf);

  % Set up time limits
  if options.start_from_standing
    N = 19;
  else
    %N = 16;
    N = options.N;
  end
  T = stride_length/2/options.speed;
  if options.start_from_standing
    tf_range = [0,10*T];
    h_min = 1/(2*N)*T; h_max_stance = 2/N*T; h_max_flight = 2/N*T;
  elseif options.add_obstacle
    tf_range = [0,10*T];
    h_min = 1/(2*N)*T; h_max_stance = 2/N*T; h_max_flight = 2/N*T;
  else
    tf_range = T*[1,1];
    h_min = 3/(4*N)*T; h_max_stance = 3/(2*N)*T; h_max_flight = 2/N*T;
  end

  % Set up cost variables
  q_nom = bsxfun(@times,qstar,ones(1,N));
  Q = eye(nq);
  Q(1,1) = 0;
  Q(2,2) = 0;
  Q(6,6) = 0;
  Qv = 1e0*eye(nq);
  %Qv(arm_idx,arm_idx) = 1e1*eye(numel(arm_idx));
  Q_comddot = diag([1,1,1]);
  Q_contact_force = 0e-4*eye(3);

  % Create collision avoidance constraints
  if options.enable_collision
    % Consider all bodies
    min_distance_constraint.flight = MinDistanceConstraint(robot,options.min_distance);
    if options.start_from_standing
      % Ignore both feet
      active_collision_options.body_idx = setdiff(1:robot.getNumBodies(),[l_foot,r_foot,l_uleg,r_uleg]);
      min_distance_constraint.double_stance = MinDistanceConstraint(robot,options.min_distance,active_collision_options);
      % Ignore left foot
      active_collision_options.body_idx = setdiff(1:robot.getNumBodies(),[l_foot]);
      min_distance_constraint.stance = MinDistanceConstraint(robot,options.min_distance,active_collision_options);
    else
      % Ignore left foot
      active_collision_options.body_idx = setdiff(1:robot.getNumBodies(),l_foot);
      min_distance_constraint.stance = MinDistanceConstraint(robot,options.min_distance,active_collision_options);
    end
    if options.n_interp_points > 0
      interp_min_distance_constraint.flight = ...
        generateInterpolatedMinDistanceConstraint(min_distance_constraint.stance,(1:options.n_interp_points)/(options.n_interp_points+1));
      interp_min_distance_constraint.stance = ...
        generateInterpolatedMinDistanceConstraint(min_distance_constraint.stance,(1:options.n_interp_points)/(options.n_interp_points+1));
      if options.start_from_standing
        interp_min_distance_constraint.double_stance = ...
          generateInterpolatedMinDistanceConstraint(min_distance_constraint.double_stance,(1:options.n_interp_points)/(options.n_interp_points+1));
      end
    end
  end

  % Create gaze constraint
  if options.constrain_head_gaze
    gaze_constraint = WorldGazeDirConstraint(robot,head,[1;0;0],[1;0;0],options.head_gaze_tol);
  end

  % Set up linearized friction cone edges
  num_edges = 3;
  FC_angles = linspace(0,2*pi,num_edges+1);FC_angles(end) = [];
  FC_axis = [0;0;1];
  FC_perp1 = rotx(pi/2)*FC_axis;
  FC_perp2 = cross(FC_axis,FC_perp1);
  FC_edge = bsxfun(@plus,FC_axis,mu*(bsxfun(@times,cos(FC_angles),FC_perp1) + ...
    bsxfun(@times,sin(FC_angles),FC_perp2)));
  FC_edge = robot.getMass()*norm(robot.getGravity)*bsxfun(@rdivide,FC_edge,sqrt(sum(FC_edge.^2,1)));

  % Create stance constraints
  if options.start_from_standing
      in_stance.right = 1:11;
      in_stance.toe  = 1:17;
      in_stance.heel = 1:14;
  else
    in_stance.right = [];
    if options.toe_first
      in_stance.toe = 5:13;
      in_stance.heel = 8:10;
    else
      %in_stance.toe = 5:13;
      %in_stance.heel = 5:10;
      touch_down_idx = ceil(N/4);
      heel_off_idx = ceil(3*N/4);
      toe_off_idx = ceil(4*N/5);
      in_stance.toe = touch_down_idx:toe_off_idx;
      in_stance.heel = touch_down_idx:heel_off_idx;
    end
  end
  in_stance.left = union(in_stance.toe,in_stance.heel);
  in_stance.left = setdiff(in_stance.left,in_stance.right);
  in_stance.total = union(in_stance.left,in_stance.right);
  in_stance.double = intersect(in_stance.left,in_stance.right);
  in_flight = setdiff(1:N,in_stance.total);

  contact_wrench_struct(1).active_knot = in_stance.toe(2:end);
  contact_wrench_struct(1).cw = ...
    LinearFrictionConeWrench(robot,l_foot,l_foot_toe,FC_edge);
  contact_wrench_struct(2).active_knot = in_stance.heel(2:end);
  contact_wrench_struct(2).cw = ...
    LinearFrictionConeWrench(robot,l_foot,l_foot_heel,FC_edge);
  if options.start_from_standing
    contact_wrench_struct(3).active_knot = in_stance.right;
    contact_wrench_struct(3).cw = ...
      LinearFrictionConeWrench(robot,r_foot,r_foot_bottom,FC_edge);
  end

  % Kinematic constraints
  lb = repmat([NaN; NaN; 0],1,2);
  ub = repmat([NaN; NaN; 0],1,2);
  position_cnstr.toe   = WorldPositionConstraint(robot,l_foot,l_foot_toe,lb,ub);
  position_cnstr.heel   = WorldPositionConstraint(robot,l_foot,l_foot_heel,lb,ub);
  stationary_cnstr.toe = WorldFixedPositionConstraint(robot,l_foot,l_foot_toe);
  stationary_cnstr.heel = WorldFixedPositionConstraint(robot,l_foot,l_foot_heel);
  stance_clearance_cnstr.toe = WorldPositionConstraint(robot,l_foot,l_foot_toe,lb,NaN(size(lb)));
  stance_clearance_cnstr.heel = WorldPositionConstraint(robot,l_foot,l_foot_heel,lb,NaN(size(lb)));
  if options.start_from_standing
    lb = repmat([NaN; NaN; 0],1,4);
    ub = repmat([NaN; NaN; 0],1,4);
    position_cnstr.right   = WorldPositionConstraint(robot,r_foot,r_foot_bottom,lb,ub);
    stationary_cnstr.right = WorldFixedBodyPoseConstraint(robot,r_foot);
  end

  % Create trajectory optimization
  cdfkp = ComDynamicsFullKinematicsPlanner(robot,N,tf_range,Q_comddot,Qv,Q,q_nom,Q_contact_force,contact_wrench_struct,options);
  if options.visualize
    cdfkp = cdfkp.addDisplayFunction(@(q)displayCallback(in_stance.total,N,q),[cdfkp.h_inds(:);reshape(cdfkp.com_inds(3,:),[],1);reshape(cdfkp.comdot_inds(1,:),[],1);cdfkp.q_inds(:)]);
  end

  % Add Timestep bounds
  cdfkp = cdfkp.addBoundingBoxConstraint(BoundingBoxConstraint(h_min*ones(numel(in_flight)-1,1),h_max_flight*ones(numel(in_flight)-1,1)),cdfkp.h_inds(in_flight(1:end-1)));
  cdfkp = cdfkp.addBoundingBoxConstraint(BoundingBoxConstraint(h_min*ones(numel(in_stance.total),1),h_max_stance*ones(numel(in_stance.total),1)),cdfkp.h_inds(in_stance.total(1:end)));

  % Add Velocity constriants
  if options.enable_velocity_limits
    arm_v_max = pi;
    cdfkp = cdfkp.addBoundingBoxConstraint(BoundingBoxConstraint(-arm_v_max*ones(numel(arm_idx),N),arm_v_max*ones(numel(arm_idx),N)),cdfkp.v_inds(arm_idx,:));
    leg_v_max = 3*pi;
    cdfkp = cdfkp.addBoundingBoxConstraint(BoundingBoxConstraint(-leg_v_max*ones(numel(leg_idx),N),leg_v_max*ones(numel(leg_idx),N)),cdfkp.v_inds(leg_idx,:));
    back_v_max = 2*pi;
    cdfkp = cdfkp.addBoundingBoxConstraint(BoundingBoxConstraint(-back_v_max*ones(numel(back_idx),N),back_v_max*ones(numel(back_idx),N)),cdfkp.v_inds(back_idx,:));
    fb_v_max = 5;
    cdfkp = cdfkp.addBoundingBoxConstraint(BoundingBoxConstraint(-fb_v_max*ones(3,N),fb_v_max*ones(3,N)),cdfkp.v_inds(1:3,:));
  end

  % Add initial contition constraints
  if options.start_from_standing
    cdfkp = cdfkp.addConstraint(ConstantConstraint([0;0]), ...
      cdfkp.com_inds(1:2,1));
    cdfkp = cdfkp.addConstraint(ConstantConstraint(q0(3:end)), ...
      cdfkp.q_inds(3:end,1));
    cdfkp = cdfkp.addConstraint(ConstantConstraint(zeros(3,1)), ...
      cdfkp.comdot_inds(1:3,1));
  else
    if options.add_obstacle
      %options.com_tol = 0.2;
      %cdfkp = cdfkp.addConstraint(BoundingBoxConstraint(q0(3:end)-options.joint_tol,q0(3:end)+options.joint_tol), ...
      %cdfkp.q_inds(3:end,1));
      not_arm_idx = setdiff(3:robot.getNumPositions(),arm_idx);
      cdfkp = cdfkp.addConstraint(BoundingBoxConstraint(q0(arm_idx)-2*options.joint_tol,q0(arm_idx)+2*options.joint_tol), ...
        cdfkp.q_inds(arm_idx,1));
      cdfkp = cdfkp.addConstraint(BoundingBoxConstraint(q0(not_arm_idx)-options.joint_tol,q0(not_arm_idx)+options.joint_tol), ...
        cdfkp.q_inds(not_arm_idx,1));
      cdfkp = cdfkp.addConstraint(ConstantConstraint(H0), ...
        cdfkp.H_inds(:,1));
      cdfkp = cdfkp.addConstraint(ConstantConstraint(Hdot0), ...
        cdfkp.Hdot_inds(:,1));
      cdfkp = cdfkp.addConstraint(BoundingBoxConstraint(-Inf,q0(1)), ...
        cdfkp.q_inds(1,1));
      cdfkp = cdfkp.addConstraint(BoundingBoxConstraint(options.comdot_0 - options.com_tol*abs(options.comdot_0),options.comdot_0 + options.com_tol*abs(options.comdot_0)), ...
        cdfkp.comdot_inds(:,1));
      %cdfkp = cdfkp.addConstraint(LinearConstraint(0,Inf,[1,-1]), ...
      %cdfkp.comdot_inds(3,[1,2]));
      cdfkp = cdfkp.addConstraint(BoundingBoxConstraint(options.com_0(3) - options.com_tol*abs(options.com_0(3)),options.com_0(3) + options.com_tol*abs(options.com_0(3))), ...
        cdfkp.com_inds(3,1));
    else
      cdfkp = cdfkp.addConstraint(ConstantConstraint([0;0]), ...
        cdfkp.com_inds(1:2,1));
    end
    cdfkp = cdfkp.addConstraint(ConstantConstraint(0), ...
      cdfkp.comdot_inds(3,1));
  end

  % Add final condition constraints
  if options.start_from_standing
    cdfkp = cdfkp.addConstraint(ConstantConstraint(options.comdot_apex), ...
      cdfkp.comdot_inds(:,end));
    cdfkp = cdfkp.addConstraint(ConstantConstraint(options.q_apex(2:end)), ...
      cdfkp.q_inds(2:end,end));
  elseif options.add_obstacle
    cdfkp = cdfkp.addConstraint(BoundingBoxConstraint(qf(arm_idx)-2*options.joint_tol,qf(arm_idx)+2*options.joint_tol), ...
      cdfkp.q_inds(arm_idx,end));
    cdfkp = cdfkp.addConstraint(BoundingBoxConstraint(qf(not_arm_idx)-options.joint_tol,qf(not_arm_idx)+options.joint_tol), ...
      cdfkp.q_inds(not_arm_idx,end));
    cdfkp = cdfkp.addConstraint(BoundingBoxConstraint(options.comdot_apex - options.com_tol*abs(options.comdot_apex),options.comdot_apex + options.com_tol*abs(options.comdot_apex)), ...
      cdfkp.comdot_inds(:,end));
    cdfkp = cdfkp.addConstraint(BoundingBoxConstraint(qf(1),Inf), ...
      cdfkp.q_inds(1,end));
    cdfkp = cdfkp.addConstraint(ConstantConstraint(Hf), ...
      cdfkp.H_inds(:,end));
    cdfkp = cdfkp.addConstraint(ConstantConstraint(Hdotf), ...
      cdfkp.Hdot_inds(:,end));
    cdfkp = cdfkp.addConstraint(BoundingBoxConstraint(options.com_apex(3) - options.com_tol*abs(options.com_apex(3)),options.com_apex(3) + options.com_tol*abs(options.com_apex(3))), ...
      cdfkp.com_inds(3,end));
  else
    cdfkp = cdfkp.addConstraint(ConstantConstraint(stride_length/2), ...
      cdfkp.com_inds(1,end));
  end

  % Immobilize specified joints
  if options.freeze_neck
    cdfkp = cdfkp.addConstraint(ConstantConstraint(repmat(q0(freeze_idx),N,1)),cdfkp.q_inds(freeze_idx,:));
  end

  % Add gaze constraint
  if options.constrain_head_gaze
    cdfkp = cdfkp.addRigidBodyConstraint(gaze_constraint,num2cell(1:N));
  end

  % Constrain arms
  if options.start_from_standing
    %cdfkp = cdfkp.addConstraint(ConstantConstraint(linspacevec(q0(wrist_idx),qf(wrist_idx),N-2)),cdfkp.q_inds(wrist_idx,2:end-1));
    %cdfkp = cdfkp.addConstraint(ConstantConstraint(linspacevec(q0(arm_idx),qf(arm_idx),N-2)),cdfkp.q_inds(arm_idx,2:end-1));
  end

  % Add periodicity constraints
  if ~(options.start_from_standing || options.add_obstacle)
    half_periodic_constraint = halfPeriodicConstraint(robot);
    cdfkp = cdfkp.addConstraint(half_periodic_constraint,cdfkp.q_inds(:,[1,end]));
    cdfkp = cdfkp.addConstraint(LinearConstraint(0,0,[1,-1]),cdfkp.v_inds(1,[1,end]));
    cdfkp = cdfkp.addConstraint(LinearConstraint(zeros(3,1),zeros(3,1),[eye(3),diag([-1,1,1])]), ...
      cdfkp.comdot_inds(1:3,[1,end]));
    %cdfkp = cdfkp.addConstraint(LinearConstraint(zeros(3,1),zeros(3,1),[eye(3),diag([-1,1,1])]), ...
      %cdfkp.comddot_inds(1:3,[1,end]));
    cdfkp = cdfkp.addConstraint(LinearConstraint(zeros(3,1),zeros(3,1),[eye(3),diag([1,-1,1])]), ...
      cdfkp.H_inds(1:3,[1,end]));
    cdfkp = cdfkp.addConstraint(LinearConstraint(zeros(3,1),zeros(3,1),[eye(3),diag([1,-1,1])]), ...
      cdfkp.Hdot_inds(1:3,[1,end]));
  end

  % Add stance kinematic constraints
  cdfkp = cdfkp.addRigidBodyConstraint(position_cnstr.toe, ...
    num2cell(in_stance.toe));
  cdfkp = cdfkp.addRigidBodyConstraint(position_cnstr.heel, ...
    num2cell(in_stance.heel));
  cdfkp = cdfkp.addRigidBodyConstraint(stationary_cnstr.toe, ...
    {in_stance.toe});
  cdfkp = cdfkp.addRigidBodyConstraint(stationary_cnstr.heel, ...
    {in_stance.heel});
  if options.start_from_standing
    cdfkp = cdfkp.addRigidBodyConstraint(position_cnstr.right, ...
      num2cell(in_stance.right));
    cdfkp = cdfkp.addRigidBodyConstraint(stationary_cnstr.right, ...
      {in_stance.right});
  end
  toe_clearance_idx = setdiff(in_stance.total,in_stance.toe);
  heel_clearance_idx = setdiff(in_stance.total,in_stance.heel);
  if ~isempty(toe_clearance_idx)
    cdfkp = cdfkp.addRigidBodyConstraint(stance_clearance_cnstr.toe, ...
      num2cell(toe_clearance_idx));
  end
  if ~isempty(heel_clearance_idx)
    cdfkp = cdfkp.addRigidBodyConstraint(stance_clearance_cnstr.heel, ...
      num2cell(heel_clearance_idx));
  end

  % Add collision avoidance constraints
  if options.enable_collision
    cdfkp = cdfkp.addRigidBodyConstraint(min_distance_constraint.flight,num2cell(in_flight));
    %cdfkp = cdfkp.addRigidBodyConstraint(min_distance_constraint.stance,num2cell(in_flight));
    cdfkp = cdfkp.addRigidBodyConstraint(min_distance_constraint.stance,num2cell(in_stance.left));
    if options.start_from_standing
      cdfkp = cdfkp.addRigidBodyConstraint(min_distance_constraint.double_stance,num2cell(in_stance.double));
    end
    if options.n_interp_points > 0
      for i = in_flight(1:end-1)
        for j = 1:numel(interp_min_distance_constraint.flight)
          cdfkp = cdfkp.addConstraint(interp_min_distance_constraint.flight{j},{cdfkp.q_inds(:,i),cdfkp.q_inds(:,i+1)});
        end
      end
      for i = in_stance.left(1:end-1)
        for j = 1:numel(interp_min_distance_constraint.stance)
          cdfkp = cdfkp.addConstraint(interp_min_distance_constraint.stance{j},{cdfkp.q_inds(:,i),cdfkp.q_inds(:,i+1)});
        end
      end
      if ~isempty(in_stance.double)
        for i = in_stance.double(1:end-1)
          for j = 1:numel(interp_min_distance_constraint.double_stance)
            cdfkp = cdfkp.addConstraint(interp_min_distance_constraint.double_stance{j},{cdfkp.q_inds(:,i),cdfkp.q_inds(:,i+1)});
          end
        end
      end
    end
  end

  % TODO: Set up seed
  if isempty(seed)
    x_seed = zeros(cdfkp.num_vars,1);
    q_seed = linspacevec(q0,qf,N);
    v_seed = gradient(q_seed);
    com_seed = linspacevec(com_0,com_f,N);
    comdot_seed = gradient(com_seed);
    comddot_seed = gradient(comdot_seed);
    lambda_seed = sqrt(2)/24;
    x_seed(cdfkp.h_inds) = T/N;
    x_seed(cdfkp.q_inds(:)) = reshape(q_seed,[],1);
    x_seed(cdfkp.v_inds(:)) = reshape(v_seed,[],1);
    x_seed(cdfkp.com_inds(:)) = reshape(com_seed,[],1);
    x_seed(cdfkp.comdot_inds(:)) = reshape(comdot_seed,[],1);
    x_seed(cdfkp.comddot_inds(:)) = reshape(comddot_seed,[],1);
    for i = 1:numel(cdfkp.lambda_inds)
      x_seed(cdfkp.lambda_inds{i}(:)) = lambda_seed;
    end
  else
    x_seed = seed.x_sol;
  end

  % Set up solver options
  cdfkp = cdfkp.setSolverOptions('snopt','iterationslimit',1e6);
  cdfkp = cdfkp.setSolverOptions('snopt','majoriterationslimit',major_iteration_limit);
  cdfkp = cdfkp.setSolverOptions('snopt','majorfeasibilitytolerance',1e-5);
  cdfkp = cdfkp.setSolverOptions('snopt','majoroptimalitytolerance',5e-4);
  cdfkp = cdfkp.setSolverOptions('snopt','superbasicslimit',2000);
  cdfkp = cdfkp.setSolverOptions('snopt','linesearchtolerance',0.9);
  cdfkp = cdfkp.setSolverOptions('snopt','print',sprintf('snopt_%s.out',suffix));

  % Solve trajectory optimization
  tic
  %profile on;
  [x_sol,~,exitflag] = cdfkp.solve(x_seed);
  %profile off;
  toc
  
  if exitflag > 10
    error(['Trajectory optimization failed. exitflag = ' num2str(exitflag)]);
  end

  % Parse trajectory optimization output
  sol.x_sol = x_sol;
  sol.q = reshape(x_sol(cdfkp.q_inds(:)),nq,N);
  sol.v = reshape(x_sol(cdfkp.v_inds(:)),nq,N);
  sol.h = reshape(x_sol(cdfkp.h_inds),1,[]);
  sol.t = cumsum([0 sol.h]);
  sol.com = reshape(x_sol(cdfkp.com_inds),3,[]);
  sol.comdot = reshape(x_sol(cdfkp.comdot_inds),3,[]);
  sol.comddot = reshape(x_sol(cdfkp.comddot_inds),3,[]);
  sol.H = reshape(x_sol(cdfkp.H_inds),3,[]);
  sol.Hdot = reshape(x_sol(cdfkp.Hdot_inds),3,[])*cdfkp.torque_multiplier;
  sol.lambda = cell(2,1);
  for i = 1:numel(cdfkp.lambda_inds)
    sol.lambda{i} = reshape(x_sol(cdfkp.lambda_inds{i}),size(cdfkp.lambda_inds{i},1),[],N);
  end
  sol.xtraj= PPTrajectory(foh(sol.t,[sol.q;sol.v]));
  sol.xtraj= sol.xtraj.setOutputFrame(robot_vis.getStateFrame);
  sol.options = options;
  sol.FC_basis_vectors = FC_edge;

  % Save results
  save(sprintf('results_%s',suffix),'sol');
end

function half_periodic_constraint = halfPeriodicConstraint(robot)
  num_symmetry = 12;
  num_equal = 8;
  nq = robot.getNumPositions();

  symmetric_matrix = zeros(2*num_symmetry,2*nq);
  equal_matrix = zeros(num_equal,2*nq);
  initial_indices = 1:nq;
  final_indices   = nq+(1:nq);

  function sym_mat = addSymmetricPair(sym_mat,rows,idx1,idx2)
    sym_mat(rows(1),[initial_indices(idx1) final_indices(idx2)]) = [1 -1];
    sym_mat(rows(2),[initial_indices(idx2) final_indices(idx1)]) = [1 -1];
  end

  function sym_mat = addAntiSymmetricPair(sym_mat,rows,idx1,idx2)
    sym_mat(rows(1),[initial_indices(idx1) final_indices(idx2)]) = [1 1];
    sym_mat(rows(2),[initial_indices(idx2) final_indices(idx1)]) = [1 1];
  end

  function eq_mat = addEquality(eq_mat,row,idx)
    eq_mat(row,[initial_indices(idx) final_indices(idx)]) = [1 -1];
  end

  function eq_mat = addOpposite(eq_mat,row,idx)
    eq_mat(row,[initial_indices(idx) final_indices(idx)]) = [1 1];
  end

  l_arm_usy_idx = robot.getBody(robot.findJointId('l_arm_shz')).position_num;
  r_arm_usy_idx = robot.getBody(robot.findJointId('r_arm_shz')).position_num;
  symmetric_matrix = addSymmetricPair(symmetric_matrix,1:2,l_arm_usy_idx,r_arm_usy_idx);

  l_arm_shx_idx = robot.getBody(robot.findJointId('l_arm_shx')).position_num;
  r_arm_shx_idx = robot.getBody(robot.findJointId('r_arm_shx')).position_num;
  symmetric_matrix = addAntiSymmetricPair(symmetric_matrix,3:4,l_arm_shx_idx,r_arm_shx_idx);

  l_arm_ely_idx = robot.getBody(robot.findJointId('l_arm_ely')).position_num;
  r_arm_ely_idx = robot.getBody(robot.findJointId('r_arm_ely')).position_num;
  symmetric_matrix = addSymmetricPair(symmetric_matrix,5:6,l_arm_ely_idx,r_arm_ely_idx);

  l_arm_elx_idx = robot.getBody(robot.findJointId('l_arm_elx')).position_num;
  r_arm_elx_idx = robot.getBody(robot.findJointId('r_arm_elx')).position_num;
  symmetric_matrix = addAntiSymmetricPair(symmetric_matrix,7:8,l_arm_elx_idx,r_arm_elx_idx);

  l_arm_uwy_idx = robot.getBody(robot.findJointId('l_arm_uwy')).position_num;
  r_arm_uwy_idx = robot.getBody(robot.findJointId('r_arm_uwy')).position_num;
  symmetric_matrix = addSymmetricPair(symmetric_matrix,9:10,l_arm_uwy_idx,r_arm_uwy_idx);

  l_arm_mwx_idx = robot.getBody(robot.findJointId('l_arm_mwx')).position_num;
  r_arm_mwx_idx = robot.getBody(robot.findJointId('r_arm_mwx')).position_num;
  symmetric_matrix = addAntiSymmetricPair(symmetric_matrix,11:12,l_arm_mwx_idx,r_arm_mwx_idx);

  l_leg_hpz_idx = robot.getBody(robot.findJointId('l_leg_hpz')).position_num;
  r_leg_hpz_idx = robot.getBody(robot.findJointId('r_leg_hpz')).position_num;
  symmetric_matrix = addAntiSymmetricPair(symmetric_matrix,13:14,l_leg_hpz_idx,r_leg_hpz_idx);

  l_leg_hpx_idx = robot.getBody(robot.findJointId('l_leg_hpx')).position_num;
  r_leg_hpx_idx = robot.getBody(robot.findJointId('r_leg_hpx')).position_num;
  symmetric_matrix = addAntiSymmetricPair(symmetric_matrix,15:16,l_leg_hpx_idx,r_leg_hpx_idx);

  l_leg_hpy_idx = robot.getBody(robot.findJointId('l_leg_hpy')).position_num;
  r_leg_hpy_idx = robot.getBody(robot.findJointId('r_leg_hpy')).position_num;
  symmetric_matrix = addSymmetricPair(symmetric_matrix,17:18,l_leg_hpy_idx,r_leg_hpy_idx);

  l_leg_kny_idx = robot.getBody(robot.findJointId('l_leg_kny')).position_num;
  r_leg_kny_idx = robot.getBody(robot.findJointId('r_leg_kny')).position_num;
  symmetric_matrix = addSymmetricPair(symmetric_matrix,19:20,l_leg_kny_idx,r_leg_kny_idx);

  l_leg_akx_idx = robot.getBody(robot.findJointId('l_leg_akx')).position_num;
  r_leg_akx_idx = robot.getBody(robot.findJointId('r_leg_akx')).position_num;
  symmetric_matrix = addAntiSymmetricPair(symmetric_matrix,21:22,l_leg_akx_idx,r_leg_akx_idx);

  l_leg_aky_idx = robot.getBody(robot.findJointId('l_leg_aky')).position_num;
  r_leg_aky_idx = robot.getBody(robot.findJointId('r_leg_aky')).position_num;
  symmetric_matrix = addSymmetricPair(symmetric_matrix,23:24,l_leg_aky_idx,r_leg_aky_idx);

  %base_y = findPositionIndices(robot,'base_y'); base_y = base_y(1);
  base_y = 2;
  equal_matrix = addOpposite(equal_matrix,1,base_y);

  base_z = 3;
  equal_matrix = addEquality(equal_matrix,2,base_z);

  base_roll = 4;
  equal_matrix = addOpposite(equal_matrix,3,base_roll);

  base_pitch = 5;
  equal_matrix = addEquality(equal_matrix,4,base_pitch);

  base_yaw = 6;
  equal_matrix = addOpposite(equal_matrix,5,base_yaw);

  back_bkz = robot.getBody(robot.findJointId('back_bkz')).position_num;
  equal_matrix = addOpposite(equal_matrix,6,back_bkz);

  back_bky = robot.getBody(robot.findJointId('back_bky')).position_num;
  equal_matrix = addEquality(equal_matrix,7,back_bky);

  back_bkx = robot.getBody(robot.findJointId('back_bkx')).position_num;
  equal_matrix = addOpposite(equal_matrix,8,back_bkx);

  lb = zeros(2*num_symmetry+num_equal,1);
  ub = lb;
  half_periodic_constraint = LinearConstraint(lb,ub,[symmetric_matrix;equal_matrix]);
end

function displayCallback(in_stance,N,x)
  h = x(1:N-1);
  ts = [0;cumsum(h)];
  com_z = x(N-1+(1:N));
  comdot_x = x(2*N-1+(1:N));
  q = reshape(x(3*N:end),[],N);
  x_data = [zeros(2,numel(ts));q;0*q];
  utime = now() * 24 * 60 * 60;
  snopt_info_vector = ones(1, size(x_data,2));
  sfigure(7);
  subplot(2,1,1);
  plot(ts,com_z,'bo-');
  hold on
  plot(ts(in_stance),com_z(in_stance),'ro-');
  title('COM Height')
  xlabel('t (s)')
  ylabel('z (m)')
  hold off
  subplot(2,1,2);
  plot(ts,comdot_x,'bo-');
  hold on
  plot(ts(in_stance),comdot_x(in_stance),'ro-');
  title('COM velocity')
  xlabel('t (s)')
  ylabel('xdot (m/s)')
  hold off
  drawnow;
end

function robot = addObstacle(robot,obstacle_x_position)
  radius = 0.1;
  len = 5;
  height = 1;
  beam = RigidBodyCapsule(radius,len,[obstacle_x_position,-1,height],[4*pi/6;0;0]);
  wall1 = RigidBodyBox(2*[0.1; 1.235; 1.0],[obstacle_x_position-0.2,-0.565-1,1],[0;0;0]);
  wall2 = RigidBodyBox(2*[0.1; 0.15; 1.0],[obstacle_x_position-0.2,1.65-1,1],[0;0;0]);
  wall3 = RigidBodyBox(2*[0.1; 1.8; 0.25],[obstacle_x_position-0.2,0-1,2.25],[0;0;0]);
  robot = robot.addGeometryToBody('world',beam);
  robot = robot.addGeometryToBody('world',wall1);
  robot = robot.addGeometryToBody('world',wall2);
  robot = robot.addGeometryToBody('world',wall3);
  robot = compile(robot);
end

function options = defaultOptionsStruct()
  options.visualize = true;
  options.toe_first = false;
  options.n_interp_points = 0;
  options.speed = 2;
  options.constrain_head_gaze = true;
  options.head_gaze_tol = 15*pi/180;
  options.freeze_neck = true;
  options.start_from_standing = false;
  options.stride_filename = 'results_ss_1p5m_2mps_22knots_A.mat';
  options.enable_collision = true;
  options.enable_velocity_limits = false;
  options.add_obstacle = false;
  options.min_distance = 0.03;
  options.N = 16;
  options.time_option = 2;
  options.joint_tol = 10*pi/180;
  options.com_tol = 0.1;
end

function options = parseOptionsStruct(options_in)
  options = defaultOptionsStruct();
  for fieldname_cell = fields(options_in)'
    fieldname = fieldname_cell{1};
    if isfield(options,fieldname)
      options.(fieldname) = options_in.(fieldname);
    end
  end
end
