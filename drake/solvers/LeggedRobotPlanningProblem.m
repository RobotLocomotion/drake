classdef LeggedRobotPlanningProblem
  % Factory class for motion planning problems involving robots that implement
  % the 'LeggedRobot' interface. Allows us to represent problems in a way
  % that's not tied to a specific solution method. This is still a work in
  % progress - at present it supports quasi-static planning with the
  % 'KinematicDirtran' class and dynamic planning with the
  % 'ComDynamicsFullKinematicsPlanner'.
  %
  % The bulk of the problem is represented by two structures
  %    * constraint_struct - Constraint objects, each with an associated tspan
  %        indicating the portion of the plan (between 0 and 1) for which the
  %        constraint is active.
  %    * support_struct    - Points on the robot which can provide support and an
  %        associated tspan indicating the portion of the plan (between 0 and 1)
  %        for which those points support the robot. These are stored stored
  %        separately fron the rest of the constraints to allow for the
  %        generation of planners that treat contacts differently (e.g.
  %        quasi-static planning vs. dynamic planning) from the same problem
  %        specification.
  %  These properties are set via the addSupport and addConstraint methods.
  %
  %  There are also several publicly set-able properties that affect the
  %  planning problem:
  %     * mu - Coefficient of friction for all support contacts
  %     * min_distance - Minimum allowable difference between collision
  %         geometries that are eligible for collision. See
  %         RigidBodyManipulator/collisionDetect for more details on which
  %         collision geometries are eligible to collide with each other.
  %     * quasistatic_shrink_factor - Shrink factor to be used in all
  %         QuasiStaticConstraint objects contained in generated planners.
  %     * n_interp_points - Number of intermediate points between knots at
  %         which collision avoidance constraints should be evaluated.
  %     * excluded_collision_groups - Structure array with two fields
  %         - name: String containing the name of the collision geometry group
  %             to be ignored
  %         - tspan: Portion of the plan between 0 and 1 for which the group
  %             should be ignored
  %     * fixed_inital_state - If 'true', then the intial state will be fixed
  %         to a value specified at the time the planner is generated.
  %     * start_from_rest - If 'true', then the robot will have zero velocity
  %         in all coordinates at the beginning of the plan.
  %     * end_at_rest - If 'true', then the robot will have zero velocity
  %         in all coordinates at the end of the plan.
  %     * Q - Weighting matrix used in planners that impose a quadratic cost on
  %         deviation from a nominal position trajectory
  %     * v_max - Upper bound on the velocities at all times. We currently
  %         assume that the velocity limits are symmetric.
  properties (SetAccess = protected)
    robot
    support_struct = struct('body',{},'pts',{},'tspan',{})
    constraint_struct = struct('constraint',{},'tspan',{});
  end

  properties
    mu = 1;
    min_distance = 0.03;
    quasistatic_shrink_factor = 0.2;
    n_interp_points = 0;
    excluded_collision_groups = struct('name',{},'tspan',{});
    fixed_initial_state = true;
    start_from_rest = true;
    end_at_rest = true;
    Q;
    v_max = 30*pi/180;
  end

  methods
    function obj = LeggedRobotPlanningProblem(robot,options)
      obj.robot = robot;
      if ~isfield(options,'fixed_initial_state')
        options.fixed_initial_state =  true;
      end
      if ~isfield(options,'start_from_rest')
        options.start_from_rest =  true;
      end
      if ~isfield(options,'end_at_rest')
        options.end_at_rest =  true;
      end
      if isfield(options,'n_interp_points')
        obj.n_interp_points = options.n_interp_points;
      end
      if isfield(options,'v_max')
        obj.v_max = options.v_max;
      end
      if isscalar(obj.v_max)
        obj.v_max = obj.v_max*ones(obj.robot.getNumVelocities(),1);
      else
        sizecheck(obj.v_max,[obj.robot.getNumVelocities(),1]);
      end
      if isfield(options,'excluded_collision_groups')
        obj.excluded_collision_groups = options.excluded_collision_groups;
      end
      obj.Q = 1e0*eye(obj.robot.getNumPositions());
      obj.Q(1,1) = 0;
      obj.Q(2,2) = 0;
      obj.Q(3,3) = 0;
      obj.Q(6,6) = 0;
      obj.fixed_initial_state = options.fixed_initial_state;
      obj.start_from_rest = options.start_from_rest;
      obj.end_at_rest = options.end_at_rest;
    end

    function obj = addSupport(obj,body_id,pts,tspan,constraints)
      % obj = addSupport(obj,body_id,pts,tspan,constraints) adds an element to
      % the support_struct for this problem.
      %
      % @param obj          --  LeggedRobotPlanningProblem object
      % @param body_id      --  Name or body index / frame id of the body or
      %                         frame which provides support
      % @param pts          --  Points on the body (or in the frame) specified
      %                         by body_id that provide support
      % @param tspan        --  Portion of the plan for which this support is
      %                         active, between 0 and 1.
      % @param constraints  --  Cell array of kinematic constraints imposed by
      %                         this contact
      support.body = obj.robot.parseBodyOrFrameID(body_id);
      support.pts = pts;
      support.tspan = tspan;
      for i = 1:numel(constraints)
        obj.constraint_struct(end+1).constraint = constraints{i};
        obj.constraint_struct(end).tspan = tspan;
      end
      obj.support_struct(end+1) = support;
    end

    function obj = addConstraint(obj, constraint, tspan)
      % obj = addConstraint(obj,constraint,tspan) adds an element to
      % the constraint_struct for this problem.
      %
      % @param obj          --  LeggedRobotPlanningProblem object
      % @param constraints  --  Constraint or RigidBodyConstraint object that
      %                         defines a kinematic constraint on the robot.
      %                         Can also be a cell array of such objects. In
      %                         that case tspan must be a cell array of the
      %                         same size.
      % @param tspan        --  Two element vector indicating the portion of
      %                         the plan for which this constraint is active,
      %                         between 0 and 1. Can also be a cell array of
      %                         such vectors.
      if iscell(constraint)
        assert(numel(constraint) == numel(tspan), ...
          'Drake:LeggedRobotPlanningProblem:BadInputDimensions', ...
          'If constraint is a cell array, tspan must be a cell array of the same size')
        for i = 1:numel(constraint)
          obj = obj.addConstraint(constraint{i},tspan{i});
        end
      else
        new_constraint.constraint = constraint;
        new_constraint.tspan = tspan;
        obj.constraint_struct(end+1) = new_constraint;
      end
    end

    function obj = addRigidBodyConstraint(obj,constraint)
      % obj = addRigidBodyConstraint(obj,constraint) adds an element to the
      % constraint_struct of this problem where the active time of the
      % constraint is taken from the tspan property of the
      % RigidBodyConstraintObject constraint.
      %
      % @param obj        --  LeggedRobotPlanningProblem object
      % @param constraint --  RigidBodyConstraint object
      if iscell(constraint)
        tspan = cell(size(constraint));
        for i = 1:numel(constraint)
          tspan{i} = min(1,max(0,constraint{i}.tspan));
        end
        obj = obj.addConstraint(constraint,tspan);
      else
        tspan = min(1,max(0,constraint.tspan));
        obj = obj.addConstraint(constraint,tspan);
      end
    end
    
    function obj = addSupportOnFlatGround(obj,body_id,pts,tspan)
      % obj = addSupportOnFlatGround(obj,body_id,pts,tspan) adds an element to
      % the support_struct of this problem for which the points on(in) the
      % specified body(frame) are in static contact with flat ground at z = 0.
      %
      % @param obj          --  LeggedRobotPlanningProblem object
      % @param body_id      --  Name or body index / frame id of the body or
      %                         frame which provides support
      % @param pts          --  Points on the body (or in the frame) specified
      %                         by body_id that provide support
      % @param tspan        --  Portion of the plan for which this support is
      %                         active, between 0 and 1.
      n_pts = min(size(pts,2),3);
      lb = repmat([NaN; NaN; 0],1,n_pts);
      ub = repmat([NaN; NaN; 0],1,n_pts);
      constraints{1} = WorldPositionConstraint(obj.robot,body_id,pts(:,1:n_pts),lb,ub);
      if n_pts == 3
        constraints{2} = WorldFixedBodyPoseConstraint(obj.robot,body_id);
      else
        constraints{2} = WorldFixedPositionConstraint(obj.robot,body_id,pts(:,1:n_pts));
      end
      obj = addSupport(obj,body_id,pts,tspan,constraints);
    end
    
    function prog = generateComDynamicsFullKinematicsPlanner(obj,N,durations,q_nom_traj,q0,Q_comddot, Qv, ...
        Q_contact_force,options)
      % prog = generateComDynamicsFullKinematicsPlanner(obj,N,durations,q_nom_traj,q0,Q_comddot, Qv, ...
      %  Q_contact_force,options) returns a ComDynamicsFullKinematicsPlanner
      %  object corresponding to this problem.
      %
      % @param obj              --  LeggedRobotPlanningProblem object
      % @param N                --  Number of knot points
      % @param durations        --  Two element vector giving the lower and
      %                             upper bounds on the final time of the plan.
      %                             The initial time is always t = 0.
      % @param q_nom_traj       --  Nominal position trajectory
      % @param q0               --  Initial position. Only used if
      %                             fixed_initial_state is set to true
      % @param Q_comddot        --  Weight on center of mass acceleration
      %                             penalty
      % @param Qv               --  Weight on joint velocity penalty
      % @param Q_contact_force  --  Weight on contact force penalty
      % @param options          --  Structure containing additional options to
      %                             be passed to the
      %                             ComDynamicsFullKinematicsPlanner
      %                             constructor
      %
      % @retval prog            --  ComDynamicsFullKinematicsPlanner object
      if nargin < 6, Q_comddot = []; end
      if nargin < 7, Qv = []; end
      if nargin < 9, Q_contact_force = []; end
      if nargin < 10, options = struct(); end
      
      % Extract stance information
      in_stance = arrayfun(@(supp)activeKnots(supp,N), obj.support_struct, ...
        'UniformOutput',false);
      
      % Set up linearized friction cone edges
      num_edges = 3;
      FC_angles = linspace(0,2*pi,num_edges+1);FC_angles(end) = [];
      FC_axis = [0;0;1];
      FC_perp1 = rotx(pi/2)*FC_axis;
      FC_perp2 = cross(FC_axis,FC_perp1);
      FC_edge = bsxfun(@plus,FC_axis,obj.mu*(bsxfun(@times,cos(FC_angles),FC_perp1) + ...
        bsxfun(@times,sin(FC_angles),FC_perp2)));
      FC_edge = obj.robot.getMass()*norm(obj.robot.getGravity)*bsxfun(@rdivide,FC_edge,sqrt(sum(FC_edge.^2,1)));
      
      % Create contact wrench struct
      for i = 1:numel(in_stance)
        contact_wrench_struct(i).active_knot = in_stance{i}(2:end);
        contact_wrench_struct(i).cw = ...
          LinearFrictionConeWrench(obj.robot,obj.support_struct(i).body,obj.support_struct(i).pts,...
          FC_edge);
      end
      
      % Set up costs
      q_nom = eval(q_nom_traj,linspace(0,1,N));
      if isempty(Q_comddot), Q_comddot = eye(3); end
      if isempty(Qv), Qv = eye(obj.robot.getNumVelocities()); end
      if isempty(Q_contact_force), Q_contact_force = zeros(3); end

      % Construct planner
      prog = ComDynamicsFullKinematicsPlanner(obj.robot, N, durations, ...
        Q_comddot, Qv, obj.Q, q_nom, Q_contact_force,contact_wrench_struct, ...
        options);
      
      % Add collision avoidance constraints
      prog = obj.addCollisionConstraintsToPlanner(prog);

      if obj.start_from_rest
        % Note: Since we use backwards Euler for the integration constraints on
        % q, we need to constrain v(:,2) as well.
        prog = prog.addConstraint(ConstantConstraint(zeros(2*obj.robot.getNumVelocities(),1)),prog.v_inds(:,1:2));
      end
      
      if obj.end_at_rest
        prog = prog.addConstraint(ConstantConstraint(zeros(obj.robot.getNumVelocities(),1)),prog.v_inds(:,end));
      end

      if obj.fixed_initial_state
        prog = prog.addConstraint(ConstantConstraint(q0),prog.q_inds(:,1));
      end
    end

    function prog = generateQuasiStaticPlanner(obj,N,durations,q_nom_traj,q0,varargin)
      % prog = generateQuasiStaticPlanner(obj,N,durations,q_nom_traj,q0,varargin)
      %   returns a KinematicDirtran object corresponding to this problem with
      %   quasi-static motion constraints (COM must remain above the convex
      %   hull of the active support points at all times).
      %
      % @param obj              --  LeggedRobotPlanningProblem object
      % @param N                --  Number of knot points
      % @param durations        --  Two element vector giving the lower and
      %                             upper bounds on the final time of the plan.
      %                             The initial time is always t = 0.
      % @param q_nom_traj       --  Nominal position trajectory
      % @param q0               --  Initial position. Only used if
      %                             fixed_initial_state is set to true
      % @param varargin         --  All additional arguments are passed
      %                             directly to the KinematicDirtran constructor
      %
      % @retval prog            -- KinematicDirtran object
      
      % Create kinematic planner
      prog = KinematicDirtran(obj.robot,N,durations,varargin{:});


      % Add collision avoidance constraints
      prog = obj.addCollisionConstraintsToPlanner(prog);

      % Add Quasistatic Constraints
      prog = obj.addQuasiStaticConstraintsToPlanner(prog);

      % Add Constraints
      prog = obj.addConstraintsToPlanner(prog);

      % Add Velocity constraints
      prog = prog.addConstraint(BoundingBoxConstraint(-repmat(obj.v_max,1,prog.N-2),repmat(obj.v_max,1,prog.N-2)),prog.v_inds(:,2:end-1));

      if obj.start_from_rest
        prog = prog.addConstraint(ConstantConstraint(zeros(obj.robot.getNumVelocities(),1)),prog.v_inds(:,1));
      end
      
      if obj.end_at_rest
        prog = prog.addConstraint(ConstantConstraint(zeros(obj.robot.getNumVelocities(),1)),prog.v_inds(:,end));
      end

      % Set up costs
      q_nom = eval(q_nom_traj,linspace(0,1,N));
      %Q = kron(eye(prog.N),double(obj.Q>0));
      Q = kron(eye(prog.N),obj.Q);
      R = 1e0*eye(prog.plant.getNumInputs);
      kT = 1e0;
      prog = prog.addCost(QuadraticConstraint(-inf,inf,Q,-Q*reshape(q_nom,[],1)),prog.q_inds(:));
      prog = prog.addRunningCost(@(h,x,u)squaredEffort(R,h,x,u));
      prog = prog.addFinalCost(@(h,xf) finalCost(kT,h,xf));

      if obj.fixed_initial_state
        prog = prog.addConstraint(ConstantConstraint(q0),prog.q_inds(:,1));
      end
    end

  end

  methods (Access = protected)
    function prog = addQuasiStaticConstraintsToPlanner(obj,prog)
      in_stance = arrayfun(@(supp)activeKnots(supp,prog.N), obj.support_struct, ...
        'UniformOutput',false);
      for i = 1:prog.N
        qsc_constraint = [];
        for j = 1:numel(obj.support_struct)
          if ismember(i,in_stance{j})
            if isempty(qsc_constraint)
              qsc_constraint = QuasiStaticConstraint(obj.robot,[-inf,inf],1);
            end
            qsc_constraint = qsc_constraint.addContact(obj.support_struct(j).body, ...
                                                       obj.support_struct(j).pts);
          end
        end
        if ~isempty(qsc_constraint)
          qsc_constraint = qsc_constraint.setShrinkFactor(obj.quasistatic_shrink_factor);
          qsc_constraint = qsc_constraint.setActive(true);
          prog = prog.addRigidBodyConstraint(qsc_constraint,i);
        end
      end
    end

    function prog = addCollisionConstraintsToPlanner(obj,prog)
      in_stance = arrayfun(@(supp)activeKnots(supp,prog.N), obj.support_struct, ...
        'UniformOutput',false);
      group_excluded = arrayfun(@(grp)activeKnots(grp,prog.N), obj.excluded_collision_groups, ...
        'UniformOutput',false);
      ignored_bodies = {};
      interpolation_parameter = (1:obj.n_interp_points)/(obj.n_interp_points+1);
      for i = 1:prog.N
        ignored_bodies{i} = [];
        for j = 1:numel(obj.support_struct)
          if ismember(i,in_stance{j})
            ignored_bodies{i}(end+1) = obj.support_struct(j).body;
          end
        end
        ignored_groups = {};
        for j = 1:numel(obj.excluded_collision_groups)
          if ismember(i,group_excluded{j})
            ignored_groups{end+1} = obj.excluded_collision_groups(j).name;
          end
        end
        ignored_bodies_current = unique(ignored_bodies{i});

        active_collision_options.body_idx = setdiff(1:obj.robot.getNumBodies(),ignored_bodies_current);
        active_collision_options.collision_groups = setdiff(unique([obj.robot.body.collision_geometry_group_names]),ignored_groups);
        min_distance_constraint = MinDistanceConstraint(obj.robot,obj.min_distance,active_collision_options);  
        prog = prog.addRigidBodyConstraint(min_distance_constraint,i);

        if i > 1
          interpolated_constraint = generateInterpolatedMinDistanceConstraint(min_distance_constraint,interpolation_parameter);
          prog = prog.addConstraint(interpolated_constraint{1},{prog.q_inds(:,i-1), prog.q_inds(:,i)});
        end
      end
    end

    function prog = addConstraintsToPlanner(obj, prog)
      active_knots = arrayfun(@(cnstr)activeKnots(cnstr,prog.N), ...
                              obj.constraint_struct, 'UniformOutput', false);
      for i = 1:numel(obj.constraint_struct)
        prog = prog.addConstraint(obj.constraint_struct(i).constraint,active_knots{i});
      end
    end

    function prog = addSupportConstraintsToPlanner(obj,prog)
      in_stance = arrayfun(@(supp)activeKnots(supp,prog.N), obj.support_struct, ...
        'UniformOutput',false);
      for i = 1:numel(obj.support_struct)
        if ~isempty(obj.support_struct(i).constraints)
          for j = 1:numel(obj.support_struct(i).constraints)
            prog = prog.addRigidBodyConstraint( ...
              obj.support_struct(i).constraints{j},in_stance{i});
          end
        end
      end
    end
    
  end
  
end

function active_knots = activeKnots(structure,N)
  active_knots = find(linspace(0,1,N)>= structure.tspan(1) & ...
                           linspace(0,1,N)<= structure.tspan(2));
end

function [f,df] = finalCost(k,h,xf)
  f = k*sum(h);
  df = k*[ones(1,numel(h)), zeros(1,numel(xf))];
end

