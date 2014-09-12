classdef LeggedRobotPlanningProblem
  properties
    robot
    supports = struct('body',{},'pts',{},'tspan',{},'constraints',{})
    rigid_body_constraints = struct('constraint',{},'tspan',{});
    mu = 1
    min_distance = 0.03;
    quasistatic_shrink_factor = 0.2;
    n_interp_points = 0;
    excluded_collision_groups = struct('name',{},'tspan',{});
    fixed_initial_state = true;
    start_from_rest = true;
    end_at_rest = true;
    Q
    v_max = 30*pi/180;
  end

  methods
    function obj = LeggedRobotPlanningProblem(robot,options)
      obj.robot = robot;
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
    end
    function obj = addSupport(obj,body_id,pts,tspan,constraints)
      support.body = obj.robot.parseBodyOrFrameID(body_id);
      support.pts = pts;
      support.tspan = tspan;
      support.constraints = constraints;
      obj.supports(end+1) = support;
    end

    function obj = addRigidBodyConstraint(obj,constraint)
      if iscell(constraint)
        for i = 1:numel(constraint)
          obj = obj.addRigidBodyConstraint(constraint{i});
        end
      else
        new_constraint.constraint = constraint;
        new_constraint.tspan = min(1,max(0,constraint.tspan));
        obj.rigid_body_constraints(end+1) = new_constraint;
      end
    end
    
    function obj = addSupportOnFlatGround(obj,body_id,pts,tspan)
      n_pts = min(size(pts,2),3);
      lb = repmat([NaN; NaN; 0],1,n_pts);
      ub = repmat([NaN; NaN; 0],1,n_pts);
      constraints{1} = WorldPositionConstraint(obj.robot,body_id,pts(:,1:n_pts),lb,ub);
      constraints{2} = WorldFixedPositionConstraint(obj.robot,body_id,pts(:,1:n_pts));
      obj = addSupport(obj,body_id,pts,tspan,constraints);
    end
    
    function prog = generateComDynamicsFullKinematicsPlanner(obj,N,durations,q_nom_traj,q0,Q_comddot, Qv, ...
        Q_contact_force,options)
      % @param N  -- Number of knot points
      if nargin < 6, Q_comddot = []; end
      if nargin < 7, Qv = []; end
      if nargin < 9, Q_contact_force = []; end
      if nargin < 10, options = struct(); end
      
      % Extract stance information
      in_stance = arrayfun(@(supp)activeKnots(supp,N), obj.supports, ...
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
          LinearFrictionConeWrench(obj.robot,obj.supports(i).body,obj.supports(i).pts,...
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
      
      % Add support constraints
      prog = obj.addSupportConstraintsToPlanner(prog);
      
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
      % Create kinematic planner
      prog = KinematicDirtran(obj.robot,N,durations,varargin{:});

      % Add kinematic constraints for supports
      prog = obj.addSupportConstraintsToPlanner(prog);

      % Add collision avoidance constraints
      prog = obj.addCollisionConstraintsToPlanner(prog);

      % Add Quasistatic Constraints
      prog = obj.addQuasiStaticConstraintsToPlanner(prog);

      % Add RigidBodyConstraints
      prog = obj.addRigidBodyConstraintsToPlanner(prog);

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
      prog = prog.addCost(QuadraticConstraint(-inf,inf,Q,Q*reshape(q_nom,[],1)),prog.q_inds(:));
      prog = prog.addRunningCost(@(h,x,u)squaredEffort(R,h,x,u));
      prog = prog.addFinalCost(@(h,xf) finalCost(kT,h,xf));

      if obj.fixed_initial_state
        prog = prog.addConstraint(ConstantConstraint(q0),prog.q_inds(:,1));
      end
    end

    function prog = addQuasiStaticConstraintsToPlanner(obj,prog)
      in_stance = arrayfun(@(supp)activeKnots(supp,prog.N), obj.supports, ...
        'UniformOutput',false);
      for i = 1:prog.N
        qsc_constraint = [];
        for j = 1:numel(obj.supports)
          if ismember(i,in_stance{j})
            if isempty(qsc_constraint)
              qsc_constraint = QuasiStaticConstraint(obj.robot,[-inf,inf],1);
            end
            qsc_constraint = qsc_constraint.addContact(obj.supports(j).body, ...
                                                       obj.supports(j).pts);
          end
        end
        if ~isempty(qsc_constraint)
          qsc_constraint = qsc_constraint.setShrinkFactor(obj.quasistatic_shrink_factor);
          qsc_constraint = qsc_constraint.setActive(true);
          prog = prog.addRigidBodyConstraint(qsc_constraint,i);
        end
      end
    end

    function prog = addRigidBodyConstraintsToPlanner(obj,prog)
      is_active = arrayfun(@(supp)activeKnots(supp,prog.N), ...
        obj.rigid_body_constraints, 'UniformOutput',false);
      for i = 1:numel(obj.rigid_body_constraints)
        prog = prog.addRigidBodyConstraint(obj.rigid_body_constraints(i).constraint,is_active{i});
      end
    end

    function prog = addCollisionConstraintsToPlanner(obj,prog)
      in_stance = arrayfun(@(supp)activeKnots(supp,prog.N), obj.supports, ...
        'UniformOutput',false);
      group_excluded = arrayfun(@(grp)activeKnots(grp,prog.N), obj.excluded_collision_groups, ...
        'UniformOutput',false);
      ignored_bodies = {};
      interpolation_parameter = (1:obj.n_interp_points)/(obj.n_interp_points+1);
      for i = 1:prog.N
        ignored_bodies{i} = [];
        for j = 1:numel(obj.supports)
          if ismember(i,in_stance{j})
            ignored_bodies{i}(end+1) = obj.supports(j).body;
          end
        end
        ignored_groups = {};
        for j = 1:numel(obj.excluded_collision_groups)
          if ismember(i,group_excluded{j})
            ignored_groups{end+1} = obj.excluded_collision_groups(j).name;
          end
        end
        if 0%i > 1
          ignored_bodies_current = unique([ignored_bodies{[i-1,i]}]);
        else
          ignored_bodies_current = unique(ignored_bodies{i});
        end

        active_collision_options.body_idx = setdiff(1:obj.robot.getNumBodies(),ignored_bodies_current);
        active_collision_options.collision_groups = setdiff(unique([obj.robot.body.collision_group_name]),ignored_groups);
        min_distance_constraint(i) = MinDistanceConstraint(obj.robot,obj.min_distance,active_collision_options);  
        prog = prog.addRigidBodyConstraint(min_distance_constraint(i),i);

        if i > 1
          %ignored_bodies_current = unique([ignored_bodies{[i-1,i]}]);
          ignored_bodies_current = unique(ignored_bodies{i});
          active_collision_options.body_idx = setdiff(1:obj.robot.getNumBodies(),ignored_bodies_current);
          min_distance_constraint_for_interp = MinDistanceConstraint(obj.robot,obj.min_distance,active_collision_options);  

          interpolated_constraint = generateInterpolatedMinDistanceConstraint(min_distance_constraint_for_interp,interpolation_parameter);
          prog = prog.addConstraint(interpolated_constraint{1},{prog.q_inds(:,i-1), prog.q_inds(:,i)});
        end
      end
    end

    function prog = addSupportConstraintsToPlanner(obj,prog)
      in_stance = arrayfun(@(supp)activeKnots(supp,prog.N), obj.supports, ...
        'UniformOutput',false);
      for i = 1:numel(obj.supports)
        if ~isempty(obj.supports(i).constraints)
          for j = 1:numel(obj.supports(i).constraints)
            prog = prog.addRigidBodyConstraint( ...
              obj.supports(i).constraints{j},in_stance{i});
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

