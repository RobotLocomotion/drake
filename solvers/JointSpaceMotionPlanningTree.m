classdef JointSpaceMotionPlanningTree < CartesianMotionPlanningTree
  properties
    rbm
    visualization_point = struct('body', 1, 'pt', [0;0;0]);
    min_distance = 0.1;
    active_collision_options = struct();
    v
    xyz_vis
    kinematic_constraints = {};
    ikoptions;
    q_nom
  end
  methods
    function obj = JointSpaceMotionPlanningTree(rbm)
      obj = obj@CartesianMotionPlanningTree(rbm.getNumPositions());
      obj.rbm = rbm;
      [obj.sampling_lb, obj.sampling_ub] = rbm.getJointLimits();
      obj.sampling_lb(isinf(obj.sampling_lb)) = -10;
      obj.sampling_ub(isinf(obj.sampling_ub)) = 10;
      %obj.v = obj.rbm.constructVisualizer(struct('use_collision_geometry',true));
      obj.ikoptions = IKoptions(rbm);
      obj.q_nom = zeros(obj.rbm.getNumPositions(),1);
    end

    function obj = compile(obj)
      obj.ikoptions = obj.ikoptions.updateRobot(obj.rbm);
      for i = 1:numel(obj.kinematic_constraints)
        if obj.rbm.getMexModelPtr ~= obj.kinematic_constraints{i}.robot.getMexModelPtr()
          obj.kinematic_constraints{i} = obj.kinematic_constraints{i}.updateRobot(obj.rbm);
        end
      end
      obj.xyz_vis = NaN(3, obj.N);
    end

    function obj = addKinematicConstraint(obj, varargin)
      for i = 1:numel(varargin)
        typecheck(varargin{i}, 'RigidBodyConstraint');
        obj.kinematic_constraints(end+1) = varargin(i);
      end
    end
    
    function obj = setNominalConfiguration(obj, q)
      sizecheck(q, size(obj.q_nom));
      obj.q_nom = q;
    end

    function [q, valid] = solveIK(obj, q_seed, q_nom, additional_constraints)
      if nargin < 2 || isempty(q_seed), q_seed = obj.q_nom; end
      if nargin < 3 || isempty(q_nom), q_nom = q_seed; end
      if nargin < 4 || isempty(additional_constraints)
        additional_constraints = {};
      end
      [q, info] = inverseKin(obj.rbm, q_nom, q_seed, obj.kinematic_constraints{:}, additional_constraints{:}, obj.ikoptions);
      valid = (info < 10);
    end

    function valid = checkKinematicConstraints(obj, q)
      valid = true;
      kinsol = obj.rbm.doKinematics(q);
      tol = 1e-3;
      for i = 1:numel(obj.kinematic_constraints)
        if isa(obj.kinematic_constraints{i}, 'QuasiStaticConstraint')
          valid = valid && obj.kinematic_constraints{i}.checkConstraint(kinsol);
        else
          if valid
            [lb, ub] = obj.kinematic_constraints{i}.bounds(0);
            if isa(obj.kinematic_constraints{i}, 'PostureConstraint')
              y = q;
            else
              y = eval(obj.kinematic_constraints{i}, 0, kinsol);
            end
            valid = all(y - lb > -tol) && all(ub - y > -tol);
          end
        end
      end
    end

    function valid = checkConstraints(obj, q)
      valid = checkConstraints@CartesianMotionPlanningTree(obj, q);
      valid = valid && obj.checkKinematicConstraints(q);
      valid = valid && obj.isCollisionFree(q);
    end

    function valid = isCollisionFree(obj, q)
      phi = obj.rbm.collisionDetect(q, false, obj.active_collision_options);
      valid = all(phi > obj.min_distance);
    end

    function obj = addGeometryToWorld(obj, geom)
      obj.rbm = obj.rbm.addGeometryToBody(1, geom);
      obj = obj.compile();
    end

    function q = randomConfig(obj)
      q = randomConfig@CartesianMotionPlanningTree(obj);
    end

    function obj = drawTree(obj, n_at_last_draw, draw_now)
      if nargin < 2, n_at_last_draw = 1; end
      if nargin < 3, draw_now = true; end
      obj.lcmgl.glColor3f(obj.line_color(1), obj.line_color(2), obj.line_color(3));
      for i = 1:obj.n
        if i >= n_at_last_draw
          kinsol = obj.rbm.doKinematics(obj.getVertex(i));
          obj.xyz_vis(:,i) = obj.rbm.forwardKin(kinsol, obj.visualization_point.body, obj.visualization_point.pt);
        end
        xyz_current = obj.xyz_vis(:, i);
        xyz_parent = obj.xyz_vis(:, obj.parent(i));
        xyz = [xyz_parent, xyz_current];
        obj.lcmgl.plot3(xyz(1,:), xyz(2,:), xyz(3,:));
      end
      if draw_now
        obj.lcmgl.switchBuffers();
      end
    end

    function drawPath(obj, varargin)
      q_path = extractPath(obj, varargin{:});
      path_length = size(q_path, 2);
      xyz = NaN(3, path_length);
      for i = 1:path_length
        kinsol = obj.rbm.doKinematics(q_path(:,i));
        xyz(:,i) = obj.rbm.forwardKin(kinsol, obj.visualization_point.body, obj.visualization_point.pt);
      end
      obj.lcmgl.glLineWidth(2);
      obj.lcmgl.glColor3f(0,1,0);
      obj.lcmgl.plot3(xyz(1,:), xyz(2,:), xyz(3,:));
      obj.lcmgl.switchBuffers();
    end
  end
end
