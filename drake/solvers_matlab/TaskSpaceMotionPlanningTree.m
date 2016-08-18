classdef TaskSpaceMotionPlanningTree < CompositeVertexArrayTree
  properties
    end_effector_id
    end_effector_pt = [0;0;0]
    position_tol = 0;
    angle_tol = 0;
    interp_weight = 0;
    tspace_idx = 1;
    cspace_idx = 2;
  end

  methods
    function obj = TaskSpaceMotionPlanningTree(r, end_effector_id, end_effector_pt)
      obj = obj@CompositeVertexArrayTree({SE3MotionPlanningTree(), ...
                                          JointSpaceMotionPlanningTree(r)}, ...
                                         {'end_effector_pose','robot_posture'}, ...
                                         [1, 0]);
      obj.end_effector_id = r.parseBodyOrFrameID(end_effector_id);
      if nargin > 2
        obj.end_effector_pt = end_effector_pt;
      end
      for i = 1:numel(r.getBody(1).collision_geometry)
        geom = r.getBody(1).collision_geometry{i};
        obj.trees{obj.tspace_idx} = obj.trees{obj.tspace_idx}.addGeometryToWorld(geom);
      end
      for i = 1:numel(r.getBody(obj.end_effector_id).collision_geometry)
        geom = r.getBody(obj.end_effector_id).collision_geometry{i};
        geom.T(1:3,4) = geom.T(1:3,4) - obj.end_effector_pt; 
        obj.trees{obj.tspace_idx} = obj.trees{obj.tspace_idx}.addGeometryToRobot(geom);
      end
      obj.trees{obj.cspace_idx}.visualization_point = struct('body', obj.end_effector_id, 'pt',...
                                          obj.end_effector_pt);
    end

    function obj = compile(obj)
      for i = 1:obj.num_trees
        obj.trees{i} = obj.trees{i}.compile();
      end
    end

    function obj = setMinDistance(obj, min_distance)
      for i = 1:obj.num_trees
        obj.trees{i}.min_distance = min_distance;
      end
    end

    function q = interpolate(obj, q1, q2, interpolation_factors)
      q = interpolate@CompositeVertexArrayTree(obj, q1, q2, interpolation_factors);
      if obj.interp_weight > 0
        for i = 1:numel(interpolation_factors)
          kinsol = obj.getRobot().doKinematics(q(obj.idx{obj.cspace_idx},i));
          xyz_quat = obj.getRobot().forwardKin(kinsol, obj.end_effector_id, obj.end_effector_pt, 2);
          q(obj.idx{obj.tspace_idx},i) = obj.trees{obj.tspace_idx}.interpolate(q(obj.idx{obj.tspace_idx},i), xyz_quat, obj.interp_weight);
        end
      end
    end

    function valid = isValid(obj, x)
      valid = isValid@CompositeVertexArrayTree(obj, x);
      tol = 1e-3;
      if valid
        kinsol = obj.getRobot().doKinematics(x(obj.idx{obj.cspace_idx}));
        ee_constraints = obj.generateEndEffectorConstraints(x);
        for i = 1:numel(ee_constraints)
          [lb, ub] = ee_constraints{i}.bounds(0);
          y = eval(ee_constraints{i}, 0, kinsol);
          valid = valid && all(y - lb > -tol) && all(ub - y > -tol);
        end
      end
    end
    
    function [x, valid] = attemptToMakeValid(obj, x, valid)
      if obj.trees{obj.tspace_idx}.isValid(x(obj.idx{obj.tspace_idx}));
        [x(obj.idx{obj.cspace_idx}), valid] = obj.trees{obj.cspace_idx}.solveIK([], [], obj.generateEndEffectorConstraints(x));
        valid = valid && obj.trees{obj.cspace_idx}.checkConstraints(x(obj.idx{obj.cspace_idx}));
      end
    end

    function obj = addKinematicConstraint(obj, varargin)
      obj.trees{obj.cspace_idx} = obj.trees{obj.cspace_idx}.addKinematicConstraint(varargin{:});
    end

    function ee_constraints = generateEndEffectorConstraints(obj, x)
      xyz = x(1:3);
      quat = x(4:7);
      ee_constraints{1} = Point2PointDistanceConstraint(obj.getRobot(), obj.end_effector_id,1,obj.end_effector_pt, xyz, 0, obj.position_tol);
      ee_constraints{2} = WorldQuatConstraint(obj.getRobot(), obj.end_effector_id, quat, obj.angle_tol);
    end

    function obj = setNominalConfiguration(obj, q)
      obj.trees{obj.cspace_idx} = obj.trees{obj.cspace_idx}.setNominalConfiguration(q);
    end

    function obj = setOrientationWeight(obj, orientation_weight)
      obj.trees{obj.tspace_idx} = obj.trees{obj.tspace_idx}.setOrientationWeight(orientation_weight);
    end

    function obj = setLCMGL(obj, varargin)
      obj = setLCMGL@CompositeVertexArrayTree(obj, varargin{:});
      obj.trees{obj.tspace_idx} = obj.trees{obj.tspace_idx}.setLCMGL(varargin{:});
    end

    function obj = drawTree(obj, varargin)
      obj.trees{obj.tspace_idx}.drawTree(varargin{:});
    end

    function obj = setTranslationSamplingBounds(obj, lb, ub)
      obj.trees{obj.tspace_idx} = ...
        obj.trees{obj.tspace_idx}.setTranslationSamplingBounds(lb, ub);
    end

    function drawPath(objA, path_ids_A, objB, path_ids_B)
      if nargin > 2
        drawPath(objA.trees{objA.tspace_idx}, path_ids_A, objB.trees{objB.tspace_idx}, path_ids_B);
      else
        drawPath(objA.trees{objA.tspace_idx}, path_ids_A);
      end
    end

    function r = getRobot(obj)
      r = obj.trees{obj.cspace_idx}.rbm;
    end
  end
end
