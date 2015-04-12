classdef VertexArrayMotionPlanningTree < MotionPlanningTree & MotionPlanningProblem
  % Partial implementation of MotionPlanningTree that uses the
  % "checkConstraints" functionality of MotionPlanningProblem and stores the
  % vertices of the tree in a num_vars-by-n array.
  properties
    % The default sampling for this class draws from a uniform distribution
    % whose lower and upper bounds for each element of the sampling_lb and sampling_ub.
    sampling_lb
    sampling_ub
  end

  properties (Access = protected)
    V
    parent
  end

  methods (Abstract)
    d = distanceMetric(obj, q1, q_array)
    q = interpolate(obj, q1, q2, interpolation_factors);
  end

  methods
    function obj = VertexArrayMotionPlanningTree(num_vars)
      obj = obj@MotionPlanningTree();
      obj = obj@MotionPlanningProblem(num_vars);
      obj.sampling_lb = zeros(num_vars, 1);
      obj.sampling_ub = ones(num_vars, 1);
    end

    function [obj, id_last] = init(obj, q_init)
      obj = init@MotionPlanningTree(obj);
      sizecheck(q_init, 'colvec');
      obj.V = NaN(obj.num_vars, obj.N);
      obj.parent = NaN(1, obj.N);
      [obj,id_last] = obj.addVertex(q_init, 1);
    end

    function q = randomSample(obj)
      q = obj.sampling_lb + (obj.sampling_ub-obj.sampling_lb).*rand(obj.num_vars,1);
    end

    function [obj, id] = addVertex(obj, q, id_parent)
      [obj, id] = addVertex@MotionPlanningTree(obj, q, id_parent);
      obj.V(:,obj.n) = q; 
      obj.parent(obj.n) = id_parent; 
    end

    function q = getVertex(obj, id)
      q = obj.V(:, id);
    end

    function id_parent = getParentId(obj, id)
      id_parent = obj.parent(id);
    end

    function valid = isValid(obj, q)
      valid = obj.checkConstraints(q);
    end

    function q = indexIntoArrayOfPoints(obj, q_array, index)
      % q = indexIntoArrayOfPoints(obj, q_array, index) returns the index-th point
      %   in the array of points q_array. This method exists to allow the
      %   MotionPlanningTree interface to be agnostic as to how its
      %   child-classes represent the points in their search spaces.
      q = q_array(:,index);
    end
  end
end
