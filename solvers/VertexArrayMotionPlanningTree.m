classdef VertexArrayMotionPlanningTree < MotionPlanningTree & MotionPlanningProblem
  % Partial implementation of MotionPlanningTree that uses the
  % "checkConstraints" functionality of MotionPlanningProblem and stores the
  % vertices of the tree in a num_vars-by-n array.
  properties
    lb
    ub
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
      obj.lb = zeros(num_vars, 1);
      obj.ub = ones(num_vars, 1);
    end

    function [obj, id_last] = init(obj, q_init)
      obj = init@MotionPlanningTree(obj);
      sizecheck(q_init, 'colvec');
      obj.V = NaN(obj.num_vars, obj.N);
      obj.parent = NaN(1, obj.N);
      [obj,id_last] = obj.addVertex(q_init, 1);
    end

    function q = randomConfig(obj)
      q = obj.lb + (obj.ub-obj.lb).*rand(obj.num_vars,1);
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

    function valid = isValidConfiguration(obj, q)
      valid = sizecheck(q, [obj.num_vars, 1]);
      valid = valid && obj.checkConstraints(q);
    end
  end
end
