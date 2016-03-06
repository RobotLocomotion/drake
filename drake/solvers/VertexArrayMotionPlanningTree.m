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
    children
    centroid
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
      obj.children = {};
      obj.centroid = q_init;
      [obj,id_last] = obj.addVertex(q_init, 1);
    end

    function q = randomSample(obj)
      q = obj.sampling_lb + (obj.sampling_ub-obj.sampling_lb).*rand(obj.num_vars,1);
    end

    function [obj, id] = addVertex(obj, q, id_parent)
      [obj, id] = addVertex@MotionPlanningTree(obj, q, id_parent);
      obj.V(:,obj.n) = q; 
      obj.parent(obj.n) = id_parent; 
      obj.children{obj.n} = [];
      obj.children{id_parent} = [obj.children{id_parent}; obj.n];
      if ~isempty(obj.centroid) && obj.n > 1
        obj.centroid = obj.centroid + (q - obj.centroid) / obj.n;
      end
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
    
    function obj = setParentId(obj, id, idParent)
      oldParent = obj.parent(id);
      obj.children{oldParent} = obj.children{oldParent}(obj.children{oldParent} ~= id);
      obj.parent(id) = idParent;
      obj.children{idParent} = [obj.children{idParent}; id];
    end
    
    function obj = removeVertices(obj, ids)
      %Rebuild children arrays
      for i = ids
        ch = obj.children{obj.getParentId(i)};
        obj.children{obj.getParentId(i)} = ch(ch ~= i);
      end
      %Rebuild the parent-child relationships
      for i = 1:obj.n
        if all(ids ~= i)
          obj.parent(i) = obj.parent(i) - nnz(ids < obj.parent(i));
        end
      end
      %Delete the vertices
      obj.V(:, ids) = [];
      obj.V(:, end+1:obj.N) = NaN(obj.num_vars, length(ids));
      obj.parent(ids) = [];
      obj.parent(end+1:obj.N) = NaN(1, length(ids));
      obj.n = obj.n - length(ids);
    end
    
    function children = getChildren(obj, parentId)
      children = obj.children{parentId};
    end
    
    function centroid = getCentroid(obj)
      centroid = obj.centroid;
    end
    
  end
end