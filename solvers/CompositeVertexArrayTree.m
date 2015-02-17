classdef CompositeVertexArrayTree < VertexArrayMotionPlanningTree
  % Concrete impmlementation of the MotionPlanningTree interface which
  % represents a tree composed of multiple trees, each in its own space. See
  % SE3MotionPlanningTree for an example of this.
  properties
    num_trees     % number of contained trees

    trees         % num_trees-element cell array of
                  %   VertexArrayMotionPlanningTrees

    tree_names    % num_trees-element cell array of strings. obj.tree_names{i}
                  % is the name of obj.trees{i}
                  
    weights       % num_trees-element vector of weights

    idx           % num_trees-element cell array of indices into the
                  %   configureation vector. q(obj.idx{i}) is the portion of the
                  %   configuration vector associated with obj.tree{i}
  end
  methods
    function obj = CompositeVertexArrayTree(trees, tree_names, weights)
      if nargin < 3, weights = ones(size(trees)); end
      typecheck(trees, 'cell');
      typecheck(tree_names, 'cell');
      typecheck(weights, 'numeric');
      sizecheck(tree_names, size(trees));
      sizecheck(weights, size(trees));
      num_vars = 0;
      idx = cell(size(trees)); %#ok
      for i = 1:numel(trees)
        typecheck(trees{i}, 'VertexArrayMotionPlanningTree');
        typecheck(tree_names{i}, 'char');
        idx{i} = num_vars + (1:trees{i}.num_vars); %#ok
        num_vars = num_vars + trees{i}.num_vars;
      end
      obj = obj@VertexArrayMotionPlanningTree(num_vars);
      obj.trees = trees;
      obj.tree_names = tree_names;
      obj.num_trees = numel(trees);
      obj.idx = idx; %#ok
      obj.weights = weights;
    end

    function d = distanceMetric(obj, q1, q_array)
      d = 0;
      for i = 1:obj.num_trees
        if obj.weights(i) > 0
          d = d + obj.weights(i)*obj.trees{i}.distanceMetric(q1(obj.idx{i},:), ...
                                                            q_array(obj.idx{i}, :));
        end
      end
    end

    function q = interpolate(obj, q1, q2, interpolation_factors)
      q = zeros(obj.num_vars, numel(interpolation_factors));
      for i = 1:obj.num_trees
        q(obj.idx{i},:) = obj.trees{i}.interpolate(q1(obj.idx{i}), ...
                                                   q2(obj.idx{i}), ...
                                                   interpolation_factors);
      end
    end

    function [obj, id_last] = init(obj, q_init)
      obj = init@VertexArrayMotionPlanningTree(obj, q_init);
      for i = 1:obj.num_trees
        [obj.trees{i}, id_last] = obj.trees{i}.init(q_init(obj.idx{i}));
      end
    end

    function valid = isValid(obj, q)
      valid = isValid@VertexArrayMotionPlanningTree(obj, q);
      for i = 1:obj.num_trees
        valid = valid && obj.trees{i}.isValid(q(obj.idx{i}));
      end
    end

    function [obj, id_new] = addVertex(obj, q, id_parent)
      obj = addVertex@VertexArrayMotionPlanningTree(obj,q,id_parent);
      for i = 1:obj.num_trees
        [obj.trees{i}, id_new] = obj.trees{i}.addVertex(q(obj.idx{i}), ...
                                                        id_parent);
      end
    end

    function q = randomSample(obj)
      q = zeros(obj.num_vars, 1);
      for i = 1:obj.num_trees
        q(obj.idx{i}) = obj.trees{i}.randomSample();
      end
    end
  end
end
