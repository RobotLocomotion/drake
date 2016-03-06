classdef MotionPlanningTree
  properties (Constant)
    TRAPPED = 0;
    REACHED = 1;
    ADVANCED = 2;
  end
  
  properties
    N = 1e4; % Number of nodes for which memory should be pre-allocated
    n = 0; % Number of nodes in the tree
    max_edge_length = 0.1;
    max_length_between_constraint_checks = 0.01;
    line_color = 0.3*[1 1 1];
    lcmgl_name = '';
    lcmgl;
  end

  methods (Abstract)
    q = randomSample(obj);
    d = distanceMetric(obj, q1, q_array)
    is_valid = isValid(obj, q);
    q = getVertex(obj, id);
    q = interpolate(obj, q1, q2, interpolation_factors);
    id_parent = getParentId(obj, id)
    q = indexIntoArrayOfPoints(obj, q_array, index);
  end

  methods
    function obj = init(obj, q_init)
      obj.n = 0;
    end

    function [obj, path_ids, info] = rrt(obj, x_start, x_goal, options)
      % [obj, path_ids, info] = rrt(obj, x_start, x_goal, options) plans a path
      % between x_start and x_goal using the RRT algorithm [1].
      %
      % @param x_start  Initial state
      % @param x_goal   Final state
      % @param options  Structure that may containthe following fields:
      %                   * N - Maximum size of the tree [1e4]
      %                   * goal_bias - Fraction of the time which x_goal will
      %                       be chosen as the target for extension. [0.05]
      %                   * visualize - Logical scalar indicating whether or not
      %                       the tree should be drawn [false]
      %                   * display_after_every - If visualize is true,
      %                       this determines the number of vertices that are
      %                       added to the tree between drawing calls
      %
      % @retval obj       The MotionPlanningTree object with all the vertices
      %                   added during the execution of this method
      % @retval path_ids  Vector of path ids such that obj.extractPath(path_ids)
      %                   returns the path from x_start to x_goal.
      % @retval info      Exitflag indicating success (1) or failure (2) of the
      %                   algorithm.
      %
      % [1] S. M. LaValle. Rapidly-exploring random trees: A new tool for path
      % planning. TR 98-11, Computer Science Dept., Iowa State Univ., Oct. 1998.
      %
      defaultOptions.display_after_every = 50;
      defaultOptions.goal_bias = .05;
      defaultOptions.N = 1e4;
      defaultOptions.visualize = false;
      options = applyDefaults(options, defaultOptions);
      obj = obj.init(x_start);
      assert(obj.isValid(x_start))
      assert(obj.isValid(x_goal))
      last_drawn_edge_num = 1;
      info = 2;
      while obj.n < options.N
        try_goal = rand<options.goal_bias;
        if try_goal
          x_sample = x_goal;
        else
          x_sample = obj.randomSample();
        end
        [obj, status] = extend(obj, x_sample);
        if options.visualize && (mod(obj.n,options.display_after_every)==0 || (try_goal && status == obj.REACHED))
          obj = obj.drawTree(last_drawn_edge_num);
          last_drawn_edge_num = obj.n-1;
        end

        if try_goal && status == obj.REACHED
          info = 1;
          path_ids = obj.getPathToVertex(obj.n);
          if options.visualize
            drawPath(obj, path_ids);
          end
          break;
        end
      end
      path_ids = obj.getPathToVertex(obj.n);
    end

    function [TA,  path_ids_A, info, TB, path_ids_B] = rrtConnect(TA, x_start, x_goal, TB, options)
      % [TA,  path_ids_A, info, TB, path_ids_B] = rrtConnect(TA, x_start, x_goal, TB, options)
      % implements the RRT-Connect algorithm based on the work of Kuffner and
      % LaValle [1]. This is a bi-directional search with a greedy connection
      % heuristic. A balanced or un-balanced form can be used.
      %
      % @param TA   MotionPlanningTree object - the forward tree
      % @param x_start  The starting point in the search space
      % @param x_goal   The goal point in the search space
      % @param TB   MotionPlanningTree object - the backward tree 
      %             @default Copy of TA
      % @param options  Structure that may contain the following fields:
      %                   * N - Number of iterations [1e4]
      %                   * goal_bias - Fraction of the iterations in which a
      %                     connection attempt is made.
      %                   * balanced - True if the trees should kept roughly the
      %                     same size. [true]
      %                   * visualize - True if the trees should be drawn during
      %                     planning. [true]
      %
      % @retval TA  The forward tree after completion of the algorithm. Contains
      %             the full path from x_start to x_goal
      % @retval path_ids_A  Vector of path ids such that TA.extractPath(path_ids_A)
      %                     returns the path from x_start to x_goal.
      % @retval info  Exitflag indicating success (1) or failure (2) of the
      %               algorithm.
      % @retval TB  The backwards tree after completion of the algorithm.
      % @retval path_ids_B  Vector of path ids such that TB.extractPath(path_ids_B)
      %                     returns the path from x_goal to the connection with
      %                     TA.
      if nargin < 5, options = struct(); end
      defaultOptions.display_after_every = 50;
      defaultOptions.goal_bias = 1; 
      defaultOptions.N = 10000;
      defaultOptions.visualize = false;
      defaultOptions.balanced = true;
      options = applyDefaults(options, defaultOptions);
      assert(TA.isValid(x_start))
      TA = TA.init(x_start);
      if nargin < 4 || isempty(TB) 
        TB = TA;
        assert(TB.isValid(x_goal))
        TB = TB.init(x_goal);
      else
        typecheck(TB, 'MotionPlanningTree');
        assert(TB.isValid(x_goal))
        TB = TB.init(x_goal);
      end
      info = 2;
      last_drawn_edge_num_A = 1;
      last_drawn_edge_num_B = 1;
      for i = 1:options.N
        % Are we doing balanced RRT-Connect?
        if options.balanced
          % if so start with TA only when it's smaller
          TA_first = (TA.n < TB.n);
        else
          % If not, switch every time
          TA_first = mod(i,2) == 0;
        end
        if TA_first
          [TA, TB, path_ids_A, path_ids_B] = rrtConnectIteration(TA, TB, options.goal_bias);
        else
          [TB, TA, path_ids_B, path_ids_A] = rrtConnectIteration(TB, TA, options.goal_bias);
        end
        if options.visualize && (TA.n + TB.n - last_drawn_edge_num_A - last_drawn_edge_num_B > options.display_after_every || ~isempty(path_ids_A))
          TA = TA.drawTree(last_drawn_edge_num_A);
          TB = TB.drawTree(last_drawn_edge_num_B);
          drawnow
          last_drawn_edge_num_A = TA.n-1;
          last_drawn_edge_num_B = TB.n-1;
        end
        if ~isempty(path_ids_A)
          assert(~isempty(path_ids_B))
          info = 1;

          % Add the remainder of the solution path onto the forward tree
          [TA, id_last] = TA.addPath(TB,fliplr(path_ids_B(1:end-1)), path_ids_A(end));
          path_ids_A = TA.getPathToVertex(id_last);
          if options.visualize
            drawPath(TA, path_ids_A);
          end
          break;
        end
      end
    end

    function [TA, TB, path_ids_A, path_ids_B] = rrtConnectIteration(TA, TB, goal_bias)
      x_sample = TA.randomSample();
      [TA, status, id_new] = extend(TA, x_sample);
%       [TA, status, id_new] = connect(TA, x_sample);
      path_ids_A = [];
      path_ids_B = [];
      if status == TA.TRAPPED && ~isempty(id_new), keyboard; end
      if status ~= TA.TRAPPED && rand < goal_bias
        [TB, status, id_last] = connect(TB, TA.getVertex(id_new));

        if status == TB.REACHED
          path_ids_A = TA.getPathToVertex(id_new);
          path_ids_B = TB.getPathToVertex(id_last);
        end
      end
    end


    function [obj, q_new, id_near] = newPoint(obj, q)
      [d, id_near] = nearestNeighbor(obj, q);
      if d < obj.max_edge_length
        q_new = q;
      else
        alpha = obj.max_edge_length/d;
        q_new = obj.interpolate(obj.getVertex(id_near), q, alpha);
        d = obj.max_edge_length;
      end
      valid = obj.isValid(q_new);
      if ~valid
        [q_new, valid] = obj.attemptToMakeValid(q_new, valid);
      end
      if valid && d > obj.max_length_between_constraint_checks
        num_interpolated_checks = 2+ceil(d/obj.max_length_between_constraint_checks);
        interpolation_factors = linspace(0, 1, num_interpolated_checks);
        q_interp_array = obj.interpolate(obj.getVertex(id_near), q_new, interpolation_factors);
        for i=2:num_interpolated_checks-1
          valid = obj.isValid(obj.indexIntoArrayOfPoints(q_interp_array, i));
          if ~valid
            [~, valid] = obj.attemptToMakeValid(obj.indexIntoArrayOfPoints(q_interp_array, i), valid);
          end
          if ~valid, break; end
        end
      end
      if ~valid
        q_new = [];
      end
    end

    function [obj, status, id_new] = extend(obj, q)
      [obj, q_new, id_near] = newPoint(obj, q);
      if isempty(q_new) || obj.distanceMetric(q_new, q) > obj.distanceMetric(obj.getVertex(id_near), q)
        id_new = [];
        status = obj.TRAPPED;
      else
        [obj, id_new] = obj.addVertex(q_new, id_near);
        if q_new == q
          status = obj.REACHED;
        else
          status = obj.ADVANCED;
        end
      end
    end

    function [obj, status, id_last] = connect(obj, q)
      status = obj.ADVANCED;
      [obj, status, id_last] = extend(obj, q);
      status_tmp = status;
      while status_tmp == obj.ADVANCED
        [obj, status_tmp, id_last_tmp] = extend(obj, q);
        if status_tmp ~= obj.TRAPPED
          status = status_tmp;
          id_last = id_last_tmp;
        end
      end
    end

    function [q, valid] = attemptToMakeValid(obj, q, valid)
      % This default implementation does nothing. Child classes can coerce
      % samples in an overloaded version of this method. See
      % JointSpaceMotionPlanningTree for an example.
    end

    function q_path = extractPath(TA, path_ids_A, TB, path_ids_B)
      if nargin > 2
        q_path = [TA.getVertex(path_ids_A), TB.getVertex(fliplr(path_ids_B(1:end-1)))];
      else
        q_path = TA.getVertex(path_ids_A);
      end
    end


    function [obj_new, id_last] = recursiveConnectSmoothing(obj, path_ids, n_iterations, visualize)
      if nargin < 4 || isempty(visualize), visualize = false; end
      if nargin < 3
        path_length = numel(path_ids);
        [obj_start, id_last] = obj.init(obj.getVertex(path_ids(1)));
        if path_length == 1
          obj_new = obj_start;
        elseif path_length == 2
          [obj_new, id_last] = obj_start.addVertex(obj.getVertex(path_ids(2)), 1);
        else
          [obj_new, status, id_last] = obj_start.connect(obj.getVertex(path_ids(end)));
          if status ~= obj.REACHED
            mid_idx = randi([floor(path_length/3),2*floor(path_length/3)]);
            [obj_new, id_last] = obj.recursiveConnectSmoothing(path_ids(1:mid_idx));
            obj_new2 = obj.recursiveConnectSmoothing(path_ids(mid_idx+1:end));
            for i = 1:obj_new2.n
              [obj_new, id_last] = obj_new.addVertex(obj_new2.getVertex(i), id_last);
            end
          end
        end
      else
        for i = 1:n_iterations
          [obj, id_last] = recursiveConnectSmoothing(obj, path_ids);
          path_ids = obj.getPathToVertex(id_last);
          if visualize
            obj.drawPath(path_ids);
          end
        end
        obj_new = obj;
      end
    end

    function obj = setLCMGL(obj, name, color)
      obj.lcmgl = LCMGLClient(name);
      sizecheck(color, 3);
      obj.line_color = color;
      obj.lcmgl_name = name;
      obj.lcmgl.switchBuffers();
    end

    function [d, id_near] = nearestNeighbor(obj, q)
      d_all = obj.distanceMetric(q, obj.getVertex(1:obj.n));
      [d, id_near] = min(d_all);
    end

    function [obj, id] = addVertex(obj, q, parent_id)
      obj.n = obj.n + 1;
      id = obj.n;
    end

    function path_ids = getPathToVertex(obj, leaf_id)
      path_ids = leaf_id;
      while path_ids(1) > 1
        path_ids = [obj.getParentId(path_ids(1)),path_ids];
      end
    end

    function [obj, id_last] = addPath(obj, T_other, path_ids, id_parent)
      path_length = numel(path_ids);
      for id = reshape(path_ids,1,[]);
        [obj, id_parent] = obj.addVertex(T_other.getVertex(id), id_parent);
      end
      id_last = id_parent;
    end

    function obj = drawTree(obj, varargin)
    end

    function obj = drawPath(obj, varargin)
    end
  end
end