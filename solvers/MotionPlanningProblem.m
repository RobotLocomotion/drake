classdef MotionPlanningProblem
  % Defines a feasible motion planning problem using Constraint
  % class objects
  %
  % Note: a motion planning problem is *not* a NonlinearProgram
  % because, for instance, it is not described by a fixed number
  % of decision variables.  Instead, we require each point in a
  % solution to be feasible given all of the constraints, and
  % therefore have a NonlinearProgram which tests that feasibility
  % as a member variable.

  properties
    num_vars
    var_names
    constraints={};  % a cell array of Constraint objects
    % todo: add differential constraints?  control affine only?
    % todo: add objectives?
    x_inds={}; % a cell array of indices into the search space. obj.constraints{i}
               % takes x(obj.x_inds{i}) as its input
  end

  methods
    function obj = MotionPlanningProblem(num_vars,var_names)
      % @num_vars the dimension of the search space (e.g. state or
      % configuration space)
      % @var_names optional variable names

      if(nargin<2)
        var_names = cellfun(@(i) sprintf('x%d',i),num2cell((1:obj.num_vars)'),'UniformOutput',false);
      else
        if(~iscellstr(var_names) || numel(var_names) ~= num_vars)
          error('Drake:MotionPlanningProblem:InvalidArgument','Argument x_name should be a cell containing %d strings',obj.num_vars);
        end
        var_names = var_names(:);
      end
      obj.num_vars = num_vars;
      obj.var_names = var_names;

    end

    function [xtraj,info] = rrt(obj,x_start,x_goal,random_sample_fcn,options)
      % Simple RRT algorithm
      % @param x_start
      % @param x_goal
      % @param tolerance scalar distance that you must achieve to the goal
      % @param random_sample_fcn function handle of the form
      %    xs = random_sample_fcn
      % which generates a nX-by-1 random sample
      % @option distance_metric_fcn function handle of the form
      %    d = distance_metric_fcn(X,xs)
      % where X is a nX-by-N list of points, and xs is an nX-by-1 sample.
      % d should be a 1-by-N list of distances @default euclideanDistance
      % @option display_fcn @default drawFirstTwoCoordinates
      % @option display_after_every draws the tree after this many points
      % are added
      % @option max_edge_length a distance by which to trim
      % @options max_length_between_constraint_checks
      % @options goal_bias @default P(try_goal)=.05

      % parameters
      sizecheck(x_start,[obj.num_vars,1]);
      sizecheck(x_goal,[obj.num_vars,1]);
      typecheck(random_sample_fcn,'function_handle');

      % options
      if nargin<5, options=struct(); end
      defaultOptions.distance_metric_fcn = @MotionPlanningProblem.euclideanDistance;
      defaultOptions.display_fcn = @MotionPlanningProblem.drawFirstTwoCoordinates;
      defaultOptions.display_after_every = 50;
      defaultOptions.max_edge_length = inf;  % because i don't have any sense of scale here
      defaultOptions.max_length_between_constraint_checks = inf;
      defaultOptions.goal_bias = .05;
      options = applyDefaults(options,defaultOptions);
      typecheck(options.distance_metric_fcn,'function_handle');

      N = 10000;  % for pre-allocating memory
      V = repmat(x_start,1,N);  % vertices
      parent = nan(1,N-1); % edges

      n=2;
      n_at_last_display=0;
      info=2; xtraj=[];  % default return values
      while n<=N
        try_goal = rand<options.goal_bias;
        if try_goal
          xs = x_goal;
        else
          xs = random_sample_fcn();
          if ~checkConstraints(obj,xs), continue; end
        end

        d = options.distance_metric_fcn(V(:,1:n-1),xs);
        [dmin,imin] = min(d);

        if dmin>options.max_edge_length
          % then linearly interpolate along that distance
          xs = V(:,imin)+(options.max_edge_length/dmin)*(xs - V(:,imin));
          dmin = options.max_edge_length;
          try_goal = false;
        end

        if dmin>options.max_length_between_constraint_checks
          % then linearly interpolate along that distance
          num_intermediate_samples = 2+ceil(dmin/options.max_length_between_constraint_checks);
          xss = linspacevec(V(:,imin),xs,num_intermediate_samples);
          valid = true;
          for i=2:num_intermediate_samples-1
            if ~checkConstraints(obj,xss(:,i)),
              valid=false;
              break;
            end
          end
          if ~valid, continue; end
        end

        V(:,n) = xs;
        parent(n-1) = imin;

        if (try_goal)  % if I get here, then I successfully connected to the goal
          path = n;
          while path(1)>1
            path = [parent(path(1)-1),path];
          end
          xtraj = PPTrajectory(foh(1:length(path),V(:,path)));
          info = 1;
        end

        if mod(n,options.display_after_every)==0 || try_goal
          options.display_fcn(V(:,1:n),parent(1:n-1),n_at_last_display);
          drawnow;
          n_at_last_display = n;
        end

        if try_goal, return; end
        n=n+1;
      end
    end

    function obj = addConstraint(obj,constraint,x_inds)
      typecheck(constraint,'Constraint');
      if nargin<3, x_inds = 1:obj.num_vars; end
      sizecheck(x_inds,constraint.xdim);

      obj.x_inds{end+1} = x_inds;
      obj.constraints = vertcat(obj.constraints,{constraint});
    end

    function valid = checkConstraints(obj,x,varargin)
      valid = true;
      for i=1:length(obj.constraints)
        y = eval(obj.constraints{i},x(obj.x_inds{i}),varargin{:});
        if any(y<obj.constraints{i}.lb) || any(y>obj.constraints{i}.ub)
          valid = false;
          return;
        end
      end
    end

  end

  methods (Static=true)
    function d = euclideanDistance(X,xs)
      d = sqrt(sum((X-repmat(xs,1,size(X,2))).^2,1));
    end

    function drawFirstTwoCoordinates(V,parent,last_drawn_edge_num)
      line([V(1,(last_drawn_edge_num+2):end);V(1,parent((last_drawn_edge_num+1):end))],[V(2,last_drawn_edge_num+2:end);V(2,parent(last_drawn_edge_num+1:end))],'Color',0.3*[1 1 1]);
    end
  end

end
