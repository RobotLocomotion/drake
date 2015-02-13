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
end
