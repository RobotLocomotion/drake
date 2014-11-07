classdef MixedIntegerConvexProgram
  properties
    vars = struct();
    nv = 0;
    c = zeros(0, 1);
    Q = sparse(0, 0);
    A = zeros(0, 0);
    b = zeros(0, 1);
    Aeq = zeros(0, 0);
    beq = zeros(0, 1);
    quadcon = struct('Qc', {}, 'q', {}, 'rhs', {});
    objcon = 0;
    cones = struct('index', {});
    polycones = struct('index', {}, 'N', {});
    symbolic_constraints;
    symbolic_objective = 0;
  end

  properties (SetAccess = protected)
    using_symbolic = false;
    symbolic_vars = [];
  end

  methods
    function obj = MixedIntegerConvexProgram(using_symbolic)
      if nargin < 1
        using_symbolic = false;
      end
      if using_symbolic
        checkDependency('yalmip');
        obj.symbolic_constraints = lmi();
      end

      obj.using_symbolic = using_symbolic;
    end

    function obj = addVariable(obj, name, type_, size_, lb, ub, start_)
      % Build a struct to hold the sizes and indices of our decision variables
      % This is a new approach that I'm experimenting with, which should offer
      % a mix of the advantages of symbolic and matrix-based optimization
      % frameworks. The idea is that we have a single Matlab struct (named just
      % 'v' for convenience) and each variable in the optimization has a
      % corresponding named field in v. For each variable, we have subfields as
      % follows:
      % type: 'B', 'I', or 'C' for binary, integer, or continuous variables
      % size: a 2x1 vector describing the shape of the variable
      % i: the indices corresponding to the variable, as a matrix of the same size as 
      % the 'size' field above. 
      % lb: lower bound, as a matrix of the same size as 'size'
      % ub: upper bound, a matrix
      % start:  the initial values as a matrix of the same size as 'size'
      %
      % After optimization, there will be an additional field added to v, called
      % 'value', which will contain the final values after optimization.
      % 
      % The 'i' field of indices is useful because when
      % we actually set up the problem in gurobi or another solver, all of the
      % optimization variables are combined into one long vector. This index
      % field lets us easily address parts of that vector. For example, to set
      % the entry in a constraint matrix A corresponding to the jth row and kth column 
      % of variable 'foo' to 1, we can do the following:
      % A(1, v.foo.i(j,k)) = 1;
      if isfield(obj.vars, name)
        error('Drake:MixedIntegerConvexProgram:DuplicateVariableName', 'Cannot add a variable with the same name as an existing one');
      end
      obj.vars.(name) = struct();
      obj.vars.(name).type = type_;
      obj.vars.(name).size = size_;
      obj.vars.(name).i = reshape(obj.nv + (1:prod(obj.vars.(name).size)), obj.vars.(name).size);
      obj.nv = obj.nv + prod(obj.vars.(name).size);
      if isscalar(lb)
        lb = repmat(lb, obj.vars.(name).size);
      end
      if isscalar(ub)
        ub = repmat(ub, obj.vars.(name).size);
      end
      obj.vars.(name).lb = lb;
      obj.vars.(name).ub = ub;
      if nargin < 7
        start_ = [];
      end
      obj.vars.(name).start = nan(obj.vars.(name).size);
      if size(start_, 1) > obj.vars.(name).size(1)
        start_ = start_(1:obj.vars.(name).size(1),:,:);
      end
      if size(start_, 2) > obj.vars.(name).size(2)
        start_ = start_(:,1:obj.vars.(name).size(2),:);
      end
      obj.vars.(name).start(1:size(start_, 1), 1:size(start_, 2), 1:size(start_, 3)) = start_;

      % Add symbolic variables if we're doing that
      if obj.using_symbolic
        size_cell =num2cell(obj.vars.(name).size);
        if strcmp(obj.vars.(name).type, 'B')
          obj.vars.(name).symb = binvar(size_cell{:});
        elseif strcmp(obj.vars.(name).type, 'I')
          obj.vars.(name).symb = intvar(size_cell{:});
        else
          obj.vars.(name).symb = sdpvar(size_cell{:});
        end
        if isempty(obj.symbolic_vars)
          obj.symbolic_vars = reshape(obj.vars.(name).symb, [], 1);
        else
          obj.symbolic_vars = [obj.symbolic_vars; reshape(obj.vars.(name).symb, [], 1)];
        end
      end

      num_new_vars = prod(obj.vars.(name).size);
      obj.c = [obj.c; zeros(num_new_vars, 1)];
      obj.Q = [obj.Q, sparse(size(obj.Q, 1), num_new_vars);
               sparse(num_new_vars, num_new_vars + size(obj.Q, 2))];
      obj.A = [obj.A, zeros(size(obj.A, 1), num_new_vars)];
      obj.Aeq = [obj.Aeq, zeros(size(obj.Aeq, 1), num_new_vars)];
      if length(obj.quadcon) > 10
        obj.biped.warning_manager.warnOnce('Drake:MixedIntegerConvexProgram:ReallocatingQuadraticConstraints', 'Reallocating matrices for many quadratic constraints. This may be inefficient. If possible, try to finish adding new variables before you start adding quadratic constraints');
      end
      for j = 1:length(obj.quadcon)
        obj.quadcon(j).Qc = [obj.quadcon(j).Qc, sparse(size(obj.quadcon(j).Qc, 1), num_new_vars);
          sparse(num_new_vars, num_new_vars + size(obj.quadcon(j).Qc, 2))];
        obj.quadcon(j).q = [obj.quadcon(j).q; zeros(num_new_vars, 1)];
      end
    end

    function obj = addVariableIfNotPresent(obj, varargin)
      name = varargin{1};
      if isfield(obj.vars, name)
        return
      else
        obj = obj.addVariable(varargin{:});
      end
    end

    function obj = addLinearConstraints(obj, A, b, Aeq, beq)
      obj.A = [obj.A; A];
      obj.b = [obj.b; b];
      obj.Aeq = [obj.Aeq; Aeq];
      obj.beq = [obj.beq; beq];
    end

    function obj = addCones(obj, cones)
      obj.cones = [obj.cones, cones];
    end

    function obj = addConesByIndex(obj, idx)
      obj = obj.addCones(struct('index', mat2cell(idx, size(idx, 1), ones(1, size(idx, 2)))));
    end

    function obj = addPolyCones(obj, polycones)
      obj.polycones = [obj.polycones, polycones];
    end

    function obj = addPolyConesByIndex(obj, idx, N)
      if length(N) == 1
        N = repmat(N, 1, size(idx, 2));
      else
        assert(length(N) == size(idx, 2));
      end
      obj = obj.addPolyCones(struct('index', mat2cell(idx, size(idx, 1), ones(1, size(idx, 2))), 'N', num2cell(N)));
    end

    function obj = addConesOrPolyConesByIndex(obj, idx, N)
      if nargin < 3 || isempty(N)
        N = 0;
      end
      if all(N == 0)
        obj = obj.addConesByIndex(idx);
      else
        assert(all(N ~= 0), 'Cannot mix cones and polycones in the same call');
        obj = obj.addPolyConesByIndex(idx, N);
      end
    end


    function obj = addQuadcon(obj, quadcon)
      obj.quadcon = [obj.quadcon, quadcon];
    end

    function obj = setLinearCost(obj, c)
      obj.c = c;
    end

    function obj = setLinearCostEntries(obj, idx, val)
      obj.c(idx) = val;
    end

    function obj = addCost(obj, Q, c, objcon)
      if ~isempty(Q)
        obj.Q = obj.Q + Q;
      end
      if ~isempty(c)
        obj.c = obj.c + c;
      end
      if ~isempty(objcon)
        obj.objcon = obj.objcon + objcon;
      end
    end

    function obj = convertPolyCones(obj)
      nconstraints = sum([obj.polycones.N]);
      A = zeros(nconstraints, obj.nv);
      b = zeros(size(A, 1), 1);
      offset = 0;
      for j = 1:length(obj.polycones)
        assert(size(obj.polycones(j).index, 1) == 3, 'polygonal cone approximation only valid for cones with 3 entries (approximates x1 <= norm([x2; x3]))')
        N = obj.polycones(j).N;
        for k = 1:N
          th = (2*pi) / N * (k-1);
          ai = rotmat(th) * [1;0];
          A(offset+1, obj.polycones(j).index(2)) = ai(1);
          A(offset+1, obj.polycones(j).index(3)) = ai(2);
          A(offset+1, obj.polycones(j).index(1)) = -1;
          offset = offset+1;
        end
      end
      assert(offset == nconstraints);
      obj = obj.addLinearConstraints(A, b, [], []);
    end

    function [obj, ok, solvertime] = solve(obj)
      if obj.using_symbolic
        [obj, ok, solvertime] = obj.solveYalmip();
      else
        [obj, ok, solvertime] = obj.solveGurobi();
      end
    end

    function [obj, ok, solvertime] = solveGurobi(obj, params)
      if nargin < 2
        params = struct();
      end
      model = obj.getGurobiModel();
      result = gurobi(model, params);
      ok = ~(strcmp(result.status, 'INFEASIBLE') || strcmp(result.status, 'INF_OR_UNBD'));
      solvertime = result.runtime;
      obj = obj.extractResult(result.x);
    end

    function model = getGurobiModel(obj)
      obj = obj.convertPolyCones();

      var_names = fieldnames(obj.vars);

      model = struct();
      model.A = sparse([obj.A; obj.Aeq]);
      model.rhs = [obj.b; obj.beq];
      model.sense = [repmat('<', size(obj.A, 1), 1); repmat('=', size(obj.Aeq, 1), 1)];
      model.start = nan(obj.nv, 1);
      model.obj = obj.c;
      model.Q = obj.Q;
      if ~isempty(obj.quadcon)
        model.quadcon = obj.quadcon;
      end
      model.objcon = obj.objcon;
      if ~isempty(obj.cones)
        model.cones = obj.cones;
      end

      % Set up defaults so we can fill them in from v
      model.vtype = repmat('C', obj.nv, 1);
      model.lb = -inf(obj.nv, 1);
      model.ub = inf(obj.nv, 1);
      for j = 1:length(var_names)
        name = var_names{j};
        i = reshape(obj.vars.(name).i, [], 1);
        model.vtype(i) = obj.vars.(name).type;
        model.lb(i) = reshape(obj.vars.(name).lb, [], 1);
        model.ub(i) = reshape(obj.vars.(name).ub, [], 1);
        model.start(i) = reshape(obj.vars.(name).start, [], 1);
      end
    end

    function obj = extractResult(obj, x)
      var_names = fieldnames(obj.vars);
      % Extract the solution
      for j = 1:length(var_names)
        name = var_names{j};
        i = reshape(obj.vars.(name).i, [], 1);
        if obj.vars.(name).type == 'I' 
          obj.vars.(name).value = reshape(round(x(i)), obj.vars.(name).size);
        elseif obj.vars.(name).type == 'B'
          obj.vars.(name).value = reshape(logical(round(x(i))), obj.vars.(name).size);
        else
          obj.vars.(name).value = reshape(x(i), obj.vars.(name).size);
        end
      end
    end

    function [obj, ok, solvertime] = solveYalmip(obj)
      constraints = obj.symbolic_constraints;
      objective = obj.symbolic_objective;

      % Now add in any constraints or objectives which were declared non-symbolically
      objective = objective + obj.symbolic_vars' * obj.Q * obj.symbolic_vars + obj.c' * obj.symbolic_vars + obj.objcon;
      constraints = [constraints,...
        obj.Aeq * obj.symbolic_vars == obj.beq,...
        obj.A * obj.symbolic_vars <= obj.b,...
        ];
      var_names = fieldnames(obj.vars);
      for j = 1:length(var_names)
        name = var_names{j};
        constraints = [constraints,...
         obj.vars.(name).lb <= obj.vars.(name).symb <= obj.vars.(name).ub];
       end
      for j = 1:length(obj.quadcon)
        constraints = [constraints,...
          obj.symbolic_vars' * obj.quadcon(j).Qc * obj.symbolic_vars + obj.quadcon(j).q' * obj.symbolic_vars <= obj.quadcon(j).rhs];
      end
      for j = 1:length(obj.cones)
        constraints = [constraints,...
          cone(obj.symbolic_vars(obj.cones(j).index(2:end)), obj.symbolic_vars(obj.cones(j).index(1)))];
      end
      for j = 1:length(obj.polycones)
        constraints = [constraints,...
          polycone(obj.symbolic_vars(obj.polycones(j).index(2:end)), obj.symbolic_vars(obj.polycones(j).index(1)), obj.polycones(j).N)];
      end

      diagnostics = optimize(constraints, objective, sdpsettings('solver', 'gurobi'))
      ok = diagnostics.problem == 0;
      solvertime = diagnostics.solvertime;
      obj = obj.extractResult(double(obj.symbolic_vars));
    end
  end
end