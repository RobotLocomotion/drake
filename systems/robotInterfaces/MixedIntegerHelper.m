classdef MixedIntegerHelper
  properties
    v = struct();
    nv = 0;
    c;
    Q;
    A;
    b;
    Aeq;
    beq;
    quadcon;
    objcon;
    cones;
    polycones;

    frozen = false;
  end

  methods
    function obj = MixedIntegerHelper()
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
      if obj.frozen
        error('This problem has been frozen (by adding constraints) so you cannot add any more new variables');
      end
      obj.v.(name) = struct();
      obj.v.(name).type = type_;
      obj.v.(name).size = size_;
      obj.v.(name).i = reshape(obj.nv + (1:prod(obj.v.(name).size)), obj.v.(name).size);
      obj.nv = obj.nv + prod(obj.v.(name).size);
      if isscalar(lb)
        lb = repmat(lb, obj.v.(name).size);
      end
      if isscalar(ub)
        ub = repmat(ub, obj.v.(name).size);
      end
      obj.v.(name).lb = lb;
      obj.v.(name).ub = ub;
      if nargin < 7
        start_ = [];
      end
      obj.v.(name).start = nan(obj.v.(name).size);
      obj.v.(name).start(1:size(start_, 1), 1:size(start_, 2)) = start_;
    end

    function obj = freeze(obj)
      % Indicate that we're done adding new variables and ready to add constraints and cost
      assert(~obj.frozen, 'cannot freeze twice');
      obj.frozen = true;
      obj.c = zeros(obj.nv, 1);
      obj.Q = sparse(obj.nv, obj.nv);
      obj.A = zeros(0, obj.nv);
      obj.b = zeros(0, 1);
      obj.Aeq = zeros(0, obj.nv);
      obj.beq = zeros(0, 1);
      obj.quadcon = struct('Qc', {}, 'q', {}, 'rhs', {});
      obj.objcon = 0;
      obj.cones = struct('index', {});
      obj.polycones = struct('index', {}, 'N', {});
    end

    function obj = addLinearConstraints(obj, A, b, Aeq, beq)
      assert(obj.frozen, 'cannot add constraints or cost until helper is frozen');
      obj.A = [obj.A; A];
      obj.b = [obj.b; b];
      obj.Aeq = [obj.Aeq; Aeq];
      obj.beq = [obj.beq; beq];
    end

    function obj = addCones(obj, cones)
      assert(obj.frozen, 'cannot add constraints or cost until helper is frozen');
      obj.cones = [obj.cones, cones];
    end

    function obj = addConesByIndex(obj, idx)
      assert(obj.frozen, 'cannot add constraints or cost until helper is frozen');
      obj = obj.addCones(struct('index', mat2cell(idx, size(idx, 1), ones(1, size(idx, 2)))));
    end

    function obj = addPolyCones(obj, polycones)
      assert(obj.frozen, 'cannot add constraints or cost until helper is frozen');
      obj.polycones = [obj.polycones, polycones];
    end

    function obj = addPolyConesByIndex(obj, idx, N)
      assert(obj.frozen, 'cannot add constraints or cost until helper is frozen');
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
      assert(obj.frozen, 'cannot add constraints or cost until helper is frozen');
      obj.quadcon = [obj.quadcon, quadcon];
    end

    function obj = setLinearCost(obj, c)
      assert(obj.frozen, 'cannot add constraints or cost until helper is frozen');
      obj.c = c;
    end

    function obj = setLinearCostEntries(obj, idx, val)
      assert(obj.frozen, 'cannot add constraints or cost until helper is frozen');
      obj.c(idx) = val;
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

    function model = getGurobiModel(obj)
      obj = obj.convertPolyCones();

      var_names = fieldnames(obj.v);

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
        i = reshape(obj.v.(name).i, [], 1);
        model.vtype(i) = obj.v.(name).type;
        model.lb(i) = reshape(obj.v.(name).lb, [], 1);
        model.ub(i) = reshape(obj.v.(name).ub, [], 1);
        model.start(i) = reshape(obj.v.(name).start, [], 1);
      end
    end

    function obj = extractResult(obj, result)
      var_names = fieldnames(obj.v);
      % Extract the solution
      for j = 1:length(var_names)
        name = var_names{j};
        i = reshape(obj.v.(name).i, [], 1);
        if obj.v.(name).type == 'I' 
          obj.v.(name).value = reshape(round(result.x(i)), obj.v.(name).size);
        elseif obj.v.(name).type == 'B'
          obj.v.(name).value = reshape(logical(round(result.x(i))), obj.v.(name).size);
        else
          obj.v.(name).value = reshape(result.x(i), obj.v.(name).size);
        end
      end
    end


    function obj = solveGurobi(obj, params)
      model = obj.getGurobiModel();
      result = gurobi(model, params);
      obj = obj.extractResult(result);
    end
  end
end