%GUROBI  Solve an LP, QP, QCP, SOCP, or MIP using the Gurobi Optimizer
%   result = GUROBI(model, params)
%
%   The Gurobi MATLAB interface can be used to solve optimization problems
%   of the form:
%
%   minimize           c'*x + x'*Q*x + alpha
%      x
%   subject to         A*x = b,
%                      l <= x <= u,
%                      some xj integral,
%                      some xk must lie within second order cones,
%                      x'*Qc*x + q'*x <= beta,
%                      some xi in SOS constraints.
%
%   Many of the model components listed here are optional.  For example,
%   integrality constraints may be omitted.
%
%   The model structure must contain the following fields:
%
%   model.A:  The linear constraint matrix. This must be a sparse matrix.
%
%   model.obj: The linear objective vector (c in the above problem
%              statement). You must specify one value for each column of A.
%              This must be a dense vector.
%
%   model.sense: The sense of the linear constraints. Allowed values are
%                '<', '=', or '>'. You must specify one value for each row
%                of A, or a single value to specify that all constraints
%                have the same sense. This must be a char array.
%
%   model.rhs: The right-hand side vector for the linear constraints (b in
%              the above problem statement). You must specify one value for
%              each row of A. This must be a dense vector.
%
%   The model structure may contain the following optional fields:
%
%   model.lb: The lower bounds on the variables. When present, you must
%             specify one value for each column of A. This must be a dense
%             vector. When absent, each variable has a lower bound of 0.
%
%   model.ub: The upper bounds on the variables. When present, you must
%             specify one value for each column of A. This must be a dense
%             vector. When absent, the variables have infinite upper
%             bounds.
%
%   model.vtype: The variable types. This char array is used to capture
%                variable integrality constraints. Allowed values are 'C'
%                (continuous), 'B' (binary), 'I' (integer), 'S'
%                (semi-continuous), or 'N' (semi-integer). When present,
%                you must specify one value for each column of A, or a
%                single value to specify that all variables should have the
%                same type. When absent, each variable is treated as
%                being continuous.
%
%   model.modelsense: The optimization sense. Allowed values are 'min'
%                     (minimize) or 'max' (maximize). When absent, the
%                     default model sense is minimization.
%
%   model.modelname: A name for the model. This is used in the log and
%                    result files.
%
%   model.objcon: The constant offset in the objective function (alpha in
%                 the above problem statement).
%
%   model.vbasis: The variable basis status vector. Used to provide an
%                 advanced starting point for the simplex algorithm. You
%                 would generally never concern yourself with contents of
%                 this array, but would instead simply copy it from the
%                 results of a previous optimization run. When present, you
%                 must specify one value for each column of A. This must be
%                 a dense vector.
%
%   model.cbasis: The constraint status vector. Used to provide an advanced
%                 starting point for the simplex algorithm. You would
%                 generally never concern yourself with contents of this
%                 array, but would instead simply copy it from the results
%                 of a previous optimization run. When present, you must
%                 specify one value for each row of A. This must be a dense
%                 vector.
%
%   model.Q: The quadratic objective matrix. When present, Q must be a
%            square matrix whose row and column counts are equal to the
%            number of columns of A. Q must be a sparse matrix.
%
%   model.cones:  The second-order cone constraints. A struct array. When
%                 present, each element in the array defines a single cone
%                 constraint:
%                     x(k)^2 >= sum(x(idx).^2),  x(k) >= 0.
%                 The constraint is defined via model.cones.index = [k idx]
%                 with the first entry in index corresponding to the index
%                 of the variable on the left-hand side of the constraint,
%                 and the remaining entries corresponding to the indices of
%                 variables on the right-hand side of the constraint.
%                 model.cones.index must be a dense vector.
%                 For example, the two second-order cone constraints:
%                  x(1)^2 >= sum(x([2 4]).^2),   x(1) >= 0,
%                  x(9)^2 >= sum(x([3 5 7]).^2), x(9) >= 0,
%                 are specified as
%                 model.cones(1).index = [1 2 4];
%                 model.cones(2).index = [9 3 5 7];
%
%   model.quadcon: The quadratic constraints. A struct array. When present,
%                  each element in the array defines a single quadratic
%                  constraint:
%                        x'*Qc*x + q'*x <= beta.
%                  The Qc matrix must be a square matrix whose row and
%                  column counts are equal to the number of columns of A.
%                  Qc must be a sparse matrix. It is stored in
%                  model.quadcon.Qc. The q vector defines the linear terms
%                  in the constraint. You must specify a value for q for
%                  each column of A. This must be a dense vector. It is
%                  stored in model.quadcon.q.  The scalar beta defines the
%                  right-hand side of the constraint. It is stored in
%                  model.quadcon.rhs. For example, the ith quadratic
%                  constraint is specified as
%                  model.quadcon(i).Qc = Qci;
%                  model.quadcon(i).q = qi;
%                  model.quadcon(i).rhs = betai;
%
%   model.sos: The Special Ordered Set (SOS) constraints. A struct
%              array.  When present, each element in the array defines
%              a single SOS constraint.  A SOS constraint can be of
%              type 1 or 2. This is specified via model.sos.type. A
%              type 1 SOS constraint is a set of variables for which
%              at most one variable in the set may take a value other
%              than zero. A type 2 SOS constraint is an ordered set of
%              variables where at most two variables in the set may
%              take non-zero values. If two take non-zeros values, they
%              must be contiguous in the ordered set.  The members of
%              an SOS constraint are specified by placing their
%              indices in model.sos.index. Optional weights associated
%              with SOS members may be defined in model.sos.weight.
%              For example, the two SOS constraints:
%              x(1) ~= 0 or x(2) ~= 0, x(1) ~= 0 or x(3) ~= 0,
%              are specified as
%              model.sos(1).index = [1 2];
%              model.sos(1).type = 1;
%              model.sos(1).weight = [1 2];
%              model.sos(2).index = [1 3];
%              model.sos(2).type = 1;
%              model.sos(2).weight = [1 2];
%
%   model.start: The MIP start vector. The MIP solver will attempt to
%                build an initial solution from this vector. When present,
%                you must specify a start value for each variable. This
%                must be a dense vector. Note that you can leave the
%                start value for a variable undefined---the MIP solver
%                will attempt to fill in values for the undefined start
%                values. This may be done by setting the start value
%                for that variable to nan. For example, to set the
%                start values for the first two variables and leave
%                the rest undefined do:
%                model.start = nan(size(model.A,2),1);
%                model.start(1:2) = [1; 0];
%
%  The params struct contains Gurobi parameters. A full list may be
%  found on the Parameter page of the reference manual:
%     http://gurobi.com/documentation/5.1/reference-manual/
%  For example:
%   params.outputflag = 0;          % Silence gurobi
%   params.resultfile = 'test.mps'; % Write out problem to MPS file
%   params.method     = 1;          % Use dual simplex method
%
%  The GUROBI function returns a struct result, with various results
%  stored in its named components. The specific results that are available
%  depend on the type of model that was solved, and the status of
%  the optimization.
%
%  The result struct will always contain the following field:
%
%   result.status:  The status of the optimization, returned as a string.
%                   The desired result is "OPTIMAL", which indicates an
%                   optimal solution to the model was found. Other status
%                   codes are possible (for example, if the model has no
%                   feasible solution). See the Optimization Status
%                   Codes page at
%                   http://gurobi.com/documentation/5.1/reference-manual/
%                   for a complete list.
%
%  The result struct may contain the following fields:
%
%   result.objval: The objective value of the computed solution.
%
%   result.objbound: The best available bound on the solution. This
%                    is a lower bound for minimization, and an
%                    upper bound for maximization.
%
%   result.runtime: The wall-clock time (in seconds) for the optimization.
%
%   result.itercount: The number of simplex iterations performed.
%
%   result.baritercount: The number of barrier iterations
%                        performed.
%
%   result.nodecount: The number of branch-and-cut nodes explored.
%
%   result.x: The computed solution. This array contains one entry for each
%             column of A.
%
%   result.slack: The constraint slack in the current solution. This array
%                 contains one entry for each row of A.
%
%   result.qcslack: The quadratic constraint slack in the current solution.
%                   This array contains one entry for each second-order
%                   cone constraint and one entry for each quadratic
%                   constraint. The slacks for the second-order cone
%                   constraints appear before the slacks for the
%                   quadratic constraints.
%
%   result.rc: The reduced cost in the current solution. This is only
%              available for continuous models. This array contains one
%              entry for each column of A.
%
%   result.pi: The dual values for the computed solution (also known as
%              shadow prices). This array contains one entry for each row
%              of A.
%
%   result.qcpi: The dual values associated with the quadratic contraints.
%                Only available when the parameter qcpdual is set to 1.
%                This array contains one entry for each second-order
%                cone constraint and one entry for each quadratic
%                constraint. The dual values for the second-order cone
%                constraints appear before the dual values for the
%                quadratic constraints.
%
%   result.vbasis: The variable basis status values for the computed
%                  optimal basis. You generally should not concern yourself
%                  with the contents of this array. If you wish to use an
%                  advanced start later, copy this array to model.vbasis.
%                  This array contains one entry for each column of A.
%
%   result.cbasis: The constraint basis status values for the computed
%                  optimal basis. If you wish to use an advanced start
%                  later, copy this array to model.cbasis. This array
%                  contains one entry for each column of A.
%
%   result.unbdray: An unbounded ray (for unbounded linear models only).
%                   This is a vector that, when added to any feasible
%                   solution, yields a new solution that is also feasible
%                   but improves the objective. Only available when
%                   parameter infunbdinfo is set to 1.
%
%   result.farkasdual: A Farkas infeasibility proof (for infeasible linear
%                      models only). This is a dual unbounded vector.
%                      Adding this vector to any feasible solution of
%                      the dual model yields a new solution that is also
%                      feasible but improves the dual objective.  Only
%                      available when parameter infunbdinfo is set to 1.
%
%   result.farkasproof: The magnitude of infeasibility violation in Farkas
%                       infeasibility proof (for infeasible linear models
%                       only). A Farkas infeasibility proof identifies a
%                       new constraint, obtained by taking a linear
%                       combination of the constraints in the model, that
%                       can never be satisfied. (the linear combination is
%                       available in the farkasdual attribute). This
%                       attribute indicates the magnitude of the violation
%                       of this aggregated constraint. Only available when
%                       parameter infunbdinfo is set to 1.
%
%  result.objbound: Best available bound on solution (lower bound
%                   for minimization, upper bound for maximization).
%
%  result.itercount: Number of simplex iterations performed.
%
%  result.baritercount: Number of barrier iterations performed.
%
%  result.nodecount: Number of branch-and-cut nodes explored.
%
% The following example shows how to form and solve the problem
%
% minimize    x +   y + 2 z
% subject to  x + 2 y + 3 z <= 4
%             x +   y       >= 1
%             x, y, z binary,
%
% with the Gurobi MATLAB interface.
%
% clear model;
% model.obj = [1; 1; 2];
% model.A = sparse([1 2 3; 1 1 0]);
% model.sense = ['<'; '>'];
% model.rhs = [4; 1];
% model.vtype = 'B';
%
% clear params;
% params.Presolve = 2;
% params.TimeLimit = 100;
%
% result = gurobi(model, params)
%
% disp(result.objval)
% disp(result.x)
%
% Copyright 2013, Gurobi Optimization, Inc.
%
% See also GUROBI_SETUP, SPARSE, STRUCT

