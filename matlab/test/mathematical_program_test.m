function mathematical_program_test

% Simple example of calling MathematicalProgram through the Python bindings.

% QP test
prog = py.pydrake.solvers.mathematicalprogram.MathematicalProgram();
x = prog.NewContinuousVariables(int32(2), 'x');
prog.AddLinearConstraint(x.item(int32(0)) >= 1.0);
prog.AddLinearConstraint(x.item(int32(1)) >= 1.0);
prog.AddQuadraticCost(py.numpy.eye(int32(2)), py.numpy.zeros(int32(2)), x);
result = prog.Solve()

% Note: int32(0) is kSolutionFound; can't reference it directly.
assert(result == py.pydrake.solvers.mathematicalprogram.SolutionResult(int32(0)));

x_sol = prog.GetSolution(x);
assert(abs(x_sol.item(int32(0)) - 1.0) < 1e-6);
assert(abs(x_sol.item(int32(1)) - 1.0) < 1e-6);
