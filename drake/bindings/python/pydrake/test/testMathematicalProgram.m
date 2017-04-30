function testMathematicalProgram

% Simple example of calling MathematicalProgram through the python
% interface.

% QP test
prog = py.pydrake.solvers.mathematicalprogram.MathematicalProgram();
x = prog.NewContinuousVariables(int32(2),'x');
prog.AddLinearConstraint(x.item(0) >= 1);
prog.AddLinearConstraint(x.item(1) >= 1);
prog.AddQuadraticCost(py.numpy.eye(2), py.numpy.zeros(2), x);
result = prog.Solve()

% Note: int32(0) is kSolutionFound; can't reference it directly.
assert(result == py.pydrake.solvers.mathematicalprogram.SolutionResult(int32(0)));

x_sol = prog.GetSolution(x);
assert(abs(x_sol.item(0)-1.0)<1e-6);
assert(abs(x_sol.item(1)-1.0)<1e-6);
