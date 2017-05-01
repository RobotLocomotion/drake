function testMathematicalProgram

% Simple example of calling MathematicalProgram through the python
% interface.

% Setup Python path for this matlab session (in case the PYTHONPATH was
% not properly set).  Note: move this to addpath_drake if it we expand 
% our use of the Python bindings.
P = py.sys.path;
drake_python_path = fullfile(fileparts(getDrakePath),'build','install','lib','python2.7','dist-packages');
if count(P,drake_python_path) == 0
    insert(P,int32(0),drake_python_path);
end


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
