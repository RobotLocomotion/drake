"""Shim module that provides vestigial names for pydrake.solvers.

Prefer not to use this import path in new code; all of the code in
this module can be imported from pydrake.solvers directly.
"""

from pydrake.common.deprecation import _warn_deprecated

from pydrake.solvers import (
    Binding,
    BoundingBoxConstraint,
    ChooseBestSolver,
    CommonSolverOption,
    Constraint,
    Cost,
    EvaluatorBase,
    ExponentialConeConstraint,
    GetAvailableSolvers,
    GetProgramType,
    L1NormCost,
    L2NormCost,
    LInfNormCost,
    LinearComplementarityConstraint,
    LinearConstraint,
    LinearCost,
    LinearEqualityConstraint,
    LinearMatrixInequalityConstraint,
    LorentzConeConstraint,
    MakeFirstAvailableSolver,
    MakeSolver,
    MathematicalProgram,
    MathematicalProgramResult,
    PerspectiveQuadraticCost,
    PositiveSemidefiniteConstraint,
    ProgramType,
    PyFunctionConstraint,
    QuadraticConstraint,
    QuadraticCost,
    RotatedLorentzConeConstraint,
    SolutionResult,
    Solve,
    SolverId,
    SolverInterface,
    SolverOptions,
    SolverType,
    VisualizationCallback,
)

_warn_deprecated(
    "Please import from the pydrake.solvers module directly, instead of the "
    f"deprecated {__name__} submodule.",
    date="2023-05-01", stacklevel=3)
