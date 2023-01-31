"""Shim module that provides vestigial names for pydrake.solvers.

Prefer not to use this import path in new code; all of the code in
this module can be imported from pydrake.solvers directly.

This module will be deprecated at some point in the future.
"""

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
