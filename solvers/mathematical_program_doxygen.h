/** @file
 Doxygen-only documentation for @ref solvers.  */

/** @addtogroup solvers
 * @{
 * Drake's MathematicalProgram class is used to solve the mathematical
 * optimization problem in the following form
 * <pre>
 *   minₓ f(x)
 *   s.t x ∈ S.
 * </pre>
 * Depending on the formulation of the objective function f, and the structure
 * of the constraint set S, this optimization problem can be grouped into
 * different categories (linear programming, quadratic programming, nonconvex
 * nonlinear programming, etc). Drake will call suitable solvers for each
 * category of optimization problem.
 *
 * Drake wraps a number of open source and commercial solvers
 * (+ a few custom solvers) to provide a common interface for convex
 * optimization, mixed-integer convex optimization, and other non-convex
 * mathematical programs.
 *
 * The MathematicalProgram class handles the coordination of decision variables,
 * objectives, and constraints.  The @ref drake::solvers::Solve() "Solve()"
 * method reflects on the accumulated objectives and constraints and will
 * dispatch to the most appropriate solver.  Alternatively, one can invoke
 * specific solver by instantiating its @ref drake::solvers::SolverInterface
 * "SolverInterface" and passing the MathematicalProgram directly to the @ref
 * drake::solvers::SolverInterface::Solve() "SolverInterface::Solve()" method.
 *
 * Our solver coverage still has many gaps, but is under active development.
 *
 * <h2>Closed-form solutions</h2>
 *
 * When the mathematical problem is formulated as the following linear system
 * <pre>
 * find x
 * s.t Ax = b,
 * </pre>
 * then @ref drake::solvers::LinearSystemSolver "LinearSystemSolver" provides
 * efficient closed form solution.
 *
 * When the mathematical problem is formulated as the following (convex)
 * quadratic program with only linear equality constraint
 * <pre>
 * min 0.5 xᵀHx + aᵀx + b
 * s.t Ax = b,
 * </pre>
 * then @ref drake::solvers::EqualityConstrainedQPSolver
 * "EqualityConstraintQPSolver" provides efficient closed form solution.
 *
 * <h2>Convex Optimization</h2>
 *
 * <table>
 * <tr>
 *   <td>Solver</td>
 *   <td><a href="https://en.wikipedia.org/wiki/Linear_programming">LP</a></td>
 *   <td><a href="https://en.wikipedia.org/wiki/Quadratic_programming">
 *     QP</a></td>
 *   <td><a href="https://en.wikipedia.org/wiki/Second-order_cone_programming">
 *     SOCP</a></td>
 *   <td><a href="https://en.wikipedia.org/wiki/Semidefinite_programming">
 *     SDP</a></td>
 *   <td><a href="https://en.wikipedia.org/wiki/Sum-of-squares_optimization">
 *     SOS</a></td>
 * </tr>
 * <tr><td><a href="https://www.gurobi.com/products/gurobi-optimizer">
 *    Gurobi</a> †</td>
 *    <td align="center">♦</td>
 *    <td align="center">♦</td>
 *    <td align="center">♦</td>
 *    <td></td>
 *    <td></td>
 *  </tr>
 * <tr><td><a href="https://www.mosek.com/products/mosek">
 *    Mosek</a> †</td>
 *    <td align="center">♦</td>
 *    <td align="center">♦</td>
 *    <td align="center">♦</td>
 *    <td align="center">♦</td>
 *    <td align="center">♦</td>
 * </tr>
 * <tr><td> <a href="https://github.com/coin-or/Clp">
 *    CLP</a></td>
 *    <td align="center">♦</td>
 *    <td></td>
 *    <td></td>
 *    <td></td>
 *    <td></td>
 * </tr>
 * <tr><td> <a href="https://github.com/coin-or/Csdp">
 *     CSDP</a></td>
 *     <td align="center">⟡</td>
 *     <td></td>
 *     <td align="center">⟡</td>
 *     <td align="center">⟡</td>
 *     <td align="center">⟡</td>
 * </tr>
 * <tr><td><a href="https://github.com/cvxgrp/scs">
 *    SCS</a></td>
 *    <td align="center">△</td>
 *    <td align="center">△</td>
 *    <td align="center">△</td>
 *    <td align="center">△</td>
 *    <td align="center">△</td>
 * </tr>
 * <tr><td><a href="https://github.com/oxfordcontrol/osqp">
 *    OSQP</a></td>
 *    <td></td>
 *    <td align="center">△</td>
 *    <td></td>
 *    <td></td>
 *    <td></td>
 * </tr>
 * <tr><td><a href="https://ccom.ucsd.edu/~optimizers/solvers/snopt/">
 *    SNOPT</a> † ‡</td>
 *     <td align="center">▢</td>
 *     <td align="center">▢</td>
 *     <td align="center">⬘</td>
 *     <td align="center">⬘</td>
 *     <td align="center">⬘</td>
 * </tr>
 * <tr><td><a href="https://projects.coin-or.org/Ipopt">Ipopt</a></td>
 *     <td align="center">▢</td>
 *     <td align="center">▢</td>
 *     <td align="center">⬘</td>
 *     <td align="center">⬘</td>
 *     <td align="center">⬘</td>
 * </tr>
 * <tr><td>
 *   <a href="http://ab-initio.mit.edu/wiki/index.php/NLopt">NLopt</a></td>
 *     <td align="center">▢</td>
 *     <td align="center">▢</td>
 *     <td align="center">⬘</td>
 *     <td align="center">⬘</td>
 *     <td align="center">⬘</td>
 * </tr>
 * </table>
 *
 * † This is a commercial solver which requires a license
 * (note that some have free licenses for academics).
 *
 * ‡ <a href="https://drake.mit.edu/installation.html">Drake's pre-compiled
 * binary releases</a> incorporate a private build of SNOPT that does not
 * require a license when invoked via Drake's SnoptSolver wrapper class.
 *
 * ♦ A preferred solver for the given category.
 *
 * ⟡ The native CSDP solver cannot handle free variables (namely all variables
 * have to be constrained within a cone). In Drake we apply special techniques
 * to handle free variables (refer to RemoveFreeVariableMethod for more
 * details). These heuristics can make the problem expensive to solve or
 * poorly conditioned.
 *
 * △ These solvers are not accurate. They implement ADMM algorithm, which
 * converges quickly to a low-accuracy solution, and requires many iterations to
 * achieve high accuracy.
 *
 * ▢ These solvers can solve the convex problems, but are not good at it. They
 * treat the convex problems as general nonlinear optimization problems.
 *
 * ⬘ These gradient-based solvers expect smooth gradients. These problems don't
 * have smooth gradients everywhere, hence even though the problem is convex,
 * these gradient-bases solvers might not converge to the globally optimal
 * solution.
 *
 * <h2>Mixed-Integer Convex Optimization</h2>
 *
 * <table>
 * <tr>
 *   <td>Solver</td>
 *   <td>MILP</a></td>
 *   <td>MIQP</a></td>
 *   <td>MISOCP</a></td>
 *   <td>MISDP</a></td>
 * </tr>
 * <tr><td><a href="https://www.gurobi.com/products/gurobi-optimizer">
 *    Gurobi</a> †</td>
 *    <td align="center">♦</td>
 *    <td align="center">♦</td>
 *    <td align="center">♦</td>
 *    <td></td>
 *  </tr>
 * <tr><td><a href="https://www.mosek.com/products/mosek">
 *    Mosek</a> †</td>
 *    <td align="center">♦</td>
 *    <td align="center">♦</td>
 *    <td align="center">♦</td>
 *    <td></td>
 * </tr>
 * <tr><td>
 * @ref drake::solvers::MixedIntegerBranchAndBound "naive branch-and-bound"
 * </td>
 *    <td align="center">◊</td>
 *    <td align="center">◊</td>
 *    <td align="center">◊</td>
 *    <td align="center">◊</td>
 * </table>
 *
 * † This is a commercial solver which requires a license
 * (note that some have free licenses for academics).
 *
 * ♦ A preferred solver for the given category.
 *
 * ◊ The naive solver's usefulness is likely restricted to small-sized problems
 * with dozens of binary variables. We implement only the basic branch-and-bound
 * algorithm, without cutting planes nor advanced branching heuristics.
 *
 * <h2>Nonconvex Programming</h2>
 *
 * <table>
 * <tr>
 *   <td>Solver</td>
 *   <td><a href="https://en.wikipedia.org/wiki/Nonlinear_programming">
 *     Nonlinear Program</a></td>
 *   <td><a href="https://en.wikipedia.org/wiki/Linear_complementarity_problem">
 *   LCP</a></td>
 *   <td><a href="https://en.wikipedia.org/wiki/Satisfiability_modulo_theories">
 *     SMT</a></td>
 * </tr>
 * <tr><td><a href="https://ccom.ucsd.edu/~optimizers/solvers/snopt/">
 *    SNOPT</a> † ‡</td></tr>
 *    <td align="center">♦</td>
 *    <td>⟐</td>
 *    <td></td>
 * <tr><td><a href="https://projects.coin-or.org/Ipopt">Ipopt</a></td></tr>
 *    <td align="center">♦</td>
 *    <td>⟐</td>
 *    <td></td>
 * <tr><td><a href="http://ab-initio.mit.edu/wiki/index.php/NLopt">
 *    NLopt</a></td></tr>
 *    <td align="center">♦</td>
 *    <td>⟐</td>
 *    <td></td>
 * <tr><td><a href="https://github.com/PositronicsLab/Moby">
 *    Moby LCP</a></td>
 *    <td></td>
 *    <td align="center">♦</td>
 *    <td></td>
 * <tr><td><a href="https://dreal.github.io/">dReal</a></td>
 *    <td></td>
 *    <td></td>
 *    <td align="center">♦</td>
 * </tr>
 * </table>
 *
 * † This is a commercial solver which requires a license
 * (note that some have free licenses for academics).
 *
 * ‡ <a href="https://drake.mit.edu/installation.html">Drake's pre-compiled
 * binary releases</a> incorporate a private build of SNOPT that does not
 * require a license when invoked via Drake's SnoptSolver wrapper class.
 *
 * ♦ A preferred solver for the given category.
 * ⟐ SNOPT/IPOPT/NLOPT might be able to solve LCP, but they are not the
 * preferred solver.
 *
 * @}
 */
