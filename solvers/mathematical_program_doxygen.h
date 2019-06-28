/** @addtogroup solvers
 * @{
 * Drake's MathematicalProgram class is used to solve the mathematical
 * optimization problem in the following form
 * <pre>
 *   minₓ f(x)
 *   s.t x ∈ S
 * </pre>
 * Depending on the formulation of the objective function f, and the structure
 * of the constraint set S, this optimization problem can be grouped into
 * different categories (linear programming, quadratic programming, nonconvex
 * nonlinear programming, etc). Drake will call suitable solvers for each
 * category of optimization problem.
 *
 * Drake wraps a number of commercial solvers (+ a few custom solvers) to
 * provide a common interface for convex optimization, mixed-integer convex
 * optimization, and other non-convex mathematical programs.
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
 * <b>Closed-form solutions</b>
 * When the mathematical problem is formulated as the following linear system
 * <pre>
 * find x
 * s.t Ax = b
 * </pre>
 * , then @ref drake::solvers::LinearSystemSolver "LinearSystemSolver" provides
 * efficient closed form solution.
 *
 * When the mathematical problem is formulated as the following (convex)
 * quadratic program with only linear equality constraint
 * <pre>
 * min 0.5 xᵀHx + aᵀx + b
 * s.t Ax = b
 * </pre>
 * , then @ref drake::solvers::EqualityConstrainedQPSolver
 * "EqualityConstraintQPSolver" provides efficient closed form solution.
 *
 * <b>Convex Optimization</b>
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
 * <tr><td>&dagger; <a href="https://www.gurobi.com/products/gurobi-optimizer">
 *    Gurobi</a></td>
 *    <td align="center">&diams;</td>
 *    <td align="center">&diams;</td>
 *    <td align="center">&diams;</td>
 *    <td></td>
 *    <td></td>
 *  </tr>
 * <tr><td>&dagger; <a href="https://www.mosek.com/products/mosek">
 *    Mosek</a></td>
 *    <td align="center">&diams;</td>
 *    <td align="center">&diams;</td>
 *    <td align="center">&diams;</td>
 *    <td align="center">&diams;</td>
 *    <td align="center">&diams;</td>
 * </tr>
 * <tr><td> <a href="https://github.com/cvxgrp/scs">
 *    SCS</a></td>
 *    <td align="center">&diams;</td>
 *    <td align="center">&diams;</td>
 *    <td align="center">&diams;</td>
 *    <td align="center">&diams;</td>
 *    <td align="center">&diams;</td>
 * </tr>
 * <tr><td> <a href="https://github.com/oxfordcontrol/osqp">
 *    OSQP</a></td>
 *    <td align="center">&diams;</td>
 *    <td align="center">&diams;</td>
 *    <td></td>
 *    <td></td>
 *    <td></td>
 * </tr>
 * <tr><td> <a href="https://github.com/coin-or/Csdp">
 *     CSDP</a></td>
 *     <td align="center">&diams;</td>
 *     <td></td>
 *     <td></td>
 *     <td align="center">&diams;</td>
 *     <td align="center">&diams;</td>
 * </tr>
 * </table>
 *
 * <b>Mixed-Integer Convex Optimization</b>
 *
 * <table>
 * <tr>
 *   <td>Solver</td>
 *   <td>MILP</a></td>
 *   <td>MIQP</a></td>
 *   <td>MISOCP</a></td>
 *   <td>MISDP</a></td>
 * </tr>
 * <tr><td>&dagger; <a href="https://www.gurobi.com/products/gurobi-optimizer">
 *    Gurobi</a></td>
 *    <td align="center">&diams;</td>
 *    <td align="center">&diams;</td>
 *    <td align="center">&diams;</td>
 *    <td></td>
 *  </tr>
 * <tr><td>&dagger; <a href="https://www.mosek.com/products/mosek">
 *    Mosek</a></td>
 *    <td align="center">&diams;</td>
 *    <td align="center">&diams;</td>
 *    <td align="center">&diams;</td>
 *    <td></td>
 * </tr>
 * </table>
 *
 * <b>Nonconvex Programming</b>
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
 * <tr><td>&dagger; &Dagger;
 *   <a href="http://www.sbsi-sol-optimize.com/asp/sol_product_snopt.htm">
 *    SNOPT</a></td></tr>
 *    <td align="center">&diams;</td>
 *    <td></td>
 *    <td></td>
 * <tr><td><a href="https://projects.coin-or.org/Ipopt">Ipopt</a></td></tr>
 *    <td align="center">&diams;</td>
 *    <td></td>
 *    <td></td>
 * <tr><td><a href="http://ab-initio.mit.edu/wiki/index.php/NLopt">
 *    NLopt</a></td></tr>
 *    <td align="center">&diams;</td>
 *    <td></td>
 *    <td></td>
 * <tr><td><a href="https://github.com/PositronicsLab/Moby">
 *    Moby LCP</a></td>
 *    <td></td>
 *    <td align="center">&diams;</td>
 *    <td></td>
 * <tr><td><a href="https://dreal.github.io/">dReal</a></td>
 *    <td></td>
 *    <td></td>
 *    <td align="center">&diams;</td>
 * </tr>
 * </table>
 *
 * &dagger; indicates that this is a commercial solver which requires a license
 * (note that some have free licenses for academics).
 *
 * &Dagger; Note that Drake's pre-compiled binary releases provide SNOPT.
 * @}
 */
