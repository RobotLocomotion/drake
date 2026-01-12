#pragma once

#include <functional>
#include <vector>

#include "drake/examples/rod2d/constraint_problem_data.h"
#include "drake/solvers/moby_lcp_solver.h"

namespace drake {
namespace examples {
namespace rod2d {

/// Solves constraint problems for constraint forces. Specifically, given
/// problem data corresponding to a rigid or multi-body system constrained
/// bilaterally and/or unilaterally and acted upon by friction, this class
/// computes the constraint forces.
///
/// This problem can be formulated as a mixed linear complementarity problem
/// (MLCP)- for 2D problems with Coulomb friction or 3D problems without Coulomb
/// friction- or a mixed complementarity problem (for 3D problems with
/// Coulomb friction). We use a polygonal approximation (of selectable accuracy)
/// to the friction cone, which yields a MLCP in all cases.
///
/// Existing algorithms for solving MLCPs, which are based upon algorithms for
/// solving "pure" linear complementarity problems (LCPs), solve smaller classes
/// of problems than the corresponding LCP versions. For example, Lemke's
/// Algorithm, which is provably able to solve the impacting problems covered by
/// this class, can solve LCPs with copositive matrices [Cottle 1992] but MLCPs
/// with only positive semi-definite matrices (the latter is a strict subset of
/// the former) [Sargent 1978].
///
/// Rather than using one of these MLCP algorithms, we instead transform the
/// problem into a pure LCP by first solving for the bilateral constraint
/// forces. This method yields an implication of which the user should be aware.
/// Bilateral constraint forces are computed before unilateral constraint
/// forces: the constraint forces will not be evenly distributed between
/// bilateral and unilateral constraints (assuming such a distribution were even
/// possible).
///
/// For the normal case of unilateral constraints admitting degrees of
/// freedom, the solution methods in this class support "softening" of the
/// constraints, as described in [Lacoursiere 2007] via the constraint force
/// mixing (CFM) and error reduction parameter (ERP) parameters that are now
/// ubiquitous in game multi-body dynamics simulation libraries.
///
/// - [Cottle 1992]   R. W. Cottle, J.-S. Pang, and R. E. Stone. The Linear
///                   Complementarity Problem. SIAM Classics in Applied
///                   Mathematics, 1992.
/// - [Judice 1992]   J. J. Judice, J. Machado, and A. Faustino. An extension of
///                   the Lemke's method for the solution of a generalized
///                   linear complementarity problem. In System Modeling and
///                   Optimization (Lecture Notes in Control and Information
///                   Sciences), Springer-Verlag, 1992.
/// - [Lacoursiere 2007]  C. Lacoursiere. Ghosts and Machines: Regularized
///                       Variational Methods for Interactive Simulations of
///                       Multibodies with Dry Frictional Contacts.
///                       Ph. D. thesis (Umea University), 2007.
/// - [Sargent 1978]  R. W. H. Sargent. An efficient implementation of the Lemke
///                   Algorithm and its extension to deal with upper and lower
///                   bounds. Mathematical Programming Study, 7, 1978.
///
/// @tparam_double_only
template <typename T>
class ConstraintSolver {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ConstraintSolver);

  /// Structure used to convert a mixed linear complementarity problem to a
  /// pure linear complementarity problem (by solving for free variables).
  struct MlcpToLcpData {
    /// Decomposition of the Delassus matrix GM⁻¹Gᵀ, where G is the bilateral
    /// constraint matrix and M is the system generalized inertia matrix.
    Eigen::CompleteOrthogonalDecomposition<MatrixX<T>> delassus_QTZ;

    /// A function pointer for solving linear systems using MLCP "A" matrix
    /// (see @ref Velocity-level-MLCPs).
    std::function<MatrixX<T>(const MatrixX<T>&)> A_solve;

    /// A function pointer for solving linear systems using only the upper left
    /// block of A⁺ in the MLCP (see @ref Velocity-level-MLCPs), where A⁺ is
    /// a singularity-robust pseudo-inverse of A, toward exploiting operations
    /// with zero blocks. For example:<pre>
    /// A⁺ | b |
    ///    | 0 |</pre> and<pre>
    /// A⁺ | B |
    ///    | 0 |
    /// </pre>
    /// where `b ∈ ℝⁿᵛ` is an arbitrary vector of dimension equal to the
    /// generalized velocities and `B ∈ ℝⁿᵛˣᵐ` is an arbitrary matrix with
    /// row dimension equal to the dimension of the generalized velocities and
    /// arbitrary number of columns (denoted `m` here).
    std::function<MatrixX<T>(const MatrixX<T>&)> fast_A_solve;
  };

  ConstraintSolver();

  // TODO(edrumwri): Describe conditions under which it is safe to replace
  // A⁻¹ by a pseudo-inverse.
  // TODO(edrumwri): Constraint solver needs to use monogram notation throughout
  // to make it consistent with the rest of Drake's multibody dynamics
  // documentation (see Issue #9080).

  /// @name Velocity-level constraint problems formulated as MLCPs.
  /// @anchor Velocity-level-MLCPs
  /// Constraint problems can be posed as mixed linear complementarity problems
  /// (MLCP), which are problems that take the form:<pre>
  /// (a)    Au + X₁y + a = 0
  /// (b)  X₂u + X₃y + x₄ ≥ 0
  /// (c)               y ≥ 0
  /// (d) vᵀ(x₄ + X₂u + X₃y) = 0
  /// </pre>
  /// where `u` are "free" variables, `y` are constrained variables, `A`, `X₁`,
  /// `X₂`, and `X₃` are given matrices (we label only the most important
  /// variables without subscripts to make them stand out) and `a` and `x₄` are
  /// given vectors. If `A` is nonsingular, `u` can be solved for:<pre>
  /// (e) u = -A⁻¹ (a + X₁y)
  /// </pre>
  /// allowing the mixed LCP to be converted to a "pure" LCP `(qq, MM)` by:<pre>
  /// (f) qq = x₄ - X₂A⁻¹a
  /// (g) MM = X₃ - X₂A⁻¹X₁
  /// </pre>
  /// This pure LCP can then be solved for `y` such that:<pre>
  /// (h)     MMv + qq ≥ 0
  /// (i)            y ≥ 0
  /// (j) yᵀ(MMv + qq) = 0
  /// </pre>
  /// and this value for `y` can be substituted into (e) to obtain the value
  /// for `u`.
  ///
  /// <h3>An MLCP-based impact model:</h3>
  /// Consider the following problem formulation of a multibody dynamics
  /// impact model (taken from [Anitescu 1997]). In a simulator, one could use
  /// this model when a collision is detected in order to compute an
  /// instantaneous change in velocity.
  /// <pre>
  /// (1) | M  -Gᵀ  -Nᵀ  -Dᵀ  0  -Lᵀ | | v⁺ | + |-Mv⁻ | = | 0  |
  ///     | G   0    0    0   0   0  | | fG | + |  kᴳ | = | 0  |
  ///     | N   0    0    0   0   0  | | fN | + |  kᴺ | = | x₅ |
  ///     | D   0    0    0   E   0  | | fD | + |  kᴰ | = | x₆ |
  ///     | 0   0    μ   -Eᵀ  0   0  | |  λ | + |   0 | = | x₇ |
  ///     | L   0    0    0   0   0  | | fL | + |  kᴸ | = | x₈ |
  /// (2) 0 ≤ fN  ⊥  x₅ ≥ 0
  /// (3) 0 ≤ fD  ⊥  x₆ ≥ 0
  /// (4) 0 ≤ λ   ⊥  x₇ ≥ 0
  /// (5) 0 ≤ fL  ⊥  x₈ ≥ 0
  /// </pre>
  /// Here, the velocity variables
  /// v⁻ ∈ ℝⁿᵛ, v ∈ ℝⁿᵛ⁺ correspond to the velocity of the system before and
  /// after impulses are applied, respectively. More details will be forthcoming
  /// but key variables are `M ∈ ℝⁿᵛˣⁿᵛ`, the generalized inertia matrix;
  /// `G ∈ ℝⁿᵇˣⁿᵛ`, `N ∈ ℝⁿᶜˣⁿᵛ`, `D ∈ ℝⁿᶜᵏˣⁿᵛ`, and `L ∈ ℝⁿᵘˣⁿᵛ` correspond to
  /// Jacobian matrices for various constraints (joints, contact, friction,
  /// generic unilateral constraints, respectively); `μ ∈ ℝⁿᶜˣⁿᶜ` is a diagonal
  /// matrix comprised of Coulomb friction coefficients; `E ∈ ℝⁿᶜᵏˣⁿᶜ` is a
  /// binary matrix used to linearize the friction cone (necessary to make
  /// this into a *linear* complementarity problem); `fG ∈ ℝⁿᵇ`, `fN ∈ ℝⁿᶜ`,
  /// `fD ∈ ℝⁿᶜᵏ`, and `fL ∈ ℝⁿᵘ` are constraint impulses; `λ ∈ ℝⁿᶜ`, `x₅`,
  /// `x₆`, `x₇`, and `x₈` can be viewed as mathematical programming "slack"
  /// variables; and `kᴳ ∈ ℝⁿᵇ`, `kᴺ ∈ ℝⁿᶜ`, `kᴰ ∈ ℝⁿᶜᵏ`, `kᴸ ∈ ℝⁿᵘ`
  /// allow customizing the problem to, e.g., correct constraint violations and
  /// simulate restitution. See "Variable definitions" in the local README for
  /// complete definitions of `nv`, `nc`, `nb`, etc.
  ///
  /// From the notation above in Equations (a)-(d), we can convert the MLCP
  /// to a "pure" linear complementarity problem (LCP), which is easier to
  /// solve, at least for active-set-type mathematical programming
  /// approaches:<pre>
  ///  A ≡ | M  -Ĝᵀ|   a ≡ |-Mv⁻ |   X₁ ≡ |-Nᵀ  -Dᵀ  0  -Lᵀ |
  ///      | Ĝ   0 |       |  kᴳ |        | 0    0   0   0  |
  ///
  /// X₂ ≡ | N   0 |   b ≡ |  kᴺ |   X₃ ≡ | 0    0   0   0  |
  ///      | D   0 |       |  kᴰ |        | 0    0   E   0  |
  ///      | 0   0 |       |  0  |        | μ   -Eᵀ  0   0  |
  ///      | L   0 |       |  kᴸ |        | 0    0   0   0  |
  ///
  ///  u ≡ | v⁺ |      y ≡ | fN  |
  ///      | fG |          | fD  |
  ///                      |  λ  |
  ///                      | fL  |
  /// </pre>
  /// Where applicable, ConstraintSolver computes solutions to linear equations
  /// with rank-deficient `A` (e.g., `AX₅ = x₆`) using a least squares approach
  /// (the complete orthogonal factorization). Now, using Equations (f) and (g)
  /// and defining `C` as the nv × nv-dimensional upper left block of
  /// `A⁻¹` (`nv` is the dimension of the generalized velocities) the pure
  /// LCP `(qq,MM)` is defined as:<pre>
  /// MM ≡ | NCNᵀ  NCDᵀ   0   NCLᵀ |
  ///      | DCNᵀ  DCDᵀ   E   DCLᵀ |
  ///      | μ      -Eᵀ   0   0    |
  ///      | LCNᵀ  LCDᵀ   0   LCLᵀ |
  ///
  /// qq ≡ | kᴺ - |N 0ⁿᵛ⁺ⁿᵇ|A⁻¹a |
  ///      | kᴰ - |D 0ⁿᵛ⁺ⁿᵇ|A⁻¹a |
  ///      |       0             |
  ///      | kᴸ - |L 0ⁿᵛ⁺ⁿᵇ|A⁻¹a |
  /// </pre>
  /// where `nb` is the number of bilateral constraint equations. The solution
  /// `y` will then take the form:<pre>
  /// y ≡ | fN |
  ///     | fD |
  ///     | λ  |
  ///     | fL |
  /// </pre>
  /// The key variables for using the MLCP-based formulations are the matrix `A`
  /// and vector `a`, as seen in documentation of MlcpToLcpData and the
  /// following methods. During its operation, ConstructBaseDiscretizedTimeLcp()
  /// constructs (and returns) functions for solving `AX=B`, where `B` is a
  /// given matrix and `X` is an unknown matrix. UpdateDiscretizedTimeLcp()
  /// computes and returns `a` during its operation.
  ///
  /// <h3>Another use of the MLCP formulation (discretized multi-body dynamics
  /// with contact and friction):</h3>
  ///
  /// Without reconstructing the entire MLCP, we now show a very similar
  /// formulation to solve the problem of discretizing
  /// the multi-body dynamics equations with contact and friction. This
  /// particular formulation provides several nice features: 1) the formulation
  /// is semi-implicit and models compliant contact efficiently, including both
  /// sticking contact and contact between very stiff surfaces; 2) all
  /// constraint forces are computed in Newtons (typical "time stepping
  /// methods" require considerable care to correctly compare constraint forces,
  /// which are impulsive, and non-constraint forces, which are non-impulsive);
  /// and 3) can be made almost symplectic by choosing a
  /// representation and computational coordinate frame that minimize
  /// velocity-dependent forces (thereby explaining the extreme stability of
  /// software like ODE and Bullet that computes dynamics in body frames
  /// (minimizing the magnitudes of velocity-dependent forces) and provides
  /// the ability to disable gyroscopic forces.
  ///
  /// The discretization problem replaces the meaning of v⁻ and v⁺ in the MLCP
  /// to mean the generalized velocity at time t and the generalized velocity
  /// at time t+h, respectively, for discretization quantum h (or, equivalently,
  /// integration step size h).  The LCP is adjusted to the form:<pre>
  /// MM ≡ | hNCNᵀ+γᴺ  hNCDᵀ   0   hNCLᵀ    |
  ///      | hDCNᵀ     hDCDᵀ   E   hDCLᵀ    |
  ///      | μ         -Eᵀ     0   0        |
  ///      | hLCNᵀ     hLCDᵀ   0   hLCLᵀ+γᴸ |
  ///
  /// qq ≡ | kᴺ - |N 0ⁿᵛ⁺ⁿᵇ|A⁻¹a |
  ///      | kᴰ - |D 0ⁿᵛ⁺ⁿᵇ|A⁻¹a |
  ///      |       0             |
  ///      | kᴸ - |L 0ⁿᵛ⁺ⁿᵇ|A⁻¹a |</pre>
  /// where `γᴺ`, `γᴸ`, `kᴺ`, `kᴸ`, and `a` are all functions of `h`;
  /// documentation that describes how to update these (and dependent) problem
  /// data to attain desired constraint stiffness and dissipation is
  /// forthcoming.
  ///
  /// The procedure one uses to formulate and solve this discretization problem
  /// is:
  /// -# Call ConstructBaseDiscretizedTimeLcp()
  /// -# Select an integration step size, dt
  /// -# Compute `kᴺ` and `kᴸ` in the problem data, accounting for dt as
  ///    necessary
  /// -# Call UpdateDiscretizedTimeLcp(), obtaining MM and qq that encode the
  ///    linear complementarity problem
  /// -# Solve the linear complementarity problem
  /// -# If LCP solved, quit.
  /// -# Reduce dt and repeat the process from 3. until success.
  ///
  /// The solution to the LCP can be used to obtain the constraint forces via
  /// PopulatePackedConstraintForcesFromLcpSolution().
  ///
  /// <h3>Obtaining the generalized constraint forces:</h3>
  ///
  /// Given the constraint forces, which have been obtained either through
  /// SolveImpactProblem() (in which case the forces are impulsive) or through
  /// direct solution of the LCP corresponding to the discretized multibody
  /// dynamics problem, followed by
  /// PopulatePackedConstraintForcesFromLcpSolution() (in which cases the forces
  /// are non-impulsive), the generalized forces/impulses due to the constraints
  /// can then be acquired via ComputeGeneralizedForceFromConstraintForces().
  // @{
  /// Computes the base time-discretization of the system using the problem
  /// data, resulting in the `MM` and `qq` described in
  /// @ref Velocity-level-MLCPs; if `MM` and `qq` are modified no further, the
  /// LCP corresponds to an impact problem (i.e., the multibody dynamics problem
  /// would not be discretized). The data output (`mlcp_to_lcp_data`, `MM`, and
  /// `qq`) can be updated using a particular time step in
  /// UpdateDiscretizedTimeLcp(), resulting in a non-impulsive problem
  /// formulation. In that case, the multibody dynamics equations *are*
  /// discretized, as described in UpdateDiscretizedTimeLcp().
  /// @note If you really do wish to solve an impact problem, you should use
  ///       SolveImpactProblem() instead.
  /// @param problem_data the constraint problem data.
  /// @param[out] mlcp_to_lcp_data a pointer to a valid MlcpToLcpData object;
  ///             the caller must ensure that this pointer remains valid through
  ///             the constraint solution process.
  /// @param[out] MM a pointer to a matrix that will contain the parts of the
  ///             LCP matrix not dependent upon the time step on return.
  /// @param[out] qq a pointer to a vector that will contain the parts of the
  ///             LCP vector not dependent upon the time step on return.
  /// @pre `mlcp_to_lcp_data`, `MM`, and `qq` are non-null on entry.
  /// @see UpdateDiscretizedTimeLcp()
  static void ConstructBaseDiscretizedTimeLcp(
      const ConstraintVelProblemData<T>& problem_data,
      MlcpToLcpData* mlcp_to_lcp_data, MatrixX<T>* MM, VectorX<T>* qq);

  /// Updates the time-discretization of the LCP initially computed in
  /// ConstructBaseDiscretizedTimeLcp() using the problem data and time step
  /// `h`. Solving the resulting pure LCP yields non-impulsive constraint forces
  /// that can be obtained from PopulatePackedConstraintForcesFromLcpSolution().
  /// @param problem_data the constraint problem data.
  /// @param[out] mlcp_to_lcp_data a pointer to a valid MlcpToLcpData object;
  ///             the caller must ensure that this pointer remains valid through
  ///             the constraint solution process.
  /// @param[out] a the vector corresponding to the MLCP vector `a`, on return.
  /// @param[out] MM a pointer to the updated LCP matrix on return.
  /// @param[out] qq a pointer to the updated LCP vector on return.
  /// @pre `mlcp_to_lcp_data`, `a`, `MM`, and `qq` are non-null on entry.
  /// @see ConstructBaseDiscretizedTimeLcp()
  static void UpdateDiscretizedTimeLcp(
      const ConstraintVelProblemData<T>& problem_data, double h,
      MlcpToLcpData* mlcp_to_lcp_data, VectorX<T>* a, MatrixX<T>* MM,
      VectorX<T>* qq);

  /// Solves the impact problem described above.
  /// @param problem_data The data used to compute the impulsive constraint
  ///            forces.
  /// @param cf The computed impulsive forces, on return, in a packed storage
  ///           format. The first `nc` elements of `cf` correspond to the
  ///           magnitudes of the contact impulses applied along the normals of
  ///           the `nc` contact points. The next elements of `cf`
  ///           correspond to the frictional impulses along the `r` spanning
  ///           directions at each point of contact. The first `r`
  ///           values (after the initial `nc` elements) correspond to the first
  ///           contact, the next `r` values correspond to the second contact,
  ///           etc. The next `ℓ` values of `cf` correspond to the impulsive
  ///           forces applied to enforce unilateral constraint functions. The
  ///           final `b` values of `cf` correspond to the forces applied to
  ///           enforce generic bilateral constraints. This packed storage
  ///           format can be turned into more useful representations through
  ///           ComputeGeneralizedForceFromConstraintForces() and
  ///           CalcContactForcesInContactFrames(). `cf` will be resized as
  ///           necessary.
  /// @pre Constraint data has been computed.
  /// @throws std::exception if the constraint forces cannot be computed
  ///         (due to, e.g., the effects of roundoff error in attempting to
  ///         solve a complementarity problem); in such cases, it is
  ///         recommended to increase regularization and attempt again.
  /// @throws std::exception if `cf` is null.
  void SolveImpactProblem(const ConstraintVelProblemData<T>& problem_data,
                          VectorX<T>* cf) const;

  /// Populates the packed constraint force vector from the solution to the
  /// linear complementarity problem (LCP) constructed using
  /// ConstructBaseDiscretizedTimeLcp() and UpdateDiscretizedTimeLcp().
  /// @param problem_data the constraint problem data.
  /// @param mlcp_to_lcp_data a reference to a MlcpToLcpData object.
  /// @param zz the solution to the LCP resulting from
  ///        UpdateDiscretizedTimeLcp().
  /// @param a the vector `a` output from UpdateDiscretizedTimeLcp().
  /// @param dt the time step used to discretize the problem.
  /// @param[out] cf the constraint forces, on return. The first `nc` elements
  ///        of `cf` correspond to the magnitudes of the contact forces applied
  ///        along the normals of the `nc` contact points. The next elements of
  ///        `cf` correspond to the frictional forces along the `r` spanning
  ///        directions at each point of contact. The first `r` values (after
  ///        the initial `nc` elements) correspond to the first contact, the
  ///        next `r` values correspond to the second contact, etc. The next
  ///        `ℓ` values of `cf` correspond to the impulsive forces applied to
  ///        enforce unilateral constraint functions. The final `b` values of
  ///        `cf` correspond to the forces applied to enforce generic bilateral
  ///        constraints. This packed storage format can be turned into more
  ///        useful representations through
  ///        ComputeGeneralizedForceFromConstraintForces() and
  ///        CalcContactForcesInContactFrames().
  /// @pre cf is non-null.
  static void PopulatePackedConstraintForcesFromLcpSolution(
      const ConstraintVelProblemData<T>& problem_data,
      const MlcpToLcpData& mlcp_to_lcp_data, const VectorX<T>& zz,
      const VectorX<T>& a, double dt, VectorX<T>* cf);
  //@}

  /// Solves the appropriate constraint problem at the acceleration level.
  /// @param problem_data The data used to compute the constraint forces.
  /// @param cf The computed constraint forces, on return, in a packed storage
  ///           format. The first `nc` elements of `cf` correspond to the
  ///           magnitudes of the contact forces applied along the normals of
  ///           the `nc` contact points. The next elements of `cf`
  ///           correspond to the frictional forces along the `r` spanning
  ///           directions at each non-sliding point of contact. The first `r`
  ///           values (after the initial `nc` elements) correspond to the first
  ///           non-sliding contact, the next `r` values correspond to the
  ///           second non-sliding contact, etc. The next `ℓ` values of `cf`
  ///           correspond to the forces applied to enforce generic unilateral
  ///           constraints. The final `b` values of `cf` correspond to the
  ///           forces applied to enforce generic bilateral constraints. This
  ///           packed storage format can be turned into more useful
  ///           representations through
  ///           ComputeGeneralizedForceFromConstraintForces() and
  ///           CalcContactForcesInContactFrames(). `cf` will be resized as
  ///           necessary.
  /// @pre Constraint data has been computed.
  /// @throws std::exception if the constraint forces cannot be computed
  ///         (due to, e.g., an "inconsistent" rigid contact configuration).
  /// @throws std::exception if `cf` is null.
  void SolveConstraintProblem(const ConstraintAccelProblemData<T>& problem_data,
                              VectorX<T>* cf) const;

  /// Computes the generalized force on the system from the constraint forces
  /// given in packed storage.
  /// @param problem_data The data used to compute the contact forces.
  /// @param cf The computed constraint forces, in the packed storage
  ///           format described in documentation for SolveConstraintProblem.
  /// @param[out] generalized_force The generalized force acting on the system
  ///             from the total constraint wrench is stored here, on return.
  ///             This method will resize `generalized_force` as necessary. The
  ///             indices of `generalized_force` will exactly match the indices
  ///             of `problem_data.f`.
  /// @throws std::exception if `generalized_force` is null or `cf`
  ///         vector is incorrectly sized.
  static void ComputeGeneralizedForceFromConstraintForces(
      const ConstraintAccelProblemData<T>& problem_data, const VectorX<T>& cf,
      VectorX<T>* generalized_force);

  /// Computes the generalized force on the system from the constraint forces
  /// given in packed storage.
  /// @param problem_data The data used to compute the contact forces.
  /// @param cf The computed constraint forces, in the packed storage
  ///           format described in documentation for
  ///           PopulatePackedConstraintForcesFromLcpSolution().
  /// @param[out] generalized_force The generalized force acting on the system
  ///             from the total constraint wrench is stored here, on return.
  ///             This method will resize `generalized_force` as necessary. The
  ///             indices of `generalized_force` will exactly match the indices
  ///             of `problem_data.f`.
  /// @throws std::exception if `generalized_force` is null or `cf`
  ///         vector is incorrectly sized.
  static void ComputeGeneralizedForceFromConstraintForces(
      const ConstraintVelProblemData<T>& problem_data, const VectorX<T>& cf,
      VectorX<T>* generalized_force);

  /// Computes the system generalized acceleration due to both external forces
  /// and constraint forces.
  /// @param problem_data The acceleration-level constraint data.
  /// @param cf The computed constraint forces, in the packed storage
  ///           format described in documentation for SolveConstraintProblem.
  /// @param[out] generalized_acceleration The generalized acceleration, on
  ///             return.
  /// @throws std::exception if @p generalized_acceleration is null or
  ///         @p cf vector is incorrectly sized.
  static void ComputeGeneralizedAcceleration(
      const ConstraintAccelProblemData<T>& problem_data, const VectorX<T>& cf,
      VectorX<T>* generalized_acceleration);

  /// Computes a first-order approximation of generalized acceleration due
  /// *only* to constraint forces.
  /// @param problem_data The velocity-level constraint data.
  /// @param cf The computed constraint forces, in the packed storage
  ///           format described in documentation for SolveConstraintProblem.
  /// @param v  The system generalized velocity at time t.
  /// @param dt The discretization time constant (i.e., the "time step" for
  ///           simulations) used to take the system's generalized velocities
  ///           from time t to time t + `dt`.
  /// @param[out] generalized_acceleration The generalized acceleration, on
  ///             return. The original will be resized (if necessary) and
  ///             overwritten.
  /// @warning This method uses the method `problem_data.solve_inertia()` in
  ///          order to compute `v(t+dt)`, so the computational demands may
  ///          be significant.
  /// @throws std::exception if `generalized_acceleration` is null or
  ///         `cf` vector is incorrectly sized.
  /// @pre `dt` is positive.
  static void ComputeGeneralizedAcceleration(
      const ConstraintVelProblemData<T>& problem_data, const VectorX<T>& v,
      const VectorX<T>& cf, double dt, VectorX<T>* generalized_acceleration);

  /// Computes the system generalized acceleration due *only* to constraint
  /// forces.
  /// @param cf The computed constraint forces, in the packed storage
  ///           format described in documentation for SolveConstraintProblem.
  /// @throws std::exception if @p generalized_acceleration is null or
  ///         @p cf vector is incorrectly sized.
  static void ComputeGeneralizedAccelerationFromConstraintForces(
      const ConstraintAccelProblemData<T>& problem_data, const VectorX<T>& cf,
      VectorX<T>* generalized_acceleration);

  /// Computes the system generalized acceleration due *only* to constraint
  /// forces.
  /// @param cf The computed constraint forces, in the packed storage
  ///           format described in documentation for SolveConstraintProblem.
  /// @throws std::exception if @p generalized_acceleration is null or
  ///         @p cf vector is incorrectly sized.
  static void ComputeGeneralizedAccelerationFromConstraintForces(
      const ConstraintVelProblemData<T>& problem_data, const VectorX<T>& cf,
      VectorX<T>* generalized_acceleration);

  /// Computes the change to the system generalized velocity from constraint
  /// impulses.
  /// @param cf The computed constraint impulses, in the packed storage
  ///           format described in documentation for SolveImpactProblem.
  /// @throws std::exception if `generalized_delta_v` is null or
  ///         `cf` vector is incorrectly sized.
  static void ComputeGeneralizedVelocityChange(
      const ConstraintVelProblemData<T>& problem_data, const VectorX<T>& cf,
      VectorX<T>* generalized_delta_v);

  /// Gets the contact forces expressed in each contact frame *for 2D contact
  /// problems* from the "packed" solution returned by SolveConstraintProblem().
  /// @param cf the output from SolveConstraintProblem()
  /// @param problem_data the problem data input to SolveConstraintProblem()
  /// @param contact_frames the contact frames corresponding to the contacts.
  ///        The first column of each matrix should give the contact normal,
  ///        while the second column gives a contact tangent. For sliding
  ///        contacts, the contact tangent should point along the direction of
  ///        sliding. For non-sliding contacts, the tangent direction should be
  ///        that used to determine `problem_data.F`. All vectors should be
  ///        expressed in the global frame.
  /// @param[out] contact_forces a non-null vector of a doublet of values, where
  ///             the iᵗʰ element represents the force along each basis
  ///             vector in the iᵗʰ contact frame.
  /// @throws std::exception if `contact_forces` is null, if
  ///         `contact_forces` is not empty, if `cf` is not the
  ///         proper size, if the number of tangent directions is not one per
  ///         non-sliding contact (indicating that the contact problem might not
  ///         be 2D), if the number of contact frames is not equal to the number
  ///         of contacts, or if a contact frame does not appear to be
  ///         orthonormal.
  /// @note On return, the contact force at the iᵗʰ contact point expressed
  ///       in the world frame is `contact_frames[i]` * `contact_forces[i]`.
  static void CalcContactForcesInContactFrames(
      const VectorX<T>& cf, const ConstraintAccelProblemData<T>& problem_data,
      const std::vector<Matrix2<T>>& contact_frames,
      std::vector<Vector2<T>>* contact_forces);

  /// Gets the contact forces expressed in each contact frame *for 2D contact
  /// problems* from a "packed" solution returned by, e.g.,
  /// SolveImpactProblem().  If the constraint forces are impulsive, the contact
  /// forces are impulsive (with units of Ns); similarly, if the constraint
  /// forces are non-impulsive, the contact forces will be non-impulsive (with
  /// units of N).
  /// @param cf the constraint forces in packed format.
  /// @param problem_data the problem data input to SolveImpactProblem()
  /// @param contact_frames the contact frames corresponding to the contacts.
  ///        The first column of each matrix should give the contact normal,
  ///        while the second column gives a contact tangent (specifically, the
  ///        tangent direction used to determine `problem_data.F`). All
  ///        vectors should be expressed in the global frame.
  /// @param[out] contact_forces a non-null vector of a doublet of values,
  ///             where the iᵗʰ element represents the force along
  ///             each basis vector in the iᵗʰ contact frame.
  /// @throws std::exception if `contact_forces` is null, if
  ///         `contact_forces` is not empty, if `cf` is not the
  ///         proper size, if the number of tangent directions is not one per
  ///         contact (indicating that the contact problem might not be 2D), if
  ///         the number of contact frames is not equal to the number
  ///         of contacts, or if a contact frame does not appear to be
  ///         orthonormal.
  /// @note On return, the contact force at the iᵗʰ contact point expressed
  ///       in the world frame is `contact_frames[i]` * `contact_forces[i]`.
  static void CalcContactForcesInContactFrames(
      const VectorX<T>& cf, const ConstraintVelProblemData<T>& problem_data,
      const std::vector<Matrix2<T>>& contact_frames,
      std::vector<Vector2<T>>* contact_forces);

 private:
  // Populates the packed constraint force vector from the solution to the
  // linear complementarity problem (LCP).
  // @param problem_data the constraint problem data.
  // @param a reference to a MlcpToLcpData object.
  // @param a the vector `a` output from UpdateDiscretizedTimeLcp().
  // @param[out] cf the constraint forces, on return.
  // @pre cf is non-null.
  static void PopulatePackedConstraintForcesFromLcpSolution(
      const ConstraintVelProblemData<T>& problem_data,
      const MlcpToLcpData& mlcp_to_lcp_data, const VectorX<T>& zz,
      const VectorX<T>& a, VectorX<T>* cf);

  static void ConstructLinearEquationSolversForMlcp(
      const ConstraintVelProblemData<T>& problem_data,
      MlcpToLcpData* mlcp_to_lcp_data);

  // Forms and solves the linear complementarity problem used to compute the
  // accelerations during ODE evaluations, as an alternative to the linear
  // system problem formulation. In contrast to
  // FormAndSolveConstraintLinearSystem(), this approach does not require the
  // active constraints to be known a priori. Significantly more computation is
  // required, however.
  // @sa FormAndSolveConstraintLinearSystem for descriptions of parameters.
  void FormAndSolveConstraintLcp(
      const ConstraintAccelProblemData<T>& problem_data,
      const VectorX<T>& trunc_neg_invA_a, VectorX<T>* cf) const;

  // Forms and solves the system of linear equations used to compute the
  // accelerations during ODE evaluations, as an alternative to the linear
  // complementarity problem formulation. If all constraints are known to be
  // active, this method will provide the correct solution rapidly.
  // @param problem_data The constraint data formulated at the acceleration
  //                     level.
  // @param trunc_neg_invA_a The first ngc elements of -A⁻¹a, where
  //        A ≡ | M Gᵀ |    (M is the generalized inertia matrix and G is the
  //            | G 0  |     Jacobian matrix for the bilateral constraints)
  //        and a ≡ | -τ |  (τ [tau] and kG are defined in `problem_data`).
  //                | kG |
  // @param[out] cf The unilateral constraint forces, on return, in a packed
  //                storage format. The first `nc` elements of `cf` correspond
  //                to the magnitudes of the contact forces applied along the
  //                normals of the `nc` contact points. The next elements of
  //                `cf` correspond to the frictional forces along the `r`
  //                spanning directions at each non-sliding point of contact.
  //                The first `r` values (after the initial `nc` elements)
  //                correspond to the first non-sliding contact, the next `r`
  //                values correspond to the second non-sliding contact, etc.
  //                The next `ℓ` values of `cf` correspond to the forces applied
  //                to enforce generic unilateral constraints. `cf` will be
  //                resized as necessary.
  void FormAndSolveConstraintLinearSystem(
      const ConstraintAccelProblemData<T>& problem_data,
      const VectorX<T>& trunc_neg_invA_a, VectorX<T>* cf) const;

  // Checks the validity of the constraint matrix. This operation is relatively
  // expensive and should only be called in debug mode. Nevertheless, it's
  // useful to debug untested constraint Jacobian operators.
  static void CheckAccelConstraintMatrix(
      const ConstraintAccelProblemData<T>& problem_data, const MatrixX<T>& MM);

  static void CheckVelConstraintMatrix(
      const ConstraintVelProblemData<T>& problem_data, const MatrixX<T>& MM);

  // Computes a constraint space compliance matrix A⋅M⁻¹⋅Bᵀ, where A ∈ ℝᵃˣᵐ
  // (realized here using an operator) and B ∈ ℝᵇˣᵐ are both Jacobian matrices
  // and M⁻¹ ∈ ℝᵐˣᵐ is the inverse of the generalized inertia matrix. Note that
  // mixing types of constraints is explicitly allowed. Aborts if A_iM_BT is
  // not of size a × b.
  static void ComputeConstraintSpaceComplianceMatrix(
      std::function<VectorX<T>(const VectorX<T>&)> A_mult, int a,
      const MatrixX<T>& M_inv_BT, Eigen::Ref<MatrixX<T>>);

  // Computes the matrix M⁻¹⋅Gᵀ, G ∈ ℝᵐˣⁿ is a constraint Jacobian matrix
  // (realized here using an operator) and M⁻¹ ∈ ℝⁿˣⁿ is the inverse of the
  // generalized inertia matrix. Resizes iM_GT as necessary.
  static void ComputeInverseInertiaTimesGT(
      std::function<MatrixX<T>(const MatrixX<T>&)> M_inv_mult,
      std::function<VectorX<T>(const VectorX<T>&)> G_transpose_mult, int m,
      MatrixX<T>* iM_GT);

  // Forms the LCP matrix and vector, which is used to determine the collisional
  // impulses.
  static void FormImpactingConstraintLcp(
      const ConstraintVelProblemData<T>& problem_data, const VectorX<T>& invA_a,
      MatrixX<T>* MM, VectorX<T>* qq);

  // Forms the LCP matrix and vector, which is used to determine the constraint
  // forces (and can also be used to determine the active set of constraints at
  // the acceleration-level).
  static void FormSustainedConstraintLcp(
      const ConstraintAccelProblemData<T>& problem_data,
      const VectorX<T>& invA_a, MatrixX<T>* MM, VectorX<T>* qq);

  // Forms the linear system matrix and vector, which is used to determine the
  // constraint forces.
  static void FormSustainedConstraintLinearSystem(
      const ConstraintAccelProblemData<T>& problem_data,
      const VectorX<T>& invA_a, MatrixX<T>* MM, VectorX<T>* qq);

  // Given a matrix A of blocks consisting of generalized inertia (M) and the
  // Jacobian of bilaterals constraints (G):
  // A ≡ | M  -Gᵀ |
  //     | G   0  |
  // this function sets a function pointer that computes X for X = A⁻¹ | B | and
  //                                                                   | 0 |
  // given B for the case where X will be premultiplied by some matrix | R 0 |,
  // where R is an arbitrary matrix. This odd operation is relatively common,
  // and optimizing for this case allows us to skip some expensive matrix
  // arithmetic.
  // @param num_generalized_velocities The dimension of the system generalized
  //        velocities.
  // @param problem_data The constraint problem data.
  // @param delassus_QTZ The factorization of the Delassus Matrix GM⁻¹Gᵀ,
  //               where G is the constraint Jacobian corresponding to the
  //               independent constraints.
  // @param[out] A_solve The operator for solving AX = B, on return.
  template <typename ProblemData>
  static void DetermineNewPartialInertiaSolveOperator(
      const ProblemData* problem_data, int num_generalized_velocities,
      const Eigen::CompleteOrthogonalDecomposition<MatrixX<T>>* delassus_QTZ,
      std::function<MatrixX<T>(const MatrixX<T>&)>* A_solve);

  // Given a matrix A of blocks consisting of generalized inertia (M) and the
  // Jacobian of bilaterals constraints (G):
  // A ≡ | M  -Gᵀ |
  //     | G   0  |
  // this function sets a function pointer that computes X for AX = B, given B.
  // @param num_generalized_velocities The dimension of the system generalized
  //        velocities.
  // @param problem_data The constraint problem data.
  // @param delassus_QTZ The factorization of the Delassus Matrix GM⁻¹Gᵀ,
  //               where G is the constraint Jacobian corresponding to the
  //               independent constraints.
  // @param[out] A_solve The operator for solving AX = B, on return.
  template <typename ProblemData>
  static void DetermineNewFullInertiaSolveOperator(
      const ProblemData* problem_data, int num_generalized_velocities,
      const Eigen::CompleteOrthogonalDecomposition<MatrixX<T>>* delassus_QTZ,
      std::function<MatrixX<T>(const MatrixX<T>&)>* A_solve);

  template <typename ProblemData>
  static ProblemData* UpdateProblemDataForUnilateralConstraints(
      const ProblemData& problem_data,
      std::function<const MatrixX<T>(const MatrixX<T>&)> modified_inertia_solve,
      int gv_dim, ProblemData* modified_problem_data);

  drake::solvers::MobyLcpSolver lcp_;
};

}  // namespace rod2d
}  // namespace examples
}  // namespace drake
