#pragma once

// GENERATED FILE DO NOT EDIT
// This file contains docstrings for the Python bindings that were
// automatically extracted by mkdoc.py.

#include <array>
#include <utility>

#if defined(__GNUG__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-variable"
#endif

// #include "drake/solvers/aggregate_costs_constraints.h"
// #include "drake/solvers/augmented_lagrangian.h"
// #include "drake/solvers/binding.h"
// #include "drake/solvers/branch_and_bound.h"
// #include "drake/solvers/choose_best_solver.h"
// #include "drake/solvers/clarabel_solver.h"
// #include "drake/solvers/clp_solver.h"
// #include "drake/solvers/common_solver_option.h"
// #include "drake/solvers/constraint.h"
// #include "drake/solvers/cost.h"
// #include "drake/solvers/create_constraint.h"
// #include "drake/solvers/create_cost.h"
// #include "drake/solvers/csdp_solver.h"
// #include "drake/solvers/decision_variable.h"
// #include "drake/solvers/equality_constrained_qp_solver.h"
// #include "drake/solvers/evaluator_base.h"
// #include "drake/solvers/function.h"
// #include "drake/solvers/get_program_type.h"
// #include "drake/solvers/gurobi_solver.h"
// #include "drake/solvers/indeterminate.h"
// #include "drake/solvers/integer_inequality_solver.h"
// #include "drake/solvers/integer_optimization_util.h"
// #include "drake/solvers/ipopt_solver.h"
// #include "drake/solvers/linear_system_solver.h"
// #include "drake/solvers/mathematical_program.h"
// #include "drake/solvers/mathematical_program_result.h"
// #include "drake/solvers/minimum_value_constraint.h"
// #include "drake/solvers/mixed_integer_optimization_util.h"
// #include "drake/solvers/mixed_integer_rotation_constraint.h"
// #include "drake/solvers/mixed_integer_rotation_constraint_internal.h"
// #include "drake/solvers/moby_lcp_solver.h"
// #include "drake/solvers/mosek_solver.h"
// #include "drake/solvers/nlopt_solver.h"
// #include "drake/solvers/non_convex_optimization_util.h"
// #include "drake/solvers/osqp_solver.h"
// #include "drake/solvers/program_attribute.h"
// #include "drake/solvers/projected_gradient_descent_solver.h"
// #include "drake/solvers/rotation_constraint.h"
// #include "drake/solvers/scs_clarabel_common.h"
// #include "drake/solvers/scs_solver.h"
// #include "drake/solvers/sdpa_free_format.h"
// #include "drake/solvers/semidefinite_relaxation.h"
// #include "drake/solvers/semidefinite_relaxation_internal.h"
// #include "drake/solvers/snopt_solver.h"
// #include "drake/solvers/solution_result.h"
// #include "drake/solvers/solve.h"
// #include "drake/solvers/solver_base.h"
// #include "drake/solvers/solver_id.h"
// #include "drake/solvers/solver_interface.h"
// #include "drake/solvers/solver_options.h"
// #include "drake/solvers/solver_type.h"
// #include "drake/solvers/solver_type_converter.h"
// #include "drake/solvers/sos_basis_generator.h"
// #include "drake/solvers/sparse_and_dense_matrix.h"
// #include "drake/solvers/specific_options.h"
// #include "drake/solvers/unrevised_lemke_solver.h"

// Symbol: pydrake_doc_solvers
constexpr struct /* pydrake_doc_solvers */ {
  // Symbol: drake
  struct /* drake */ {
    // Symbol: drake::solvers
    struct /* solvers */ {
      // Symbol: drake::solvers::AddBilinearProductMcCormickEnvelopeMultipleChoice
      struct /* AddBilinearProductMcCormickEnvelopeMultipleChoice */ {
        // Source: drake/solvers/mixed_integer_optimization_util.h
        const char* doc =
R"""(Add constraints to the optimization program, such that the bilinear
product x * y is approximated by w, using Mixed Integer constraint
with "Multiple Choice" model. To do so, we assume that the range of x
is [x_min, x_max], and the range of y is [y_min, y_max]. We first
consider two arrays φˣ, φʸ, satisfying


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    x_min = φˣ₀ < φˣ₁ < ... < φˣₘ = x_max
    y_min = φʸ₀ < φʸ₁ < ... < φʸₙ = y_max

.. raw:: html

    </details>

, and divide the range of x into intervals [φˣ₀, φˣ₁], [φˣ₁, φˣ₂], ...
, [φˣₘ₋₁, φˣₘ] and the range of y into intervals [φʸ₀, φʸ₁], [φʸ₁,
φʸ₂], ... , [φʸₙ₋₁, φʸₙ]. The xy plane is thus cut into rectangles,
with each rectangle as [φˣᵢ, φˣᵢ₊₁] x [φʸⱼ, φʸⱼ₊₁]. The convex hull of
the surface z = x * y for x, y in each rectangle is a tetrahedron. We
then approximate the bilinear product x * y with w, such that (x, y,
w) is in one of the tetrahedrons.

Parameter ``prog``:
    The optimization problem to which the constraints will be added.

Parameter ``x``:
    A variable in the bilinear product.

Parameter ``y``:
    A variable in the bilinear product.

Parameter ``w``:
    The expression that approximates the bilinear product x * y.

Parameter ``phi_x``:
    φˣ in the documentation above. Will be used to cut the range of x
    into small intervals.

Parameter ``phi_y``:
    φʸ in the documentation above. Will be used to cut the range of y
    into small intervals.

Parameter ``Bx``:
    The binary-valued expression indicating which interval x is in.
    Bx(i) = 1 => φˣᵢ ≤ x ≤ φˣᵢ₊₁.

Parameter ``By``:
    The binary-valued expression indicating which interval y is in.
    By(i) = 1 => φʸⱼ ≤ y ≤ φʸⱼ₊₁.

One formulation of the constraint is


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    x = ∑ᵢⱼ x̂ᵢⱼ
    y = ∑ᵢⱼ ŷᵢⱼ
    Bˣʸᵢⱼ = Bˣᵢ ∧ Bʸⱼ
    ∑ᵢⱼ Bˣʸᵢⱼ = 1
    φˣᵢ Bˣʸᵢⱼ ≤ x̂ᵢⱼ ≤ φˣᵢ₊₁ Bˣʸᵢⱼ
    φʸⱼ Bˣʸᵢⱼ ≤ ŷᵢⱼ ≤ φʸⱼ₊₁ Bˣʸᵢⱼ
    w ≥ ∑ᵢⱼ (x̂ᵢⱼ φʸⱼ   + φˣᵢ   ŷᵢⱼ - φˣᵢ  φʸⱼ   Bˣʸᵢⱼ)
    w ≥ ∑ᵢⱼ (x̂ᵢⱼ φʸⱼ₊₁ + φˣᵢ₊₁ ŷᵢⱼ - φˣᵢ₊₁ φʸⱼ₊₁ Bˣʸᵢⱼ)
    w ≤ ∑ᵢⱼ (x̂ᵢⱼ φʸⱼ   + φˣᵢ₊₁ ŷᵢⱼ - φˣᵢ₊₁ φʸⱼ   Bˣʸᵢⱼ)
    w ≤ ∑ᵢⱼ (x̂ᵢⱼ φʸⱼ₊₁ + φˣᵢ   ŷᵢⱼ - φˣᵢ   φʸⱼ₊₁ Bˣʸᵢⱼ)

.. raw:: html

    </details>

The "logical and" constraint Bˣʸᵢⱼ = Bˣᵢ ∧ Bʸⱼ can be imposed as


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    Bˣʸᵢⱼ ≥ Bˣᵢ + Bʸⱼ - 1
    Bˣʸᵢⱼ ≤ Bˣᵢ
    Bˣʸᵢⱼ ≤ Bʸⱼ
    0 ≤ Bˣʸᵢⱼ ≤ 1

.. raw:: html

    </details>

This formulation will introduce slack variables x̂, ŷ and Bˣʸ, in
total 3 * m * n variables.

In order to reduce the number of slack variables, we can further
simplify these constraints, by defining two vectors ``x̅ ∈ ℝⁿ``, `y̅ ∈
ℝᵐ` as


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    x̅ⱼ = ∑ᵢ x̂ᵢⱼ
    y̅ᵢ = ∑ⱼ ŷᵢⱼ

.. raw:: html

    </details>

and the constraints above can be re-formulated using ``x̅`` and ``y̅``
as


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    x = ∑ⱼ x̅ⱼ
    y = ∑ᵢ y̅ᵢ
    Bˣʸᵢⱼ = Bˣᵢ ∧ Bʸⱼ
    ∑ᵢⱼ Bˣʸᵢⱼ = 1
    ∑ᵢ φˣᵢ Bˣʸᵢⱼ ≤ x̅ⱼ ≤ ∑ᵢ φˣᵢ₊₁ Bˣʸᵢⱼ
    ∑ⱼ φʸⱼ Bˣʸᵢⱼ ≤ y̅ᵢ ≤ ∑ⱼ φʸⱼ₊₁ Bˣʸᵢⱼ
    w ≥ ∑ⱼ( x̅ⱼ φʸⱼ   ) + ∑ᵢ( φˣᵢ   y̅ᵢ ) - ∑ᵢⱼ( φˣᵢ   φʸⱼ   Bˣʸᵢⱼ )
    w ≥ ∑ⱼ( x̅ⱼ φʸⱼ₊₁ ) + ∑ᵢ( φˣᵢ₊₁ y̅ᵢ ) - ∑ᵢⱼ( φˣᵢ₊₁ φʸⱼ₊₁ Bˣʸᵢⱼ )
    w ≤ ∑ⱼ( x̅ⱼ φʸⱼ   ) + ∑ᵢ( φˣᵢ₊₁ y̅ⱼ ) - ∑ᵢⱼ( φˣᵢ₊₁ φʸⱼ   Bˣʸᵢⱼ )
    w ≤ ∑ⱼ( x̅ⱼ φʸⱼ₊₁ ) + ∑ᵢ( φˣᵢ   y̅ᵢ ) - ∑ᵢⱼ( φˣᵢ   φʸⱼ₊₁ Bˣʸᵢⱼ ).

.. raw:: html

    </details>

In this formulation, we introduce new continuous variables ``x̅``,
`y̅`, ``Bˣʸ``. The total number of new variables is m + n + m * n.

In section 3.3 of Mixed-Integer Models for Nonseparable Piecewise
Linear Optimization: Unifying Framework and Extensions by Juan P
Vielma, Shabbir Ahmed and George Nemhauser, this formulation is called
"Multiple Choice Model".

Note:
    We DO NOT add the constraint Bx(i) ∈ {0, 1}, By(j) ∈ {0, 1} in
    this function. It is the user's responsibility to ensure that
    these binary constraints are enforced. The users can also add
    cutting planes ∑ᵢBx(i) = 1, ∑ⱼBy(j) = 1. Without these two cutting
    planes, (x, y, w) is still in the McCormick envelope of z = x * y,
    but these two cutting planes "might" improve the computation speed
    in the mixed-integer solver.)""";
      } AddBilinearProductMcCormickEnvelopeMultipleChoice;
      // Symbol: drake::solvers::AddBilinearProductMcCormickEnvelopeSos2
      struct /* AddBilinearProductMcCormickEnvelopeSos2 */ {
        // Source: drake/solvers/mixed_integer_optimization_util.h
        const char* doc =
R"""(Add constraints to the optimization program, such that the bilinear
product x * y is approximated by w, using Special Ordered Set of Type
2 (sos2) constraint. To do so, we assume that the range of x is
[x_min, x_max], and the range of y is [y_min, y_max]. We first
consider two arrays φˣ, φʸ, satisfying


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    x_min = φˣ₀ < φˣ₁ < ... < φˣₘ = x_max
    y_min = φʸ₀ < φʸ₁ < ... < φʸₙ = y_max

.. raw:: html

    </details>

, and divide the range of x into intervals [φˣ₀, φˣ₁], [φˣ₁, φˣ₂], ...
, [φˣₘ₋₁, φˣₘ] and the range of y into intervals [φʸ₀, φʸ₁], [φʸ₁,
φʸ₂], ... , [φʸₙ₋₁, φʸₙ]. The xy plane is thus cut into rectangles,
with each rectangle as [φˣᵢ, φˣᵢ₊₁] x [φʸⱼ, φʸⱼ₊₁]. The convex hull of
the surface z = x * y for x, y in each rectangle is a tetrahedron. We
then approximate the bilinear product x * y with w, such that (x, y,
w) is in one of the tetrahedrons.

We use two different encoding schemes on the binary variables, to
determine which interval is active. We can choose either linear or
logarithmic binning. When using linear binning, for a variable with N
intervals, we use N binary variables, and B(i) = 1 indicates the
variable is in the i'th interval. When using logarithmic binning, we
use ⌈log₂(N)⌉ binary variables. If these binary variables represent
integer M in the reflected Gray code, then the continuous variable is
in the M'th interval.

Parameter ``prog``:
    The program to which the bilinear product constraint is added

Parameter ``x``:
    The decision variable.

Parameter ``y``:
    The decision variable.

Parameter ``w``:
    The expression to approximate x * y

Parameter ``phi_x``:
    The end points of the intervals for ``x``.

Parameter ``phi_y``:
    The end points of the intervals for ``y``.

Parameter ``Bx``:
    The binary variables for the interval in which x stays encoded as
    described above.

Parameter ``By``:
    The binary variables for the interval in which y stays encoded as
    described above.

Parameter ``binning``:
    Determine whether to use linear binning or logarithmic binning.

Returns:
    lambda The auxiliary continuous variables.

The constraints we impose are


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    x = (φˣ)ᵀ * ∑ⱼ λᵢⱼ
    y = (φʸ)ᵀ * ∑ᵢ λᵢⱼ
    w = ∑ᵢⱼ φˣᵢ * φʸⱼ * λᵢⱼ
    Both ∑ⱼ λᵢⱼ = λ.rowwise().sum() and ∑ᵢ λᵢⱼ = λ.colwise().sum() satisfy SOS2
    constraint.

.. raw:: html

    </details>

If x ∈ [φx(M), φx(M+1)] and y ∈ [φy(N), φy(N+1)], then only λ(M, N),
λ(M + 1, N), λ(M, N + 1) and λ(M+1, N+1) can be strictly positive, all
other λ(i, j) are zero.

Note:
    We DO NOT add the constraint Bx(i) ∈ {0, 1}, By(j) ∈ {0, 1} in
    this function. It is the user's responsibility to ensure that
    these constraints are enforced.)""";
      } AddBilinearProductMcCormickEnvelopeSos2;
      // Symbol: drake::solvers::AddBoundingBoxConstraintsImpliedByRollPitchYawLimits
      struct /* AddBoundingBoxConstraintsImpliedByRollPitchYawLimits */ {
        // Source: drake/solvers/rotation_constraint.h
        const char* doc =
R"""(Applies *very conservative* limits on the entries of R for the cases
when rotations can be limited (for instance, if you want to search
over rotations, but there is an obvious symmetry in the problem so
that e.g. 0 < pitch < PI need not be considered). A matrix so
constrained may still contain rotations outside of this envelope.
Note: For simple rotational symmetry over PI, prefer
kPitch_NegPI_2_to_PI_2 (over 0_to_PI) because it adds one more
constraint (when combined with constraints on roll and yaw). Note: The
Roll-Pitch-Yaw angles follow the convention in RollPitchYaw, namely
extrinsic rotations about Space-fixed x-y-z axes, respectively.)""";
      } AddBoundingBoxConstraintsImpliedByRollPitchYawLimits;
      // Symbol: drake::solvers::AddLogarithmicSos1Constraint
      struct /* AddLogarithmicSos1Constraint */ {
        // Source: drake/solvers/mixed_integer_optimization_util.h
        const char* doc_4args =
R"""(Adds the special ordered set of type 1 (SOS1) constraint. Namely


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    λ(0) + ... + λ(n-1) = 1
      λ(i) ≥ 0 ∀i
      ∃ j ∈ {0, 1, ..., n-1}, s.t λ(j) = 1

.. raw:: html

    </details>

where one and only one of λ(i) is 1, all other λ(j) are 0. We will
need to add ⌈log₂(n)⌉ binary variables, where n is the number of rows
in λ. For more information, please refer to Modeling Disjunctive
Constraints with a Logarithmic Number of Binary Variables and
Constraints by J. Vielma and G. Nemhauser, 2011.

Parameter ``prog``:
    The program to which the SOS1 constraint is added.

Parameter ``lambda``:
    lambda is in SOS1.

Parameter ``y``:
    The binary variables indicating which λ is positive. For a given
    assignment on the binary variable ``y``, if (y(0), ...,
    y(⌈log₂(n)⌉) represents integer M in ``binary_encoding``, then
    only λ(M) is positive. Namely, if (y(0), ..., y(⌈log₂(n)⌉) equals
    to binary_encoding.row(M), then λ(M) = 1

Parameter ``binary_encoding``:
    A n x ⌈log₂(n)⌉ matrix. binary_encoding.row(i) represents integer
    i. No two rows of ``binary_encoding`` can be the same.

Raises:
    RuntimeError if ``binary_encoding`` has a non-binary entry (0, 1).)""";
        // Source: drake/solvers/mixed_integer_optimization_util.h
        const char* doc_2args =
R"""(Adds the special ordered set of type 1 (SOS1) constraint. Namely


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    λ(0) + ... + λ(n-1) = 1
      λ(i) ≥ 0 ∀i
      ∃ j ∈ {0, 1, ..., n-1}, s.t λ(j) = 1

.. raw:: html

    </details>

where one and only one of λ(i) is 1, all other λ(j) are 0. We will
need to add ⌈log₂(n)⌉ binary variables, where n is the number of rows
in λ. For more information, please refer to Modeling Disjunctive
Constraints with a Logarithmic Number of Binary Variables and
Constraints by J. Vielma and G. Nemhauser, 2011.

Parameter ``prog``:
    The program to which the SOS1 constraint is added.

Parameter ``num_lambda``:
    n in the documentation above.

Returns:
    (lambda, y) lambda is λ in the documentation above. Notice that λ
    are declared as continuous variables, but they only admit binary
    solutions. y are binary variables of size ⌈log₂(n)⌉. When this
    sos1 constraint is satisfied, suppose that λ(i)=1 and λ(j)=0 ∀
    j≠i, then y is the Reflected Gray code of i. For example, suppose
    n = 8, i = 5, then y is a vector of size ⌈log₂(n)⌉ = 3, and the
    value of y is (1, 1, 0) which equals to 5 according to reflected
    Gray code.)""";
      } AddLogarithmicSos1Constraint;
      // Symbol: drake::solvers::AddLogarithmicSos2Constraint
      struct /* AddLogarithmicSos2Constraint */ {
        // Source: drake/solvers/mixed_integer_optimization_util.h
        const char* doc_3args_MathematicalProgram_constEigenMatrixBase_conststdstring =
R"""(Adds the special ordered set 2 (SOS2) constraint,


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    λ(0) + ... + λ(n) = 1
      ∀i. λ(i) ≥ 0
      ∃ j ∈ {0, 1, ..., n-1}, s.t λ(i) = 0 if i ≠ j and i ≠ j + 1

.. raw:: html

    </details>

Namely at most two entries in λ can be strictly positive, and these
two entries have to be adjacent. All other λ should be zero. Moreover,
the non-zero λ satisfies λ(j) + λ(j + 1) = 1. We will need to add
⌈log₂(n - 1)⌉ binary variables, where n is the number of rows in λ.
For more information, please refer to Modeling Disjunctive Constraints
with a Logarithmic Number of Binary Variables and Constraints by J.
Vielma and G. Nemhauser, 2011.

Parameter ``prog``:
    Add the SOS2 constraint to this mathematical program.

Parameter ``lambda``:
    At most two entries in λ can be strictly positive, and these two
    entries have to be adjacent. All other entries are zero.

Returns:
    y The newly added binary variables. The assignment of the binary
    variable y implies which two λ can be strictly positive. With a
    binary assignment on y, and suppose the integer M corresponds to
    (y(0), y(1), ..., y(⌈log₂(n - 1)⌉)) in Gray code, then only λ(M)
    and λ(M + 1) can be non-zero. For example, if the assignment of y
    = (1, 1), in Gray code, (1, 1) represents integer 2, so only λ(2)
    and λ(3) can be strictly positive.)""";
        // Source: drake/solvers/mixed_integer_optimization_util.h
        const char* doc_3args_prog_lambda_y =
R"""(Adds the special ordered set 2 (SOS2) constraint,

See also:
    AddLogarithmicSos2Constraint.)""";
      } AddLogarithmicSos2Constraint;
      // Symbol: drake::solvers::AddRelaxNonConvexQuadraticConstraintInTrustRegion
      struct /* AddRelaxNonConvexQuadraticConstraintInTrustRegion */ {
        // Source: drake/solvers/non_convex_optimization_util.h
        const char* doc =
R"""(For a non-convex quadratic constraint lb ≤ xᵀQ₁x - xᵀQ₂x + pᵀy ≤ ub
where Q₁, Q₂ are both positive semidefinite matrices. ``y`` is a
vector that can overlap with ``x``. We relax this non-convex
constraint by several convex constraints. The steps are 1. Introduce
two new variables z₁, z₂, to replace xᵀQ₁x and xᵀQ₂x respectively. The
constraint becomes


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    lb ≤ z₁ - z₂ + pᵀy ≤ ub              (1)

.. raw:: html

    </details>

2. Ideally, we would like to enforce z₁ = xᵀQ₁x and z₂ = xᵀQ₂x through convex
   constraints. To this end, we first bound z₁ and z₂ from below, as
   


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    z₁ ≥ xᵀQ₁x                            (2)
         z₂ ≥ xᵀQ₂x                            (3)

.. raw:: html

    </details>

These two constraints are second order cone constraints. 3. To bound
z₁ and z₂ from above, we linearize the quadratic forms xᵀQ₁x and xᵀQ₂x
at a point x₀. Due to the convexity of the quadratic form, we know
that given a positive scalar d > 0, there exists a neighbourhood N(x₀)
around x₀, s.t ∀ x ∈ N(x₀)


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    xᵀQ₁x ≤ 2 x₀ᵀQ₁(x - x₀) + x₀ᵀQ₁x₀ + d   (4)
       xᵀQ₂x ≤ 2 x₀ᵀQ₂(x - x₀) + x₀ᵀQ₂x₀ + d   (5)

.. raw:: html

    </details>

Notice N(x₀) is the intersection of two ellipsoids, as formulated in
(4) and (5). Therefore, we also enforce the linear constraints


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    z₁ ≤ 2 x₀ᵀQ₁(x - x₀) + x₀ᵀQ₁x₀ + d    (6)
         z₂ ≤ 2 x₀ᵀQ₂(x - x₀) + x₀ᵀQ₂x₀ + d    (7)

.. raw:: html

    </details>

So we relax the original non-convex constraint, with the convex
constraints (1)-(3), (6) and (7).

The trust region is the neighbourhood N(x₀) around x₀, such that the
inequalities (4), (5) are satisfied ∀ x ∈ N(x₀).

The positive scalar d controls both how much the constraint relaxation
is (the original constraint can be violated by at most d), and how big
the trust region is.

If there is a solution satisfying the relaxed constraint, this
solution can violate the original non-convex constraint by at most d;
on the other hand, if there is not a solution satisfying the relaxed
constraint, it proves that the original non-convex constraint does not
have a solution in the trust region.

This approach is outlined in section III of On Time Optimization of
Centroidal Momentum Dynamics by Brahayam Ponton, Alexander Herzog,
Stefan Schaal and Ludovic Righetti, ICRA, 2018

The special cases are when Q₁ = 0 or Q₂ = 0. 1. When Q₁ = 0, the
original constraint becomes lb ≤ -xᵀQ₂x + pᵀy ≤ ub If ub = +∞, then
the original constraint is the convex rotated Lorentz cone constraint
xᵀQ₂x ≤ pᵀy - lb. The user should not call this function to relax this
convex constraint.

Raises:
    RuntimeError if Q₁ = 0 and ub = +∞. If ub < +∞, then we introduce
    a new variable z, with the constraints lb ≤ -z + pᵀy ≤ ub z ≥
    xᵀQ₂x z ≤ 2 x₀ᵀQ₂(x - x₀) + x₀ᵀQ₂x₀ + d 2. When Q₂ = 0, the
    constraint becomes lb ≤ xᵀQ₁x + pᵀy ≤ ub If lb = -∞, then the
    original constraint is the convex rotated Lorentz cone constraint
    xᵀQ₁x ≤ ub - pᵀy. The user should not call this function to relax
    this convex constraint.

Raises:
    RuntimeError if Q₂ = 0 and lb = -∞. If lb > -∞, then we introduce
    a new variable z, with the constraints lb ≤ z + pᵀy ≤ ub z ≥ xᵀQ₁x
    z ≤ 2 x₀ᵀQ₁(x - x₀) + x₀ᵀQ₁x₀ + d 3. If both Q₁ and Q₂ are zero,
    then the original constraint is a convex linear constraint lb ≤
    pᵀx ≤ ub. The user should not call this function to relax this
    convex constraint. Throw a runtime error.

Parameter ``prog``:
    The MathematicalProgram to which the relaxed constraints are
    added.

Parameter ``x``:
    The decision variables which appear in the original non-convex
    constraint.

Parameter ``Q1``:
    A positive semidefinite matrix.

Parameter ``Q2``:
    A positive semidefinite matrix.

Parameter ``y``:
    A vector, the variables in the linear term of the quadratic form.

Parameter ``p``:
    A vector, the linear coefficients of the quadratic form.

Parameter ``linearization_point``:
    The vector ``x₀`` in the documentation above.

Parameter ``lower_bound``:
    The left-hand side of the original non-convex constraint.

Parameter ``upper_bound``:
    The right-hand side of the original non-convex constraint.

Parameter ``trust_region_gap``:
    The user-specified positive scalar, ``d`` in the documentation
    above. This gap determines both the maximal constraint violation
    and the size of the trust region. @retval <linear_constraint,
    rotated_lorentz_cones, z> linear_constraint includes (1)(6)(7)
    rotated_lorentz_cones are (2) (3) When either Q1 or Q2 is zero,
    rotated_lorentz_cones contains only one rotated Lorentz cone,
    either (2) or (3). z is the newly added variable.

Precondition:
1. Q1, Q2 are positive semidefinite.
     2. d is positive.
     3. Q1, Q2, x, x₀ are all of the consistent size.
     4. p and y are of the consistent size.
     5. lower_bound ≤ upper_bound.

    $Raises:

RuntimeError when the precondition is not satisfied.)""";
      } AddRelaxNonConvexQuadraticConstraintInTrustRegion;
      // Symbol: drake::solvers::AddRotationMatrixBoxSphereIntersectionMilpConstraints
      struct /* AddRotationMatrixBoxSphereIntersectionMilpConstraints */ {
        // Source: drake/solvers/mixed_integer_rotation_constraint.h
        const char* doc =
R"""(Adds binary variables that constrain the value of the column *and* row
vectors of R, in order to add the following (in some cases non-convex)
constraints as an MILP. Specifically, for column vectors Ri, we
constrain:

- forall i, |Ri| = 1 ± envelope,
- forall i,j. i ≠ j, Ri.dot(Rj) = 0 ± envelope,
- R2 = R0.cross(R1) ± envelope,
     and again for R0=R1.cross(R2), and R1=R2.cross(R0).

Then all of the same constraints are also added to R^T. The size of
the envelope decreases quickly as num_binary_variables_per_half_axis
is is increased.

Note:
    Creates ``9*2*num_binary_variables_per_half_axis binary``
    variables named "BRpos*(*,*)" and "BRneg*(*,*)", and the same
    number of continuous variables named "CRpos*(*,*)" and
    "CRneg*(*,*)".

Note:
    The particular representation/algorithm here was developed in an
    attempt: - to enable efficient reuse of the variables between the
    constraints between multiple rows/columns (e.g. the constraints on
    Rᵀ use the same variables as the constraints on R), and - to
    facilitate branch-and-bound solution techniques -- binary regions
    are layered so that constraining one region establishes
    constraints on large portions of SO(3), and confers hopefully
    "useful" constraints the on other binary variables.

Parameter ``R``:
    The rotation matrix

Parameter ``num_intervals_per_half_axis``:
    number of intervals for a half axis.

Parameter ``prog``:
    The mathematical program to which the constraints are added.

Note:
    This method uses the same approach as
    MixedIntegerRotationConstraintGenerator with
    kBoxSphereIntersection, namely the feasible sets to both
    relaxation are the same. But they use different sets of binary
    variables, and thus the computation speed can be different inside
    optimization solvers.)""";
      } AddRotationMatrixBoxSphereIntersectionMilpConstraints;
      // Symbol: drake::solvers::AddRotationMatrixBoxSphereIntersectionReturn
      struct /* AddRotationMatrixBoxSphereIntersectionReturn */ {
        // Source: drake/solvers/mixed_integer_rotation_constraint.h
        const char* doc =
R"""(Some of the newly added variables in function
AddRotationMatrixBoxSphereIntersectionMilpConstraints. CRpos, CRneg,
BRpos and BRneg can only take value 0 or 1. ``CRpos`` and ``CRneg``
are declared as continuous variables, while ``BRpos`` and ``BRneg``
are declared as binary variables. The definition for these variables
are


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    CRpos[k](i, j) = 1 => k / N <= R(i, j) <= (k+1) / N
      CRneg[k](i, j) = 1 => -(k+1) / N <= R(i, j) <= -k / N
      BRpos[k](i, j) = 1 => R(i, j) >= k / N
      BRneg[k](i, j) = 1 => R(i, j) <= -k / N

.. raw:: html

    </details>

where ``N`` is ``num_intervals_per_half_axis``, one of the input
argument of AddRotationMatrixBoxSphereIntersectionMilpConstraints.)""";
        // Symbol: drake::solvers::AddRotationMatrixBoxSphereIntersectionReturn::BRneg
        struct /* BRneg */ {
          // Source: drake/solvers/mixed_integer_rotation_constraint.h
          const char* doc = R"""()""";
        } BRneg;
        // Symbol: drake::solvers::AddRotationMatrixBoxSphereIntersectionReturn::BRpos
        struct /* BRpos */ {
          // Source: drake/solvers/mixed_integer_rotation_constraint.h
          const char* doc = R"""()""";
        } BRpos;
        // Symbol: drake::solvers::AddRotationMatrixBoxSphereIntersectionReturn::CRneg
        struct /* CRneg */ {
          // Source: drake/solvers/mixed_integer_rotation_constraint.h
          const char* doc = R"""()""";
        } CRneg;
        // Symbol: drake::solvers::AddRotationMatrixBoxSphereIntersectionReturn::CRpos
        struct /* CRpos */ {
          // Source: drake/solvers/mixed_integer_rotation_constraint.h
          const char* doc = R"""()""";
        } CRpos;
      } AddRotationMatrixBoxSphereIntersectionReturn;
      // Symbol: drake::solvers::AddRotationMatrixOrthonormalSocpConstraint
      struct /* AddRotationMatrixOrthonormalSocpConstraint */ {
        // Source: drake/solvers/rotation_constraint.h
        const char* doc =
R"""(Adds a set of convex constraints which approximate the set of
orthogonal matrices, O(3). Adds the bilinear constraints that the each
column Ri has length <= 1 and that Ri'Rj approx 0 via -2 + |Ri|^2 +
|Rj|^2 <= 2Ri'Rj <= 2 - |Ri|^2 - |Rj|^2 (for all i!=j), using a
second-order-cone relaxation. Additionally, the same constraints are
applied to all of the rows.)""";
      } AddRotationMatrixOrthonormalSocpConstraint;
      // Symbol: drake::solvers::AddRotationMatrixSpectrahedralSdpConstraint
      struct /* AddRotationMatrixSpectrahedralSdpConstraint */ {
        // Source: drake/solvers/rotation_constraint.h
        const char* doc =
R"""(Adds constraint (10) from https://arxiv.org/pdf/1403.4914.pdf , which
exactly represents the convex hull of all rotation matrices in 3D.)""";
      } AddRotationMatrixSpectrahedralSdpConstraint;
      // Symbol: drake::solvers::AddSos2Constraint
      struct /* AddSos2Constraint */ {
        // Source: drake/solvers/mixed_integer_optimization_util.h
        const char* doc =
R"""(Adds the special ordered set 2 (SOS2) constraint. y(i) takes binary
values (either 0 or 1).


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    y(i) = 1 => λ(i) + λ(i + 1) = 1.

.. raw:: html

    </details>

See also:
    AddLogarithmicSos2Constraint for a complete explanation on SOS2
    constraint.

Parameter ``prog``:
    The optimization program to which the SOS2 constraint is added.

Parameter ``lambda``:
    At most two entries in λ can be strictly positive, and these two
    entries have to be adjacent. All other entries are zero. Moreover,
    these two entries should sum up to 1.

Parameter ``y``:
    y(i) takes binary value, and determines which two entries in λ can
    be strictly positive. Throw a runtime error if y.rows() !=
    lambda.rows() - 1.)""";
      } AddSos2Constraint;
      // Symbol: drake::solvers::AggregateBoundingBoxConstraints
      struct /* AggregateBoundingBoxConstraints */ {
        // Source: drake/solvers/aggregate_costs_constraints.h
        const char* doc_was_unable_to_choose_unambiguous_names =
R"""(Aggregates many bounding box constraints, returns the intersection
(the tightest bounds) of these constraints.

Parameter ``bounding_box_constraints``:
    The constraints to be aggregated.

Returns ``aggregated_bounds``:
    aggregated_bounds[var.get_id()] returns the (lower, upper) bounds
    of that variable as the tightest bounds of
    ``bounding_box_constraints``.)""";
      } AggregateBoundingBoxConstraints;
      // Symbol: drake::solvers::AggregateDuplicateVariables
      struct /* AggregateDuplicateVariables */ {
        // Source: drake/solvers/aggregate_costs_constraints.h
        const char* doc =
R"""(For linear expression A * vars where ``vars`` might contain duplicated
entries, rewrite this linear expression as A_new * vars_new where
vars_new doesn't contain duplicated entries.)""";
      } AggregateDuplicateVariables;
      // Symbol: drake::solvers::AggregateLinearCosts
      struct /* AggregateLinearCosts */ {
        // Source: drake/solvers/aggregate_costs_constraints.h
        const char* doc =
R"""(Given many linear costs, aggregate them into

aᵀ*x + b,

Parameter ``linear_costs``:
    the linear costs to be aggregated.

Parameter ``linear_coeff``:
    a in the documentation above.

Parameter ``vars``:
    x in the documentation above.

Parameter ``constant_cost``:
    b in the documentation above.)""";
      } AggregateLinearCosts;
      // Symbol: drake::solvers::AggregateQuadraticAndLinearCosts
      struct /* AggregateQuadraticAndLinearCosts */ {
        // Source: drake/solvers/aggregate_costs_constraints.h
        const char* doc =
R"""(Given many linear and quadratic costs, aggregate them into

0.5*x₁ᵀQx₁ + bᵀx₂ + c where x₁ and x₂ don't need to be the same.

Parameter ``quadratic_costs``:
    The quadratic costs to be aggregated.

Parameter ``linear_costs``:
    The linear costs to be aggregated.

Parameter ``Q_lower``:
    The lower triangular part of the matrix Q.

Parameter ``quadratic_vars``:
    x₁ in the documentation above.

Parameter ``linear_coeff``:
    b in the documentation above.

Parameter ``linear_vars``:
    x₂ in the documentation above.

Parameter ``constant_cost``:
    c in the documentation above.)""";
      } AggregateQuadraticAndLinearCosts;
      // Symbol: drake::solvers::AreRequiredAttributesSupported
      struct /* AreRequiredAttributesSupported */ {
        // Source: drake/solvers/program_attribute.h
        const char* doc =
R"""(Returns true iff ``required`` is a subset of ``supported``.

Parameter ``unsupported_message``:
    (Optional) When provided, if this function returns false, the
    message will be set to a phrase describing the unsupported
    attributes; or if this function returns true, the message will be
    set to the empty string.)""";
      } AreRequiredAttributesSupported;
      // Symbol: drake::solvers::AugmentedLagrangianNonsmooth
      struct /* AugmentedLagrangianNonsmooth */ {
        // Source: drake/solvers/augmented_lagrangian.h
        const char* doc =
R"""(Compute the augmented Lagrangian (AL) of a given mathematical program

min f(x) s.t h(x) = 0 l <= g(x) <= u x_lo <= x <= x_up

We first turn it into an equality constrained program with
non-negative slack variable s as follows

min f(x) s.t h(x) = 0 c(x) - s = 0 s >= 0

Depending on the option include_x_bounds, the constraint h(x)=0,
c(x)>=0 may or may not include the bounding box constraint x_lo <= x
<= x_up.

the (non-smooth) augmented Lagrangian is defined as

L(x, λ, μ) = f(x) − λ₁ᵀh(x) + μ/2 h(x)ᵀh(x) - λ₂ᵀ(c(x)-s) + μ/2
(c(x)-s)ᵀ(c(x)-s)

where s = max(c(x) - λ₂/μ, 0).

For more details, refer to section 17.4 of Numerical Optimization by
Jorge Nocedal and Stephen Wright, Edition 1, 1999 (This formulation
isn't presented in Edition 2, but to stay consistent with Edition 2,
we use μ/2 as the coefficient of the quadratic penalty term instead of
1/(2μ) in Edition 1). Note that the augmented Lagrangian L(x, λ, μ) is
NOT a smooth function of x, since s = max(c(x) - λ₂/μ, 0) is
non-smooth at c(x) - λ₂/μ = 0.)""";
        // Symbol: drake::solvers::AugmentedLagrangianNonsmooth::AugmentedLagrangianNonsmooth
        struct /* ctor */ {
          // Source: drake/solvers/augmented_lagrangian.h
          const char* doc =
R"""(Parameter ``prog``:
    The mathematical program we will evaluate.

Parameter ``include_x_bounds``:
    Whether the Lagrangian and the penalty for the bounds x_lo <= x <=
    x_up are included in the augmented Lagrangian L(x, λ, μ) or not.)""";
        } ctor;
        // Symbol: drake::solvers::AugmentedLagrangianNonsmooth::Eval
        struct /* Eval */ {
          // Source: drake/solvers/augmented_lagrangian.h
          const char* doc =
R"""(Parameter ``x``:
    The value of all the decision variables.

Parameter ``lambda_val``:
    The estimated Lagrangian multipliers. The order of the Lagrangian
    multiplier is as this: We first call to evaluate all constraints.
    Then for each row of the constraint, if it is an equality
    constraint, we append one single Lagrangian multiplier. Otherwise
    we append the Lagrangian multiplier for the lower and upper bounds
    (where the lower comes before the upper), if the corresponding
    bound is not ±∞. The order of evaluating all the constraints is
    the same as prog.GetAllConstraints() except for
    prog.bounding_box_constraints(). If include_x_bounds=true, then we
    aggregate all the bounding_box_constraints() and evaluate them at
    the end of all constraints.

Parameter ``mu``:
    μ in the documentation above. The constant for penalty term
    weight. This should be a strictly positive number.

Parameter ``constraint_residue``:
    The value of the all the constraints. For an equality constraint
    c(x)=0 or the inequality constraint c(x)>= 0, the residue is c(x).
    Depending on include_x_bounds, ``constraint_residue`` may or may
    not contain the residue for bounding box constraints x_lo <= x <=
    x_up at the end.

Parameter ``cost``:
    The value of the cost function f(x).

Returns:
    The evaluated Augmented Lagrangian (AL) L(x, λ, μ).)""";
        } Eval;
        // Symbol: drake::solvers::AugmentedLagrangianNonsmooth::include_x_bounds
        struct /* include_x_bounds */ {
          // Source: drake/solvers/augmented_lagrangian.h
          const char* doc =
R"""(Returns:
    Whether the bounding box constraint x_lo <= x <= x_up is included
    in the augmented Lagrangian L(x, λ, μ).)""";
        } include_x_bounds;
        // Symbol: drake::solvers::AugmentedLagrangianNonsmooth::is_equality
        struct /* is_equality */ {
          // Source: drake/solvers/augmented_lagrangian.h
          const char* doc =
R"""(Returns:
    Whether each constraint is equality or not. The order of the
    constraint is explained in the class documentation.)""";
        } is_equality;
        // Symbol: drake::solvers::AugmentedLagrangianNonsmooth::lagrangian_size
        struct /* lagrangian_size */ {
          // Source: drake/solvers/augmented_lagrangian.h
          const char* doc =
R"""(Returns:
    The size of the Lagrangian multiplier λ.)""";
        } lagrangian_size;
        // Symbol: drake::solvers::AugmentedLagrangianNonsmooth::prog
        struct /* prog */ {
          // Source: drake/solvers/augmented_lagrangian.h
          const char* doc =
R"""(Returns:
    The mathematical program for which the augmented Lagrangian is
    computed.)""";
        } prog;
        // Symbol: drake::solvers::AugmentedLagrangianNonsmooth::x_lo
        struct /* x_lo */ {
          // Source: drake/solvers/augmented_lagrangian.h
          const char* doc =
R"""(Returns:
    all the lower bounds of x.)""";
        } x_lo;
        // Symbol: drake::solvers::AugmentedLagrangianNonsmooth::x_up
        struct /* x_up */ {
          // Source: drake/solvers/augmented_lagrangian.h
          const char* doc =
R"""(Returns:
    all the upper bounds of x.)""";
        } x_up;
      } AugmentedLagrangianNonsmooth;
      // Symbol: drake::solvers::AugmentedLagrangianSmooth
      struct /* AugmentedLagrangianSmooth */ {
        // Source: drake/solvers/augmented_lagrangian.h
        const char* doc =
R"""(Compute the augmented Lagrangian (AL) of a given mathematical program

min f(x) s.t h(x) = 0 l <= g(x) <= u x_lo <= x <= x_up

We first turn it into an equality constrained program with
non-negative slack variable s as follows

min f(x) s.t h(x) = 0 c(x) - s = 0 s >= 0

We regard this as an optimization problem on variable (x, s), with
equality constraints h(x) = 0, c(x)-s = 0, and the bound constraint s
>= 0.

Depending on the option include_x_bounds, the constraint h(x)=0,
c(x)>=0 may or may not include the bounding box constraint x_lo <= x
<= x_up.

The (smooth) augmented Lagrangian is defined as

L(x, s, λ, μ) = f(x) − λ₁ᵀh(x) + μ/2 h(x)ᵀh(x) - λ₂ᵀ(c(x)-s) + μ/2
(c(x)-s)ᵀ(c(x)-s)

For more details, refer to section 17.4 of Numerical Optimization by
Jorge Nocedal and Stephen Wright, Edition 2, 2006. Note that the
augmented Lagrangian L(x, s, λ, μ) is a smooth function of (x, s),

This is the implementation used in LANCELOT. To solve the nonlinear
optimization through this Augmented Lagrangian, the nonlinear solve
should be able to handle bounding box constraints on the decision
variables.)""";
        // Symbol: drake::solvers::AugmentedLagrangianSmooth::AugmentedLagrangianSmooth
        struct /* ctor */ {
          // Source: drake/solvers/augmented_lagrangian.h
          const char* doc =
R"""(Parameter ``prog``:
    The mathematical program we will evaluate.

Parameter ``include_x_bounds``:
    Whether the Lagrangian and the penalty for the bounds x_lo <= x <=
    x_up are included in the augmented Lagrangian L(x, s, λ, μ) or
    not.)""";
        } ctor;
        // Symbol: drake::solvers::AugmentedLagrangianSmooth::Eval
        struct /* Eval */ {
          // Source: drake/solvers/augmented_lagrangian.h
          const char* doc =
R"""(Parameter ``x``:
    The value of all the decision variables in prog().

Parameter ``s``:
    The value of all slack variables s.

Parameter ``lambda_val``:
    The estimated Lagrangian multipliers. The order of the Lagrangian
    multiplier is as follows: We first call to evaluate all
    constraints. Then for each row of the constraint, if it is an
    equality constraint, we append one single Lagrangian multiplier.
    Otherwise we append the Lagrangian multiplier for the lower and
    upper bounds (where the lower comes before the upper), if the
    corresponding bound is not ±∞. The order of evaluating all the
    constraints is the same as prog.GetAllConstraints() except for
    prog.bounding_box_constraints(). If include_x_bounds=true, then we
    aggregate all the bounding_box_constraints() and evaluate them at
    the end of all constraints.

Parameter ``mu``:
    μ in the documentation above. The constant for penalty term
    weight. This should be a strictly positive number.

Parameter ``constraint_residue``:
    The value of the all the constraints. For an equality constraint
    c(x)=0, the residue is c(x); for an inequality constraint c(x)>=0,
    the residue is c(x)-s where s is the corresponding slack variable.
    Depending on include_x_bounds, ``constraint_residue`` may or may
    not contain the residue for bounding box constraints x_lo <= x <=
    x_up at the end.

Parameter ``cost``:
    The value of the cost function f(x).

Returns:
    The evaluated Augmented Lagrangian (AL) L(x, s, λ, μ).

Note:
    This Eval function differs from
    AugmentedLagrangianNonsmooth∷Eval() function as ``s`` is an input
    argument.)""";
        } Eval;
        // Symbol: drake::solvers::AugmentedLagrangianSmooth::include_x_bounds
        struct /* include_x_bounds */ {
          // Source: drake/solvers/augmented_lagrangian.h
          const char* doc =
R"""(Returns:
    Whether the bounding box constraint x_lo <= x <= x_up is included
    in the augmented Lagrangian L(x, λ, μ).)""";
        } include_x_bounds;
        // Symbol: drake::solvers::AugmentedLagrangianSmooth::is_equality
        struct /* is_equality */ {
          // Source: drake/solvers/augmented_lagrangian.h
          const char* doc =
R"""(Returns:
    Whether each constraint is equality or not. The order of the
    constraint is explained in the class documentation.)""";
        } is_equality;
        // Symbol: drake::solvers::AugmentedLagrangianSmooth::lagrangian_size
        struct /* lagrangian_size */ {
          // Source: drake/solvers/augmented_lagrangian.h
          const char* doc =
R"""(Returns:
    The size of the Lagrangian multiplier λ.)""";
        } lagrangian_size;
        // Symbol: drake::solvers::AugmentedLagrangianSmooth::prog
        struct /* prog */ {
          // Source: drake/solvers/augmented_lagrangian.h
          const char* doc =
R"""(Returns:
    The mathematical program for which the augmented Lagrangian is
    computed.)""";
        } prog;
        // Symbol: drake::solvers::AugmentedLagrangianSmooth::s_size
        struct /* s_size */ {
          // Source: drake/solvers/augmented_lagrangian.h
          const char* doc =
R"""(Returns:
    The size of the slack variable s.)""";
        } s_size;
        // Symbol: drake::solvers::AugmentedLagrangianSmooth::x_lo
        struct /* x_lo */ {
          // Source: drake/solvers/augmented_lagrangian.h
          const char* doc =
R"""(Returns:
    All the lower bounds of x.)""";
        } x_lo;
        // Symbol: drake::solvers::AugmentedLagrangianSmooth::x_up
        struct /* x_up */ {
          // Source: drake/solvers/augmented_lagrangian.h
          const char* doc =
R"""(Returns:
    All the upper bounds of x.)""";
        } x_up;
      } AugmentedLagrangianSmooth;
      // Symbol: drake::solvers::Binding
      struct /* Binding */ {
        // Source: drake/solvers/binding.h
        const char* doc =
R"""(A binding on constraint type C is a mapping of the decision variables
onto the inputs of C. This allows the constraint to operate on a
vector made up of different elements of the decision variables.)""";
        // Symbol: drake::solvers::Binding::Binding<C>
        struct /* ctor */ {
          // Source: drake/solvers/binding.h
          const char* doc =
R"""(Concatenates each VectorDecisionVariable object in ``v`` into a single
column vector, binds this column vector of decision variables with the
constraint ``c``.)""";
        } ctor;
        // Symbol: drake::solvers::Binding::ContainsVariable
        struct /* ContainsVariable */ {
          // Source: drake/solvers/binding.h
          const char* doc =
R"""(Returns true iff the given ``var`` is included in this Binding.)""";
        } ContainsVariable;
        // Symbol: drake::solvers::Binding::GetNumElements
        struct /* GetNumElements */ {
          // Source: drake/solvers/binding.h
          const char* doc =
R"""(Returns the number of variables associated with this evaluator.)""";
        } GetNumElements;
        // Symbol: drake::solvers::Binding::ToLatex
        struct /* ToLatex */ {
          // Source: drake/solvers/binding.h
          const char* doc =
R"""(Returns a LaTeX description of this Binding. Does not include any
characters to enter/exit math mode; you might want, e.g. "$$" +
evaluator.ToLatex() + "$$".)""";
        } ToLatex;
        // Symbol: drake::solvers::Binding::evaluator
        struct /* evaluator */ {
          // Source: drake/solvers/binding.h
          const char* doc = R"""()""";
        } evaluator;
        // Symbol: drake::solvers::Binding::operator!=
        struct /* operator_ne */ {
          // Source: drake/solvers/binding.h
          const char* doc = R"""()""";
        } operator_ne;
        // Symbol: drake::solvers::Binding::to_string
        struct /* to_string */ {
          // Source: drake/solvers/binding.h
          const char* doc =
R"""(Returns string representation of Binding.)""";
        } to_string;
        // Symbol: drake::solvers::Binding::variables
        struct /* variables */ {
          // Source: drake/solvers/binding.h
          const char* doc = R"""()""";
        } variables;
      } Binding;
      // Symbol: drake::solvers::Bound
      struct /* Bound */ {
        // Source: drake/solvers/aggregate_costs_constraints.h
        const char* doc =
R"""(Stores the lower and upper bound of a variable.)""";
        // Symbol: drake::solvers::Bound::lower
        struct /* lower */ {
          // Source: drake/solvers/aggregate_costs_constraints.h
          const char* doc = R"""(Lower bound.)""";
        } lower;
        // Symbol: drake::solvers::Bound::upper
        struct /* upper */ {
          // Source: drake/solvers/aggregate_costs_constraints.h
          const char* doc = R"""(Upper bound.)""";
        } upper;
      } Bound;
      // Symbol: drake::solvers::BoundingBoxConstraint
      struct /* BoundingBoxConstraint */ {
        // Source: drake/solvers/constraint.h
        const char* doc =
R"""(Implements a constraint of the form :math:`lb <= x <= ub`

Note: the base Constraint class (as implemented at the moment) could
play this role. But this class enforces that it is ONLY a bounding box
constraint, and not something more general. Some solvers use this
information to handle bounding box constraints differently than
general constraints, so use of this form is encouraged.)""";
        // Symbol: drake::solvers::BoundingBoxConstraint::BoundingBoxConstraint
        struct /* ctor */ {
          // Source: drake/solvers/constraint.h
          const char* doc = R"""()""";
        } ctor;
      } BoundingBoxConstraint;
      // Symbol: drake::solvers::CeilLog2
      struct /* CeilLog2 */ {
        // Source: drake/solvers/mixed_integer_optimization_util.h
        const char* doc =
R"""(Return ⌈log₂(n)⌉, namely the minimal integer no smaller than log₂(n),
with base 2.

Parameter ``n``:
    A positive integer.

Returns:
    The minimal integer no smaller than log₂(n).)""";
      } CeilLog2;
      // Symbol: drake::solvers::ChooseBestSolver
      struct /* ChooseBestSolver */ {
        // Source: drake/solvers/choose_best_solver.h
        const char* doc =
R"""(Choose the best solver given the formulation in the optimization
program and the availability of the solvers.

Raises:
    RuntimeError if there is no available solver for ``prog``.)""";
      } ChooseBestSolver;
      // Symbol: drake::solvers::ClarabelSolver
      struct /* ClarabelSolver */ {
        // Source: drake/solvers/clarabel_solver.h
        const char* doc =
R"""(An interface to wrap Clarabel
https://github.com/oxfordcontrol/Clarabel.cpp)""";
        // Symbol: drake::solvers::ClarabelSolver::ClarabelSolver
        struct /* ctor */ {
          // Source: drake/solvers/clarabel_solver.h
          const char* doc = R"""()""";
        } ctor;
        // Symbol: drake::solvers::ClarabelSolver::Details
        struct /* Details */ {
          // Source: drake/solvers/clarabel_solver.h
          const char* doc =
R"""(Type of details stored in MathematicalProgramResult.)""";
        } Details;
        // Symbol: drake::solvers::ClarabelSolver::ProgramAttributesSatisfied
        struct /* ProgramAttributesSatisfied */ {
          // Source: drake/solvers/clarabel_solver.h
          const char* doc = R"""()""";
        } ProgramAttributesSatisfied;
        // Symbol: drake::solvers::ClarabelSolver::UnsatisfiedProgramAttributes
        struct /* UnsatisfiedProgramAttributes */ {
          // Source: drake/solvers/clarabel_solver.h
          const char* doc = R"""()""";
        } UnsatisfiedProgramAttributes;
        // Symbol: drake::solvers::ClarabelSolver::id
        struct /* id */ {
          // Source: drake/solvers/clarabel_solver.h
          const char* doc = R"""()""";
        } id;
        // Symbol: drake::solvers::ClarabelSolver::is_available
        struct /* is_available */ {
          // Source: drake/solvers/clarabel_solver.h
          const char* doc = R"""()""";
        } is_available;
        // Symbol: drake::solvers::ClarabelSolver::is_enabled
        struct /* is_enabled */ {
          // Source: drake/solvers/clarabel_solver.h
          const char* doc = R"""()""";
        } is_enabled;
      } ClarabelSolver;
      // Symbol: drake::solvers::ClarabelSolverDetails
      struct /* ClarabelSolverDetails */ {
        // Source: drake/solvers/clarabel_solver.h
        const char* doc =
R"""(The Clarabel solver details after calling the Solve() function. The
user can call
MathematicalProgramResult∷get_solver_details<ClarabelSolver>() to
obtain the details.)""";
        // Symbol: drake::solvers::ClarabelSolverDetails::iterations
        struct /* iterations */ {
          // Source: drake/solvers/clarabel_solver.h
          const char* doc = R"""(Number of iterations in Clarabel.)""";
        } iterations;
        // Symbol: drake::solvers::ClarabelSolverDetails::solve_time
        struct /* solve_time */ {
          // Source: drake/solvers/clarabel_solver.h
          const char* doc =
R"""(The solve time inside Clarabel in seconds.)""";
        } solve_time;
        // Symbol: drake::solvers::ClarabelSolverDetails::status
        struct /* status */ {
          // Source: drake/solvers/clarabel_solver.h
          const char* doc = R"""(The status from Clarabel.)""";
        } status;
      } ClarabelSolverDetails;
      // Symbol: drake::solvers::ClpSolver
      struct /* ClpSolver */ {
        // Source: drake/solvers/clp_solver.h
        const char* doc =
R"""(A wrapper to call CLP using Drake's MathematicalProgram.

Note:
    Currently our ClpSolver has a memory issue when solving a QP. The
    user should be aware of this risk.

Note:
    The authors can adjust the problem scaling option by setting
    "scaling" as mentioned in
    https://github.com/coin-or/Clp/blob/43129ba1a7fd66ce70fe0761fcd696951917ed2e/src/ClpModel.hpp#L705-L706
    For example prog.SetSolverOption(ClpSolver∷id(), "scaling", 0);
    will do "no scaling". The default is 1.)""";
        // Symbol: drake::solvers::ClpSolver::ClpSolver
        struct /* ctor */ {
          // Source: drake/solvers/clp_solver.h
          const char* doc = R"""()""";
        } ctor;
        // Symbol: drake::solvers::ClpSolver::Details
        struct /* Details */ {
          // Source: drake/solvers/clp_solver.h
          const char* doc = R"""()""";
        } Details;
        // Symbol: drake::solvers::ClpSolver::ProgramAttributesSatisfied
        struct /* ProgramAttributesSatisfied */ {
          // Source: drake/solvers/clp_solver.h
          const char* doc = R"""()""";
        } ProgramAttributesSatisfied;
        // Symbol: drake::solvers::ClpSolver::UnsatisfiedProgramAttributes
        struct /* UnsatisfiedProgramAttributes */ {
          // Source: drake/solvers/clp_solver.h
          const char* doc = R"""()""";
        } UnsatisfiedProgramAttributes;
        // Symbol: drake::solvers::ClpSolver::id
        struct /* id */ {
          // Source: drake/solvers/clp_solver.h
          const char* doc = R"""()""";
        } id;
        // Symbol: drake::solvers::ClpSolver::is_available
        struct /* is_available */ {
          // Source: drake/solvers/clp_solver.h
          const char* doc = R"""()""";
        } is_available;
        // Symbol: drake::solvers::ClpSolver::is_enabled
        struct /* is_enabled */ {
          // Source: drake/solvers/clp_solver.h
          const char* doc = R"""()""";
        } is_enabled;
      } ClpSolver;
      // Symbol: drake::solvers::ClpSolverDetails
      struct /* ClpSolverDetails */ {
        // Source: drake/solvers/clp_solver.h
        const char* doc =
R"""(The CLP solver details after calling Solve() function. The user can
call MathematicalProgramResult∷get_solver_details<ClpSolver>() to
obtain the details.)""";
        // Symbol: drake::solvers::ClpSolverDetails::clp_version
        struct /* clp_version */ {
          // Source: drake/solvers/clp_solver.h
          const char* doc = R"""(The CLP_VERSION from the Clp build.)""";
        } clp_version;
        // Symbol: drake::solvers::ClpSolverDetails::status
        struct /* status */ {
          // Source: drake/solvers/clp_solver.h
          const char* doc =
R"""(Refer to ClpModel∷status() function for the meaning of the status
code. - -1: unknown error. - 0: optimal. - 1: primal infeasible - 2:
dual infeasible - 3: stopped on iterations or time. - 4: stopped due
to errors - 5: stopped by event handler)""";
        } status;
      } ClpSolverDetails;
      // Symbol: drake::solvers::CommonSolverOption
      struct /* CommonSolverOption */ {
        // Source: drake/solvers/common_solver_option.h
        const char* doc =
R"""(Some options can be applied to not one solver, but many solvers (for
example, many solvers support printing out the progress in each
iteration). CommonSolverOption contain the names of these supported
options. The user can use these options as "key" in
SolverOption∷SetOption(). If the solver doesn't support the option,
the option is ignored.)""";
        // Symbol: drake::solvers::CommonSolverOption::kMaxThreads
        struct /* kMaxThreads */ {
          // Source: drake/solvers/common_solver_option.h
          const char* doc =
R"""(Some solvers are multi-threaded. The user can request the maximum
number of threads used by the solver with this ``int`` option. When
not set, the value defaults to Parallelism.Max().num_threads(), which
can be controlled via the drake∷Parallelism "DRAKE_NUM_THREADS"
environment variable.

Precondition:
    The number of threads must be greater than 0.

Note:
    Setting this value higher than the actual hardware concurrency may
    result in a degraded performance. It is recommended to set this
    value lower than or equal to Parallelism.Max().num_threads().

Note:
    A solver may choose to use fewer threads than the value specified.

Note:
    This options does NOT disable multi-threading in BLAS/LAPACK which
    is used by many solvers under the hood. Therefore, some internal
    operations of the solvers may still be multi-core.)""";
        } kMaxThreads;
        // Symbol: drake::solvers::CommonSolverOption::kPrintFileName
        struct /* kPrintFileName */ {
          // Source: drake/solvers/common_solver_option.h
          const char* doc =
R"""(Many solvers support printing the progress of each iteration to a
file. The user can call SolverOptions∷SetOption(kPrintFileName,
"filename.log") to enable this. To disable, set the option to the
empty string ``""``, which indicates that no file should be written.)""";
        } kPrintFileName;
        // Symbol: drake::solvers::CommonSolverOption::kPrintToConsole
        struct /* kPrintToConsole */ {
          // Source: drake/solvers/common_solver_option.h
          const char* doc =
R"""(Many solvers support printing the progress of each iteration to the
console. The user can call ``SolverOptions∷SetOption(kPrintToConsole,
1)`` to enable this, or use ``0`` to turn off printing to the console.)""";
        } kPrintToConsole;
        // Symbol: drake::solvers::CommonSolverOption::kStandaloneReproductionFileName
        struct /* kStandaloneReproductionFileName */ {
          // Source: drake/solvers/common_solver_option.h
          const char* doc =
R"""(Some solvers support writing a standalone (e.g., it does not depend on
Drake) minimal reproduction of the problem to a file. This is
especially useful for sending bug reports upstream to the developers
of the solver. The user can call
``SolverOptions∷SetOption(kStandaloneReproductionFileName,
"filename.txt")`` to enable this. To disable, set the option to the
empty string ``""``, which indicates that no file should be written.)""";
        } kStandaloneReproductionFileName;
      } CommonSolverOption;
      // Symbol: drake::solvers::ConcatenateIndeterminatesRefList
      struct /* ConcatenateIndeterminatesRefList */ {
        // Source: drake/solvers/indeterminate.h
        const char* doc =
R"""(Concatenates each element in ``var_list`` into a single Eigen vector
of indeterminates, returns this concatenated vector.)""";
      } ConcatenateIndeterminatesRefList;
      // Symbol: drake::solvers::ConcatenateVariableRefList
      struct /* ConcatenateVariableRefList */ {
        // Source: drake/solvers/decision_variable.h
        const char* doc =
R"""(Concatenates each element in ``var_list`` into a single Eigen vector
of decision variables, returns this concatenated vector.)""";
      } ConcatenateVariableRefList;
      // Symbol: drake::solvers::Constraint
      struct /* Constraint */ {
        // Source: drake/solvers/constraint.h
        const char* doc =
R"""(A constraint is a function + lower and upper bounds.

Solver interfaces must acknowledge that these constraints are mutable.
Parameters can change after the constraint is constructed and before
the call to Solve().

It should support evaluating the constraint, and adding it to an
optimization problem.)""";
        // Symbol: drake::solvers::Constraint::CheckSatisfied
        struct /* CheckSatisfied */ {
          // Source: drake/solvers/constraint.h
          const char* doc =
R"""(Return whether this constraint is satisfied by the given value, ``x``.

Parameter ``x``:
    A ``num_vars`` x 1 vector.

Parameter ``tol``:
    A tolerance for bound checking.

Raises:
    RuntimeError if the size of x isn't correct.)""";
        } CheckSatisfied;
        // Symbol: drake::solvers::Constraint::Constraint
        struct /* ctor */ {
          // Source: drake/solvers/constraint.h
          const char* doc_5args =
R"""(Constructs a constraint which has ``num_constraints`` rows, with an
input ``num_vars`` x 1 vector.

Parameter ``num_constraints``:
    The number of rows in the constraint output.

Parameter ``num_vars``:
    The number of rows in the input. If the input dimension is
    unknown, then set ``num_vars`` to Eigen∷Dynamic.

Parameter ``lb``:
    Lower bound, which must be a ``num_constraints`` x 1 vector, lb
    cannot contain NAN.

Parameter ``ub``:
    Upper bound, which must be a ``num_constraints`` x 1 vector, ub
    cannot contain NAN.

See also:
    Eval(...))""";
          // Source: drake/solvers/constraint.h
          const char* doc_2args =
R"""(Constructs a constraint which has ``num_constraints`` rows, with an
input ``num_vars`` x 1 vector, with no bounds.

Parameter ``num_constraints``:
    The number of rows in the constraint output.

Parameter ``num_vars``:
    The number of rows in the input. If the input dimension is
    unknown, then set ``num_vars`` to Eigen∷Dynamic.

See also:
    Eval(...))""";
        } ctor;
        // Symbol: drake::solvers::Constraint::DoCheckSatisfied
        struct /* DoCheckSatisfied */ {
          // Source: drake/solvers/constraint.h
          const char* doc = R"""()""";
        } DoCheckSatisfied;
        // Symbol: drake::solvers::Constraint::UpdateLowerBound
        struct /* UpdateLowerBound */ {
          // Source: drake/solvers/constraint.h
          const char* doc =
R"""(Updates the lower bound.

Note:
    if the users want to expose this method in a sub-class, do using
    Constraint∷UpdateLowerBound, as in LinearConstraint.)""";
        } UpdateLowerBound;
        // Symbol: drake::solvers::Constraint::UpdateUpperBound
        struct /* UpdateUpperBound */ {
          // Source: drake/solvers/constraint.h
          const char* doc =
R"""(Updates the upper bound.

Note:
    if the users want to expose this method in a sub-class, do using
    Constraint∷UpdateUpperBound, as in LinearConstraint.)""";
        } UpdateUpperBound;
        // Symbol: drake::solvers::Constraint::lower_bound
        struct /* lower_bound */ {
          // Source: drake/solvers/constraint.h
          const char* doc = R"""()""";
        } lower_bound;
        // Symbol: drake::solvers::Constraint::num_constraints
        struct /* num_constraints */ {
          // Source: drake/solvers/constraint.h
          const char* doc =
R"""(Number of rows in the output constraint.)""";
        } num_constraints;
        // Symbol: drake::solvers::Constraint::set_bounds
        struct /* set_bounds */ {
          // Source: drake/solvers/constraint.h
          const char* doc =
R"""(Set the upper and lower bounds of the constraint.

Parameter ``new_lb``:
    A ``num_constraints`` x 1 vector.

Parameter ``new_ub``:
    A ``num_constraints`` x 1 vector.

Note:
    If the users want to expose this method in a sub-class, do using
    Constraint∷set_bounds, as in LinearConstraint.)""";
        } set_bounds;
        // Symbol: drake::solvers::Constraint::upper_bound
        struct /* upper_bound */ {
          // Source: drake/solvers/constraint.h
          const char* doc = R"""()""";
        } upper_bound;
      } Constraint;
      // Symbol: drake::solvers::ConstructMonomialBasis
      struct /* ConstructMonomialBasis */ {
        // Source: drake/solvers/sos_basis_generator.h
        const char* doc =
R"""(Given input polynomial p, outputs a set M of monomials with the
following guarantee: if p = f1*f1 + f2*f2 + ... + fn*fn for some
(unknown) polynomials f1, f2, ..., fn, then the span of M contains f1,
f2, ..., fn, Given M, one can then find the polynomials fi using
semidefinite programming; see, e.g., Chapter 3 of Semidefinite
Optimization and Convex Algebraic Geometry by G. Blekherman, P.
Parrilo, R. Thomas.

Parameter ``p``:
    A polynomial

Returns:
    A vector whose entries are the elements of M)""";
      } ConstructMonomialBasis;
      // Symbol: drake::solvers::Cost
      struct /* Cost */ {
        // Source: drake/solvers/cost.h
        const char* doc =
R"""(Provides an abstract base for all costs.)""";
        // Symbol: drake::solvers::Cost::Cost
        struct /* ctor */ {
          // Source: drake/solvers/cost.h
          const char* doc =
R"""(Constructs a cost evaluator.

Parameter ``num_vars``:
    Number of input variables.

Parameter ``description``:
    Human-friendly description.)""";
        } ctor;
      } Cost;
      // Symbol: drake::solvers::CreateBinaryCodeMatchConstraint
      struct /* CreateBinaryCodeMatchConstraint */ {
        // Source: drake/solvers/integer_optimization_util.h
        const char* doc =
R"""(Create linear constraints such that, when these constraints are
satisfied, match = 1 if and only if code == expected, otherwise match
= 0

Parameter ``code``:
    code(i) should only take binary values.

Parameter ``expected``:
    The expected matched value for code.

Parameter ``match``:
    an expression that takes binary value, representing if code ==
    expected

Returns:
    the linear constraints.

This function is useful integer optimization, for example, if we have
a constraint match = ((b1 == 0) && (b2 == 1) && (b3 == 1)), we can
call the function CreateBinaryCodeMatchConstraint({b1, b2, b3}, {0, 1,
1}, match) to create the constraint.)""";
      } CreateBinaryCodeMatchConstraint;
      // Symbol: drake::solvers::CreateLogicalAndConstraint
      struct /* CreateLogicalAndConstraint */ {
        // Source: drake/solvers/integer_optimization_util.h
        const char* doc =
R"""(Adds linear constraints, such that when b1, b2, b1_and_b2 satisfy the
constraints, and b1, b2 take binary values, it is guaranteed that
b1_and_b2 = b1 ∧ b2 (b1 and b2). The constraints are


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    b1_and_b2 >= b1 + b2 - 1
      b1_and_b2 <= b1
      b1_and_b2 <= b2
      0 <= b1_and_b2 <= 1

.. raw:: html

    </details>

Parameter ``b1``:
    An expression that should only take a binary value.

Parameter ``b2``:
    An expression that should only take a binary value.

Parameter ``b1_and_b2``:
    Should be the logical and between ``b1`` and ``b2``.

Returns:
    The newly added constraints, such that when b1, b2, b1_and_b2
    satisfy the constraints, it is guaranteed that b1_and_b2 = b1 ∧
    b2.

Precondition:
    b1, b2, b1_and_b2 are all linear expressions.)""";
      } CreateLogicalAndConstraint;
      // Symbol: drake::solvers::CreateLogicalOrConstraint
      struct /* CreateLogicalOrConstraint */ {
        // Source: drake/solvers/integer_optimization_util.h
        const char* doc =
R"""(Adds linear constraints, such that when b1, b2, b1_or_b2 satisfy the
constraints, and b1, b2 take binary values, it is guaranteed that
b1_or_b2 = b1 ∨ b2 (b1 or b2). The constraints are


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    b1_or_b2 <= b1 + b2
      b1_or_b2 >= b1
      b1_or_b2 >= b2
      0 <= b1_or_b2 <= 1

.. raw:: html

    </details>

Parameter ``b1``:
    An expression that should only take a binary value.

Parameter ``b2``:
    An expression that should only take a binary value.

Parameter ``b1_or_b2``:
    Should be the logical or between ``b1`` and ``b2``.

Returns:
    The newly added constraints, such that when b1, b2, b1_or_b2
    satisfy the constraints, it is guaranteed that b1_or_b2 = b1 ∨ b2.

Precondition:
    b1, b2, b1_or_b2 are all linear expressions.)""";
      } CreateLogicalOrConstraint;
      // Symbol: drake::solvers::CreateLogicalXorConstraint
      struct /* CreateLogicalXorConstraint */ {
        // Source: drake/solvers/integer_optimization_util.h
        const char* doc =
R"""(Add linear constraints, such that when b1, b2, b1_xor_b2 satisfy the
constraints, and b1, b2 take binary values, it is guaranteed that
b1_xor_b2 = b1 ⊕ b2 (b1 exclusive xor b2). The constraints are


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    b1_xor_b2 <= b1 + b2
      b1_xor_b2 >= b1 - b2
      b1_xor_b2 >= b2 - b1
      b1_xor_b2 <= 2 - b1 - b2
      0 <= b1_xor_b2 <= 1

.. raw:: html

    </details>

Parameter ``b1``:
    An expression that should only take a binary value.

Parameter ``b2``:
    An expression that should only take a binary value.

Parameter ``b1_xor_b2``:
    Should be the logical exclusive or between ``b1`` and ``b2``.

Returns:
    The newly added constraints, such that when b1, b2, b1_xor_b2
    satisfy the constraints, it is guaranteed that b1_xor_b2 = b1 ⊕
    b2.

Precondition:
    b1, b2, b1_xor_b2 are all linear expressions.)""";
      } CreateLogicalXorConstraint;
      // Symbol: drake::solvers::CsdpSolver
      struct /* CsdpSolver */ {
        // Source: drake/solvers/csdp_solver.h
        const char* doc =
R"""(Wrap CSDP solver such that it can solve a
drake∷solvers∷MathematicalProgram.

Note:
    CSDP doesn't accept free variables, while
    drake∷solvers∷MathematicalProgram does. In order to convert
    MathematicalProgram into CSDP format, we provide several
    approaches to remove free variables. You can set the approach
    through


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    {cc}
    SolverOptions solver_options;
    solver_options.SetOption(CsdpSolver∷id(),
       "drake∷RemoveFreeVariableMethod",
       static_cast<int>(RemoveFreeVariableMethod∷kNullspace));
    CsdpSolver solver;
    auto result = solver.Solve(prog, std∷nullopt, solver_options);

.. raw:: html

    </details>

For more details, check out RemoveFreeVariableMethod.)""";
        // Symbol: drake::solvers::CsdpSolver::CsdpSolver
        struct /* ctor */ {
          // Source: drake/solvers/csdp_solver.h
          const char* doc = R"""(Default constructor)""";
        } ctor;
        // Symbol: drake::solvers::CsdpSolver::Details
        struct /* Details */ {
          // Source: drake/solvers/csdp_solver.h
          const char* doc = R"""()""";
        } Details;
        // Symbol: drake::solvers::CsdpSolver::ProgramAttributesSatisfied
        struct /* ProgramAttributesSatisfied */ {
          // Source: drake/solvers/csdp_solver.h
          const char* doc = R"""()""";
        } ProgramAttributesSatisfied;
        // Symbol: drake::solvers::CsdpSolver::id
        struct /* id */ {
          // Source: drake/solvers/csdp_solver.h
          const char* doc = R"""()""";
        } id;
        // Symbol: drake::solvers::CsdpSolver::is_available
        struct /* is_available */ {
          // Source: drake/solvers/csdp_solver.h
          const char* doc = R"""()""";
        } is_available;
        // Symbol: drake::solvers::CsdpSolver::is_enabled
        struct /* is_enabled */ {
          // Source: drake/solvers/csdp_solver.h
          const char* doc = R"""()""";
        } is_enabled;
      } CsdpSolver;
      // Symbol: drake::solvers::CsdpSolverDetails
      struct /* CsdpSolverDetails */ {
        // Source: drake/solvers/csdp_solver.h
        const char* doc =
R"""(The CSDP solver details after calling Solve() function. The user can
call MathematicalProgramResult∷get_solver_details<CsdpSolver>() to
obtain the details.)""";
        // Symbol: drake::solvers::CsdpSolverDetails::Z_val
        struct /* Z_val */ {
          // Source: drake/solvers/csdp_solver.h
          const char* doc = R"""()""";
        } Z_val;
        // Symbol: drake::solvers::CsdpSolverDetails::dual_objective
        struct /* dual_objective */ {
          // Source: drake/solvers/csdp_solver.h
          const char* doc = R"""(The dual objective value.)""";
        } dual_objective;
        // Symbol: drake::solvers::CsdpSolverDetails::primal_objective
        struct /* primal_objective */ {
          // Source: drake/solvers/csdp_solver.h
          const char* doc = R"""(The primal objective value.)""";
        } primal_objective;
        // Symbol: drake::solvers::CsdpSolverDetails::return_code
        struct /* return_code */ {
          // Source: drake/solvers/csdp_solver.h
          const char* doc =
R"""(Refer to the Return Codes section of CSDP 6.2.0 User's Guide for
explanation on the return code. Some of the common return codes are

0 Problem is solved to optimality. 1 Problem is primal infeasible. 2
Problem is dual infeasible. 3 Problem solved to near optimality. 4
Maximum iterations reached. 5 Stuck at edge of primal feasibility. 6
Stuck at edge of dual feasibility. 7 Lack of progress. 8 X, Z, or O is
singular. 9 NaN or Inf values encountered.)""";
        } return_code;
        // Symbol: drake::solvers::CsdpSolverDetails::y_val
        struct /* y_val */ {
          // Source: drake/solvers/csdp_solver.h
          const char* doc =
R"""(CSDP solves a primal problem of the form

max tr(C*X) s.t tr(Aᵢ*X) = aᵢ X ≽ 0

The dual form is

min aᵀy s.t ∑ᵢ yᵢAᵢ - C = Z Z ≽ 0

y, Z are the variables for the dual problem. y_val, Z_val are the
solutions to the dual problem.)""";
        } y_val;
      } CsdpSolverDetails;
      // Symbol: drake::solvers::DecisionVariable
      struct /* DecisionVariable */ {
        // Source: drake/solvers/decision_variable.h
        const char* doc = R"""()""";
      } DecisionVariable;
      // Symbol: drake::solvers::DecomposeNonConvexQuadraticForm
      struct /* DecomposeNonConvexQuadraticForm */ {
        // Source: drake/solvers/non_convex_optimization_util.h
        const char* doc =
R"""(For a non-convex homogeneous quadratic form xᵀQx, where Q is not
necessarily a positive semidefinite matrix, we decompose it as a
difference between two convex homogeneous quadratic forms xᵀQx = xᵀQ₁x
- xᵀQ₂x, Q₁, Q₂ are positive semidefinite. To find the optimal Q₁ and
Q₂, we solve the following semidefinite programming problem min s s.t
s >= trace(Q₁) s >= trace(Q₂) Q₁ - Q₂ = (Q + Qᵀ) / 2 Q₁, Q₂ are
positive semidefinite The decomposition Q = Q₁ - Q₂ can be used later,
to solve the non-convex optimization problem involving a quadratic
form xᵀQx. For more information, please refer to the papers on
difference of convex decomposition, for example Undominated d.c
Decompositions of Quadratic Functions and Applications to
Branch-and-Bound Approaches By I.M.Bomze and M. Locatelli
Computational Optimization and Applications, 2004 DC Decomposition of
Nonconvex Polynomials with Algebraic Techniques By A. A. Ahmadi and G.
Hall Mathematical Programming, 2015

Parameter ``Q``:
    A square matrix.

Raises:
    RuntimeError if Q is not square.

Returns:
    The optimal decomposition (Q₁, Q₂))""";
      } DecomposeNonConvexQuadraticForm;
      // Symbol: drake::solvers::EnumerateIntegerSolutions
      struct /* EnumerateIntegerSolutions */ {
        // Source: drake/solvers/integer_inequality_solver.h
        const char* doc =
R"""(Finds all integer solutions x to the linear inequalities


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    Ax <= b,
                       x <= upper_bound,
                       x >= lower_bound.

.. raw:: html

    </details>

Parameter ``A``:
    An (m x n) integer matrix.

Parameter ``b``:
    An (m x 1) integer vector.

Parameter ``upper_bound``:
    A (n x 1) integer vector.

Parameter ``lower_bound``:
    A (n x 1) integer vector.

Returns:
    A (p x n) matrix whose rows are the solutions.)""";
      } EnumerateIntegerSolutions;
      // Symbol: drake::solvers::EqualityConstrainedQPSolver
      struct /* EqualityConstrainedQPSolver */ {
        // Source: drake/solvers/equality_constrained_qp_solver.h
        const char* doc =
R"""(Solves a quadratic program with equality constraint.

This program doesn't depend on the initial guess.

The user can set the following options:

- FeasibilityTolOptionName(). The feasible solution (both primal and dual
  variables) should satisfy their constraints, with error no larger than
  this value. The default is Eigen∷dummy_precision().)""";
        // Symbol: drake::solvers::EqualityConstrainedQPSolver::EqualityConstrainedQPSolver
        struct /* ctor */ {
          // Source: drake/solvers/equality_constrained_qp_solver.h
          const char* doc = R"""()""";
        } ctor;
        // Symbol: drake::solvers::EqualityConstrainedQPSolver::FeasibilityTolOptionName
        struct /* FeasibilityTolOptionName */ {
          // Source: drake/solvers/equality_constrained_qp_solver.h
          const char* doc =
R"""(Returns:
    string key for SolverOptions to set the feasibility tolerance.)""";
        } FeasibilityTolOptionName;
        // Symbol: drake::solvers::EqualityConstrainedQPSolver::ProgramAttributesSatisfied
        struct /* ProgramAttributesSatisfied */ {
          // Source: drake/solvers/equality_constrained_qp_solver.h
          const char* doc = R"""()""";
        } ProgramAttributesSatisfied;
        // Symbol: drake::solvers::EqualityConstrainedQPSolver::id
        struct /* id */ {
          // Source: drake/solvers/equality_constrained_qp_solver.h
          const char* doc = R"""()""";
        } id;
        // Symbol: drake::solvers::EqualityConstrainedQPSolver::is_available
        struct /* is_available */ {
          // Source: drake/solvers/equality_constrained_qp_solver.h
          const char* doc = R"""()""";
        } is_available;
        // Symbol: drake::solvers::EqualityConstrainedQPSolver::is_enabled
        struct /* is_enabled */ {
          // Source: drake/solvers/equality_constrained_qp_solver.h
          const char* doc = R"""()""";
        } is_enabled;
      } EqualityConstrainedQPSolver;
      // Symbol: drake::solvers::EvaluatorBase
      struct /* EvaluatorBase */ {
        // Source: drake/solvers/evaluator_base.h
        const char* doc =
R"""(Provides an abstract interface to represent an expression, mapping a
fixed or dynamic number of inputs to a fixed number of outputs, that
may be evaluated on a scalar type of double or AutoDiffXd.

These objects, and its derivatives, are meant to be bound to a given
set of variables using the Binding<> class.)""";
        // Symbol: drake::solvers::EvaluatorBase::Display
        struct /* Display */ {
          // Source: drake/solvers/evaluator_base.h
          const char* doc_2args =
R"""(Formats this evaluator into the given stream using ``vars`` for the
bound decision variable names.

The size of ``vars`` must match the ``num_vars()`` declared by this
evaluator. (If ``num_vars()`` is ``Eigen∷Dynamic``, then ``vars`` may
be any size.))""";
          // Source: drake/solvers/evaluator_base.h
          const char* doc_1args =
R"""(Formats this evaluator into the given stream, without displaying the
decision variables it is bound to.)""";
        } Display;
        // Symbol: drake::solvers::EvaluatorBase::DoDisplay
        struct /* DoDisplay */ {
          // Source: drake/solvers/evaluator_base.h
          const char* doc =
R"""(NVI implementation of Display. The default implementation will report
the NiceTypeName, get_description, and list the bound variables.
Subclasses may override to customize the message.

Precondition:
    vars size is consistent with num_vars".)""";
        } DoDisplay;
        // Symbol: drake::solvers::EvaluatorBase::DoEval
        struct /* DoEval */ {
          // Source: drake/solvers/evaluator_base.h
          const char* doc_was_unable_to_choose_unambiguous_names =
R"""(Implements expression evaluation for scalar type double.

Parameter ``x``:
    Input vector.

Parameter ``y``:
    Output vector.

Precondition:
    x must be of size ``num_vars`` x 1.

Postcondition:
    y will be of size ``num_outputs`` x 1.)""";
        } DoEval;
        // Symbol: drake::solvers::EvaluatorBase::DoToLatex
        struct /* DoToLatex */ {
          // Source: drake/solvers/evaluator_base.h
          const char* doc = R"""()""";
        } DoToLatex;
        // Symbol: drake::solvers::EvaluatorBase::Eval
        struct /* Eval */ {
          // Source: drake/solvers/evaluator_base.h
          const char* doc =
R"""(Evaluates the expression.

Parameter ``x``:
    A ``num_vars`` x 1 input vector.

Parameter ``y``:
    A ``num_outputs`` x 1 output vector.)""";
        } Eval;
        // Symbol: drake::solvers::EvaluatorBase::EvaluatorBase
        struct /* ctor */ {
          // Source: drake/solvers/evaluator_base.h
          const char* doc =
R"""(Constructs a evaluator.

Parameter ``num_outputs``:
    The number of rows in the output.

Parameter ``num_vars``:
    The number of rows in the input. If the input dimension is not
    known, then set ``num_vars`` to Eigen∷Dynamic.

Parameter ``description``:
    A human-friendly description.

See also:
    Eval(...))""";
        } ctor;
        // Symbol: drake::solvers::EvaluatorBase::SetGradientSparsityPattern
        struct /* SetGradientSparsityPattern */ {
          // Source: drake/solvers/evaluator_base.h
          const char* doc =
R"""(Set the sparsity pattern of the gradient matrix ∂y/∂x (the gradient of
y value in Eval, w.r.t x in Eval) . gradient_sparsity_pattern contains
*all* the pairs of (row_index, col_index) for which the corresponding
entries could have non-zero value in the gradient matrix ∂y/∂x.)""";
        } SetGradientSparsityPattern;
        // Symbol: drake::solvers::EvaluatorBase::ToLatex
        struct /* ToLatex */ {
          // Source: drake/solvers/evaluator_base.h
          const char* doc =
R"""(Returns a LaTeX string describing this evaluator. Does not include any
characters to enter/exit math mode; you might want, e.g. "$$" +
evaluator.ToLatex() + "$$".)""";
        } ToLatex;
        // Symbol: drake::solvers::EvaluatorBase::get_description
        struct /* get_description */ {
          // Source: drake/solvers/evaluator_base.h
          const char* doc =
R"""(Getter for a human-friendly description for the evaluator.)""";
        } get_description;
        // Symbol: drake::solvers::EvaluatorBase::gradient_sparsity_pattern
        struct /* gradient_sparsity_pattern */ {
          // Source: drake/solvers/evaluator_base.h
          const char* doc =
R"""(Returns the vector of (row_index, col_index) that contains all the
entries in the gradient of Eval function (∂y/∂x) whose value could be
non-zero, namely if ∂yᵢ/∂xⱼ could be non-zero, then the pair (i, j) is
in gradient_sparsity_pattern.

Returns ``gradient_sparsity_pattern``:
    If nullopt, then we regard all entries of the gradient as
    potentially non-zero.)""";
        } gradient_sparsity_pattern;
        // Symbol: drake::solvers::EvaluatorBase::is_thread_safe
        struct /* is_thread_safe */ {
          // Source: drake/solvers/evaluator_base.h
          const char* doc =
R"""(Returns whether it is safe to call Eval in parallel.)""";
        } is_thread_safe;
        // Symbol: drake::solvers::EvaluatorBase::num_outputs
        struct /* num_outputs */ {
          // Source: drake/solvers/evaluator_base.h
          const char* doc =
R"""(Getter for the number of outputs, namely the number of rows in y, as
used in Eval(x, y).)""";
        } num_outputs;
        // Symbol: drake::solvers::EvaluatorBase::num_vars
        struct /* num_vars */ {
          // Source: drake/solvers/evaluator_base.h
          const char* doc =
R"""(Getter for the number of variables, namely the number of rows in x, as
used in Eval(x, y).)""";
        } num_vars;
        // Symbol: drake::solvers::EvaluatorBase::set_description
        struct /* set_description */ {
          // Source: drake/solvers/evaluator_base.h
          const char* doc =
R"""(Set a human-friendly description for the evaluator.)""";
        } set_description;
        // Symbol: drake::solvers::EvaluatorBase::set_is_thread_safe
        struct /* set_is_thread_safe */ {
          // Source: drake/solvers/evaluator_base.h
          const char* doc = R"""()""";
        } set_is_thread_safe;
        // Symbol: drake::solvers::EvaluatorBase::set_num_outputs
        struct /* set_num_outputs */ {
          // Source: drake/solvers/evaluator_base.h
          const char* doc = R"""()""";
        } set_num_outputs;
      } EvaluatorBase;
      // Symbol: drake::solvers::EvaluatorConstraint
      struct /* EvaluatorConstraint */ {
        // Source: drake/solvers/constraint.h
        const char* doc =
R"""(A constraint that may be specified using another (potentially
nonlinear) evaluator.

Template parameter ``EvaluatorType``:
    The nested evaluator.)""";
        // Symbol: drake::solvers::EvaluatorConstraint::EvaluatorConstraint<EvaluatorType>
        struct /* ctor */ {
          // Source: drake/solvers/constraint.h
          const char* doc =
R"""(Constructs an evaluator constraint, given the EvaluatorType instance
(which will specify the number of constraints and variables), and will
forward the remaining arguments to the Constraint constructor.

Parameter ``evaluator``:
    EvaluatorType instance.

Parameter ``args``:
    Arguments to be forwarded to the constraint constructor.)""";
        } ctor;
        // Symbol: drake::solvers::EvaluatorConstraint::evaluator
        struct /* evaluator */ {
          // Source: drake/solvers/constraint.h
          const char* doc = R"""(Reference to the nested evaluator.)""";
        } evaluator;
      } EvaluatorConstraint;
      // Symbol: drake::solvers::EvaluatorCost
      struct /* EvaluatorCost */ {
        // Source: drake/solvers/cost.h
        const char* doc =
R"""(A cost that may be specified using another (potentially nonlinear)
evaluator.

Template parameter ``EvaluatorType``:
    The nested evaluator.)""";
        // Symbol: drake::solvers::EvaluatorCost::DoEval
        struct /* DoEval */ {
          // Source: drake/solvers/cost.h
          const char* doc = R"""()""";
        } DoEval;
        // Symbol: drake::solvers::EvaluatorCost::EvaluatorCost<EvaluatorType>
        struct /* ctor */ {
          // Source: drake/solvers/cost.h
          const char* doc =
R"""(This cost computes a.dot(evaluator(x)) + b

Precondition:
    a.rows() == evaluator->num_outputs())""";
        } ctor;
        // Symbol: drake::solvers::EvaluatorCost::evaluator
        struct /* evaluator */ {
          // Source: drake/solvers/cost.h
          const char* doc = R"""()""";
        } evaluator;
      } EvaluatorCost;
      // Symbol: drake::solvers::ExponentialConeConstraint
      struct /* ExponentialConeConstraint */ {
        // Source: drake/solvers/constraint.h
        const char* doc =
R"""(An exponential cone constraint is a special type of convex cone
constraint. We constrain A * x + b to be in the exponential cone,
where A has 3 rows, and b is in ℝ³, x is the decision variable. A
vector z in ℝ³ is in the exponential cone, if {z₀, z₁, z₂ | z₀ ≥ z₁ *
exp(z₂ / z₁), z₁ > 0}. Equivalently, this constraint can be
refomulated with logarithm function {z₀, z₁, z₂ | z₂ ≤ z₁ * log(z₀ /
z₁), z₀ > 0, z₁ > 0}

The Eval function implemented in this class is z₀ - z₁ * exp(z₂ / z₁)
>= 0, z₁ > 0 where z = A * x + b. It is not recommended to solve an
exponential cone constraint through generic nonlinear optimization. It
is possible that the nonlinear solver can accidentally set z₁ = 0,
where the constraint is not well defined. Instead, the user should
consider to solve the program through conic solvers that can exploit
exponential cone, such as MOSEK™ and SCS.)""";
        // Symbol: drake::solvers::ExponentialConeConstraint::A
        struct /* A */ {
          // Source: drake/solvers/constraint.h
          const char* doc = R"""(Getter for matrix A.)""";
        } A;
        // Symbol: drake::solvers::ExponentialConeConstraint::DoEval
        struct /* DoEval */ {
          // Source: drake/solvers/constraint.h
          const char* doc = R"""()""";
        } DoEval;
        // Symbol: drake::solvers::ExponentialConeConstraint::DoEvalGeneric
        struct /* DoEvalGeneric */ {
          // Source: drake/solvers/constraint.h
          const char* doc = R"""()""";
        } DoEvalGeneric;
        // Symbol: drake::solvers::ExponentialConeConstraint::DoToLatex
        struct /* DoToLatex */ {
          // Source: drake/solvers/constraint.h
          const char* doc = R"""()""";
        } DoToLatex;
        // Symbol: drake::solvers::ExponentialConeConstraint::ExponentialConeConstraint
        struct /* ctor */ {
          // Source: drake/solvers/constraint.h
          const char* doc =
R"""(Constructor for exponential cone. Constrains A * x + b to be in the
exponential cone.

Precondition:
    A has 3 rows.)""";
        } ctor;
        // Symbol: drake::solvers::ExponentialConeConstraint::b
        struct /* b */ {
          // Source: drake/solvers/constraint.h
          const char* doc = R"""(Getter for vector b.)""";
        } b;
      } ExponentialConeConstraint;
      // Symbol: drake::solvers::ExponentiallySmoothedHingeLoss
      struct /* ExponentiallySmoothedHingeLoss */ {
        // Source: drake/solvers/minimum_value_constraint.h
        const char* doc =
R"""(A hinge loss function smoothed by exponential function. This loss
function is differentiable everywhere. The formulation is described in
section II.C of [2]. The penalty is <pre class="unicode-art"> ⎧ 0 if x
≥ 0 φ(x) = ⎨ ⎩ -x exp(1/x) if x < 0. </pre> [2] "Whole-body Motion
Planning with Centroidal Dynamics and Full Kinematics" by Hongkai Dai,
Andres Valenzuela and Russ Tedrake, IEEE-RAS International Conference
on Humanoid Robots, 2014.)""";
      } ExponentiallySmoothedHingeLoss;
      // Symbol: drake::solvers::ExpressionConstraint
      struct /* ExpressionConstraint */ {
        // Source: drake/solvers/constraint.h
        const char* doc =
R"""(Impose a generic (potentially nonlinear) constraint represented as a
vector of symbolic Expression. Expression∷Evaluate is called on every
constraint evaluation.

Uses symbolic∷Jacobian to provide the gradients to the AutoDiff
method.)""";
        // Symbol: drake::solvers::ExpressionConstraint::DoDisplay
        struct /* DoDisplay */ {
          // Source: drake/solvers/constraint.h
          const char* doc = R"""()""";
        } DoDisplay;
        // Symbol: drake::solvers::ExpressionConstraint::DoEval
        struct /* DoEval */ {
          // Source: drake/solvers/constraint.h
          const char* doc = R"""()""";
        } DoEval;
        // Symbol: drake::solvers::ExpressionConstraint::DoToLatex
        struct /* DoToLatex */ {
          // Source: drake/solvers/constraint.h
          const char* doc = R"""()""";
        } DoToLatex;
        // Symbol: drake::solvers::ExpressionConstraint::ExpressionConstraint
        struct /* ctor */ {
          // Source: drake/solvers/constraint.h
          const char* doc = R"""()""";
        } ctor;
        // Symbol: drake::solvers::ExpressionConstraint::expressions
        struct /* expressions */ {
          // Source: drake/solvers/constraint.h
          const char* doc =
R"""(Returns:
    the symbolic expressions.)""";
        } expressions;
        // Symbol: drake::solvers::ExpressionConstraint::vars
        struct /* vars */ {
          // Source: drake/solvers/constraint.h
          const char* doc =
R"""(Returns:
    the list of the variables involved in the vector of expressions,
    in the order that they are expected to be received during DoEval.
    Any Binding that connects this constraint to decision variables
    should pass this list of variables to the Binding.)""";
        } vars;
      } ExpressionConstraint;
      // Symbol: drake::solvers::ExpressionCost
      struct /* ExpressionCost */ {
        // Source: drake/solvers/cost.h
        const char* doc =
R"""(Impose a generic (potentially nonlinear) cost represented as a
symbolic Expression. Expression∷Evaluate is called on every constraint
evaluation.

Uses symbolic∷Jacobian to provide the gradients to the AutoDiff
method.)""";
        // Symbol: drake::solvers::ExpressionCost::DoDisplay
        struct /* DoDisplay */ {
          // Source: drake/solvers/cost.h
          const char* doc = R"""()""";
        } DoDisplay;
        // Symbol: drake::solvers::ExpressionCost::DoEval
        struct /* DoEval */ {
          // Source: drake/solvers/cost.h
          const char* doc = R"""()""";
        } DoEval;
        // Symbol: drake::solvers::ExpressionCost::DoToLatex
        struct /* DoToLatex */ {
          // Source: drake/solvers/cost.h
          const char* doc = R"""()""";
        } DoToLatex;
        // Symbol: drake::solvers::ExpressionCost::ExpressionCost
        struct /* ctor */ {
          // Source: drake/solvers/cost.h
          const char* doc = R"""()""";
        } ctor;
        // Symbol: drake::solvers::ExpressionCost::expression
        struct /* expression */ {
          // Source: drake/solvers/cost.h
          const char* doc =
R"""(Returns:
    the symbolic expression.)""";
        } expression;
        // Symbol: drake::solvers::ExpressionCost::vars
        struct /* vars */ {
          // Source: drake/solvers/cost.h
          const char* doc =
R"""(Returns:
    the list of the variables involved in the vector of expressions,
    in the order that they are expected to be received during DoEval.
    Any Binding that connects this constraint to decision variables
    should pass this list of variables to the Binding.)""";
        } vars;
      } ExpressionCost;
      // Symbol: drake::solvers::FunctionEvaluator
      struct /* FunctionEvaluator */ {
        // Source: drake/solvers/evaluator_base.h
        const char* doc =
R"""(An evaluator that may be specified using a callable object. Consider
constructing these instances using MakeFunctionEvaluator(...).

Template parameter ``F``:
    The function / functor's type.)""";
        // Symbol: drake::solvers::FunctionEvaluator::FunctionEvaluator<F>
        struct /* ctor */ {
          // Source: drake/solvers/evaluator_base.h
          const char* doc =
R"""(Constructs an instance by copying from an lvalue or rvalue of ``F``.

Template parameter ``FF``:
    Perfect-forwarding type of ``F`` (e.g., ``const F&``, `F&&`).

Parameter ``f``:
    The callable object. If rvalue, this value will be std∷move'd.
    Otherwise, it will be copied.

Parameter ``args``:
    Arguments to be forwarded to EvaluatorBase constructor.)""";
        } ctor;
      } FunctionEvaluator;
      // Symbol: drake::solvers::GenerateSDPA
      struct /* GenerateSDPA */ {
        // Source: drake/solvers/sdpa_free_format.h
        const char* doc =
R"""(SDPA is a format to record an SDP problem

max tr(C*X) s.t tr(Aᵢ*X) = gᵢ X ≽ 0

or the dual of the problem

min gᵀy s.t ∑ᵢ yᵢAᵢ - C ≽ 0

where X is a symmetric block diagonal matrix. The format is described
in http://plato.asu.edu/ftp/sdpa_format.txt. Many solvers, such as
CSDP, DSDP, SDPA, sedumi and SDPT3, accept an SDPA format file as the
input. This function reads a MathematicalProgram that can be
formulated as above, and write an SDPA file.

Parameter ``prog``:
    a program that contains an optimization program.

Parameter ``file_name``:
    The name of the file, note that the extension will be added
    automatically.

Parameter ``method``:
    If ``prog`` contains free variables (i.e., variables without
    bounds), then we need to remove these free variables to write the
    program in the SDPA format. Please refer to
    RemoveFreeVariableMethod for details on how to remove the free
    variables. $*Default:* is RemoveFreeVariableMethod∷kNullspace.

Returns ``is_success``:
    . Returns true if we can generate the SDPA file. The failure could
    be 1. ``prog`` cannot be captured by the formulation above. 2.
    ``prog`` cannot create a file with the given name, etc.)""";
      } GenerateSDPA;
      // Symbol: drake::solvers::GetAvailableSolvers
      struct /* GetAvailableSolvers */ {
        // Source: drake/solvers/choose_best_solver.h
        const char* doc =
R"""(Returns the list of available and enabled solvers that definitely
accept all programs of the given program type. The order of the
returned SolverIds reflects an approximate order of preference, from
most preferred (front) to least preferred (back). Because we are
analyzing only based on the program type rather than a specific
program, it's possible that solvers later in the list would perform
better in certain situations. To obtain the truly best solver, using
ChooseBestSolver() instead.

Note:
    If a solver only accepts a subset of the program type, then that
    solver is not included in the returned results. For example
    EqualityConstrainedQPSolver doesn't accept programs with
    inequality linear constraints, so it doesn't show up in the return
    of GetAvailableSolvers(ProgramType∷kQP).)""";
      } GetAvailableSolvers;
      // Symbol: drake::solvers::GetKnownSolvers
      struct /* GetKnownSolvers */ {
        // Source: drake/solvers/choose_best_solver.h
        const char* doc =
R"""(Returns the set of solvers known to ChooseBestSolver.)""";
      } GetKnownSolvers;
      // Symbol: drake::solvers::GetProgramType
      struct /* GetProgramType */ {
        // Source: drake/solvers/get_program_type.h
        const char* doc =
R"""(Returns the type of the optimization program (LP, QP, etc), based on
the properties of its cost/constraints/variables. Each mathematical
program should be characterized by a unique type. If a program can be
characterized as either type A or type B (for example, a program with
linear constraint and linear costs can be characterized as either an
LP or an SDP), then we choose the type corresponding to a smaller set
of programs (LP in this case).)""";
      } GetProgramType;
      // Symbol: drake::solvers::GetVariableValue
      struct /* GetVariableValue */ {
        // Source: drake/solvers/mathematical_program_result.h
        const char* doc_3args_var_variable_index_variable_values =
R"""(Retrieve the value of a single variable ``var`` from
``variable_values``.

Parameter ``var``:
    The variable whose value is going to be retrieved.
    ``var``.get_id() must be a key in ``variable_index``.

Parameter ``variable_index``:
    maps the variable ID to its index in ``variable_values``.

Parameter ``variable_values``:
    The values of all variables.

Returns:
    variable_values(variable_index[var.get_id()]) if var.get_id() is a
    valid key of ``variable_index``.

Raises:
    RuntimeError if var.get_id() is not a valid key of
    ``variable_index``.

Precondition:
    All the mapped value in variable_index is in the range [0,
    variable_values.rows()))""";
        // Source: drake/solvers/mathematical_program_result.h
        const char* doc_3args_constEigenMatrixBase_conststdoptional_constEigenRef =
R"""(Overload GetVariableValue() function, but for an Eigen matrix of
decision variables.)""";
      } GetVariableValue;
      // Symbol: drake::solvers::GurobiSolver
      struct /* GurobiSolver */ {
        // Source: drake/solvers/gurobi_solver.h
        const char* doc =
R"""(An implementation of SolverInterface for the commercially-licensed
Gurobi solver (https://www.gurobi.com/).

The default build of Drake is not configured to use Gurobi, so
therefore SolverInterface∷available() will return false. You must
compile Drake from source in order to link against Gurobi. For
details, refer to the documentation at
https://drake.mit.edu/bazel.html#proprietary-solvers.

The GRB_LICENSE_FILE environment variable controls whether or not
SolverInterface∷enabled() returns true. If it is set to any non-empty
value, then the solver is enabled; otherwise, the solver is not
enabled.

Gurobi solver supports options/parameters listed in
https://docs.gurobi.com/projects/optimizer/en/12.0/concepts/parameters.html.
On top of these options, we provide the following additional options
1. "GRBwrite", set to a file name so that Gurobi solver will write the
optimization model to this file, check
https://www.docs.gurobi.com/projects/optimizer/en/12.0/reference/python/model.html#Write
for more details, such as all supported file extensions. Set this
option to "" if you don't want to write to file. Default is not to
write to a file. 2. "GRBcomputeIIS", set to 1 to compute an
Irreducible Inconsistent Subsystem (IIS) when the problem is
infeasible. Refer to
https://docs.gurobi.com/projects/optimizer/en/12.0/reference/python/model.html#Model.computeIIS
for more details. Often this method is called together with setting
GRBwrite to "FILENAME.ilp" to write IIS to a file with extension
"ilp". Default is not to compute IIS.

GurobiSolver supports parallelization during Solve(). If both the
"Threads" integer solver option and CommonSolverOption∷kMaxThreads
have been set by the user, then the value in "Threads" will be used as
the number of threads.

If neither the "Threads" integer solver option nor
CommonSolverOption∷kMaxThreads has been set by the user, then
GurobiSolver uses the environment variable GUROBI_NUM_THREADS (if set)
as a default value for "Threads".

If none of "Threads", CommonSolverOption∷kMaxThreads, or
GUROBI_NUM_THREADS are set, then Drake's default maximum parallelism
will be used.)""";
        // Symbol: drake::solvers::GurobiSolver::AcquireLicense
        struct /* AcquireLicense */ {
          // Source: drake/solvers/gurobi_solver.h
          const char* doc =
R"""(This acquires a Gurobi license environment shared among all
GurobiSolver instances. The environment will stay valid as long as at
least one shared_ptr returned by this function is alive. GurobiSolver
calls this method on each Solve().

If the license file contains the string ``HOSTID``, then we treat this
as confirmation that the license is attached to the local host, and
maintain an internal copy of the shared_ptr for the lifetime of the
process. Otherwise the default behavior is to only hold the license
while at least one GurobiSolver instance is alive.

Call this method directly and maintain the shared_ptr ONLY if you must
use different MathematicalProgram instances at different instances in
time, and repeatedly acquiring the license is costly (e.g., requires
contacting a license server).

Returns:
    A shared pointer to a license environment that will stay valid as
    long as any shared_ptr returned by this function is alive. If
    Gurobi is not available in your build, this will return a null
    (empty) shared_ptr.

Raises:
    RuntimeError if Gurobi is available but a license cannot be
    obtained.)""";
        } AcquireLicense;
        // Symbol: drake::solvers::GurobiSolver::AddMipNodeCallback
        struct /* AddMipNodeCallback */ {
          // Source: drake/solvers/gurobi_solver.h
          const char* doc =
R"""(Registers a callback to be called at intermediate solutions during the
solve.

Parameter ``callback``:
    User callback function.)""";
        } AddMipNodeCallback;
        // Symbol: drake::solvers::GurobiSolver::AddMipSolCallback
        struct /* AddMipSolCallback */ {
          // Source: drake/solvers/gurobi_solver.h
          const char* doc =
R"""(Registers a callback to be called at feasible solutions during the
solve.

Parameter ``callback``:
    User callback function.)""";
        } AddMipSolCallback;
        // Symbol: drake::solvers::GurobiSolver::Details
        struct /* Details */ {
          // Source: drake/solvers/gurobi_solver.h
          const char* doc =
R"""(Type of details stored in MathematicalProgramResult.)""";
        } Details;
        // Symbol: drake::solvers::GurobiSolver::GurobiSolver
        struct /* ctor */ {
          // Source: drake/solvers/gurobi_solver.h
          const char* doc = R"""()""";
        } ctor;
        // Symbol: drake::solvers::GurobiSolver::MipNodeCallbackFunction
        struct /* MipNodeCallbackFunction */ {
          // Source: drake/solvers/gurobi_solver.h
          const char* doc =
R"""(Users can supply a callback to be called when the Gurobi solver finds
an intermediate solution node, which may not be feasible. See Gurobi
reference manual for more detail on callbacks:
https://docs.gurobi.com/projects/optimizer/en/12.0/reference/numericcodes/callbacks.html
The user may supply a partial solution in the VectorXd and
VectorXDecisionVariable arguments that will be passed to Gurobi as a
candidate feasible solution. See gurobi_solver_test.cc for an example
of using std∷bind to create a callback of this signature, while
allowing additional data to be passed through.

Parameter ``MathematicalProgram``:
    & The optimization wrapper, whose current variable values
    (accessible via MathematicalProgram∷GetSolution) will be set to
    the intermediate solution values.

Parameter ``SolveStatusInfo``:
    & Intermediate solution status information values queried from
    Gurobi.

Parameter ``VectorXd*``:
    User may assign this to be the values of the variable assignments.

Parameter ``VectorXDecisionVariable*``:
    User may assign this to be the decision variables being assigned.
    Must have the same number of elements as the VectorXd assignment.)""";
        } MipNodeCallbackFunction;
        // Symbol: drake::solvers::GurobiSolver::MipSolCallbackFunction
        struct /* MipSolCallbackFunction */ {
          // Source: drake/solvers/gurobi_solver.h
          const char* doc =
R"""(Users can supply a callback to be called when the Gurobi solver finds
a feasible solution. See Gurobi reference manual for more detail on
callbacks:
https://docs.gurobi.com/projects/optimizer/en/12.0/reference/numericcodes/callbacks.html
See gurobi_solver_test.cc for an example of using std∷bind to create a
callback of this signature, while allowing additional data to be
passed through.

Parameter ``MathematicalProgram``:
    & The optimization wrapper, whose current variable values
    (accessible via MathematicalProgram∷GetSolution) will be set to
    the intermediate solution values.

Parameter ``SolveStatusInfo``:
    & Intermediate solution status information values queried from
    Gurobi.

Parameter ``void*``:
    Arbitrary data supplied during callback registration.)""";
        } MipSolCallbackFunction;
        // Symbol: drake::solvers::GurobiSolver::ProgramAttributesSatisfied
        struct /* ProgramAttributesSatisfied */ {
          // Source: drake/solvers/gurobi_solver.h
          const char* doc = R"""()""";
        } ProgramAttributesSatisfied;
        // Symbol: drake::solvers::GurobiSolver::SolveStatusInfo
        struct /* SolveStatusInfo */ {
          // Source: drake/solvers/gurobi_solver.h
          const char* doc =
R"""(Contains info returned to a user function that handles a Node or
Solution callback.

See also:
    MipNodeCallbackFunction

See also:
    MipSolCallbackFunction)""";
          // Symbol: drake::solvers::GurobiSolver::SolveStatusInfo::best_bound
          struct /* best_bound */ {
            // Source: drake/solvers/gurobi_solver.h
            const char* doc = R"""(Best known objective lower bound.)""";
          } best_bound;
          // Symbol: drake::solvers::GurobiSolver::SolveStatusInfo::best_objective
          struct /* best_objective */ {
            // Source: drake/solvers/gurobi_solver.h
            const char* doc = R"""(Objective of best solution yet.)""";
          } best_objective;
          // Symbol: drake::solvers::GurobiSolver::SolveStatusInfo::current_objective
          struct /* current_objective */ {
            // Source: drake/solvers/gurobi_solver.h
            const char* doc = R"""(Objective of current solution.)""";
          } current_objective;
          // Symbol: drake::solvers::GurobiSolver::SolveStatusInfo::explored_node_count
          struct /* explored_node_count */ {
            // Source: drake/solvers/gurobi_solver.h
            const char* doc = R"""(Number of nodes explored so far.)""";
          } explored_node_count;
          // Symbol: drake::solvers::GurobiSolver::SolveStatusInfo::feasible_solutions_count
          struct /* feasible_solutions_count */ {
            // Source: drake/solvers/gurobi_solver.h
            const char* doc = R"""(Number of feasible sols found so far.)""";
          } feasible_solutions_count;
          // Symbol: drake::solvers::GurobiSolver::SolveStatusInfo::reported_runtime
          struct /* reported_runtime */ {
            // Source: drake/solvers/gurobi_solver.h
            const char* doc = R"""(Runtime as of this callback.)""";
          } reported_runtime;
        } SolveStatusInfo;
        // Symbol: drake::solvers::GurobiSolver::UnsatisfiedProgramAttributes
        struct /* UnsatisfiedProgramAttributes */ {
          // Source: drake/solvers/gurobi_solver.h
          const char* doc = R"""()""";
        } UnsatisfiedProgramAttributes;
        // Symbol: drake::solvers::GurobiSolver::id
        struct /* id */ {
          // Source: drake/solvers/gurobi_solver.h
          const char* doc = R"""()""";
        } id;
        // Symbol: drake::solvers::GurobiSolver::is_available
        struct /* is_available */ {
          // Source: drake/solvers/gurobi_solver.h
          const char* doc = R"""()""";
        } is_available;
        // Symbol: drake::solvers::GurobiSolver::is_enabled
        struct /* is_enabled */ {
          // Source: drake/solvers/gurobi_solver.h
          const char* doc =
R"""(Returns true iff the environment variable GRB_LICENSE_FILE has been
set to a non-empty value.)""";
        } is_enabled;
      } GurobiSolver;
      // Symbol: drake::solvers::GurobiSolverDetails
      struct /* GurobiSolverDetails */ {
        // Source: drake/solvers/gurobi_solver.h
        const char* doc =
R"""(The Gurobi solver details after calling Solve() function. The user can
call MathematicalProgramResult∷get_solver_details<GurobiSolver>() to
obtain the details.)""";
        // Symbol: drake::solvers::GurobiSolverDetails::error_code
        struct /* error_code */ {
          // Source: drake/solvers/gurobi_solver.h
          const char* doc =
R"""(The error message returned from Gurobi call. Please refer to
https://docs.gurobi.com/projects/optimizer/en/12.0/reference/numericcodes/errors.html)""";
        } error_code;
        // Symbol: drake::solvers::GurobiSolverDetails::objective_bound
        struct /* objective_bound */ {
          // Source: drake/solvers/gurobi_solver.h
          const char* doc =
R"""(The best known bound on the optimal objective. This is used in mixed
integer optimization. Please refer to
https://docs.gurobi.com/projects/optimizer/en/12.0/reference/attributes/model.html#objbound)""";
        } objective_bound;
        // Symbol: drake::solvers::GurobiSolverDetails::optimization_status
        struct /* optimization_status */ {
          // Source: drake/solvers/gurobi_solver.h
          const char* doc =
R"""(The status code when the optimize call has returned. Please refer to
https://docs.gurobi.com/projects/optimizer/en/12.0/reference/numericcodes/statuscodes.html)""";
        } optimization_status;
        // Symbol: drake::solvers::GurobiSolverDetails::optimizer_time
        struct /* optimizer_time */ {
          // Source: drake/solvers/gurobi_solver.h
          const char* doc =
R"""(The gurobi optimization time. Please refer to
https://docs.gurobi.com/projects/optimizer/en/12.0/reference/attributes/model.html#attrruntime)""";
        } optimizer_time;
      } GurobiSolverDetails;
      // Symbol: drake::solvers::IndeterminatesRefList
      struct /* IndeterminatesRefList */ {
        // Source: drake/solvers/indeterminate.h
        const char* doc = R"""()""";
      } IndeterminatesRefList;
      // Symbol: drake::solvers::IntervalBinning
      struct /* IntervalBinning */ {
        // Source: drake/solvers/mixed_integer_optimization_util.h
        const char* doc =
R"""(For a continuous variable whose range is cut into small intervals, we
will use binary variables to represent which interval the continuous
variable is in. We support two representations, either using
logarithmic number of binary variables, or linear number of binary
variables. For more details,

See also:
    AddLogarithmicSos2Constraint and AddSos2Constraint)""";
        // Symbol: drake::solvers::IntervalBinning::kLinear
        struct /* kLinear */ {
          // Source: drake/solvers/mixed_integer_optimization_util.h
          const char* doc = R"""()""";
        } kLinear;
        // Symbol: drake::solvers::IntervalBinning::kLogarithmic
        struct /* kLogarithmic */ {
          // Source: drake/solvers/mixed_integer_optimization_util.h
          const char* doc = R"""()""";
        } kLogarithmic;
      } IntervalBinning;
      // Symbol: drake::solvers::IpoptSolver
      struct /* IpoptSolver */ {
        // Source: drake/solvers/ipopt_solver.h
        const char* doc =
R"""(A wrapper to call <a href="https://coin-or.github.io/Ipopt/">Ipopt</a>
using Drake's MathematicalProgram.)""";
        // Symbol: drake::solvers::IpoptSolver::Details
        struct /* Details */ {
          // Source: drake/solvers/ipopt_solver.h
          const char* doc =
R"""(Type of details stored in MathematicalProgramResult.)""";
        } Details;
        // Symbol: drake::solvers::IpoptSolver::IpoptSolver
        struct /* ctor */ {
          // Source: drake/solvers/ipopt_solver.h
          const char* doc = R"""()""";
        } ctor;
        // Symbol: drake::solvers::IpoptSolver::ProgramAttributesSatisfied
        struct /* ProgramAttributesSatisfied */ {
          // Source: drake/solvers/ipopt_solver.h
          const char* doc = R"""()""";
        } ProgramAttributesSatisfied;
        // Symbol: drake::solvers::IpoptSolver::id
        struct /* id */ {
          // Source: drake/solvers/ipopt_solver.h
          const char* doc = R"""()""";
        } id;
        // Symbol: drake::solvers::IpoptSolver::is_available
        struct /* is_available */ {
          // Source: drake/solvers/ipopt_solver.h
          const char* doc = R"""()""";
        } is_available;
        // Symbol: drake::solvers::IpoptSolver::is_enabled
        struct /* is_enabled */ {
          // Source: drake/solvers/ipopt_solver.h
          const char* doc = R"""()""";
        } is_enabled;
      } IpoptSolver;
      // Symbol: drake::solvers::IpoptSolverDetails
      struct /* IpoptSolverDetails */ {
        // Source: drake/solvers/ipopt_solver.h
        const char* doc =
R"""(The Ipopt solver details after calling Solve() function. The user can
call MathematicalProgramResult∷get_solver_details<IpoptSolver>() to
obtain the details.)""";
        // Symbol: drake::solvers::IpoptSolverDetails::ConvertStatusToString
        struct /* ConvertStatusToString */ {
          // Source: drake/solvers/ipopt_solver.h
          const char* doc =
R"""(Convert status field to string. This function is useful if you want to
interpret the meaning of status.)""";
        } ConvertStatusToString;
        // Symbol: drake::solvers::IpoptSolverDetails::g
        struct /* g */ {
          // Source: drake/solvers/ipopt_solver.h
          const char* doc =
R"""(The final value for the constraint function.)""";
        } g;
        // Symbol: drake::solvers::IpoptSolverDetails::lambda
        struct /* lambda */ {
          // Source: drake/solvers/ipopt_solver.h
          const char* doc =
R"""(The final value for the constraint multiplier.)""";
        } lambda;
        // Symbol: drake::solvers::IpoptSolverDetails::status
        struct /* status */ {
          // Source: drake/solvers/ipopt_solver.h
          const char* doc =
R"""(The final status of the solver. Please refer to section 6 in
Introduction to Ipopt: A tutorial for downloading, installing, and
using Ipopt. You could also find the meaning of the status as
Ipopt∷SolverReturn defined in IpAlgTypes.hpp)""";
        } status;
        // Symbol: drake::solvers::IpoptSolverDetails::z_L
        struct /* z_L */ {
          // Source: drake/solvers/ipopt_solver.h
          const char* doc =
R"""(The final value for the lower bound multiplier.)""";
        } z_L;
        // Symbol: drake::solvers::IpoptSolverDetails::z_U
        struct /* z_U */ {
          // Source: drake/solvers/ipopt_solver.h
          const char* doc =
R"""(The final value for the upper bound multiplier.)""";
        } z_U;
      } IpoptSolverDetails;
      // Symbol: drake::solvers::L1NormCost
      struct /* L1NormCost */ {
        // Source: drake/solvers/cost.h
        const char* doc =
R"""(Implements a cost of the form ‖Ax + b‖₁. Note that this cost is
non-differentiable when any element of Ax + b equals zero.)""";
        // Symbol: drake::solvers::L1NormCost::A
        struct /* A */ {
          // Source: drake/solvers/cost.h
          const char* doc = R"""()""";
        } A;
        // Symbol: drake::solvers::L1NormCost::DoDisplay
        struct /* DoDisplay */ {
          // Source: drake/solvers/cost.h
          const char* doc = R"""()""";
        } DoDisplay;
        // Symbol: drake::solvers::L1NormCost::DoEval
        struct /* DoEval */ {
          // Source: drake/solvers/cost.h
          const char* doc = R"""()""";
        } DoEval;
        // Symbol: drake::solvers::L1NormCost::DoToLatex
        struct /* DoToLatex */ {
          // Source: drake/solvers/cost.h
          const char* doc = R"""()""";
        } DoToLatex;
        // Symbol: drake::solvers::L1NormCost::L1NormCost
        struct /* ctor */ {
          // Source: drake/solvers/cost.h
          const char* doc =
R"""(Construct a cost of the form ‖Ax + b‖₁.

Parameter ``A``:
    Linear term.

Parameter ``b``:
    Constant term.

Raises:
    RuntimeError if the size of A and b don't match.)""";
        } ctor;
        // Symbol: drake::solvers::L1NormCost::UpdateCoefficients
        struct /* UpdateCoefficients */ {
          // Source: drake/solvers/cost.h
          const char* doc =
R"""(Updates the coefficients of the cost. Note that the number of
variables (columns of A) cannot change.

Parameter ``new_A``:
    New linear term.

Parameter ``new_b``:
    New constant term.)""";
        } UpdateCoefficients;
        // Symbol: drake::solvers::L1NormCost::b
        struct /* b */ {
          // Source: drake/solvers/cost.h
          const char* doc = R"""()""";
        } b;
        // Symbol: drake::solvers::L1NormCost::update_A_entry
        struct /* update_A_entry */ {
          // Source: drake/solvers/cost.h
          const char* doc =
R"""(Updates A(i, j) = val.

Raises:
    if i or j are invalid indices.)""";
        } update_A_entry;
        // Symbol: drake::solvers::L1NormCost::update_b_entry
        struct /* update_b_entry */ {
          // Source: drake/solvers/cost.h
          const char* doc =
R"""(Updates b(i) = val.

Raises:
    if i is an invalid index.)""";
        } update_b_entry;
      } L1NormCost;
      // Symbol: drake::solvers::L2NormCost
      struct /* L2NormCost */ {
        // Source: drake/solvers/cost.h
        const char* doc =
R"""(Implements a cost of the form ‖Ax + b‖₂.)""";
        // Symbol: drake::solvers::L2NormCost::DoDisplay
        struct /* DoDisplay */ {
          // Source: drake/solvers/cost.h
          const char* doc = R"""()""";
        } DoDisplay;
        // Symbol: drake::solvers::L2NormCost::DoEval
        struct /* DoEval */ {
          // Source: drake/solvers/cost.h
          const char* doc = R"""()""";
        } DoEval;
        // Symbol: drake::solvers::L2NormCost::DoToLatex
        struct /* DoToLatex */ {
          // Source: drake/solvers/cost.h
          const char* doc = R"""()""";
        } DoToLatex;
        // Symbol: drake::solvers::L2NormCost::GetDenseA
        struct /* GetDenseA */ {
          // Source: drake/solvers/cost.h
          const char* doc = R"""()""";
        } GetDenseA;
        // Symbol: drake::solvers::L2NormCost::L2NormCost
        struct /* ctor */ {
          // Source: drake/solvers/cost.h
          const char* doc_dense_A =
R"""(Construct a cost of the form ‖Ax + b‖₂.

Parameter ``A``:
    Linear term.

Parameter ``b``:
    Constant term.

Raises:
    RuntimeError if the size of A and b don't match.)""";
          // Source: drake/solvers/cost.h
          const char* doc_sparse_A =
R"""(Overloads constructor with a sparse A matrix.)""";
        } ctor;
        // Symbol: drake::solvers::L2NormCost::UpdateCoefficients
        struct /* UpdateCoefficients */ {
          // Source: drake/solvers/cost.h
          const char* doc_dense_A =
R"""(Updates the coefficients of the cost. Note that the number of
variables (columns of A) cannot change.

Parameter ``new_A``:
    New linear term.

Parameter ``new_b``:
    New constant term.)""";
          // Source: drake/solvers/cost.h
          const char* doc_sparse_A =
R"""(Overloads UpdateCoefficients but with a sparse A matrix.)""";
        } UpdateCoefficients;
        // Symbol: drake::solvers::L2NormCost::b
        struct /* b */ {
          // Source: drake/solvers/cost.h
          const char* doc = R"""()""";
        } b;
        // Symbol: drake::solvers::L2NormCost::get_sparse_A
        struct /* get_sparse_A */ {
          // Source: drake/solvers/cost.h
          const char* doc = R"""()""";
        } get_sparse_A;
      } L2NormCost;
      // Symbol: drake::solvers::LInfNormCost
      struct /* LInfNormCost */ {
        // Source: drake/solvers/cost.h
        const char* doc =
R"""(Implements a cost of the form ‖Ax + b‖∞. Note that this cost is
non-differentiable when any two or more elements of Ax + b are equal.)""";
        // Symbol: drake::solvers::LInfNormCost::A
        struct /* A */ {
          // Source: drake/solvers/cost.h
          const char* doc = R"""()""";
        } A;
        // Symbol: drake::solvers::LInfNormCost::DoDisplay
        struct /* DoDisplay */ {
          // Source: drake/solvers/cost.h
          const char* doc = R"""()""";
        } DoDisplay;
        // Symbol: drake::solvers::LInfNormCost::DoEval
        struct /* DoEval */ {
          // Source: drake/solvers/cost.h
          const char* doc = R"""()""";
        } DoEval;
        // Symbol: drake::solvers::LInfNormCost::DoToLatex
        struct /* DoToLatex */ {
          // Source: drake/solvers/cost.h
          const char* doc = R"""()""";
        } DoToLatex;
        // Symbol: drake::solvers::LInfNormCost::LInfNormCost
        struct /* ctor */ {
          // Source: drake/solvers/cost.h
          const char* doc =
R"""(Construct a cost of the form ‖Ax + b‖∞.

Parameter ``A``:
    Linear term.

Parameter ``b``:
    Constant term.

Raises:
    RuntimeError if the size of A and b don't match.)""";
        } ctor;
        // Symbol: drake::solvers::LInfNormCost::UpdateCoefficients
        struct /* UpdateCoefficients */ {
          // Source: drake/solvers/cost.h
          const char* doc =
R"""(Updates the coefficients of the cost. Note that the number of
variables (columns of A) cannot change.

Parameter ``new_A``:
    New linear term.

Parameter ``new_b``:
    New constant term.)""";
        } UpdateCoefficients;
        // Symbol: drake::solvers::LInfNormCost::b
        struct /* b */ {
          // Source: drake/solvers/cost.h
          const char* doc = R"""()""";
        } b;
        // Symbol: drake::solvers::LInfNormCost::update_A_entry
        struct /* update_A_entry */ {
          // Source: drake/solvers/cost.h
          const char* doc =
R"""(Updates A(i, j) = val.

Raises:
    if i or j are invalid indices.)""";
        } update_A_entry;
        // Symbol: drake::solvers::LInfNormCost::update_b_entry
        struct /* update_b_entry */ {
          // Source: drake/solvers/cost.h
          const char* doc =
R"""(Updates b(i) = val.

Raises:
    if i is an invalid index.)""";
        } update_b_entry;
      } LInfNormCost;
      // Symbol: drake::solvers::LinearComplementarityConstraint
      struct /* LinearComplementarityConstraint */ {
        // Source: drake/solvers/constraint.h
        const char* doc =
R"""(Implements a constraint of the form:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    Mx + q ≥ 0
      x ≥ 0
      x'(Mx + q) == 0

.. raw:: html

    </details>

Often this is summarized with the short-hand:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    0 ≤ z ⊥ Mz+q ≥ 0

.. raw:: html

    </details>

An implied slack variable complements any 0 component of x. To get the
slack values at a given solution x, use Eval(x).)""";
        // Symbol: drake::solvers::LinearComplementarityConstraint::DoCheckSatisfied
        struct /* DoCheckSatisfied */ {
          // Source: drake/solvers/constraint.h
          const char* doc = R"""()""";
        } DoCheckSatisfied;
        // Symbol: drake::solvers::LinearComplementarityConstraint::DoEval
        struct /* DoEval */ {
          // Source: drake/solvers/constraint.h
          const char* doc = R"""()""";
        } DoEval;
        // Symbol: drake::solvers::LinearComplementarityConstraint::DoToLatex
        struct /* DoToLatex */ {
          // Source: drake/solvers/constraint.h
          const char* doc = R"""()""";
        } DoToLatex;
        // Symbol: drake::solvers::LinearComplementarityConstraint::LinearComplementarityConstraint
        struct /* ctor */ {
          // Source: drake/solvers/constraint.h
          const char* doc = R"""()""";
        } ctor;
        // Symbol: drake::solvers::LinearComplementarityConstraint::M
        struct /* M */ {
          // Source: drake/solvers/constraint.h
          const char* doc = R"""()""";
        } M;
        // Symbol: drake::solvers::LinearComplementarityConstraint::q
        struct /* q */ {
          // Source: drake/solvers/constraint.h
          const char* doc = R"""()""";
        } q;
      } LinearComplementarityConstraint;
      // Symbol: drake::solvers::LinearConstraint
      struct /* LinearConstraint */ {
        // Source: drake/solvers/constraint.h
        const char* doc =
R"""(Implements a constraint of the form :math:`lb <= Ax <= ub`)""";
        // Symbol: drake::solvers::LinearConstraint::A_
        struct /* A_ */ {
          // Source: drake/solvers/constraint.h
          const char* doc = R"""()""";
        } A_;
        // Symbol: drake::solvers::LinearConstraint::DoDisplay
        struct /* DoDisplay */ {
          // Source: drake/solvers/constraint.h
          const char* doc = R"""()""";
        } DoDisplay;
        // Symbol: drake::solvers::LinearConstraint::DoEval
        struct /* DoEval */ {
          // Source: drake/solvers/constraint.h
          const char* doc = R"""()""";
        } DoEval;
        // Symbol: drake::solvers::LinearConstraint::DoToLatex
        struct /* DoToLatex */ {
          // Source: drake/solvers/constraint.h
          const char* doc = R"""()""";
        } DoToLatex;
        // Symbol: drake::solvers::LinearConstraint::GetDenseA
        struct /* GetDenseA */ {
          // Source: drake/solvers/constraint.h
          const char* doc =
R"""(Get the matrix A as a dense matrix.

Note:
    this might involve memory allocation to convert a sparse matrix to
    a dense one, for better performance you should call get_sparse_A()
    which returns a sparse matrix.)""";
        } GetDenseA;
        // Symbol: drake::solvers::LinearConstraint::LinearConstraint
        struct /* ctor */ {
          // Source: drake/solvers/constraint.h
          const char* doc_dense_A =
R"""(Construct the linear constraint lb <= A*x <= ub

Throws if A has any entry which is not finite.)""";
          // Source: drake/solvers/constraint.h
          const char* doc_sparse_A =
R"""(Overloads constructor with a sparse A matrix. Throws if A has any
entry which is not finite.)""";
        } ctor;
        // Symbol: drake::solvers::LinearConstraint::RemoveTinyCoefficient
        struct /* RemoveTinyCoefficient */ {
          // Source: drake/solvers/constraint.h
          const char* doc =
R"""(Sets A(i, j) to zero if abs(A(i, j)) <= tol. Oftentimes the
coefficient A is computed numerically with round-off errors. Such
small round-off errors can cause numerical issues for certain
optimization solvers. Hence it is recommended to remove the tiny
coefficients to achieve numerical robustness.

Parameter ``tol``:
    The entries in A with absolute value <= tol will be set to 0.

Note:
    tol>= 0.)""";
        } RemoveTinyCoefficient;
        // Symbol: drake::solvers::LinearConstraint::UpdateCoefficients
        struct /* UpdateCoefficients */ {
          // Source: drake/solvers/constraint.h
          const char* doc_dense_A =
R"""(Updates the linear term, upper and lower bounds in the linear
constraint. The updated constraint is: new_lb <= new_A * x <= new_ub
Note that the size of constraints (number of rows) can change, but the
number of variables (number of cols) cannot.

Throws if new_A has any entry which is not finite or if new_A, new_lb,
and new_ub don't all have the same number of rows.

Parameter ``new_A``:
    new linear term

Parameter ``new_lb``:
    new lower bound

Parameter ``new_ub``:
    new upper bound)""";
          // Source: drake/solvers/constraint.h
          const char* doc_sparse_A =
R"""(Overloads UpdateCoefficients but with a sparse A matrix.

Throws if new_A has any entry which is not finite or if new_A, new_lb,
and new_ub don't all have the same number of rows.)""";
        } UpdateCoefficients;
        // Symbol: drake::solvers::LinearConstraint::get_sparse_A
        struct /* get_sparse_A */ {
          // Source: drake/solvers/constraint.h
          const char* doc = R"""()""";
        } get_sparse_A;
        // Symbol: drake::solvers::LinearConstraint::is_dense_A_constructed
        struct /* is_dense_A_constructed */ {
          // Source: drake/solvers/constraint.h
          const char* doc =
R"""(Returns true iff this constraint already has a dense representation,
i.e, if GetDenseA() will be cheap.)""";
        } is_dense_A_constructed;
      } LinearConstraint;
      // Symbol: drake::solvers::LinearCost
      struct /* LinearCost */ {
        // Source: drake/solvers/cost.h
        const char* doc =
R"""(Implements a cost of the form

.. math:: a'x + b

.)""";
        // Symbol: drake::solvers::LinearCost::DoDisplay
        struct /* DoDisplay */ {
          // Source: drake/solvers/cost.h
          const char* doc = R"""()""";
        } DoDisplay;
        // Symbol: drake::solvers::LinearCost::DoEval
        struct /* DoEval */ {
          // Source: drake/solvers/cost.h
          const char* doc = R"""()""";
        } DoEval;
        // Symbol: drake::solvers::LinearCost::DoToLatex
        struct /* DoToLatex */ {
          // Source: drake/solvers/cost.h
          const char* doc = R"""()""";
        } DoToLatex;
        // Symbol: drake::solvers::LinearCost::GetSparseMatrix
        struct /* GetSparseMatrix */ {
          // Source: drake/solvers/cost.h
          const char* doc = R"""()""";
        } GetSparseMatrix;
        // Symbol: drake::solvers::LinearCost::LinearCost
        struct /* ctor */ {
          // Source: drake/solvers/cost.h
          const char* doc =
R"""(Construct a linear cost of the form

.. math:: a'x + b

.

Parameter ``a``:
    Linear term.

Parameter ``b``:
    (optional) Constant term.)""";
        } ctor;
        // Symbol: drake::solvers::LinearCost::UpdateCoefficients
        struct /* UpdateCoefficients */ {
          // Source: drake/solvers/cost.h
          const char* doc =
R"""(Updates the coefficients of the cost. Note that the number of
variables (size of a) cannot change.

Parameter ``new_a``:
    New linear term.

Parameter ``new_b``:
    (optional) New constant term.)""";
        } UpdateCoefficients;
        // Symbol: drake::solvers::LinearCost::a
        struct /* a */ {
          // Source: drake/solvers/cost.h
          const char* doc = R"""()""";
        } a;
        // Symbol: drake::solvers::LinearCost::b
        struct /* b */ {
          // Source: drake/solvers/cost.h
          const char* doc = R"""()""";
        } b;
        // Symbol: drake::solvers::LinearCost::update_coefficient_entry
        struct /* update_coefficient_entry */ {
          // Source: drake/solvers/cost.h
          const char* doc =
R"""(Updates one entry in the coefficient of the cost. a[i] = val.

Parameter ``i``:
    The index of the coefficient to be updated.

Parameter ``val``:
    The value of that updated entry.)""";
        } update_coefficient_entry;
        // Symbol: drake::solvers::LinearCost::update_constant_term
        struct /* update_constant_term */ {
          // Source: drake/solvers/cost.h
          const char* doc =
R"""(Updates the constant term in the cost to ``new_b``.)""";
        } update_constant_term;
      } LinearCost;
      // Symbol: drake::solvers::LinearEqualityConstraint
      struct /* LinearEqualityConstraint */ {
        // Source: drake/solvers/constraint.h
        const char* doc =
R"""(Implements a constraint of the form :math:`Ax = b`)""";
        // Symbol: drake::solvers::LinearEqualityConstraint::LinearEqualityConstraint
        struct /* ctor */ {
          // Source: drake/solvers/constraint.h
          const char* doc_dense_Aeq =
R"""(Constructs the linear equality constraint Aeq * x = beq.

Throws is any entry in Aeq or beq is not finite.)""";
          // Source: drake/solvers/constraint.h
          const char* doc_sparse_Aeq =
R"""(Overloads the constructor with a sparse matrix Aeq.)""";
          // Source: drake/solvers/constraint.h
          const char* doc_row_a =
R"""(Constructs the linear equality constraint a.dot(x) = beq)""";
        } ctor;
        // Symbol: drake::solvers::LinearEqualityConstraint::UpdateCoefficients
        struct /* UpdateCoefficients */ {
          // Source: drake/solvers/constraint.h
          const char* doc =
R"""(Overloads UpdateCoefficients but with a sparse A matrix.

Throws if any entry of beq or Aeq is not finite.)""";
        } UpdateCoefficients;
      } LinearEqualityConstraint;
      // Symbol: drake::solvers::LinearMatrixInequalityConstraint
      struct /* LinearMatrixInequalityConstraint */ {
        // Source: drake/solvers/constraint.h
        const char* doc =
R"""(Impose the matrix inequality constraint on variable x

.. math:: F_0 + x_1  F_1 + ... + x_n  F_n \text{ is p.s.d}

where p.s.d stands for positive semidefinite. :math:`F_0, F_1, ...,
F_n` are all given symmetric matrices of the same size.

Note:
    if the matrices Fᵢ all have 1 row, then it is better to impose a
    linear inequality constraints; if they all have 2 rows, then it is
    better to impose a rotated Lorentz cone constraint, since a 2 x 2
    matrix X being p.s.d is equivalent to the constraint [X(0, 0),
    X(1, 1), X(0, 1)] in the rotated Lorentz cone.)""";
        // Symbol: drake::solvers::LinearMatrixInequalityConstraint::DoEval
        struct /* DoEval */ {
          // Source: drake/solvers/constraint.h
          const char* doc_was_unable_to_choose_unambiguous_names =
R"""(Evaluate the eigen values of the linear matrix.)""";
        } DoEval;
        // Symbol: drake::solvers::LinearMatrixInequalityConstraint::DoToLatex
        struct /* DoToLatex */ {
          // Source: drake/solvers/constraint.h
          const char* doc = R"""()""";
        } DoToLatex;
        // Symbol: drake::solvers::LinearMatrixInequalityConstraint::F
        struct /* F */ {
          // Source: drake/solvers/constraint.h
          const char* doc = R"""()""";
        } F;
        // Symbol: drake::solvers::LinearMatrixInequalityConstraint::LinearMatrixInequalityConstraint
        struct /* ctor */ {
          // Source: drake/solvers/constraint.h
          const char* doc =
R"""(Parameter ``F``:
    Each symmetric matrix F[i] should be of the same size.

Parameter ``symmetry_tolerance``:
    The precision to determine if the input matrices Fi are all
    symmetric.

See also:
    math∷IsSymmetric().)""";
        } ctor;
        // Symbol: drake::solvers::LinearMatrixInequalityConstraint::WarnOnSmallMatrixSize
        struct /* WarnOnSmallMatrixSize */ {
          // Source: drake/solvers/constraint.h
          const char* doc = R"""(Warn if the matrix size is 1x1 or 2x2.)""";
        } WarnOnSmallMatrixSize;
        // Symbol: drake::solvers::LinearMatrixInequalityConstraint::matrix_rows
        struct /* matrix_rows */ {
          // Source: drake/solvers/constraint.h
          const char* doc =
R"""(Gets the number of rows in the matrix inequality constraint. Namely Fi
are all matrix_rows() x matrix_rows() matrices.)""";
        } matrix_rows;
      } LinearMatrixInequalityConstraint;
      // Symbol: drake::solvers::LinearSystemSolver
      struct /* LinearSystemSolver */ {
        // Source: drake/solvers/linear_system_solver.h
        const char* doc =
R"""(Finds the least-square solution to the linear system A * x = b.)""";
        // Symbol: drake::solvers::LinearSystemSolver::LinearSystemSolver
        struct /* ctor */ {
          // Source: drake/solvers/linear_system_solver.h
          const char* doc = R"""()""";
        } ctor;
        // Symbol: drake::solvers::LinearSystemSolver::ProgramAttributesSatisfied
        struct /* ProgramAttributesSatisfied */ {
          // Source: drake/solvers/linear_system_solver.h
          const char* doc = R"""()""";
        } ProgramAttributesSatisfied;
        // Symbol: drake::solvers::LinearSystemSolver::UnsatisfiedProgramAttributes
        struct /* UnsatisfiedProgramAttributes */ {
          // Source: drake/solvers/linear_system_solver.h
          const char* doc = R"""()""";
        } UnsatisfiedProgramAttributes;
        // Symbol: drake::solvers::LinearSystemSolver::id
        struct /* id */ {
          // Source: drake/solvers/linear_system_solver.h
          const char* doc = R"""()""";
        } id;
        // Symbol: drake::solvers::LinearSystemSolver::is_available
        struct /* is_available */ {
          // Source: drake/solvers/linear_system_solver.h
          const char* doc = R"""()""";
        } is_available;
        // Symbol: drake::solvers::LinearSystemSolver::is_enabled
        struct /* is_enabled */ {
          // Source: drake/solvers/linear_system_solver.h
          const char* doc = R"""()""";
        } is_enabled;
      } LinearSystemSolver;
      // Symbol: drake::solvers::LogarithmicSos2NewBinaryVariables
      struct /* LogarithmicSos2NewBinaryVariables */ {
        // Source: drake/solvers/mixed_integer_optimization_util.h
        const char* doc =
R"""(The size of the new binary variables in the compile time, for Special
Ordered Set of type 2 (SOS2) constraint. The SOS2 constraint says that


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    λ(0) + ... + λ(n) = 1
      ∀i. λ(i) ≥ 0
      ∃ j ∈ {0, 1, ..., n-1}, s.t λ(j) + λ(j + 1) = 1

.. raw:: html

    </details>

Template parameter ``NumLambda``:
    The length of the lambda vector. NumLambda = n + 1.)""";
        // Symbol: drake::solvers::LogarithmicSos2NewBinaryVariables::type
        struct /* type */ {
          // Source: drake/solvers/mixed_integer_optimization_util.h
          const char* doc = R"""()""";
        } type;
      } LogarithmicSos2NewBinaryVariables;
      // Symbol: drake::solvers::LorentzConeConstraint
      struct /* LorentzConeConstraint */ {
        // Source: drake/solvers/constraint.h
        const char* doc =
R"""(Constraining the linear expression :math:`z=Ax+b` lies within the
Lorentz cone. A vector z ∈ ℝ ⁿ lies within Lorentz cone if

.. math:: z_0 \ge \sqrt{z_1^2+...+z_{n-1}^2}

where A ∈ ℝ ⁿˣᵐ, b ∈ ℝ ⁿ are given matrices. Ideally this constraint
should be handled by a second-order cone solver. In case the user
wants to enforce this constraint through general nonlinear
optimization, we provide three different formulations on the Lorentz
cone constraint 1. [kConvex] g(z) = z₀ - sqrt(z₁² + ... + zₙ₋₁²) ≥ 0
This formulation is not differentiable at z₁=...=zₙ₋₁=0 2.
[kConvexSmooth] g(z) = z₀ - sqrt(z₁² + ... + zₙ₋₁²) ≥ 0 but the
gradient of g(z) is approximated as ∂g(z)/∂z = [1, -z₁/sqrt(z₁² + ...
zₙ₋₁² + ε), ..., -zₙ₋₁/sqrt(z₁²+...+zₙ₋₁²+ε)] where ε is a small
positive number. 3. [kNonconvex] z₀²-(z₁²+...+zₙ₋₁²) ≥ 0 z₀ ≥ 0 This
constraint is differentiable everywhere, but z₀²-(z₁²+...+zₙ₋₁²) ≥ 0
is non-convex. For more information and visualization, please refer to
https://www.epfl.ch/labs/disopt/wp-content/uploads/2018/09/7.pdf and
https://docs.mosek.com/modeling-cookbook/cqo.html (Fig 3.1))""";
        // Symbol: drake::solvers::LorentzConeConstraint::A
        struct /* A */ {
          // Source: drake/solvers/constraint.h
          const char* doc = R"""(Getter for A.)""";
        } A;
        // Symbol: drake::solvers::LorentzConeConstraint::A_dense
        struct /* A_dense */ {
          // Source: drake/solvers/constraint.h
          const char* doc = R"""(Getter for dense version of A.)""";
        } A_dense;
        // Symbol: drake::solvers::LorentzConeConstraint::EvalType
        struct /* EvalType */ {
          // Source: drake/solvers/constraint.h
          const char* doc =
R"""(We provide three possible Eval functions to represent the Lorentz cone
constraint z₀ ≥ sqrt(z₁² + ... + zₙ₋₁²). For more explanation on the
three formulations, refer to LorentzConeConstraint documentation.)""";
          // Symbol: drake::solvers::LorentzConeConstraint::EvalType::kConvex
          struct /* kConvex */ {
            // Source: drake/solvers/constraint.h
            const char* doc =
R"""(The constraint is g(z) = z₀ - sqrt(z₁² + ... + zₙ₋₁²) ≥ 0. Note this
formulation is non-differentiable at z₁= ...= zₙ₋₁=0)""";
          } kConvex;
          // Symbol: drake::solvers::LorentzConeConstraint::EvalType::kConvexSmooth
          struct /* kConvexSmooth */ {
            // Source: drake/solvers/constraint.h
            const char* doc =
R"""(Same as kConvex, but with approximated gradient that exists
everywhere..)""";
          } kConvexSmooth;
          // Symbol: drake::solvers::LorentzConeConstraint::EvalType::kNonconvex
          struct /* kNonconvex */ {
            // Source: drake/solvers/constraint.h
            const char* doc =
R"""(Nonconvex constraint z₀²-(z₁²+...+zₙ₋₁²) ≥ 0 and z₀ ≥ 0. Note this
formulation is differentiable, but at z₁= ...= zₙ₋₁=0 the gradient is
also 0, so a gradient-based nonlinear solver can get stuck.)""";
          } kNonconvex;
        } EvalType;
        // Symbol: drake::solvers::LorentzConeConstraint::LorentzConeConstraint
        struct /* ctor */ {
          // Source: drake/solvers/constraint.h
          const char* doc =
R"""(Raises:
    RuntimeError if A.row() < 2.)""";
        } ctor;
        // Symbol: drake::solvers::LorentzConeConstraint::UpdateCoefficients
        struct /* UpdateCoefficients */ {
          // Source: drake/solvers/constraint.h
          const char* doc =
R"""(Updates the coefficients, the updated constraint is z=new_A * x +
new_b in the Lorentz cone.

Raises:
    RuntimeError if the new_A.cols() != A.cols(), namely the variable
    size should not change.

Precondition:
    ``new_A`` has to have at least 2 rows and new_A.rows() ==
    new_b.rows().)""";
        } UpdateCoefficients;
        // Symbol: drake::solvers::LorentzConeConstraint::b
        struct /* b */ {
          // Source: drake/solvers/constraint.h
          const char* doc = R"""(Getter for b.)""";
        } b;
        // Symbol: drake::solvers::LorentzConeConstraint::eval_type
        struct /* eval_type */ {
          // Source: drake/solvers/constraint.h
          const char* doc = R"""(Getter for eval type.)""";
        } eval_type;
      } LorentzConeConstraint;
      // Symbol: drake::solvers::Make2NormSquaredCost
      struct /* Make2NormSquaredCost */ {
        // Source: drake/solvers/cost.h
        const char* doc =
R"""(Creates a quadratic cost of the form |Ax-b|²=(Ax-b)ᵀ(Ax-b))""";
      } Make2NormSquaredCost;
      // Symbol: drake::solvers::MakeFirstAvailableSolver
      struct /* MakeFirstAvailableSolver */ {
        // Source: drake/solvers/choose_best_solver.h
        const char* doc =
R"""(Makes the first available and enabled solver. If no solvers are
available, throws a RuntimeError.)""";
      } MakeFirstAvailableSolver;
      // Symbol: drake::solvers::MakeFunctionCost
      struct /* MakeFunctionCost */ {
        // Source: drake/solvers/cost.h
        const char* doc =
R"""(Converts an input of type ``F`` to a nonlinear cost.

Template parameter ``FF``:
    The forwarded function type (e.g., ``const F&``, `F&&`, ...). The
    class ``F`` should have functions numInputs(), numOutputs(), and
    eval(x, y).)""";
      } MakeFunctionCost;
      // Symbol: drake::solvers::MakeFunctionEvaluator
      struct /* MakeFunctionEvaluator */ {
        // Source: drake/solvers/evaluator_base.h
        const char* doc =
R"""(Creates a FunctionEvaluator instance bound to a given callable object.

Template parameter ``FF``:
    Perfect-forwarding type of ``F`` (e.g., ``const F&``, `F&&`).

Parameter ``f``:
    Callable function object.

Returns:
    An implementation of EvaluatorBase using the callable object.)""";
      } MakeFunctionEvaluator;
      // Symbol: drake::solvers::MakeQuadraticErrorCost
      struct /* MakeQuadraticErrorCost */ {
        // Source: drake/solvers/cost.h
        const char* doc =
R"""(Creates a cost term of the form (x-x_desired)'*Q*(x-x_desired).)""";
      } MakeQuadraticErrorCost;
      // Symbol: drake::solvers::MakeSemidefiniteRelaxation
      struct /* MakeSemidefiniteRelaxation */ {
        // Source: drake/solvers/semidefinite_relaxation.h
        const char* doc_2args =
R"""(Constructs a new MathematicalProgram which represents the semidefinite
programming convex relaxation of the (likely nonconvex) program
``prog``. This method currently supports only linear and quadratic
costs and constraints, but may be extended in the future with broader
support.

See https://underactuated.mit.edu/optimization.html#sdp_relaxation for
references and examples.

Note: Currently, programs using LinearEqualityConstraint will give
tighter relaxations than programs using LinearConstraint or
BoundingBoxConstraint, even if lower_bound == upper_bound. Prefer
LinearEqualityConstraint.

Raises:
    RuntimeError if ``prog`` has costs and constraints which are not
    linear nor quadratic.)""";
        // Source: drake/solvers/semidefinite_relaxation.h
        const char* doc_3args =
R"""(A version of MakeSemidefiniteRelaxation that allows for specifying the
sparsity of the relaxation.

For each group in ``variable_groups``, the costs and constraints whose
variables are a subset of the group will be jointly relaxed into a
single, dense semidefinite program in the same manner as
MakeSemidefiniteRelaxation(prog).

Each of these semidefinite relaxations are aggregated into a single
program, and their semidefinite variables are made to agree where the
variable groups overlap.

The returned program will always have the same number of PSD variables
as variable groups.

Costs and constraints whose variables are not a subset of any of the
groups are not relaxed and are simply added to the aggregated program.
If these costs and constraints are non-convex, then this method will
throw.

As an example, consider the following program. min x₂ᵀ * Q * x₂
subject to x₁ + x₂ ≤ 1 x₂ + x₃ ≤ 2 x₁ + x₃ ≤ 3

And suppose we call MakeSemidefiniteRelaxation(prog,
std∷vector<Variables>{{x₁, x₂}, {x₂,x₃}}).

The resulting relaxation would have two semidefinite variables,
namely: [U₁, U₂, x₁] [W₁, W₂, x₂] [U₂, U₃, x₂], [W₂, W₃, x₃] [x₁ᵀ,
x₂ᵀ, 1] [x₂ᵀ, x₃ᵀ, 1]

The first semidefinite variable would be associated to the
semidefinite relaxation of the subprogram: min x₁ᵀ * Q * x₁ subject to
x₁ + x₂ ≤ 1 And the implied constraints from x₁ + x₂ ≤ 1 would be
added to the first semidefinite variable. These implied constraints
are additional constraints that can be placed on the matrix [U₁, U₂,
x₁] [U₂, U₃, x₂] [x₁ᵀ, x₂ᵀ, 1] which are redundant in the non-convex
program, but are not redundant in the semidefinite relaxation. See
https://underactuated.mit.edu/optimization.html#sdp_relaxation for
references and examples.

The second semidefinite variable would be associated to the
semidefinite relaxation of the subprogram: min x₂ᵀ * Q * x₂ subject to
x₂ + x₃ ≤ 2 And the implied constraints from x₂ + x₃ ≤ 2 would be
added to the second semidefinite variable.

Since the constraint x₁ + x₃ ≤ 3 is not a subset of any of the
variable groups, it will be added to the overall relaxation, but will
not be used to generate implied constraints on any semidefinite
variable.

The total relaxation would also include an equality constraint that U₃
== W₁ so that the quadratic relaxation of x₂ is consistent between the
two semidefinite variables.

Note: 1) Costs are only associated to a single variable group, so that
the resulting aggregated program has a relaxed cost with the same
scaling. 2) The homogenization variable "1" is re-used in every
semidefinite variable.

Raises:
    RuntimeError if there is a non-convex cost or constraint whose
    variables do not intersect with any of the variable groups.)""";
      } MakeSemidefiniteRelaxation;
      // Symbol: drake::solvers::MakeSolver
      struct /* MakeSolver */ {
        // Source: drake/solvers/choose_best_solver.h
        const char* doc =
R"""(Given the solver ID, create the solver with the matching ID.

Raises:
    RuntimeError if there is no matching solver.)""";
      } MakeSolver;
      // Symbol: drake::solvers::MathematicalProgram
      struct /* MathematicalProgram */ {
        // Source: drake/solvers/mathematical_program.h
        const char* doc =
R"""(MathematicalProgram stores the decision variables, the constraints and
costs of an optimization problem. The user can solve the problem by
calling solvers∷Solve() function, and obtain the results of the
optimization.)""";
        // Symbol: drake::solvers::MathematicalProgram::Add2NormSquaredCost
        struct /* Add2NormSquaredCost */ {
          // Source: drake/solvers/mathematical_program.h
          const char* doc =
R"""(Adds a quadratic cost of the form |Ax-b|²=(Ax-b)ᵀ(Ax-b))""";
        } Add2NormSquaredCost;
        // Symbol: drake::solvers::MathematicalProgram::AddBoundingBoxConstraint
        struct /* AddBoundingBoxConstraint */ {
          // Source: drake/solvers/mathematical_program.h
          const char* doc_3args_lb_ub_vars =
R"""(Adds bounding box constraints referencing potentially a subset of the
decision variables.

Parameter ``lb``:
    The lower bound.

Parameter ``ub``:
    The upper bound.

Parameter ``vars``:
    Will imposes constraint lb(i, j) <= vars(i, j) <= ub(i, j).

Returns:
    The newly constructed BoundingBoxConstraint.)""";
          // Source: drake/solvers/mathematical_program.h
          const char* doc_3args_lb_ub_var =
R"""(Adds bounds for a single variable.

Parameter ``lb``:
    Lower bound.

Parameter ``ub``:
    Upper bound.

Parameter ``var``:
    The decision variable.)""";
          // Source: drake/solvers/mathematical_program.h
          const char* doc_3args_double_double_constEigenMatrixBase =
R"""(Adds the same scalar lower and upper bound to every variable in
``vars``.

Template parameter ``Derived``:
    An Eigen∷Matrix with Variable as the scalar type. The matrix has
    unknown number of columns at compile time, or has more than one
    column.

Parameter ``lb``:
    Lower bound.

Parameter ``ub``:
    Upper bound.

Parameter ``vars``:
    The decision variables.)""";
        } AddBoundingBoxConstraint;
        // Symbol: drake::solvers::MathematicalProgram::AddConstraint
        struct /* AddConstraint */ {
          // Source: drake/solvers/mathematical_program.h
          const char* doc_1args_binding =
R"""(Adds a generic constraint to the program. This should only be used if
a more specific type of constraint is not available, as it may require
the use of a significantly more expensive solver.

Note:
    If ``binding``.evaluator()->num_constraints() == 0, then this
    constraint is not added into the MathematicalProgram. We return
    ``binding`` directly.)""";
          // Source: drake/solvers/mathematical_program.h
          const char* doc_3args_e_lb_ub =
R"""(Adds one row of constraint lb <= e <= ub where ``e`` is a symbolic
expression.

Raises:
    RuntimeError if 1. ``lb <= e <= ub`` is a trivial constraint such
    as 1 <= 2 <= 3. 2. ``lb <= e <= ub`` is unsatisfiable such as 1 <=
    -5 <= 3

Parameter ``e``:
    A symbolic expression of the decision variables.

Parameter ``lb``:
    A scalar, the lower bound.

Parameter ``ub``:
    A scalar, the upper bound.

The resulting constraint may be a BoundingBoxConstraint,
LinearConstraint, LinearEqualityConstraint, QuadraticConstraint, or
ExpressionConstraint, depending on the arguments. Constraints of the
form x == 1 (which could be created as a BoundingBoxConstraint or
LinearEqualityConstraint) will be constructed as a
LinearEqualityConstraint.)""";
          // Source: drake/solvers/mathematical_program.h
          const char* doc_1args_f =
R"""(Add a constraint represented by a symbolic formula to the program. The
input formula ``f`` can be of the following forms:

1. e1 <= e2
2. e1 >= e2
3. e1 == e2
4. A conjunction of relational formulas where each conjunct is
   a relational formula matched by 1, 2, or 3.

Note that first two cases might return an object of
Binding<BoundingBoxConstraint>, Binding<LinearConstraint>, or
Binding<ExpressionConstraint>, depending on ``f``. Also the third case
might return an object of Binding<LinearEqualityConstraint> or
Binding<ExpressionConstraint>.

It throws an exception if 1. ``f`` is not matched with one of the
above patterns. Especially, strict inequalities (<, >) are not
allowed. 2. ``f`` is either a trivial constraint such as "1 <= 2" or
an unsatisfiable constraint such as "2 <= 1". 3. It is not possible to
find numerical bounds of ``e1`` and ``e2`` where ``f`` = e1 ≃ e2. We
allow ``e1`` and ``e2`` to be infinite but only if there are no other
terms. For example, ``x <= ∞`` is allowed. However, ``x - ∞ <= 0`` is
not allowed because ``x ↦ ∞`` introduces ``nan`` in the evaluation.)""";
          // Source: drake/solvers/mathematical_program.h
          const char* doc_1args_constEigenDenseBase =
R"""(Adds a constraint represented by an Eigen∷Matrix<symbolic∷Formula> or
Eigen∷Array<symbolic∷Formula> to the program. A common use-case of
this function is to add a constraint with the element-wise comparison
between two Eigen matrices, using ``A.array() <= B.array()``. See the
following example.


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    MathematicalProgram prog;
      Eigen∷Matrix<double, 2, 2> A = ...;
      Eigen∷Vector2d b = ...;
      auto x = prog.NewContinuousVariables(2, "x");
      prog.AddConstraint((A * x).array() <= b.array());

.. raw:: html

    </details>

A formula in ``formulas`` can be of the following forms:

1. e1 <= e2
2. e1 >= e2
3. e1 == e2

It throws an exception if AddConstraint(const symbolic∷Formula& f)
throws an exception for f ∈ ``formulas``.

@overload Binding<Constraint> AddConstraint(const symbolic∷Formula& f)

Template parameter ``Derived``:
    Eigen∷Matrix or Eigen∷Array with Formula as the Scalar.)""";
          // Source: drake/solvers/mathematical_program.h
          const char* doc_2args_con_vars =
R"""(Adds a generic constraint to the program. This should only be used if
a more specific type of constraint is not available, as it may require
the use of a significantly more expensive solver.)""";
        } AddConstraint;
        // Symbol: drake::solvers::MathematicalProgram::AddCost
        struct /* AddCost */ {
          // Source: drake/solvers/mathematical_program.h
          const char* doc_1args_binding_cost =
R"""(Adds a generic cost to the optimization program.)""";
          // Source: drake/solvers/mathematical_program.h
          const char* doc_2args_obj_vars =
R"""(Adds a cost type to the optimization program.

Parameter ``obj``:
    The added objective.

Parameter ``vars``:
    The decision variables on which the cost depend.)""";
          // Source: drake/solvers/mathematical_program.h
          const char* doc_1args_binding =
R"""(Adds an L2 norm cost |Ax+b|₂ (notice this cost is not quadratic since
we don't take the square of the L2 norm). Refer to AddL2NormCost for
more details.)""";
          // Source: drake/solvers/mathematical_program.h
          const char* doc_1args_e =
R"""(Adds a cost in the symbolic form.

Returns:
    The newly created cost, together with the bound variables.)""";
        } AddCost;
        // Symbol: drake::solvers::MathematicalProgram::AddDecisionVariables
        struct /* AddDecisionVariables */ {
          // Source: drake/solvers/mathematical_program.h
          const char* doc =
R"""(Appends new variables to the end of the existing variables.

Parameter ``decision_variables``:
    The newly added decision_variables.

Precondition:
    ``decision_variables`` should not intersect with the existing
    indeterminates in the optimization program.

Raises:
    RuntimeError if the preconditions are not satisfied.)""";
        } AddDecisionVariables;
        // Symbol: drake::solvers::MathematicalProgram::AddEqualityConstraintBetweenPolynomials
        struct /* AddEqualityConstraintBetweenPolynomials */ {
          // Source: drake/solvers/mathematical_program.h
          const char* doc =
R"""(Constraining that two polynomials are the same (i.e., they have the
same coefficients for each monomial). This function is often used in
sum-of-squares optimization. We will impose the linear equality
constraint that the coefficient of a monomial in ``p1`` is the same as
the coefficient of the same monomial in ``p2``.

Parameter ``p1``:
    Note that p1's indeterminates should have been registered as
    indeterminates in this MathematicalProgram object, and p1's
    coefficients are affine functions of decision variables in this
    MathematicalProgram object.

Parameter ``p2``:
    Note that p2's indeterminates should have been registered as
    indeterminates in this MathematicalProgram object, and p2's
    coefficients are affine functions of decision variables in this
    MathematicalProgram object.

Note:
    It calls ``Reparse`` to enforce ``p1`` and ``p2`` to have this
    MathematicalProgram's indeterminates.)""";
        } AddEqualityConstraintBetweenPolynomials;
        // Symbol: drake::solvers::MathematicalProgram::AddExponentialConeConstraint
        struct /* AddExponentialConeConstraint */ {
          // Source: drake/solvers/mathematical_program.h
          const char* doc_3args =
R"""(Adds an exponential cone constraint, that z = A * vars + b should be
in the exponential cone. Namely {z₀, z₁, z₂ | z₀ ≥ z₁ * exp(z₂ / z₁),
z₁ > 0}, or equivalently (using the logarithm function), {z₀, z₁, z₂ |
z₂ ≤ z₁ * log(z₀ / z₁), z₀ > 0, z₁ > 0}.

Parameter ``A``:
    The A matrix in the documentation above. A must have 3 rows.

Parameter ``b``:
    The b vector in the documentation above.

Parameter ``vars``:
    The variables bound with this constraint.)""";
          // Source: drake/solvers/mathematical_program.h
          const char* doc_1args =
R"""(Add the constraint that z is in the exponential cone.

Parameter ``z``:
    The expression in the exponential cone.

Precondition:
    each entry in ``z`` is a linear expression of the decision
    variables.)""";
        } AddExponentialConeConstraint;
        // Symbol: drake::solvers::MathematicalProgram::AddIndeterminate
        struct /* AddIndeterminate */ {
          // Source: drake/solvers/mathematical_program.h
          const char* doc =
R"""(Adds indeterminate. This method appends an indeterminate to the end of
the program's old indeterminates, if ``new_indeterminate`` is not
already in the program's old indeterminates.

Parameter ``new_indeterminate``:
    The indeterminate to be appended to the program's old
    indeterminates.

Returns:
    indeterminate_index The index of the added indeterminate in the
    program's indeterminates. i.e.
    prog.indeterminates()(indeterminate_index) = new_indeterminate.

Precondition:
    ``new_indeterminate`` should not intersect with the program's
    decision variables.

Precondition:
    new_indeterminate should be of CONTINUOUS type.)""";
        } AddIndeterminate;
        // Symbol: drake::solvers::MathematicalProgram::AddIndeterminates
        struct /* AddIndeterminates */ {
          // Source: drake/solvers/mathematical_program.h
          const char* doc =
R"""(Adds indeterminates. This method appends some indeterminates to the
end of the program's old indeterminates.

Parameter ``new_indeterminates``:
    The indeterminates to be appended to the program's old
    indeterminates.

Precondition:
    ``new_indeterminates`` should not intersect with the program's old
    decision variables.

Precondition:
    Each entry in new_indeterminates should be of CONTINUOUS type.)""";
        } AddIndeterminates;
        // Symbol: drake::solvers::MathematicalProgram::AddL1NormCostInEpigraphForm
        struct /* AddL1NormCostInEpigraphForm */ {
          // Source: drake/solvers/mathematical_program.h
          const char* doc =
R"""(Adds an L1 norm cost min |Ax+b|₁ as a linear cost min Σᵢsᵢ on the
slack variables sᵢ, together with the constraints (for each i) sᵢ ≥
(|Ax+b|)ᵢ, which itself is written sᵢ ≥ (Ax+b)ᵢ and sᵢ ≥ -(Ax+b)ᵢ.

Returns:
    (s, linear_cost, linear_constraint). ``s`` is the vector of slack
    variables, ``linear_cost`` is the cost on ``s``, and
    ``linear_constraint`` is the constraint encoding s ≥ Ax+b and s ≥
    -(Ax+b).)""";
        } AddL1NormCostInEpigraphForm;
        // Symbol: drake::solvers::MathematicalProgram::AddL2NormCost
        struct /* AddL2NormCost */ {
          // Source: drake/solvers/mathematical_program.h
          const char* doc_3args_A_b_vars =
R"""(Adds an L2 norm cost |Ax+b|₂ (notice this cost is not quadratic since
we don't take the square of the L2 norm).

Note:
    Currently kL2NormCost is supported by SnoptSolver, IpoptSolver,
    NloptSolver, GurobiSolver, MosekSolver, ClarabelSolver, and
    SCSSolver.)""";
          // Source: drake/solvers/mathematical_program.h
          const char* doc_3args_A_b_vars_list =
R"""(Adds an L2 norm cost |Ax+b|₂ (notice this cost is not quadratic since
we don't take the square of the L2 norm))""";
          // Source: drake/solvers/mathematical_program.h
          const char* doc_expression =
R"""(Adds an L2 norm cost |Ax+b|₂ from a symbolic expression which can be
decomposed into sqrt((Ax+b)'(Ax+b)). See
symbolic∷DecomposeL2NormExpression for details on the tolerance
parameters.

Raises:
    RuntimeError if ``e`` cannot be decomposed into an L2 norm.)""";
        } AddL2NormCost;
        // Symbol: drake::solvers::MathematicalProgram::AddL2NormCostUsingConicConstraint
        struct /* AddL2NormCostUsingConicConstraint */ {
          // Source: drake/solvers/mathematical_program.h
          const char* doc =
R"""(Adds an L2 norm cost min |Ax+b|₂ as a linear cost min s on the slack
variable s, together with a Lorentz cone constraint s ≥ |Ax+b|₂ Many
conic optimization solvers (Gurobi, MOSEK™, SCS, etc) natively prefers
this form of linear cost + conic constraints. So if you are going to
use one of these conic solvers, then add the L2 norm cost using this
function instead of AddL2NormCost().

Returns:
    (s, linear_cost, lorentz_cone_constraint). ``s`` is the slack
    variable (with variable name string as "slack"), ``linear_cost``
    is the cost on ``s``, and ``lorentz_cone_constraint`` is the
    constraint s≥|Ax+b|₂)""";
        } AddL2NormCostUsingConicConstraint;
        // Symbol: drake::solvers::MathematicalProgram::AddLinearComplementarityConstraint
        struct /* AddLinearComplementarityConstraint */ {
          // Source: drake/solvers/mathematical_program.h
          const char* doc =
R"""(Adds a linear complementarity constraints referencing a subset of the
decision variables.)""";
        } AddLinearComplementarityConstraint;
        // Symbol: drake::solvers::MathematicalProgram::AddLinearConstraint
        struct /* AddLinearConstraint */ {
          // Source: drake/solvers/mathematical_program.h
          const char* doc_4args_A_lb_ub_dense =
R"""(Adds linear constraints referencing potentially a subset of the
decision variables (defined in the vars parameter).)""";
          // Source: drake/solvers/mathematical_program.h
          const char* doc_4args_A_lb_ub_sparse =
R"""(Adds sparse linear constraints referencing potentially a subset of the
decision variables (defined in the vars parameter).)""";
          // Source: drake/solvers/mathematical_program.h
          const char* doc_4args_a_lb_ub_vars =
R"""(Adds one row of linear constraint referencing potentially a subset of
the decision variables (defined in the vars parameter). lb <= a*vars
<= ub

Parameter ``a``:
    A row vector.

Parameter ``lb``:
    A scalar, the lower bound.

Parameter ``ub``:
    A scalar, the upper bound.

Parameter ``vars``:
    The decision variables on which to impose the linear constraint.)""";
          // Source: drake/solvers/mathematical_program.h
          const char* doc_3args_e_lb_ub =
R"""(Adds one row of linear constraint lb <= e <= ub where ``e`` is a
symbolic expression.

Raises:
    RuntimeError if 1. ``e`` is a non-linear expression. 2. ``lb <= e
    <= ub`` is a trivial constraint such as 1 <= 2 <= 3. 3. ``lb <= e
    <= ub`` is unsatisfiable such as 1 <= -5 <= 3

Parameter ``e``:
    A linear symbolic expression in the form of ``c0 + c1 * v1 + ... +
    cn * vn`` where ``c_i`` is a constant and @v_i is a variable.

Parameter ``lb``:
    A scalar, the lower bound.

Parameter ``ub``:
    A scalar, the upper bound.)""";
          // Source: drake/solvers/mathematical_program.h
          const char* doc_3args_v_lb_ub =
R"""(Adds linear constraints represented by symbolic expressions to the
program. It throws if @v includes a non-linear expression or ``lb <= v
<= ub`` includes trivial/unsatisfiable constraints.)""";
          // Source: drake/solvers/mathematical_program.h
          const char* doc_1args_f =
R"""(Add a linear constraint represented by a symbolic formula to the
program. The input formula ``f`` can be of the following forms:

1. e1 <= e2
2. e1 >= e2
3. e1 == e2
4. A conjunction of relational formulas where each conjunct is
   a relational formula matched by 1, 2, or 3.

Note that first two cases might return an object of
Binding<BoundingBoxConstraint> depending on ``f``. Also the third case
returns an object of Binding<LinearEqualityConstraint>.

It throws an exception if 1. ``f`` is not matched with one of the
above patterns. Especially, strict inequalities (<, >) are not
allowed. 2. ``f`` includes a non-linear expression. 3. ``f`` is either
a trivial constraint such as "1 <= 2" or an unsatisfiable constraint
such as "2 <= 1". 4. It is not possible to find numerical bounds of
``e1`` and ``e2`` where ``f`` = e1 ≃ e2. We allow ``e1`` and ``e2`` to
be infinite but only if there are no other terms. For example, ``x <=
∞`` is allowed. However, ``x - ∞ <= 0`` is not allowed because ``x ↦
∞`` introduces ``nan`` in the evaluation.)""";
          // Source: drake/solvers/mathematical_program.h
          const char* doc_1args_formulas =
R"""(Add a linear constraint represented by an
Eigen∷Array<symbolic∷Formula> to the program. A common use-case of
this function is to add a linear constraint with the element-wise
comparison between two Eigen matrices, using ``A.array() <=
B.array()``. See the following example.


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    MathematicalProgram prog;
      Eigen∷Matrix<double, 2, 2> A;
      auto x = prog.NewContinuousVariables(2, "x");
      Eigen∷Vector2d b;
      ... // set up A and b
      prog.AddLinearConstraint((A * x).array() <= b.array());

.. raw:: html

    </details>

A formula in ``formulas`` can be of the following forms:

1. e1 <= e2 2. e1 >= e2 3. e1 == e2

It throws an exception if AddLinearConstraint(const symbolic∷Formula&
f) throws an exception for f ∈ ``formulas``.

Template parameter ``Derived``:
    An Eigen Array type of Formula.)""";
        } AddLinearConstraint;
        // Symbol: drake::solvers::MathematicalProgram::AddLinearCost
        struct /* AddLinearCost */ {
          // Source: drake/solvers/mathematical_program.h
          const char* doc_1args =
R"""(Adds a linear cost term of the form a'*x + b.

Parameter ``e``:
    A linear symbolic expression.

Precondition:
    e is a linear expression a'*x + b, where each entry of x is a
    decision variable in the mathematical program.

Returns:
    The newly added linear constraint, together with the bound
    variables.)""";
          // Source: drake/solvers/mathematical_program.h
          const char* doc_3args =
R"""(Adds a linear cost term of the form a'*x + b. Applied to a subset of
the variables and pushes onto the linear cost data structure.)""";
          // Source: drake/solvers/mathematical_program.h
          const char* doc_2args =
R"""(Adds a linear cost term of the form a'*x. Applied to a subset of the
variables and pushes onto the linear cost data structure.)""";
        } AddLinearCost;
        // Symbol: drake::solvers::MathematicalProgram::AddLinearEqualityConstraint
        struct /* AddLinearEqualityConstraint */ {
          // Source: drake/solvers/mathematical_program.h
          const char* doc_2args_e_b =
R"""(Adds one row of linear constraint e = b where ``e`` is a symbolic
expression.

Raises:
    RuntimeError if 1. ``e`` is a non-linear expression. 2. ``e`` is a
    constant.

Parameter ``e``:
    A linear symbolic expression in the form of ``c0 + c1 * x1 + ... +
    cn * xn`` where ``c_i`` is a constant and @x_i is a variable.

Parameter ``b``:
    A scalar.

Returns:
    The newly added linear equality constraint, together with the
    bound variable.)""";
          // Source: drake/solvers/mathematical_program.h
          const char* doc_1args_f =
R"""(Adds a linear equality constraint represented by a symbolic formula to
the program. The input formula ``f`` is either an equality formula
(``e1 == e2``) or a conjunction of equality formulas.

It throws an exception if

1. ``f`` is neither an equality formula nor a conjunction of equalities.
2. ``f`` includes a non-linear expression.)""";
          // Source: drake/solvers/mathematical_program.h
          const char* doc_1args_formulas =
R"""(Adds a linear equality constraint represented by an
Eigen∷Array<symbolic∷Formula> to the program. A common use-case of
this function is to add a linear constraint with the element-wise
comparison between two Eigen matrices, using ``A.array() ==
B.array()``. See the following example.


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    MathematicalProgram prog;
      Eigen∷Matrix<double, 2, 2> A;
      auto x = prog.NewContinuousVariables(2, "x");
      Eigen∷Vector2d b;
      ... // set up A and b
      prog.AddLinearConstraint((A * x).array() == b.array());

.. raw:: html

    </details>

It throws an exception if AddLinearConstraint(const symbolic∷Formula&
f) throws an exception for f ∈ ``formulas``.

Template parameter ``Derived``:
    An Eigen Array type of Formula.)""";
          // Source: drake/solvers/mathematical_program.h
          const char* doc_2args_constEigenMatrixBase_constEigenMatrixBase =
R"""(Adds linear equality constraints :math:`v = b`, where ``v(i)`` is a
symbolic linear expression.

Raises:
    RuntimeError if 1. ``v(i)`` is a non-linear expression. 2.
    ``v(i)`` is a constant.

Template parameter ``DerivedV``:
    An Eigen Matrix type of Expression. A column vector.

Template parameter ``DerivedB``:
    An Eigen Matrix type of double. A column vector.

Parameter ``v``:
    v(i) is a linear symbolic expression in the form of `` c0 + c1 *
    x1 + ... + cn * xn `` where ci is a constant and @xi is a
    variable.

Parameter ``b``:
    A vector of doubles.

Returns:
    The newly added linear equality constraint, together with the
    bound variables.)""";
          // Source: drake/solvers/mathematical_program.h
          const char* doc_3args_Aeq_beq_dense =
R"""(AddLinearEqualityConstraint

Adds linear equality constraints referencing potentially a subset of
the decision variables.

Example: to add two equality constraints which only depend on two of
the elements of x, you could use


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    auto x = prog.NewContinuousVariables(6,"myvar");
      Eigen∷Matrix2d Aeq;
      Aeq << -1, 2,
              1, 1;
      Eigen∷Vector2d beq(1, 3);
      // Imposes constraint
      // -x(0) + 2x(1) = 1
      //  x(0) +  x(1) = 3
      prog.AddLinearEqualityConstraint(Aeq, beq, x.head<2>());

.. raw:: html

    </details>)""";
          // Source: drake/solvers/mathematical_program.h
          const char* doc_3args_Aeq_beq_sparse =
R"""(AddLinearEqualityConstraint

Adds linear equality constraints referencing potentially a subset of
the decision variables using a sparse A matrix.)""";
          // Source: drake/solvers/mathematical_program.h
          const char* doc_3args_a_beq_vars =
R"""(Adds one row of linear equality constraint referencing potentially a
subset of decision variables.

.. math:: ax = beq

Parameter ``a``:
    A row vector.

Parameter ``beq``:
    A scalar.

Parameter ``vars``:
    The decision variables on which the constraint is imposed.)""";
        } AddLinearEqualityConstraint;
        // Symbol: drake::solvers::MathematicalProgram::AddLinearMatrixInequalityConstraint
        struct /* AddLinearMatrixInequalityConstraint */ {
          // Source: drake/solvers/mathematical_program.h
          const char* doc_2args =
R"""(Adds a linear matrix inequality constraint to the program.)""";
          // Source: drake/solvers/mathematical_program.h
          const char* doc_1args =
R"""(Adds a linear matrix inequality constraint on a symmetric matrix of
symbolic expressions ``X``, namely ``X`` is positive semidefinite, and
each entry in ``X`` is a linear (affine) expression of decision
variables.

Parameter ``X``:
    Imposes constraint "X is positive semidefinite".

Precondition:
    X is symmetric.

Precondition:
    X(i, j) is linear (affine) for all i, j

Returns:
    The newly added linear matrix inequality constraint.)""";
        } AddLinearMatrixInequalityConstraint;
        // Symbol: drake::solvers::MathematicalProgram::AddLogDeterminantLowerBoundConstraint
        struct /* AddLogDeterminantLowerBoundConstraint */ {
          // Source: drake/solvers/mathematical_program.h
          const char* doc =
R"""(Impose the constraint log(det(X)) >= lower. See log_determinant for
more details.

Parameter ``X``:
    A symmetric positive semidefinite matrix X.

Parameter ``lower``:
    The lower bound of log(det(X))

Returns:
    (constraint, t, Z) constraint is ∑ᵢt(i) >= lower, we also return
    the newly created slack variables t and the lower triangular
    matrix Z. Note that Z is not a matrix of symbolic∷Variable but
    symbolic∷Expression, because the upper-diagonal entries of Z are
    not variable, but expression 0.

Precondition:
    X is a symmetric matrix.)""";
        } AddLogDeterminantLowerBoundConstraint;
        // Symbol: drake::solvers::MathematicalProgram::AddLorentzConeConstraint
        struct /* AddLorentzConeConstraint */ {
          // Source: drake/solvers/mathematical_program.h
          const char* doc_formula =
R"""(Adds a Lorentz cone constraint of the form Ax+b >= |Cx+d|₂ from a
symbolic formula with one side which can be decomposed into
sqrt((Cx+d)'(Cx+d)).

Parameter ``eval_type``:
    The evaluation type when evaluating the lorentz cone constraint in
    generic optimization. Refer to LorentzConeConstraint∷EvalType for
    more details.

See symbolic∷DecomposeL2NormExpression for details on the tolerance
parameters, ``psd_tol`` and ``coefficient_tol``. Consider using the
overload which takes a vector of expressions to avoid the numerical
decomposition.

Raises:
    RuntimeError if ``f`` cannot be decomposed into a Lorentz cone.)""";
          // Source: drake/solvers/mathematical_program.h
          const char* doc_2args_v_eval_type =
R"""(Adds Lorentz cone constraint referencing potentially a subset of the
decision variables.

Parameter ``v``:
    An Eigen∷Vector of symbolic∷Expression. Constraining that

.. math:: v_0 \ge \sqrt{v_1^2 + ... + v_{n-1}^2}

Returns:
    The newly constructed Lorentz cone constraint with the bounded
    variables. For example, to add the Lorentz cone constraint

x+1 >= sqrt(y² + 2y + x² + 5), = sqrt((y+1)²+x²+2²) The user could
call


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    {cc}
    Vector4<symbolic∷Expression> v(x+1, y+1, x, 2.);
    prog.AddLorentzConeConstraint(v);

.. raw:: html

    </details>

Parameter ``eval_type``:
    The evaluation type when evaluating the lorentz cone constraint in
    generic optimization. Refer to LorentzConeConstraint∷EvalType for
    more details.)""";
          // Source: drake/solvers/mathematical_program.h
          const char* doc_4args_linear_expression_quadratic_expression_tol_eval_type =
R"""(Adds Lorentz cone constraint on the linear expression v1 and quadratic
expression v2, such that v1 >= sqrt(v2)

Parameter ``linear_expression``:
    The linear expression v1.

Parameter ``quadratic_expression``:
    The quadratic expression v2.

Parameter ``tol``:
    The tolerance to determine if the matrix in v2 is positive
    semidefinite or not.

See also:
    DecomposePositiveQuadraticForm for more explanation. $*Default:*
    is 0.

Parameter ``eval_type``:
    The evaluation type when evaluating the lorentz cone constraint in
    generic optimization. Refer to LorentzConeConstraint∷EvalType for
    more details.

Returns ``binding``:
    The newly added Lorentz cone constraint, together with the bound
    variables.

Precondition:
1. ``v1`` is a linear expression, in the form of c'*x + d.
2. ``v2`` is a quadratic expression, in the form of
   


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    x'*Q*x + b'x + a

.. raw:: html

    </details>

Also the quadratic expression has to be convex, namely Q is a positive
semidefinite matrix, and the quadratic expression needs to be
non-negative for any x.

Raises:
    RuntimeError if the preconditions are not satisfied.

Notice this constraint is equivalent to the vector [z;y] is within a
Lorentz cone, where


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    z = v1
     y = R * x + d

.. raw:: html

    </details>

while (R, d) satisfies y'*y = x'*Q*x + b'*x + a For example, to add
the Lorentz cone constraint

x+1 >= sqrt(y² + 2y + x² + 4), the user could call


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    {cc}
    prog.AddLorentzConeConstraint(x+1, pow(y, 2) + 2 * y + pow(x, 2) + 4);

.. raw:: html

    </details>)""";
          // Source: drake/solvers/mathematical_program.h
          const char* doc_4args_A_b_vars_eval_type =
R"""(Adds Lorentz cone constraint referencing potentially a subset of the
decision variables (defined in the vars parameter). The linear
expression :math:`z=Ax+b` is in the Lorentz cone. A vector :math:`z
\in\mathbb{R}^n` is in the Lorentz cone, if

.. math:: z_0 \ge \sqrt{z_1^2 + ... + z_{n-1}^2}

Parameter ``A``:
    A :math:`\mathbb{R}^{n\times m}` matrix, whose number of columns
    equals to the size of the decision variables.

Parameter ``b``:
    A :math:`\mathbb{R}^n` vector, whose number of rows equals to the
    size of the decision variables.

Parameter ``vars``:
    The Eigen vector of :math:`m` decision variables.

Parameter ``eval_type``:
    The evaluation type when evaluating the lorentz cone constraint in
    generic optimization. Refer to LorentzConeConstraint∷EvalType for
    more details.

Returns:
    The newly added Lorentz cone constraint.

For example, to add the Lorentz cone constraint

x+1 >= sqrt(y² + 2y + x² + 5) = sqrt((y+1)² + x² + 2²), the user could
call


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    {cc}
    Eigen∷Matrix<double, 4, 2> A;
    Eigen∷Vector4d b;
    A << 1, 0, 0, 1, 1, 0, 0, 0;
    b << 1, 1, 0, 2;
    // A * [x;y] + b = [x+1; y+1; x; 2]
    prog.AddLorentzConeConstraint(A, b, Vector2<symbolic∷Variable>(x, y));

.. raw:: html

    </details>)""";
        } AddLorentzConeConstraint;
        // Symbol: drake::solvers::MathematicalProgram::AddMaximizeGeometricMeanCost
        struct /* AddMaximizeGeometricMeanCost */ {
          // Source: drake/solvers/mathematical_program.h
          const char* doc_3args =
R"""(An overloaded version of maximize_geometric_mean.

Returns:
    cost The added cost (note that since MathematicalProgram only
    minimizes the cost, the returned cost evaluates to -power(∏ᵢz(i),
    1/n) where z = A*x+b.

Precondition:
    A.rows() == b.rows(), A.rows() >= 2.)""";
          // Source: drake/solvers/mathematical_program.h
          const char* doc_2args =
R"""(An overloaded version of maximize_geometric_mean. We add the cost to
maximize the geometric mean of x, i.e., c*power(∏ᵢx(i), 1/n).

Parameter ``c``:
    The positive coefficient of the geometric mean cost, $*Default:*
    is 1.

Returns:
    cost The added cost (note that since MathematicalProgram only
    minimizes the cost, the returned cost evaluates to -c *
    power(∏ᵢx(i), 1/n).

Precondition:
    x.rows() >= 2.

Precondition:
    c > 0.)""";
        } AddMaximizeGeometricMeanCost;
        // Symbol: drake::solvers::MathematicalProgram::AddMaximizeLogDeterminantCost
        struct /* AddMaximizeLogDeterminantCost */ {
          // Source: drake/solvers/mathematical_program.h
          const char* doc =
R"""(Maximize the log determinant. See log_determinant for more details.

Parameter ``X``:
    A symmetric positive semidefinite matrix X, whose log(det(X)) will
    be maximized.

Returns:
    (cost, t, Z) cost is -∑ᵢt(i), we also return the newly created
    slack variables t and the lower triangular matrix Z. Note that Z
    is not a matrix of symbolic∷Variable but symbolic∷Expression,
    because the upper-diagonal entries of Z are not variable, but
    expression 0.

Precondition:
    X is a symmetric matrix.)""";
        } AddMaximizeLogDeterminantCost;
        // Symbol: drake::solvers::MathematicalProgram::AddPolynomialConstraint
        struct /* AddPolynomialConstraint */ {
          // Source: drake/solvers/mathematical_program.h
          const char* doc =
R"""(Adds a polynomial constraint to the program referencing a subset of
the decision variables (defined in the vars parameter).)""";
        } AddPolynomialConstraint;
        // Symbol: drake::solvers::MathematicalProgram::AddPolynomialCost
        struct /* AddPolynomialCost */ {
          // Source: drake/solvers/mathematical_program.h
          const char* doc =
R"""(Adds a cost term in the polynomial form.

Parameter ``e``:
    A symbolic expression in the polynomial form.

Returns:
    The newly created cost and the bound variables.)""";
        } AddPolynomialCost;
        // Symbol: drake::solvers::MathematicalProgram::AddPositiveDiagonallyDominantDualConeMatrixConstraint
        struct /* AddPositiveDiagonallyDominantDualConeMatrixConstraint */ {
          // Source: drake/solvers/mathematical_program.h
          const char* doc_expression =
R"""(This is an overloaded variant of add_dd_dual "diagonally dominant dual
cone constraint"

Parameter ``X``:
    The matrix X. We will use 0.5(X+Xᵀ) as the "symmetric version" of
    X.

Precondition:
    X(i, j) should be a linear expression of decision variables.

Returns:
    A linear constraint of size n² encoding vᵢᵀXvᵢ ≥ 0)""";
          // Source: drake/solvers/mathematical_program.h
          const char* doc_variable =
R"""(This is an overloaded variant of add_dd_dual "diagonally dominant dual
cone constraint"

Parameter ``X``:
    The matrix X. We will use 0.5(X+Xᵀ) as the "symmetric version" of
    X.

Returns:
    A linear constraint of size n² encoding vᵢᵀXvᵢ ≥ 0)""";
        } AddPositiveDiagonallyDominantDualConeMatrixConstraint;
        // Symbol: drake::solvers::MathematicalProgram::AddPositiveDiagonallyDominantMatrixConstraint
        struct /* AddPositiveDiagonallyDominantMatrixConstraint */ {
          // Source: drake/solvers/mathematical_program.h
          const char* doc =
R"""(Adds the constraint that a symmetric matrix is diagonally dominant
with non-negative diagonal entries. A symmetric matrix X is diagonally
dominant with non-negative diagonal entries if X(i, i) >= ∑ⱼ |X(i, j)|
∀ j ≠ i namely in each row, the diagonal entry is larger than the sum
of the absolute values of all other entries in the same row. A matrix
being diagonally dominant with non-negative diagonals is a sufficient
(but not necessary) condition of a matrix being positive semidefinite.
Internally we will create a matrix Y as slack variables, such that
Y(i, j) represents the absolute value |X(i, j)| ∀ j ≠ i. The diagonal
entries Y(i, i) = X(i, i) The users can refer to "DSOS and SDSOS
Optimization: More Tractable Alternatives to Sum of Squares and
Semidefinite Optimization" by Amir Ali Ahmadi and Anirudha Majumdar,
with arXiv link https://arxiv.org/abs/1706.02586

Parameter ``X``:
    The matrix X. We will use 0.5(X+Xᵀ) as the "symmetric version" of
    X.

Returns:
    Y The slack variable. Y(i, j) represents |X(i, j)| ∀ j ≠ i, with
    the constraint Y(i, j) >= X(i, j) and Y(i, j) >= -X(i, j). Y is a
    symmetric matrix. The diagonal entries Y(i, i) = X(i, i))""";
        } AddPositiveDiagonallyDominantMatrixConstraint;
        // Symbol: drake::solvers::MathematicalProgram::AddPositiveSemidefiniteConstraint
        struct /* AddPositiveSemidefiniteConstraint */ {
          // Source: drake/solvers/mathematical_program.h
          const char* doc_1args_symmetric_matrix_var =
R"""(Adds a positive semidefinite constraint on a symmetric matrix.

Raises:
    RuntimeError in Debug mode if ``symmetric_matrix_var`` is not
    symmetric.

Parameter ``symmetric_matrix_var``:
    A symmetric MatrixDecisionVariable object.)""";
          // Source: drake/solvers/mathematical_program.h
          const char* doc_1args_e =
R"""(Adds a positive semidefinite constraint on a symmetric matrix of
symbolic expressions ``e``. We create a new symmetric matrix of
variables M being positive semidefinite, with the linear equality
constraint e == M.

Parameter ``e``:
    Imposes constraint "e is positive semidefinite".

Precondition:
    e is symmetric.

Precondition:
    e(i, j) is linear for all i, j

Returns:
    The newly added positive semidefinite constraint, with the bound
    variable M that are also newly added.

For example, to add a constraint that

⌈x + 1 2x + 3 x+y⌉ |2x+ 3 2 0| is positive semidefinite ⌊x + y 0 x⌋
The user could call


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    {cc}
    Matrix3<symbolic∷Expression> e
    e << x+1, 2*x+3, x+y,
         2*x+3,   2,   0,
         x+y,     0,   x;
    prog.AddPositiveSemidefiniteConstraint(e);

.. raw:: html

    </details>

Note:
    This function will add additional variables and linear equality
    constraints. Consider calling
    AddLinearMatrixInequalityConstraint(e), which doesn't introduce
    new variables or linear equality constraints.)""";
        } AddPositiveSemidefiniteConstraint;
        // Symbol: drake::solvers::MathematicalProgram::AddPrincipalSubmatrixIsPsdConstraint
        struct /* AddPrincipalSubmatrixIsPsdConstraint */ {
          // Source: drake/solvers/mathematical_program.h
          const char* doc_2args_symmetric_matrix_var_minor_indices =
R"""(Adds a constraint that the principal submatrix of a symmetric matrix
composed of the indices in minor_indices is positive semidefinite.

Precondition:
    The passed ``symmetric_matrix_var`` is a symmetric matrix.

Precondition:
    All values in ``minor_indices`` lie in the range [0,
    symmetric_matrix_var.rows() - 1].

Parameter ``symmetric_matrix_var``:
    A symmetric MatrixDecisionVariable object.

See also:
    AddPositiveSemidefiniteConstraint)""";
          // Source: drake/solvers/mathematical_program.h
          const char* doc_2args_e_minor_indices =
R"""(Adds a constraint the that the principal submatrix of a symmetric
matrix of expressions composed of the indices in minor_indices is
positive semidefinite.

Precondition:
    The passed ``symmetric_matrix_var`` is a symmetric matrix.

Precondition:
    All values in ``minor_indices`` lie in the range [0,
    symmetric_matrix_var.rows() - 1].

Parameter ``e``:
    Imposes constraint "e is positive semidefinite".

See also:
    AddLinearMatrixInequalityConstraint.

Note:
    the return type is Binding<LinearMatrixInequalityConstraint>,
    different from the overloaded function above which returns
    Binding<PositiveSemidefiniteConstraint>. We impose the constraint
    as an LMI so as to add fewer additional variables and constraints.)""";
        } AddPrincipalSubmatrixIsPsdConstraint;
        // Symbol: drake::solvers::MathematicalProgram::AddQuadraticAsRotatedLorentzConeConstraint
        struct /* AddQuadraticAsRotatedLorentzConeConstraint */ {
          // Source: drake/solvers/mathematical_program.h
          const char* doc =
R"""(Add the convex quadratic constraint 0.5xᵀQx + bᵀx + c <= 0 as a
rotated Lorentz cone constraint [rᵀx+s, 1, Px+q] is in the rotated
Lorentz cone. When solving the optimization problem using conic
solvers (like Mosek, Gurobi, SCS, etc), it is numerically preferable
to impose the convex quadratic constraint as rotated Lorentz cone
constraint. See
https://docs.mosek.com/11.1/capi/prob-def-quadratic.html#a-recommendation

Raises:
    exception if this quadratic constraint is not convex (Q is not
    positive semidefinite)

Parameter ``Q``:
    The Hessian of the quadratic constraint. Should be positive
    semidefinite.

Parameter ``b``:
    The linear coefficient of the quadratic constraint.

Parameter ``c``:
    The constant term of the quadratic constraint.

Parameter ``vars``:
    x in the documentation above.

Parameter ``psd_tol``:
    If the minimal eigenvalue of Q is smaller than -psd_tol, then
    throw an exception. $*Default:* = 0.)""";
        } AddQuadraticAsRotatedLorentzConeConstraint;
        // Symbol: drake::solvers::MathematicalProgram::AddQuadraticConstraint
        struct /* AddQuadraticConstraint */ {
          // Source: drake/solvers/mathematical_program.h
          const char* doc_6args =
R"""(Adds quadratic constraint lb ≤ .5 xᵀQx + bᵀx ≤ ub Notice that if your
quadratic constraint is convex, and you intend to solve the problem
with a convex solver (like Mosek), then it is better to reformulate it
with a second order cone constraint. See
https://docs.mosek.com/11.1/capi/prob-def-quadratic.html#a-recommendation
for an explanation.

Parameter ``vars``:
    x in the documentation above.

Parameter ``hessian_type``:
    Whether the Hessian is positive semidefinite, negative
    semidefinite or indefinite. Drake will check the type if
    hessian_type=std∷nullopt. Specifying the hessian type will speed
    this method up.

Precondition:
    hessian_type should be correct if it is not std∷nullopt, as we
    will blindly trust it in the downstream code.)""";
          // Source: drake/solvers/mathematical_program.h
          const char* doc_4args =
R"""(Overloads AddQuadraticConstraint, impose lb <= e <= ub where ``e`` is
a quadratic expression. Notice that if your quadratic constraint is
convex, and you intend to solve the problem with a convex solver (like
Mosek), then it is better to reformulate it with a second order cone
constraint. See
https://docs.mosek.com/11.1/capi/prob-def-quadratic.html#a-recommendation
for an explanation.)""";
        } AddQuadraticConstraint;
        // Symbol: drake::solvers::MathematicalProgram::AddQuadraticCost
        struct /* AddQuadraticCost */ {
          // Source: drake/solvers/mathematical_program.h
          const char* doc_2args =
R"""(Add a quadratic cost term of the form 0.5*x'*Q*x + b'*x + c.

Parameter ``e``:
    A quadratic symbolic expression.

Parameter ``is_convex``:
    Whether the cost is already known to be convex. If
    is_convex=nullopt (the default), then Drake will determine if
    ``e`` is a convex quadratic cost or not. To improve the
    computation speed, the user can set is_convex if the user knows
    whether the cost is convex or not.

Raises:
    RuntimeError if the expression is not quadratic.

Returns:
    The newly added cost together with the bound variables.)""";
          // Source: drake/solvers/mathematical_program.h
          const char* doc_5args =
R"""(Adds a cost term of the form 0.5*x'*Q*x + b'x + c Applied to subset of
the variables.

Parameter ``is_convex``:
    Whether the cost is already known to be convex. If
    is_convex=nullopt (the default), then Drake will determine if this
    is a convex quadratic cost or not. To improve the computation
    speed, the user can set is_convex if the user knows whether the
    cost is convex or not.)""";
          // Source: drake/solvers/mathematical_program.h
          const char* doc_4args =
R"""(Adds a cost term of the form 0.5*x'*Q*x + b'x Applied to subset of the
variables.

Parameter ``is_convex``:
    Whether the cost is already known to be convex. If
    is_convex=nullopt (the default), then Drake will determine if this
    is a convex quadratic cost or not. To improve the computation
    speed, the user can set is_convex if the user knows whether the
    cost is convex or not.)""";
        } AddQuadraticCost;
        // Symbol: drake::solvers::MathematicalProgram::AddQuadraticErrorCost
        struct /* AddQuadraticErrorCost */ {
          // Source: drake/solvers/mathematical_program.h
          const char* doc_3args_w_x_desired_vars =
R"""(Adds a cost term of the form w*|x-x_desired|^2.)""";
          // Source: drake/solvers/mathematical_program.h
          const char* doc_3args_Q_x_desired_vars =
R"""(Adds a cost term of the form (x-x_desired)'*Q*(x-x_desired).)""";
        } AddQuadraticErrorCost;
        // Symbol: drake::solvers::MathematicalProgram::AddRotatedLorentzConeConstraint
        struct /* AddRotatedLorentzConeConstraint */ {
          // Source: drake/solvers/mathematical_program.h
          const char* doc_4args_linear_expression1_linear_expression2_quadratic_expression_tol =
R"""(Adds rotated Lorentz cone constraint on the linear expression v1, v2
and quadratic expression u, such that v1 * v2 >= u, v1 >= 0, v2 >= 0

Parameter ``linear_expression1``:
    The linear expression v1.

Parameter ``linear_expression2``:
    The linear expression v2.

Parameter ``quadratic_expression``:
    The quadratic expression u.

Parameter ``tol``:
    The tolerance to determine if the matrix in v2 is positive
    semidefinite or not.

See also:
    DecomposePositiveQuadraticForm for more explanation. $*Default:*
    is 0.

Returns ``binding``:
    The newly added rotated Lorentz cone constraint, together with the
    bound variables.

Precondition:
1. ``linear_expression1`` is a linear (affine) expression, in the form of
   v1 = c1'*x + d1.
2. ``linear_expression2`` is a linear (affine) expression, in the form of
   v2 = c2'*x + d2.
2. ``quadratic_expression`` is a quadratic expression, in the form of
   


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    u = x'*Q*x + b'x + a

.. raw:: html

    </details>

Also the quadratic expression has to be convex, namely Q is a positive
semidefinite matrix, and the quadratic expression needs to be
non-negative for any x.

Raises:
    RuntimeError if the preconditions are not satisfied.

For example, to add the rotated Lorentz cone constraint

(x+1)(x+y) >= x²+z²+2z+5 x+1 >= 0 x+y >= 0 The user could call


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    {cc}
    prog.AddRotatedLorentzConeConstraint(x+1, x+y, pow(x, 2) + pow(z, 2) +
    2*z+5);

.. raw:: html

    </details>)""";
          // Source: drake/solvers/mathematical_program.h
          const char* doc_1args_v =
R"""(Adds a constraint that a symbolic expression ``v`` is in the rotated
Lorentz cone, i.e.,

.. math:: v_0v_1 \ge v_2^2 + ... + v_{n-1}^2\
v_0 \ge 0, v_1 \ge 0

Parameter ``v``:
    A linear expression of variables, :math:`v = A x + b`, where
    :math:`A, b` are given matrices of the correct size, :math:`x` is
    the vector of decision variables.

Returns ``binding``:
    The newly added rotated Lorentz cone constraint, together with the
    bound variables.

For example, to add the rotated Lorentz cone constraint

(x+1)(x+y) >= x²+z²+2z+5 = x² + (z+1)² + 2² x+1 >= 0 x+y >= 0 The user
could call


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    {cc}
    Eigen∷Matrix<symbolic∷Expression, 5, 1> v;
    v << x+1, x+y, x, z+1, 2;
    prog.AddRotatedLorentzConeConstraint(v);

.. raw:: html

    </details>)""";
          // Source: drake/solvers/mathematical_program.h
          const char* doc_3args_A_b_vars =
R"""(Adds a rotated Lorentz cone constraint referencing potentially a
subset of decision variables, The linear expression :math:`z=Ax+b` is
in rotated Lorentz cone. A vector :math:`z \in\mathbb{R}^n` is in the
rotated Lorentz cone, if

.. math:: z_0z_1 \ge z_2^2 + ... + z_{n-1}^2

where :math:`A\in\mathbb{R}^{n\times m}, b\in\mathbb{R}^n` are given
matrices.

Parameter ``A``:
    A matrix whose number of columns equals to the size of the
    decision variables.

Parameter ``b``:
    A vector whose number of rows equals to the size of the decision
    variables.

Parameter ``vars``:
    The decision variables on which the constraint is imposed.)""";
          // Source: drake/solvers/mathematical_program.h
          const char* doc_1args_vars =
R"""(Impose that a vector :math:`x\in\mathbb{R}^m` is in rotated Lorentz
cone. Namely

.. math:: x_0 x_1 \ge x_2^2 + ... + x_{m-1}^2\
x_0 \ge 0, x_1 \ge 0

Parameter ``vars``:
    The stacked column of vars lies in the rotated Lorentz cone.

Returns:
    The newly added rotated Lorentz cone constraint.)""";
        } AddRotatedLorentzConeConstraint;
        // Symbol: drake::solvers::MathematicalProgram::AddScaledDiagonallyDominantDualConeMatrixConstraint
        struct /* AddScaledDiagonallyDominantDualConeMatrixConstraint */ {
          // Source: drake/solvers/mathematical_program.h
          const char* doc_expression =
R"""(This is an overloaded variant of add_sdd_dual "scaled diagonally
dominant dual cone constraint"

Parameter ``X``:
    The matrix X. We will use 0.5(X+Xᵀ) as the "symmetric version" of
    X.

Precondition:
    X(i, j) should be a linear expression of decision variables.

Returns:
    A vector of RotatedLorentzConeConstraint constraints of length 1/2
    * n * (n-1) encoding VᵢⱼᵀXVᵢⱼ is psd)""";
          // Source: drake/solvers/mathematical_program.h
          const char* doc_variable =
R"""(This is an overloaded variant of add_sdd_dual "scaled diagonally
dominant dual cone constraint"

Parameter ``X``:
    The matrix X. We will use 0.5(X+Xᵀ) as the "symmetric version" of
    X.

Returns:
    A vector of RotatedLorentzConeConstraint constraints of length 1/2
    * n * (n-1) encoding VᵢⱼᵀXVᵢⱼ is psd)""";
        } AddScaledDiagonallyDominantDualConeMatrixConstraint;
        // Symbol: drake::solvers::MathematicalProgram::AddScaledDiagonallyDominantMatrixConstraint
        struct /* AddScaledDiagonallyDominantMatrixConstraint */ {
          // Source: drake/solvers/mathematical_program.h
          const char* doc_expression =
R"""(This is an overloaded variant of addsdd "scaled diagonally dominant
matrix constraint"

Parameter ``X``:
    The matrix X to be constrained scaled diagonally dominant. X.

Precondition:
    X(i, j) should be a linear expression of decision variables.

Returns:
    M A vector of vectors of 2 x 2 symmetric matrices M. For i < j,
    M[i][j] is


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    [Mⁱʲ(i, i), Mⁱʲ(i, j)]
    [Mⁱʲ(i, j), Mⁱʲ(j, j)].

.. raw:: html

    </details>

Note that M[i][j](0, 1) = Mⁱʲ(i, j) = (X(i, j) + X(j, i)) / 2 for i >=
j, M[i][j] is the zero matrix.)""";
          // Source: drake/solvers/mathematical_program.h
          const char* doc_variable =
R"""(This is an overloaded variant of addsdd "scaled diagonally dominant
matrix constraint"

Parameter ``X``:
    The symmetric matrix X to be constrained scaled diagonally
    dominant.

Returns:
    M For i < j M[i][j] contains the slack variables, mentioned in
    addsdd "scaled diagonally dominant matrix constraint". For i >= j,
    M[i][j] contains default-constructed variables (with get_id() ==
    0).)""";
        } AddScaledDiagonallyDominantMatrixConstraint;
        // Symbol: drake::solvers::MathematicalProgram::AddSosConstraint
        struct /* AddSosConstraint */ {
          // Source: drake/solvers/mathematical_program.h
          const char* doc_4args_p_monomial_basis_type_gram_name =
R"""(Adds constraints that a given polynomial ``p`` is a sums-of-squares
(SOS), that is, ``p`` can be decomposed into ``mᵀQm``, where m is the
``monomial_basis``. It returns the coefficients matrix Q, which is
positive semidefinite.

Parameter ``type``:
    The type of the polynomial. $*Default:* is kSos, but the user can
    also use kSdsos and kDsos. Refer to NonnegativePolynomial for
    details on different types of sos polynomials.

Parameter ``gram_name``:
    The name of the gram matrix for print out.

Note:
    It calls ``Reparse`` to enforce ``p`` to have this
    MathematicalProgram's indeterminates if necessary.)""";
          // Source: drake/solvers/mathematical_program.h
          const char* doc_3args_p_type_gram_name =
R"""(Adds constraints that a given polynomial ``p`` is a sums-of-squares
(SOS), that is, ``p`` can be decomposed into ``mᵀQm``, where m is a
monomial basis selected from the sparsity of ``p``. It returns a pair
of constraint bindings expressing: - The coefficients matrix Q, which
is positive semidefinite. - The monomial basis m.

Parameter ``type``:
    The type of the polynomial. $*Default:* is kSos, but the user can
    also use kSdsos and kDsos. Refer to NonnegativePolynomial for the
    details on different type of sos polynomials.

Parameter ``gram_name``:
    The name of the gram matrix for print out.

Note:
    It calls ``Reparse`` to enforce ``p`` to have this
    MathematicalProgram's indeterminates if necessary.)""";
          // Source: drake/solvers/mathematical_program.h
          const char* doc_4args_e_monomial_basis_type_gram_name =
R"""(Adds constraints that a given symbolic expression ``e`` is a
sums-of-squares (SOS), that is, ``p`` can be decomposed into ``mᵀQm``,
where m is the ``monomial_basis``. Note that it decomposes ``e`` into
a polynomial with respect to ``indeterminates()`` in this mathematical
program. It returns the coefficients matrix Q, which is positive
semidefinite.

Parameter ``type``:
    Refer to NonnegativePolynomial class documentation.

Parameter ``gram_name``:
    The name of the gram matrix for print out.)""";
          // Source: drake/solvers/mathematical_program.h
          const char* doc_3args_e_type_gram_name =
R"""(Adds constraints that a given symbolic expression ``e`` is a
sums-of-squares (SOS), that is, ``e`` can be decomposed into ``mᵀQm``.
Note that it decomposes ``e`` into a polynomial with respect to
``indeterminates()`` in this mathematical program. It returns a pair
expressing: - The coefficients matrix Q, which is positive
semidefinite. - The monomial basis m.

Parameter ``type``:
    Refer to NonnegativePolynomial class documentation.

Parameter ``gram_name``:
    The name of the gram matrix for print out.)""";
        } AddSosConstraint;
        // Symbol: drake::solvers::MathematicalProgram::AddVisualizationCallback
        struct /* AddVisualizationCallback */ {
          // Source: drake/solvers/mathematical_program.h
          const char* doc =
R"""(Adds a callback method to visualize intermediate results of the
optimization.

Note:
    Just like other costs/constraints, not all solvers support
    callbacks. Adding a callback here will force
    MathematicalProgram∷Solve to select a solver that support
    callbacks. For instance, adding a visualization callback to a
    quadratic programming problem may result in using a nonlinear
    programming solver as the default solver.

Parameter ``callback``:
    a std∷function that accepts an Eigen∷Vector of doubles
    representing the bound decision variables.

Parameter ``vars``:
    the decision variables that should be passed to the callback.)""";
        } AddVisualizationCallback;
        // Symbol: drake::solvers::MathematicalProgram::CheckSatisfied
        struct /* CheckSatisfied */ {
          // Source: drake/solvers/mathematical_program.h
          const char* doc =
R"""(Evaluates CheckSatisfied for the constraint in ``binding`` using the
value of ALL of the decision variables in this program.

Raises:
    RuntimeError if the size of ``prog_var_vals`` is invalid.)""";
          // Source: drake/solvers/mathematical_program.h
          const char* doc_vector =
R"""(Evaluates CheckSatisfied for the all of the constraints in ``binding``
using the value of ALL of the decision variables in this program.

Returns:
    true iff all of the constraints are satisfied.

Raises:
    RuntimeError if the size of ``prog_var_vals`` is invalid.)""";
        } CheckSatisfied;
        // Symbol: drake::solvers::MathematicalProgram::CheckSatisfiedAtInitialGuess
        struct /* CheckSatisfiedAtInitialGuess */ {
          // Source: drake/solvers/mathematical_program.h
          const char* doc =
R"""(Evaluates CheckSatisfied for the constraint in ``binding`` at the
initial guess.)""";
          // Source: drake/solvers/mathematical_program.h
          const char* doc_vector =
R"""(Evaluates CheckSatisfied for the all of the constraints in
``bindings`` at the initial guess.

Returns:
    true iff all of the constraints are satisfied.)""";
        } CheckSatisfiedAtInitialGuess;
        // Symbol: drake::solvers::MathematicalProgram::ClearVariableScaling
        struct /* ClearVariableScaling */ {
          // Source: drake/solvers/mathematical_program.h
          const char* doc =
R"""(Clears the scaling factors for decision variables.

See variable_scaling "Variable scaling" for more information.)""";
        } ClearVariableScaling;
        // Symbol: drake::solvers::MathematicalProgram::Clone
        struct /* Clone */ {
          // Source: drake/solvers/mathematical_program.h
          const char* doc =
R"""(Clones an optimization program. The clone will be functionally
equivalent to the source program with the same:

- decision variables
- constraints
- costs
- solver settings
- initial guess

Note that this is currently a *shallow* clone. The costs and
constraints are not themselves cloned.

Returns ``new_prog``:
    . The newly constructed mathematical program.)""";
        } Clone;
        // Symbol: drake::solvers::MathematicalProgram::EvalBinding
        struct /* EvalBinding */ {
          // Source: drake/solvers/mathematical_program.h
          const char* doc =
R"""(Evaluates the value of some binding, for some input value for all
decision variables.

Parameter ``binding``:
    A Binding whose variables are decision variables in this program.

Parameter ``prog_var_vals``:
    The value of all the decision variables in this program.

Raises:
    RuntimeError if the size of ``prog_var_vals`` is invalid.)""";
        } EvalBinding;
        // Symbol: drake::solvers::MathematicalProgram::EvalBindingAtInitialGuess
        struct /* EvalBindingAtInitialGuess */ {
          // Source: drake/solvers/mathematical_program.h
          const char* doc =
R"""(Evaluates the evaluator in ``binding`` at the initial guess.

Returns:
    The value of ``binding`` at the initial guess.)""";
        } EvalBindingAtInitialGuess;
        // Symbol: drake::solvers::MathematicalProgram::EvalBindings
        struct /* EvalBindings */ {
          // Source: drake/solvers/mathematical_program.h
          const char* doc =
R"""(Evaluates a set of bindings (plural version of ``EvalBinding``).

Parameter ``bindings``:
    List of bindings.

Parameter ``prog_var_vals``:
    The value of all the decision variables in this program.

Returns:
    All binding values, concatenated into a single vector.

Raises:
    RuntimeError if the size of ``prog_var_vals`` is invalid.)""";
        } EvalBindings;
        // Symbol: drake::solvers::MathematicalProgram::EvalVisualizationCallbacks
        struct /* EvalVisualizationCallbacks */ {
          // Source: drake/solvers/mathematical_program.h
          const char* doc =
R"""(Evaluates all visualization callbacks registered with the
MathematicalProgram.

Parameter ``prog_var_vals``:
    The value of all the decision variables in this program.

Raises:
    RuntimeError if the size does not match.)""";
        } EvalVisualizationCallbacks;
        // Symbol: drake::solvers::MathematicalProgram::FindDecisionVariableIndex
        struct /* FindDecisionVariableIndex */ {
          // Source: drake/solvers/mathematical_program.h
          const char* doc =
R"""(Returns the index of the decision variable. Internally the solvers
thinks all variables are stored in an array, and it accesses each
individual variable using its index. This index is used when adding
constraints and costs for each solver.

Precondition:
    {``var`` is a decision variable in the mathematical program,
    otherwise this function throws a runtime error.})""";
        } FindDecisionVariableIndex;
        // Symbol: drake::solvers::MathematicalProgram::FindDecisionVariableIndices
        struct /* FindDecisionVariableIndices */ {
          // Source: drake/solvers/mathematical_program.h
          const char* doc =
R"""(Returns the indices of the decision variables. Internally the solvers
thinks all variables are stored in an array, and it accesses each
individual variable using its index. This index is used when adding
constraints and costs for each solver.

Precondition:
    {``vars`` are decision variables in the mathematical program,
    otherwise this function throws a runtime error.})""";
        } FindDecisionVariableIndices;
        // Symbol: drake::solvers::MathematicalProgram::FindIndeterminateIndex
        struct /* FindIndeterminateIndex */ {
          // Source: drake/solvers/mathematical_program.h
          const char* doc =
R"""(Returns the index of the indeterminate. Internally a solver thinks all
indeterminates are stored in an array, and it accesses each individual
indeterminate using its index. This index is used when adding
constraints and costs for each solver.

Precondition:
    ``var`` is a indeterminate in the mathematical program, otherwise
    this function throws a runtime error.)""";
        } FindIndeterminateIndex;
        // Symbol: drake::solvers::MathematicalProgram::GetAllConstraints
        struct /* GetAllConstraints */ {
          // Source: drake/solvers/mathematical_program.h
          const char* doc =
R"""(Getter for returning all constraints.

Returns:
    Vector of all constraint bindings.

Note:
    The group ordering may change as more constraint types are added.)""";
        } GetAllConstraints;
        // Symbol: drake::solvers::MathematicalProgram::GetAllCosts
        struct /* GetAllCosts */ {
          // Source: drake/solvers/mathematical_program.h
          const char* doc =
R"""(Getter returning all costs.

Returns:
    Vector of all cost bindings.

Note:
    The group ordering may change as more cost types are added.)""";
        } GetAllCosts;
        // Symbol: drake::solvers::MathematicalProgram::GetAllLinearConstraints
        struct /* GetAllLinearConstraints */ {
          // Source: drake/solvers/mathematical_program.h
          const char* doc =
R"""(Getter returning all linear constraints (both linear equality and
inequality constraints). Note that this does *not* include bounding
box constraints, which are technically also linear.

Returns:
    Vector of all linear constraint bindings.)""";
        } GetAllLinearConstraints;
        // Symbol: drake::solvers::MathematicalProgram::GetBindingVariableValues
        struct /* GetBindingVariableValues */ {
          // Source: drake/solvers/mathematical_program.h
          const char* doc =
R"""(Given the value of all decision variables, namely
this.decision_variable(i) takes the value prog_var_vals(i), returns
the vector that contains the value of the variables in
binding.variables().

Parameter ``binding``:
    binding.variables() must be decision variables in this
    MathematicalProgram.

Parameter ``prog_var_vals``:
    The value of ALL the decision variables in this program.

Returns:
    binding_variable_vals binding_variable_vals(i) is the value of
    binding.variables()(i) in prog_var_vals.)""";
        } GetBindingVariableValues;
        // Symbol: drake::solvers::MathematicalProgram::GetInitialGuess
        struct /* GetInitialGuess */ {
          // Source: drake/solvers/mathematical_program.h
          const char* doc_1args_decision_variable =
R"""(Gets the initial guess for a single variable.

Precondition:
    ``decision_variable`` has been registered in the optimization
    program.

Raises:
    RuntimeError if the pre condition is not satisfied.)""";
          // Source: drake/solvers/mathematical_program.h
          const char* doc_1args_constEigenMatrixBase =
R"""(Gets the initial guess for some variables.

Precondition:
    Each variable in ``decision_variable_mat`` has been registered in
    the optimization program.

Raises:
    RuntimeError if the pre condition is not satisfied.)""";
        } GetInitialGuess;
        // Symbol: drake::solvers::MathematicalProgram::GetVariableScaling
        struct /* GetVariableScaling */ {
          // Source: drake/solvers/mathematical_program.h
          const char* doc =
R"""(Returns the mapping from a decision variable index to its scaling
factor.

See variable_scaling "Variable scaling" for more information.)""";
        } GetVariableScaling;
        // Symbol: drake::solvers::MathematicalProgram::IsThreadSafe
        struct /* IsThreadSafe */ {
          // Source: drake/solvers/mathematical_program.h
          const char* doc =
R"""(Returns whether it is safe to solve this mathematical program
concurrently. A mathematical program is safe to solve concurrently if
all of its cost, constraints, and visualization callbacks are marked
as thread safe.)""";
        } IsThreadSafe;
        // Symbol: drake::solvers::MathematicalProgram::MakeCost
        struct /* MakeCost */ {
          // Source: drake/solvers/mathematical_program.h
          const char* doc =
R"""(Convert an input of type ``F`` to a FunctionCost object.

Template parameter ``F``:
    This class should have functions numInputs(), numOutputs and
    eval(x, y).)""";
        } MakeCost;
        // Symbol: drake::solvers::MathematicalProgram::MakePolynomial
        struct /* MakePolynomial */ {
          // Source: drake/solvers/mathematical_program.h
          const char* doc =
R"""(Creates a symbolic polynomial from the given expression ``e``. It uses
this MathematicalProgram's ``indeterminates()`` in constructing the
polynomial.

This method helps a user create a polynomial with the right set of
indeterminates which are declared in this MathematicalProgram. We
recommend users to use this method over an explicit call to Polynomial
constructors to avoid a possible mismatch between this
MathematicalProgram's indeterminates and the user-specified
indeterminates (or unspecified, which then includes all symbolic
variables in the expression ``e``). Consider the following example.

e = ax + bx + c

MP.indeterminates() = {x} MP.decision_variables() = {a, b}

- ``MP.MakePolynomial(e)`` create a polynomial, ``(a + b)x + c``.  Here only
  ``x`` is an indeterminate of this polynomial.

- In contrast, ``symbolic∷Polynomial(e)`` returns ``ax + bx + c`` where all
  variables ``{a, b, x}`` are indeterminates. Note that this is problematic
  as its indeterminates, ``{a, b, x}`` and the MathematicalProgram's decision
  variables, ``{a, b}`` overlap.

Note:
    This function does not require that the decision variables in
    ``e`` is a subset of the decision variables in
    MathematicalProgram.)""";
        } MakePolynomial;
        // Symbol: drake::solvers::MathematicalProgram::MathematicalProgram
        struct /* ctor */ {
          // Source: drake/solvers/mathematical_program.h
          const char* doc = R"""()""";
        } ctor;
        // Symbol: drake::solvers::MathematicalProgram::NewBinaryVariables
        struct /* NewBinaryVariables */ {
          // Source: drake/solvers/mathematical_program.h
          const char* doc_3args =
R"""(Adds binary variables, appending them to an internal vector of any
existing vars. The initial guess values for the new variables are set
to NaN, to indicate that an initial guess has not been assigned.
Callers are expected to add costs and/or constraints to have any
effect during optimization. Callers can also set the initial guess of
the decision variables through SetInitialGuess() or
SetInitialGuessForAllVariables().

Template parameter ``Rows``:
    The number of rows in the new variables.

Template parameter ``Cols``:
    The number of columns in the new variables.

Parameter ``rows``:
    The number of rows in the new variables.

Parameter ``cols``:
    The number of columns in the new variables.

Parameter ``name``:
    The commonly shared name of the new variables.

Returns:
    The MatrixDecisionVariable of size rows x cols, containing the new
    vars (not all the vars stored).

Example:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    MathematicalProgram prog;
    auto b = prog.NewBinaryVariables(2, 3, "b");

.. raw:: html

    </details>

This adds a 2 x 3 matrix decision variables into the program.

The name of the variable is only used for the user in order to ease
readability.)""";
          // Source: drake/solvers/mathematical_program.h
          const char* doc_1args =
R"""(Adds a matrix of binary variables into the optimization program.

Template parameter ``Rows``:
    The number of rows in the newly added binary variables.

Template parameter ``Cols``:
    The number of columns in the new variables. The default is 1.

Parameter ``name``:
    Each newly added binary variable will share the same name. The
    default name is "b".

Returns:
    A matrix containing the newly added variables.)""";
          // Source: drake/solvers/mathematical_program.h
          const char* doc_2args =
R"""(Adds binary variables to this MathematicalProgram. The new variables
are viewed as a column vector, with size ``rows`` x 1.

See also:
    NewBinaryVariables(int rows, int cols, const
    std∷vector<std∷string>& names);)""";
        } NewBinaryVariables;
        // Symbol: drake::solvers::MathematicalProgram::NewContinuousVariables
        struct /* NewContinuousVariables */ {
          // Source: drake/solvers/mathematical_program.h
          const char* doc_2args =
R"""(Adds continuous variables, appending them to an internal vector of any
existing vars. The initial guess values for the new variables are set
to NaN, to indicate that an initial guess has not been assigned.
Callers are expected to add costs and/or constraints to have any
effect during optimization. Callers can also set the initial guess of
the decision variables through SetInitialGuess() or
SetInitialGuessForAllVariables().

Parameter ``rows``:
    The number of rows in the new variables.

Parameter ``name``:
    The name of the newly added variables

Returns:
    The VectorDecisionVariable of size rows x 1, containing the new
    vars (not all the vars stored).

Example:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    MathematicalProgram prog;
    auto x = prog.NewContinuousVariables(2, "x");

.. raw:: html

    </details>

This adds a 2 x 1 vector containing decision variables into the
program. The names of the variables are "x(0)" and "x(1)".

The name of the variable is only used for the user in order to ease
readability.)""";
          // Source: drake/solvers/mathematical_program.h
          const char* doc_3args =
R"""(Adds continuous variables, appending them to an internal vector of any
existing vars. The initial guess values for the new variables are set
to NaN, to indicate that an initial guess has not been assigned.
Callers are expected to add costs and/or constraints to have any
effect during optimization. Callers can also set the initial guess of
the decision variables through SetInitialGuess() or
SetInitialGuessForAllVariables().

Template parameter ``Rows``:
    The number of rows of the new variables, in the compile time.

Template parameter ``Cols``:
    The number of columns of the new variables, in the compile time.

Parameter ``rows``:
    The number of rows in the new variables. When Rows is not
    Eigen∷Dynamic, rows is ignored.

Parameter ``cols``:
    The number of columns in the new variables. When Cols is not
    Eigen∷Dynamic, cols is ignored.

Parameter ``name``:
    All variables will share the same name, but different index.

Returns:
    The MatrixDecisionVariable of size Rows x Cols, containing the new
    vars (not all the vars stored).

Example:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    MathematicalProgram prog;
    auto x = prog.NewContinuousVariables(2, 3, "X");
    auto y = prog.NewContinuousVariables<2, 3>(2, 3, "X");

.. raw:: html

    </details>

This adds a 2 x 3 matrix decision variables into the program.

The name of the variable is only used for the user in order to ease
readability.)""";
          // Source: drake/solvers/mathematical_program.h
          const char* doc_1args =
R"""(Adds continuous variables, appending them to an internal vector of any
existing vars. The initial guess values for the new variables are set
to NaN, to indicate that an initial guess has not been assigned.
Callers are expected to add costs and/or constraints to have any
effect during optimization. Callers can also set the initial guess of
the decision variables through SetInitialGuess() or
SetInitialGuessForAllVariables().

Template parameter ``Rows``:
    The number of rows in the new variables.

Template parameter ``Cols``:
    The number of columns in the new variables. The default is 1.

Parameter ``name``:
    All variables will share the same name, but different index.

Returns:
    The MatrixDecisionVariable of size rows x cols, containing the new
    vars (not all the vars stored).

Example:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    MathematicalProgram prog;
    auto x = prog.NewContinuousVariables<2, 3>("X");

.. raw:: html

    </details>

This adds a 2 x 3 matrix decision variables into the program.

The name of the variable is only used for the user in order to ease
readability.)""";
        } NewContinuousVariables;
        // Symbol: drake::solvers::MathematicalProgram::NewEvenDegreeDsosPolynomial
        struct /* NewEvenDegreeDsosPolynomial */ {
          // Source: drake/solvers/mathematical_program.h
          const char* doc =
R"""(see even_degree_nonnegative_polynomial for details. Variant that
produces a DSOS polynomial. Same as NewEvenDegreeSosPolynomial, except
the returned polynomial is diagonally dominant sum of squares (dsos).)""";
        } NewEvenDegreeDsosPolynomial;
        // Symbol: drake::solvers::MathematicalProgram::NewEvenDegreeFreePolynomial
        struct /* NewEvenDegreeFreePolynomial */ {
          // Source: drake/solvers/mathematical_program.h
          const char* doc =
R"""(Returns a free polynomial that only contains even degree monomials. A
monomial is even degree if its total degree (sum of all variables'
degree) is even. For example, xy is an even degree monomial (degree 2)
while x²y is not (degree 3).

Parameter ``indeterminates``:
    The monomial basis is over these indeterminates.

Parameter ``degree``:
    The highest degree of the polynomial.

Parameter ``coeff_name``:
    The coefficients of the polynomial are decision variables with
    this name as a base. The variable name would be "a1", "a2", etc.)""";
        } NewEvenDegreeFreePolynomial;
        // Symbol: drake::solvers::MathematicalProgram::NewEvenDegreeNonnegativePolynomial
        struct /* NewEvenDegreeNonnegativePolynomial */ {
          // Source: drake/solvers/mathematical_program.h
          const char* doc =
R"""(See even_degree_nonnegative_polynomial for more details. Variant that
produces different non-negative polynomials depending on ``type``.

Parameter ``type``:
    The returned polynomial p(x) can be either SOS, SDSOS or DSOS,
    depending on ``type``.)""";
        } NewEvenDegreeNonnegativePolynomial;
        // Symbol: drake::solvers::MathematicalProgram::NewEvenDegreeSdsosPolynomial
        struct /* NewEvenDegreeSdsosPolynomial */ {
          // Source: drake/solvers/mathematical_program.h
          const char* doc =
R"""(see even_degree_nonnegative_polynomial for details. Variant that
produces an SDSOS polynomial.)""";
        } NewEvenDegreeSdsosPolynomial;
        // Symbol: drake::solvers::MathematicalProgram::NewEvenDegreeSosPolynomial
        struct /* NewEvenDegreeSosPolynomial */ {
          // Source: drake/solvers/mathematical_program.h
          const char* doc =
R"""(See even_degree_nonnegative_polynomial for more details. Variant that
produces a SOS polynomial.)""";
        } NewEvenDegreeSosPolynomial;
        // Symbol: drake::solvers::MathematicalProgram::NewFreePolynomial
        struct /* NewFreePolynomial */ {
          // Source: drake/solvers/mathematical_program.h
          const char* doc =
R"""(Returns a free polynomial in a monomial basis over ``indeterminates``
of a given ``degree``. It uses ``coeff_name`` to make new decision
variables and use them as coefficients. For example,
``NewFreePolynomial({x₀, x₁}, 2)`` returns a₀x₁² + a₁x₀x₁ + a₂x₀² +
a₃x₁ + a₄x₀ + a₅.)""";
        } NewFreePolynomial;
        // Symbol: drake::solvers::MathematicalProgram::NewIndeterminates
        struct /* NewIndeterminates */ {
          // Source: drake/solvers/mathematical_program.h
          const char* doc_2args =
R"""(Adds indeterminates to this MathematicalProgram, with default name
"x".

See also:
    NewIndeterminates(int rows, int cols, const
    std∷vector<std∷string>& names);)""";
          // Source: drake/solvers/mathematical_program.h
          const char* doc_3args =
R"""(Adds indeterminates to this MathematicalProgram, with default name
"X". The new variables are returned and viewed as a matrix, with size
``rows`` x ``cols``.

See also:
    NewIndeterminates(int rows, int cols, const
    std∷vector<std∷string>& names);)""";
        } NewIndeterminates;
        // Symbol: drake::solvers::MathematicalProgram::NewOddDegreeFreePolynomial
        struct /* NewOddDegreeFreePolynomial */ {
          // Source: drake/solvers/mathematical_program.h
          const char* doc =
R"""(Returns a free polynomial that only contains odd degree monomials. A
monomial is odd degree if its total degree (sum of all variables'
degree) is even. For example, xy is not an odd degree monomial (degree
2) while x²y is (degree 3).

Parameter ``indeterminates``:
    The monomial basis is over these indeterminates.

Parameter ``degree``:
    The highest degree of the polynomial.

Parameter ``coeff_name``:
    The coefficients of the polynomial are decision variables with
    this name as a base. The variable name would be "a1", "a2", etc.)""";
        } NewOddDegreeFreePolynomial;
        // Symbol: drake::solvers::MathematicalProgram::NewSosPolynomial
        struct /* NewSosPolynomial */ {
          // Source: drake/solvers/mathematical_program.h
          const char* doc_3args_monomial_basis_type_gram_name =
R"""(Returns a pair of a SOS polynomial p = mᵀQm and the Gramian matrix Q,
where m is the ``monomial`` basis. For example,
``NewSosPolynomial(Vector2<Monomial>{x,y})`` returns a polynomial p =
Q₍₀,₀₎x² + 2Q₍₁,₀₎xy + Q₍₁,₁₎y² and Q. Depending on the type of the
polynomial, we will impose different constraint on the polynomial. -
if type = kSos, we impose the polynomial being SOS. - if type =
kSdsos, we impose the polynomial being SDSOS. - if type = kDsos, we
impose the polynomial being DSOS.

Parameter ``gram_name``:
    The name of the gram matrix for print out.

Note:
    Q is a symmetric monomial_basis.rows() x monomial_basis.rows()
    matrix.)""";
          // Source: drake/solvers/mathematical_program.h
          const char* doc_3args_gramian_monomial_basis_type =
R"""(Overloads NewSosPolynomial, except the Gramian matrix Q is an input
instead of an output.)""";
          // Source: drake/solvers/mathematical_program.h
          const char* doc_4args_indeterminates_degree_type_gram_name =
R"""(Overloads NewSosPolynomial. Returns a pair of a SOS polynomial p =
m(x)ᵀQm(x) of degree ``degree`` and the Gramian matrix Q that should
be PSD, where m(x) is the result of calling
``MonomialBasis(indeterminates, degree/2)``. For example,
``NewSosPolynomial({x}, 4)`` returns a pair of a polynomial p =
Q₍₀,₀₎x⁴ + 2Q₍₁,₀₎ x³ + (2Q₍₂,₀₎ + Q₍₁,₁₎)x² + 2Q₍₂,₁₎x + Q₍₂,₂₎ and
Q.

Parameter ``type``:
    Depending on the type of the polynomial, we will impose different
    constraint on the polynomial. - if type = kSos, we impose the
    polynomial being SOS. - if type = kSdsos, we impose the polynomial
    being SDSOS. - if type = kDsos, we impose the polynomial being
    DSOS.

Parameter ``gram_name``:
    The name of the gram matrix for print out.

Raises:
    RuntimeError if ``degree`` is not a positive even integer.

See also:
    MonomialBasis.)""";
        } NewSosPolynomial;
        // Symbol: drake::solvers::MathematicalProgram::NewSymmetricContinuousVariables
        struct /* NewSymmetricContinuousVariables */ {
          // Source: drake/solvers/mathematical_program.h
          const char* doc_2args =
R"""(Adds a runtime sized symmetric matrix as decision variables to this
MathematicalProgram. The optimization will only use the stacked
columns of the lower triangular part of the symmetric matrix as
decision variables.

Parameter ``rows``:
    The number of rows in the symmetric matrix.

Parameter ``name``:
    The name of the matrix. It is only used the for user to understand
    the optimization program. The default name is "Symmetric", and
    each variable will be named as


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    Symmetric(0, 0)     Symmetric(1, 0)     ... Symmetric(rows-1, 0)
    Symmetric(1, 0)     Symmetric(1, 1)     ... Symmetric(rows-1, 1)
               ...
    Symmetric(rows-1,0) Symmetric(rows-1,1) ... Symmetric(rows-1, rows-1)

.. raw:: html

    </details>

Notice that the (i,j)'th entry and (j,i)'th entry has the same name.

Returns:
    The newly added decision variables.)""";
          // Source: drake/solvers/mathematical_program.h
          const char* doc_1args =
R"""(Adds a static sized symmetric matrix as decision variables to this
MathematicalProgram. The optimization will only use the stacked
columns of the lower triangular part of the symmetric matrix as
decision variables.

Template parameter ``rows``:
    The number of rows in the symmetric matrix.

Parameter ``name``:
    The name of the matrix. It is only used the for user to understand
    the optimization program. The default name is "Symmetric", and
    each variable will be named as


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    Symmetric(0, 0)     Symmetric(1, 0)     ... Symmetric(rows-1, 0)
    Symmetric(1, 0)     Symmetric(1, 1)     ... Symmetric(rows-1, 1)
               ...
    Symmetric(rows-1,0) Symmetric(rows-1,1) ... Symmetric(rows-1, rows-1)

.. raw:: html

    </details>

Notice that the (i,j)'th entry and (j,i)'th entry has the same name.

Returns:
    The newly added decision variables.)""";
        } NewSymmetricContinuousVariables;
        // Symbol: drake::solvers::MathematicalProgram::NonnegativePolynomial
        struct /* NonnegativePolynomial */ {
          // Source: drake/solvers/mathematical_program.h
          const char* doc =
R"""(Types of non-negative polynomial that can be found through conic
optimization. We currently support SOS, SDSOS and DSOS. For more
information about these polynomial types, please refer to "DSOS and
SDSOS Optimization: More Tractable Alternatives to Sum of Squares and
Semidefinite Optimization" by Amir Ali Ahmadi and Anirudha Majumdar,
with arXiv link https://arxiv.org/abs/1706.02586)""";
          // Symbol: drake::solvers::MathematicalProgram::NonnegativePolynomial::kDsos
          struct /* kDsos */ {
            // Source: drake/solvers/mathematical_program.h
            const char* doc =
R"""(A diagonally dominant sum-of-squares polynomial.)""";
          } kDsos;
          // Symbol: drake::solvers::MathematicalProgram::NonnegativePolynomial::kSdsos
          struct /* kSdsos */ {
            // Source: drake/solvers/mathematical_program.h
            const char* doc =
R"""(A scaled-diagonally dominant sum-of-squares polynomial.)""";
          } kSdsos;
          // Symbol: drake::solvers::MathematicalProgram::NonnegativePolynomial::kSos
          struct /* kSos */ {
            // Source: drake/solvers/mathematical_program.h
            const char* doc = R"""(A sum-of-squares polynomial.)""";
          } kSos;
        } NonnegativePolynomial;
        // Symbol: drake::solvers::MathematicalProgram::RelaxPsdConstraintToDdDualCone
        struct /* RelaxPsdConstraintToDdDualCone */ {
          // Source: drake/solvers/mathematical_program.h
          const char* doc =
R"""(1. Relaxes the positive semidefinite ``constraint`` with a diagonally
dominant dual cone constraint. 2. Adds the diagonally dominant dual
cone constraint into this MathematicalProgram. 3. Removes the positive
semidefinite ``constraint``, if it had already been registered in this
MathematicalProgram.

This provides a polyhedral (i.e. linear) necessary, but not
sufficient, condition for the variables in ``constraint`` to be
positive semidefinite.

Precondition:
    The decision variables contained in constraint have been
    registered with this MathematicalProgram.

Returns:
    The return of
    AddPositiveDiagonallyDominantDualConeMatrixConstraint applied to
    the variables in ``constraint``.)""";
        } RelaxPsdConstraintToDdDualCone;
        // Symbol: drake::solvers::MathematicalProgram::RelaxPsdConstraintToSddDualCone
        struct /* RelaxPsdConstraintToSddDualCone */ {
          // Source: drake/solvers/mathematical_program.h
          const char* doc =
R"""(1. Relaxes the positive semidefinite ``constraint`` with a scaled
diagonally dominant dual cone constraint. 2. Adds the scaled
diagonally dominant dual cone constraint into this
MathematicalProgram. 3. Removes the positive semidefinite
``constraint``, if it had already been registered in this
MathematicalProgram.

This provides a second-order cone necessary, but not sufficient,
condition for the variables in ``constraint`` to be positive
semidefinite.

Precondition:
    The decision variables contained in constraint have been
    registered with this MathematicalProgram.

Returns:
    The return of AddScaledDiagonallyDominantDualConeMatrixConstraint
    applied to the variables in ``constraint``.)""";
        } RelaxPsdConstraintToSddDualCone;
        // Symbol: drake::solvers::MathematicalProgram::RemoveConstraint
        struct /* RemoveConstraint */ {
          // Source: drake/solvers/mathematical_program.h
          const char* doc =
R"""(Removes ``constraint`` from this mathematical program. See
remove_cost_constraint "Remove costs, constraints or callbacks" for
more details.

Returns:
    number of constraint objects removed from this program. If this
    program doesn't contain ``constraint``, then returns 0. If this
    program contains multiple ``constraint`` objects, then returns the
    repetition of ``constraint`` in this program.)""";
        } RemoveConstraint;
        // Symbol: drake::solvers::MathematicalProgram::RemoveCost
        struct /* RemoveCost */ {
          // Source: drake/solvers/mathematical_program.h
          const char* doc =
R"""(Removes ``cost`` from this mathematical program. See
remove_cost_constraint "Remove costs, constraints or callbacks" for
more details.

Returns:
    number of cost objects removed from this program. If this program
    doesn't contain ``cost``, then returns 0. If this program contains
    multiple ``cost`` objects, then returns the repetition of ``cost``
    in this program.)""";
        } RemoveCost;
        // Symbol: drake::solvers::MathematicalProgram::RemoveDecisionVariable
        struct /* RemoveDecisionVariable */ {
          // Source: drake/solvers/mathematical_program.h
          const char* doc =
R"""(Remove ``var`` from this program's decision variable.

Note:
    after removing the variable, the indices of some remaining
    variables inside this MathematicalProgram will change.

Returns:
    the index of ``var`` in this optimization program. return -1 if
    ``var`` is not a decision variable.

Raises:
    exception if ``var`` is bound with any cost or constraint.

Raises:
    exception if ``var`` is not a decision variable of the program.)""";
        } RemoveDecisionVariable;
        // Symbol: drake::solvers::MathematicalProgram::RemoveVisualizationCallback
        struct /* RemoveVisualizationCallback */ {
          // Source: drake/solvers/mathematical_program.h
          const char* doc =
R"""(Removes ``callback`` from this mathematical program. See
remove_cost_constraint "Remove costs, constraints or callbacks" for
more details.

Returns:
    number of callback objects removed from this program. If this
    program doesn't contain ``callback``, then returns 0. If this
    program contains multiple ``callback`` objects, then returns the
    repetition of ``callback`` in this program.)""";
        } RemoveVisualizationCallback;
        // Symbol: drake::solvers::MathematicalProgram::Reparse
        struct /* Reparse */ {
          // Source: drake/solvers/mathematical_program.h
          const char* doc =
R"""(Reparses the polynomial ``p`` using this MathematicalProgram's
indeterminates.)""";
        } Reparse;
        // Symbol: drake::solvers::MathematicalProgram::SetDecisionVariableValueInVector
        struct /* SetDecisionVariableValueInVector */ {
          // Source: drake/solvers/mathematical_program.h
          const char* doc_3args_decision_variable_decision_variable_new_value_values =
R"""(Updates the value of a single ``decision_variable`` inside the
``values`` vector to be ``decision_variable_new_value``. The other
decision variables' values in ``values`` are unchanged.

Parameter ``decision_variable``:
    a registered decision variable in this program.

Parameter ``decision_variable_new_value``:
    the variable's new values.

Parameter ``values``:
    The vector to be tweaked; must be of size num_vars().)""";
          // Source: drake/solvers/mathematical_program.h
          const char* doc_3args_decision_variables_decision_variables_new_values_values =
R"""(Updates the values of some ``decision_variables`` inside the
``values`` vector to be ``decision_variables_new_values``. The other
decision variables' values in ``values`` are unchanged.

Parameter ``decision_variables``:
    registered decision variables in this program.

Parameter ``decision_variables_new_values``:
    the variables' respective new values; must have the same rows()
    and cols() sizes and ``decision_variables``.

Parameter ``values``:
    The vector to be tweaked; must be of size num_vars().)""";
        } SetDecisionVariableValueInVector;
        // Symbol: drake::solvers::MathematicalProgram::SetInitialGuess
        struct /* SetInitialGuess */ {
          // Source: drake/solvers/mathematical_program.h
          const char* doc_2args_decision_variable_variable_guess_value =
R"""(Sets the initial guess for a single variable ``decision_variable``.
The guess is stored as part of this program.

Precondition:
    decision_variable is a registered decision variable in the
    program.

Raises:
    RuntimeError if precondition is not satisfied.)""";
          // Source: drake/solvers/mathematical_program.h
          const char* doc_2args_constEigenMatrixBase_constEigenMatrixBase =
R"""(Sets the initial guess for the decision variables stored in
``decision_variable_mat`` to be ``x0``. The guess is stored as part of
this program.)""";
        } SetInitialGuess;
        // Symbol: drake::solvers::MathematicalProgram::SetInitialGuessForAllVariables
        struct /* SetInitialGuessForAllVariables */ {
          // Source: drake/solvers/mathematical_program.h
          const char* doc =
R"""(Set the initial guess for ALL decision variables. Note that variables
begin with a default initial guess of NaN to indicate that no guess is
available.

Parameter ``x0``:
    A vector of appropriate size (num_vars() x 1).)""";
        } SetInitialGuessForAllVariables;
        // Symbol: drake::solvers::MathematicalProgram::SetSolverOption
        struct /* SetSolverOption */ {
          // Source: drake/solvers/mathematical_program.h
          const char* doc_double_option =
R"""(See set_solver_option for more details. Set the double-valued options.)""";
          // Source: drake/solvers/mathematical_program.h
          const char* doc_int_option =
R"""(See set_solver_option for more details. Set the integer-valued
options.)""";
          // Source: drake/solvers/mathematical_program.h
          const char* doc_string_option =
R"""(See set_solver_option for more details. Set the string-valued options.)""";
        } SetSolverOption;
        // Symbol: drake::solvers::MathematicalProgram::SetSolverOptions
        struct /* SetSolverOptions */ {
          // Source: drake/solvers/mathematical_program.h
          const char* doc =
R"""(Overwrite the stored solver options inside MathematicalProgram with
the provided solver options.)""";
        } SetSolverOptions;
        // Symbol: drake::solvers::MathematicalProgram::SetVariableScaling
        struct /* SetVariableScaling */ {
          // Source: drake/solvers/mathematical_program.h
          const char* doc =
R"""(Setter for the scaling ``s`` of decision variable ``var``.

Parameter ``var``:
    the decision variable to be scaled.

Parameter ``s``:
    scaling factor (must be positive).

See variable_scaling "Variable scaling" for more information.)""";
        } SetVariableScaling;
        // Symbol: drake::solvers::MathematicalProgram::TightenPsdConstraintToDd
        struct /* TightenPsdConstraintToDd */ {
          // Source: drake/solvers/mathematical_program.h
          const char* doc =
R"""(1. Tightens the positive semidefinite ``constraint`` with a positive
diagonally dominant constraint. 2. Adds the positive diagonally
dominant constraint into this MathematicalProgram. 3. Removes the
positive semidefinite ``constraint``, if it had already been
registered in this MathematicalProgram.

This provides a polyhedral (i.e. linear) sufficient, but not
necessary, condition for the variables in ``constraint`` to be
positive semidefinite.

Precondition:
    The decision variables contained in constraint have been
    registered with this MathematicalProgram.

Returns:
    The return of AddPositiveDiagonallyDominantMatrixConstraint
    applied to the variables in ``constraint``.)""";
        } TightenPsdConstraintToDd;
        // Symbol: drake::solvers::MathematicalProgram::TightenPsdConstraintToSdd
        struct /* TightenPsdConstraintToSdd */ {
          // Source: drake/solvers/mathematical_program.h
          const char* doc =
R"""(1. Tightens the positive semidefinite ``constraint`` with a scaled
diagonally dominant constraint. 2. Adds the scaled diagonally dominant
constraint into this MathematicalProgram. 3. Removes the positive
semidefinite ``constraint``, if it had already been registered in this
MathematicalProgram.

This provides a second-order cone sufficient, but not necessary,
condition for the variables in ``constraint`` to be positive
semidefinite.

Precondition:
    The decision variables contained in constraint have been
    registered with this MathematicalProgram.

Returns:
    The return of AddScaledDiagonallyDominantMatrixConstraint applied
    to the variables in ``constraint``.)""";
        } TightenPsdConstraintToSdd;
        // Symbol: drake::solvers::MathematicalProgram::ToLatex
        struct /* ToLatex */ {
          // Source: drake/solvers/mathematical_program.h
          const char* doc =
R"""(Returns a string representation of this program in LaTeX.

This can be particularly useful e.g. in a Jupyter (python) notebook:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    from IPython.display import Markdown, display
    display(Markdown(prog.ToLatex()))

.. raw:: html

    </details>

Note that by default, we do not require variables to have unique
names. Providing useful variable names and calling
Evaluator∷set_description() to describe the costs and constraints can
dramatically improve the readability of the output. See the tutorial
``debug_mathematical_program.ipynb`` for more information.)""";
        } ToLatex;
        // Symbol: drake::solvers::MathematicalProgram::VarType
        struct /* VarType */ {
          // Source: drake/solvers/mathematical_program.h
          const char* doc = R"""()""";
        } VarType;
        // Symbol: drake::solvers::MathematicalProgram::bounding_box_constraints
        struct /* bounding_box_constraints */ {
          // Source: drake/solvers/mathematical_program.h
          const char* doc = R"""(Getter for all bounding box constraints)""";
        } bounding_box_constraints;
        // Symbol: drake::solvers::MathematicalProgram::decision_variable
        struct /* decision_variable */ {
          // Source: drake/solvers/mathematical_program.h
          const char* doc =
R"""(Getter for the decision variable with index ``i`` in the program.)""";
        } decision_variable;
        // Symbol: drake::solvers::MathematicalProgram::decision_variable_index
        struct /* decision_variable_index */ {
          // Source: drake/solvers/mathematical_program.h
          const char* doc =
R"""(Returns the mapping from a decision variable ID to its index in the
vector containing all the decision variables in the mathematical
program.)""";
        } decision_variable_index;
        // Symbol: drake::solvers::MathematicalProgram::decision_variables
        struct /* decision_variables */ {
          // Source: drake/solvers/mathematical_program.h
          const char* doc =
R"""(Getter for all decision variables in the program.)""";
        } decision_variables;
        // Symbol: drake::solvers::MathematicalProgram::exponential_cone_constraints
        struct /* exponential_cone_constraints */ {
          // Source: drake/solvers/mathematical_program.h
          const char* doc =
R"""(Getter for exponential cone constraints.)""";
        } exponential_cone_constraints;
        // Symbol: drake::solvers::MathematicalProgram::generic_constraints
        struct /* generic_constraints */ {
          // Source: drake/solvers/mathematical_program.h
          const char* doc = R"""(Getter for all generic constraints)""";
        } generic_constraints;
        // Symbol: drake::solvers::MathematicalProgram::generic_costs
        struct /* generic_costs */ {
          // Source: drake/solvers/mathematical_program.h
          const char* doc = R"""(Getter for all generic costs.)""";
        } generic_costs;
        // Symbol: drake::solvers::MathematicalProgram::indeterminate
        struct /* indeterminate */ {
          // Source: drake/solvers/mathematical_program.h
          const char* doc =
R"""(Getter for the indeterminate with index ``i`` in the program.)""";
        } indeterminate;
        // Symbol: drake::solvers::MathematicalProgram::indeterminates
        struct /* indeterminates */ {
          // Source: drake/solvers/mathematical_program.h
          const char* doc =
R"""(Getter for all indeterminates in the program.)""";
        } indeterminates;
        // Symbol: drake::solvers::MathematicalProgram::indeterminates_index
        struct /* indeterminates_index */ {
          // Source: drake/solvers/mathematical_program.h
          const char* doc =
R"""(Returns the mapping from an indeterminate ID to its index in the
vector containing all the indeterminates in the mathematical program.)""";
        } indeterminates_index;
        // Symbol: drake::solvers::MathematicalProgram::initial_guess
        struct /* initial_guess */ {
          // Source: drake/solvers/mathematical_program.h
          const char* doc = R"""(Getter for the initial guess)""";
        } initial_guess;
        // Symbol: drake::solvers::MathematicalProgram::l2norm_costs
        struct /* l2norm_costs */ {
          // Source: drake/solvers/mathematical_program.h
          const char* doc = R"""(Getter for l2norm costs.)""";
        } l2norm_costs;
        // Symbol: drake::solvers::MathematicalProgram::linear_complementarity_constraints
        struct /* linear_complementarity_constraints */ {
          // Source: drake/solvers/mathematical_program.h
          const char* doc =
R"""(Getter for all linear complementarity constraints.)""";
        } linear_complementarity_constraints;
        // Symbol: drake::solvers::MathematicalProgram::linear_constraints
        struct /* linear_constraints */ {
          // Source: drake/solvers/mathematical_program.h
          const char* doc =
R"""(Getter for linear *inequality* constraints. Note that this does not
include linear_equality_constraints() nor bounding_box_constraints().
See also GetAllLinearConstraints().)""";
        } linear_constraints;
        // Symbol: drake::solvers::MathematicalProgram::linear_costs
        struct /* linear_costs */ {
          // Source: drake/solvers/mathematical_program.h
          const char* doc = R"""(Getter for linear costs.)""";
        } linear_costs;
        // Symbol: drake::solvers::MathematicalProgram::linear_equality_constraints
        struct /* linear_equality_constraints */ {
          // Source: drake/solvers/mathematical_program.h
          const char* doc =
R"""(Getter for linear equality constraints. Note that this only includes
constraints that were added explicitly as LinearEqualityConstraint or
which were added symbolically (and their equality constraint nature
was uncovered). There may be bounding_box_constraints() and
linear_constraints() whose lower bounds also equal their upper bounds.)""";
        } linear_equality_constraints;
        // Symbol: drake::solvers::MathematicalProgram::linear_matrix_inequality_constraints
        struct /* linear_matrix_inequality_constraints */ {
          // Source: drake/solvers/mathematical_program.h
          const char* doc =
R"""(Getter for linear matrix inequality constraints.)""";
        } linear_matrix_inequality_constraints;
        // Symbol: drake::solvers::MathematicalProgram::lorentz_cone_constraints
        struct /* lorentz_cone_constraints */ {
          // Source: drake/solvers/mathematical_program.h
          const char* doc = R"""(Getter for Lorentz cone constraints.)""";
        } lorentz_cone_constraints;
        // Symbol: drake::solvers::MathematicalProgram::num_indeterminates
        struct /* num_indeterminates */ {
          // Source: drake/solvers/mathematical_program.h
          const char* doc =
R"""(Gets the number of indeterminates in the optimization program)""";
        } num_indeterminates;
        // Symbol: drake::solvers::MathematicalProgram::num_vars
        struct /* num_vars */ {
          // Source: drake/solvers/mathematical_program.h
          const char* doc =
R"""(Getter for number of variables in the optimization program)""";
        } num_vars;
        // Symbol: drake::solvers::MathematicalProgram::positive_semidefinite_constraints
        struct /* positive_semidefinite_constraints */ {
          // Source: drake/solvers/mathematical_program.h
          const char* doc =
R"""(Getter for positive semidefinite constraints.)""";
        } positive_semidefinite_constraints;
        // Symbol: drake::solvers::MathematicalProgram::quadratic_constraints
        struct /* quadratic_constraints */ {
          // Source: drake/solvers/mathematical_program.h
          const char* doc = R"""(Getter for quadratic constraints.)""";
        } quadratic_constraints;
        // Symbol: drake::solvers::MathematicalProgram::quadratic_costs
        struct /* quadratic_costs */ {
          // Source: drake/solvers/mathematical_program.h
          const char* doc = R"""(Getter for quadratic costs.)""";
        } quadratic_costs;
        // Symbol: drake::solvers::MathematicalProgram::required_capabilities
        struct /* required_capabilities */ {
          // Source: drake/solvers/mathematical_program.h
          const char* doc =
R"""(Getter for the required capability on the solver, given the
cost/constraint/variable types in the program.)""";
        } required_capabilities;
        // Symbol: drake::solvers::MathematicalProgram::rotated_lorentz_cone_constraints
        struct /* rotated_lorentz_cone_constraints */ {
          // Source: drake/solvers/mathematical_program.h
          const char* doc =
R"""(Getter for rotated Lorentz cone constraints.)""";
        } rotated_lorentz_cone_constraints;
        // Symbol: drake::solvers::MathematicalProgram::solver_options
        struct /* solver_options */ {
          // Source: drake/solvers/mathematical_program.h
          const char* doc =
R"""(Returns the solver options stored inside MathematicalProgram.)""";
        } solver_options;
        // Symbol: drake::solvers::MathematicalProgram::to_string
        struct /* to_string */ {
          // Source: drake/solvers/mathematical_program.h
          const char* doc =
R"""(Returns string representation of this program, listing the decision
variables, costs, and constraints.

Note that by default, we do not require variables to have unique
names. Providing useful variable names and calling
Evaluator∷set_description() to describe the costs and constraints can
dramatically improve the readability of the output. See the tutorial
``debug_mathematical_program.ipynb`` for more information.)""";
        } to_string;
        // Symbol: drake::solvers::MathematicalProgram::visualization_callbacks
        struct /* visualization_callbacks */ {
          // Source: drake/solvers/mathematical_program.h
          const char* doc = R"""(Getter for all callbacks.)""";
        } visualization_callbacks;
      } MathematicalProgram;
      // Symbol: drake::solvers::MathematicalProgramResult
      struct /* MathematicalProgramResult */ {
        // Source: drake/solvers/mathematical_program_result.h
        const char* doc =
R"""(The result returned by MathematicalProgram∷Solve(). It stores the
solvers∷SolutionResult (whether the program is solved to optimality,
detected infeasibility, etc), the optimal value for the decision
variables, the optimal cost, and solver specific details.)""";
        // Symbol: drake::solvers::MathematicalProgramResult::AddSuboptimalSolution
        struct /* AddSuboptimalSolution */ {
          // Source: drake/solvers/mathematical_program_result.h
          const char* doc =
R"""(Adds the suboptimal solution to the result. See solution_pools
"solution pools".

Parameter ``suboptimal_objective``:
    The objective value computed from this suboptimal solution.

Parameter ``suboptimal_x``:
    The values of the decision variables in this suboptimal solution.)""";
        } AddSuboptimalSolution;
        // Symbol: drake::solvers::MathematicalProgramResult::EvalBinding
        struct /* EvalBinding */ {
          // Source: drake/solvers/mathematical_program_result.h
          const char* doc =
R"""(Evaluate a Binding at the solution.

Parameter ``binding``:
    A binding between a constraint/cost and the variables.

Precondition:
    The binding.variables() must be the within the decision variables
    in the MathematicalProgram that generated this
    MathematicalProgramResult.

Precondition:
    The user must have called set_decision_variable_index() function.)""";
        } EvalBinding;
        // Symbol: drake::solvers::MathematicalProgramResult::GetDualSolution
        struct /* GetDualSolution */ {
          // Source: drake/solvers/mathematical_program_result.h
          const char* doc =
R"""(Gets the dual solution associated with a constraint.

For constraints in the form lower <= f(x) <= upper (including linear
inequality, linear equality, bounding box constraints, and general
nonlinear constraints), we interpret the dual variable value as the
"shadow price" of the original problem. Namely if we change the
constraint bound by one unit (each unit is infinitesimally small), the
change of the optimal cost is the value of the dual solution times the
unit. Mathematically dual_solution = ∂optimal_cost / ∂bound.

For a linear equality constraint Ax = b where b ∈ ℝⁿ, the vector of
dual variables has n rows, and dual_solution(i) is the value of the
dual variable for the constraint A(i,:)*x = b(i).

For a linear inequality constraint lower <= A*x <= upper where lower
and upper ∈ ℝⁿ, dual_solution also has n rows. dual_solution(i) is the
value of the dual variable for constraint lower(i) <= A(i,:)*x <=
upper(i). If neither side of the constraint is active, then
dual_solution(i) is 0. If the left hand-side lower(i) <= A(i, :)*x is
active (meaning lower(i) = A(i, :)*x at the solution), then
dual_solution(i) is non-negative (because the objective is to minimize
a cost, increasing the lower bound means the constraint set is
tighter, hence the optimal solution cannot decrease. Thus the shadow
price is non-negative). If the right hand-side A(i, :)*x<=upper(i) is
active (meaning A(i,:)*x=upper(i) at the solution), then
dual_solution(i) is non-positive.

For a bounding box constraint lower <= x <= upper, the interpretation
of the dual solution is the same as the linear inequality constraint.

For a Lorentz cone or rotated Lorentz cone constraint that Ax + b is
in the cone, depending on the solver, the dual solution has different
meanings: 1. If the solver is Gurobi, then the user can only obtain
the dual solution by explicitly setting the options for computing dual
solution.


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    auto constraint = prog.AddLorentzConeConstraint(...);
       GurobiSolver solver;
       // Explicitly tell the solver to compute the dual solution for Lorentz
       // cone or rotated Lorentz cone constraint, check
       // https://docs.gurobi.com/projects/optimizer/en/12.0/reference/parameters.html#qcpdual
       // for more information.
       SolverOptions options;
       options.SetOption(GurobiSolver∷id(), "QCPDual", 1);
       MathematicalProgramResult result = solver.Solve(prog, {}, options);
       Eigen∷VectorXd dual_solution = result.GetDualSolution(constraint);

.. raw:: html

    </details>

The dual solution has size 1, dual_solution(0) is the shadow price for
the constraint z₁² + ... +zₙ² ≤ z₀² for Lorentz cone constraint, and
the shadow price for the constraint z₂² + ... +zₙ² ≤ z₀z₁ for rotated
Lorentz cone constraint, where z is the slack variable representing z
= A*x+b and z in the Lorentz cone/rotated Lorentz cone. 2. For
nonlinear solvers like IPOPT, the dual solution for Lorentz cone
constraint (with EvalType∷kConvex) is the shadow price for z₀ -
sqrt(z₁² + ... +zₙ²) ≥ 0, where z = Ax+b. 3. For other convex conic
solver such as SCS, MOSEK™, CSDP, etc, the dual solution to the
(rotated) Lorentz cone constraint doesn't have the "shadow price"
interpretation, but should lie in the dual cone, and satisfy the KKT
condition. For more information, refer to
https://docs.mosek.com/11.1/capi/prob-def-conic.html#duality-for-conic-optimization
as an explanation.

The interpretation for the dual variable to conic constraint x ∈ K can
be different. Here K is a convex cone, including exponential cone,
power cone, psd cone, etc. When the problem is solved by a convex
solver (like SCS, MOSEK™, CSDP, etc), often it has a dual variable z ∈
K*, where K* is the dual cone. Here the dual variable DOESN'T have the
interpretation of "shadow price", but should satisfy the KKT
condition, while the dual variable stays inside the dual cone.

When K is a psd cone, the returned dual solution is the lower triangle
of the dual symmetric psd matrix. Namely for the primal problem

min trace(C*X) s.t A(X) = b X is psd

the dual is

max b'*y s.t A'(y) - C = Z Z is psd.

We return the lower triangular part of Z. You can call
drake∷math∷ToSymmetricMatrixFromLowerTriangularColumns to get the
matrix Z.)""";
        } GetDualSolution;
        // Symbol: drake::solvers::MathematicalProgramResult::GetInfeasibleConstraintNames
        struct /* GetInfeasibleConstraintNames */ {
          // Source: drake/solvers/mathematical_program_result.h
          const char* doc =
R"""(See get_infeasible_constraints for more information.

Parameter ``prog``:
    The MathematicalProgram that was solved to obtain ``this``
    MathematicalProgramResult.

Parameter ``tolerance``:
    A positive tolerance to check the constraint violation. If no
    tolerance is provided, this method will attempt to obtain the
    constraint tolerance from the solver, or insert a conservative
    default tolerance.

Note: Currently most constraints have the empty string as the
description, so the NiceTypeName of the Constraint is used instead.
Use e.g. ``prog.AddConstraint(x ==
1).evaluator().set_description(str)`` to make this method more
specific/useful.)""";
        } GetInfeasibleConstraintNames;
        // Symbol: drake::solvers::MathematicalProgramResult::GetInfeasibleConstraints
        struct /* GetInfeasibleConstraints */ {
          // Source: drake/solvers/mathematical_program_result.h
          const char* doc =
R"""(See get_infeasible_constraints for more information.

Parameter ``prog``:
    The MathematicalProgram that was solved to obtain ``this``
    MathematicalProgramResult.

Parameter ``tolerance``:
    A positive tolerance to check the constraint violation. If no
    tolerance is provided, this method will attempt to obtain the
    constraint tolerance from the solver, or insert a conservative
    default tolerance.

Returns:
    infeasible_bindings A vector of all infeasible bindings
    (constraints together with the associated variables) at the
    best-effort solution.)""";
        } GetInfeasibleConstraints;
        // Symbol: drake::solvers::MathematicalProgramResult::GetSolution
        struct /* GetSolution */ {
          // Source: drake/solvers/mathematical_program_result.h
          const char* doc_0args =
R"""(Gets the solution of all decision variables.)""";
          // Source: drake/solvers/mathematical_program_result.h
          const char* doc_1args_constEigenMatrixBase =
R"""(Gets the solution of an Eigen matrix of decision variables.

Template parameter ``Derived``:
    An Eigen matrix containing Variable.

Parameter ``var``:
    The decision variables.

Returns:
    The value of the decision variable after solving the problem.)""";
          // Source: drake/solvers/mathematical_program_result.h
          const char* doc_1args_var =
R"""(Gets the solution of a single decision variable.

Parameter ``var``:
    The decision variable.

Returns:
    The value of the decision variable after solving the problem.

Raises:
    RuntimeError if ``var`` is not captured in the mapping
    ``decision_variable_index``, as the input argument of
    set_decision_variable_index().)""";
          // Source: drake/solvers/mathematical_program_result.h
          const char* doc_1args_e =
R"""(Substitutes the value of all decision variables into the Expression.

Parameter ``e``:
    The decision variable.

Returns:
    the Expression that is the result of the substitution.)""";
          // Source: drake/solvers/mathematical_program_result.h
          const char* doc_1args_p =
R"""(Substitutes the value of all decision variables into the coefficients
of the symbolic polynomial.

Parameter ``p``:
    A symbolic polynomial. Its indeterminates can't intersect with the
    set of decision variables of the MathematicalProgram from which
    this result is obtained.

Returns:
    the symbolic∷Polynomial as the result of the substitution.)""";
        } GetSolution;
        // Symbol: drake::solvers::MathematicalProgramResult::GetSuboptimalSolution
        struct /* GetSuboptimalSolution */ {
          // Source: drake/solvers/mathematical_program_result.h
          const char* doc_2args_constEigenMatrixBase_int =
R"""(@name Solution Pools Some solvers (like Gurobi, Cplex, etc) can store
a pool of (suboptimal) solutions for mixed integer programming model.
Gets the suboptimal solution corresponding to a matrix of decision
variables. See solution_pools "solution pools"

Parameter ``var``:
    The decision variables.

Parameter ``solution_number``:
    The index of the sub-optimal solution.

Precondition:
    ``solution_number`` should be in the range [0,
    num_suboptimal_solution()).

Returns:
    The suboptimal values of the decision variables after solving the
    problem.)""";
          // Source: drake/solvers/mathematical_program_result.h
          const char* doc_2args_var_solution_number =
R"""(Gets the suboptimal solution of a decision variable. See
solution_pools "solution pools"

Parameter ``var``:
    The decision variable.

Parameter ``solution_number``:
    The index of the sub-optimal solution.

Precondition:
    ``solution_number`` should be in the range [0,
    num_suboptimal_solution()).

Returns:
    The suboptimal value of the decision variable after solving the
    problem.)""";
        } GetSuboptimalSolution;
        // Symbol: drake::solvers::MathematicalProgramResult::MathematicalProgramResult
        struct /* ctor */ {
          // Source: drake/solvers/mathematical_program_result.h
          const char* doc =
R"""(Constructs the result.

Note:
    The solver_details is set to nullptr.)""";
        } ctor;
        // Symbol: drake::solvers::MathematicalProgramResult::SetSolution
        struct /* SetSolution */ {
          // Source: drake/solvers/mathematical_program_result.h
          const char* doc =
R"""(Resets the solution of a single decision variable that is already
registered with this result.

Raises:
    RuntimeError if ``var`` is not captured in the mapping
    ``decision_variable_index``, as the input argument of
    set_decision_variable_index().)""";
        } SetSolution;
        // Symbol: drake::solvers::MathematicalProgramResult::SetSolverDetailsType
        struct /* SetSolverDetailsType */ {
          // Source: drake/solvers/mathematical_program_result.h
          const char* doc =
R"""((Advanced.) Forces the solver_details to be stored using the given
type ``T``. Typically, only an implementation of SolverInterface will
call this method. If the storage was already typed as T, this is a
no-op. If there were not any solver_details previously, or if it was
of a different type, initializes the storage to a default-constructed
T. Returns a reference to the mutable solver_details object. The
reference remains valid until the next call to this method, or until
this MathematicalProgramResult is destroyed.)""";
        } SetSolverDetailsType;
        // Symbol: drake::solvers::MathematicalProgramResult::get_abstract_solver_details
        struct /* get_abstract_solver_details */ {
          // Source: drake/solvers/mathematical_program_result.h
          const char* doc =
R"""((Advanced.) Gets the type-erased solver details. Most users should use
get_solver_details() instead. Throws an error if the solver_details
has not been set.)""";
        } get_abstract_solver_details;
        // Symbol: drake::solvers::MathematicalProgramResult::get_decision_variable_index
        struct /* get_decision_variable_index */ {
          // Source: drake/solvers/mathematical_program_result.h
          const char* doc = R"""(Gets decision_variable_index.)""";
        } get_decision_variable_index;
        // Symbol: drake::solvers::MathematicalProgramResult::get_optimal_cost
        struct /* get_optimal_cost */ {
          // Source: drake/solvers/mathematical_program_result.h
          const char* doc = R"""(Gets the optimal cost.)""";
        } get_optimal_cost;
        // Symbol: drake::solvers::MathematicalProgramResult::get_solution_result
        struct /* get_solution_result */ {
          // Source: drake/solvers/mathematical_program_result.h
          const char* doc = R"""(Gets SolutionResult.)""";
        } get_solution_result;
        // Symbol: drake::solvers::MathematicalProgramResult::get_solver_details
        struct /* get_solver_details */ {
          // Source: drake/solvers/mathematical_program_result.h
          const char* doc =
R"""(Gets the solver details for the ``Solver`` that solved the program.
Throws an error if the solver_details has not been set.)""";
        } get_solver_details;
        // Symbol: drake::solvers::MathematicalProgramResult::get_solver_id
        struct /* get_solver_id */ {
          // Source: drake/solvers/mathematical_program_result.h
          const char* doc = R"""(Gets the solver ID.)""";
        } get_solver_id;
        // Symbol: drake::solvers::MathematicalProgramResult::get_suboptimal_objective
        struct /* get_suboptimal_objective */ {
          // Source: drake/solvers/mathematical_program_result.h
          const char* doc =
R"""(Gets the suboptimal objective value. See solution_pools "solution
pools".

Parameter ``solution_number``:
    The index of the sub-optimal solution.

Precondition:
    ``solution_number`` should be in the range [0,
    num_suboptimal_solution()).)""";
        } get_suboptimal_objective;
        // Symbol: drake::solvers::MathematicalProgramResult::get_x_val
        struct /* get_x_val */ {
          // Source: drake/solvers/mathematical_program_result.h
          const char* doc = R"""(Gets the decision variable values.)""";
        } get_x_val;
        // Symbol: drake::solvers::MathematicalProgramResult::is_success
        struct /* is_success */ {
          // Source: drake/solvers/mathematical_program_result.h
          const char* doc =
R"""(Returns true if the optimization problem is solved successfully; false
otherwise. For more information on the solution status, the user could
call get_solver_details() to obtain the solver-specific solution
status.)""";
        } is_success;
        // Symbol: drake::solvers::MathematicalProgramResult::num_suboptimal_solution
        struct /* num_suboptimal_solution */ {
          // Source: drake/solvers/mathematical_program_result.h
          const char* doc =
R"""(Number of suboptimal solutions stored inside
MathematicalProgramResult. See solution_pools "solution pools".)""";
        } num_suboptimal_solution;
        // Symbol: drake::solvers::MathematicalProgramResult::set_decision_variable_index
        struct /* set_decision_variable_index */ {
          // Source: drake/solvers/mathematical_program_result.h
          const char* doc =
R"""(Sets decision_variable_index mapping, that maps each decision variable
to its index in the aggregated vector containing all decision
variables in MathematicalProgram. Initialize x_val to NAN.)""";
        } set_decision_variable_index;
        // Symbol: drake::solvers::MathematicalProgramResult::set_dual_solution
        struct /* set_dual_solution */ {
          // Source: drake/solvers/mathematical_program_result.h
          const char* doc =
R"""(Sets the dual solution associated with a given constraint.)""";
        } set_dual_solution;
        // Symbol: drake::solvers::MathematicalProgramResult::set_optimal_cost
        struct /* set_optimal_cost */ {
          // Source: drake/solvers/mathematical_program_result.h
          const char* doc = R"""(Sets the optimal cost.)""";
        } set_optimal_cost;
        // Symbol: drake::solvers::MathematicalProgramResult::set_solution_result
        struct /* set_solution_result */ {
          // Source: drake/solvers/mathematical_program_result.h
          const char* doc = R"""(Sets SolutionResult.)""";
        } set_solution_result;
        // Symbol: drake::solvers::MathematicalProgramResult::set_solver_id
        struct /* set_solver_id */ {
          // Source: drake/solvers/mathematical_program_result.h
          const char* doc = R"""(Sets the solver ID.)""";
        } set_solver_id;
        // Symbol: drake::solvers::MathematicalProgramResult::set_x_val
        struct /* set_x_val */ {
          // Source: drake/solvers/mathematical_program_result.h
          const char* doc = R"""(Sets the decision variable values.)""";
        } set_x_val;
      } MathematicalProgramResult;
      // Symbol: drake::solvers::MatrixXDecisionVariable
      struct /* MatrixXDecisionVariable */ {
        // Source: drake/solvers/decision_variable.h
        const char* doc = R"""()""";
      } MatrixXDecisionVariable;
      // Symbol: drake::solvers::MatrixXIndeterminate
      struct /* MatrixXIndeterminate */ {
        // Source: drake/solvers/indeterminate.h
        const char* doc =
R"""(MatrixXIndeterminate is used as an alias for
Eigen∷Matrix<symbolic∷Variable, Eigen∷Dynamic, Eigen∷Dynamic>.

See also:
    MatrixIndeterminate<int, int>)""";
      } MatrixXIndeterminate;
      // Symbol: drake::solvers::MinimumValueLowerBoundConstraint
      struct /* MinimumValueLowerBoundConstraint */ {
        // Source: drake/solvers/minimum_value_constraint.h
        const char* doc =
R"""(Constrain min(v) >= lb where v=f(x). Namely all elements of the vector
``v`` returned by the user-provided function f(x) to be no smaller
than a specified value ``lb``.

The formulation of the constraint is


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    SmoothOverMax( φ((vᵢ - v_influence)/(v_influence - lb)) / φ(-1) ) ≤ 1

.. raw:: html

    </details>

where vᵢ is the i-th value returned by the user-provided function,
``lb`` is the minimum allowable value. v_influence is the "influence
value" (the value below which an element influences the constraint or,
conversely, the value above which an element is ignored), φ is a
solvers∷MinimumValuePenaltyFunction, and SmoothOverMax(v) is a smooth,
over approximation of max(v) (i.e. SmoothOverMax(v) >= max(v), for all
v). We require that lb < v_influence. The input scaling (vᵢ -
v_influence)/(v_influence - lb) ensures that at the boundary of the
feasible set (when vᵢ == lb), we evaluate the penalty function at -1,
where it is required to have a non-zero gradient. The user-provided
function may return a vector with up to ``max_num_values`` elements.
If it returns a vector with fewer than ``max_num_values`` elements,
the remaining elements are assumed to be greater than the "influence
value".)""";
        // Symbol: drake::solvers::MinimumValueLowerBoundConstraint::MinimumValueLowerBoundConstraint
        struct /* ctor */ {
          // Source: drake/solvers/minimum_value_constraint.h
          const char* doc =
R"""(Constructs a MinimumValueLowerBoundConstraint. min(v) >= lb And we set
ub to infinity in min(v) <= ub.

Parameter ``num_vars``:
    The number of inputs to ``value_function``

Parameter ``minimum_value_lower``:
    The minimum allowed value, lb, for all elements of the vector
    returned by ``value_function``.

Parameter ``influence_value_offset``:
    The difference between the influence value, v_influence, and the
    minimum value, lb (see class documentation). This value must be
    finite and strictly positive, as it is used to scale the values
    returned by ``value_function``. Smaller values may decrease the
    amount of computation required for each constraint evaluation if
    ``value_function`` can quickly determine that some elements will
    be larger than the influence value and skip the computation
    associated with those elements.

Parameter ``max_num_values``:
    The maximum number of elements in the vector returned by
    ``value_function``.

Parameter ``value_function``:
    User-provided function that takes a ``num_vars``-element vector
    and the influence distance as inputs and returns a vector with up
    to ``max_num_values`` elements. The function can omit from the
    return vector any elements larger than the provided influence
    distance.

Parameter ``value_function_double``:
    Optional user-provide function that computes the same values as
    ``value_function`` but for double rather than AutoDiffXd. If
    omitted, ``value_function`` will be called (and the gradients
    discarded) when this constraint is evaluated for doubles.

Precondition:
    ``value_function_double(ExtractValue(x), v_influence) ==
    ExtractValue(value_function(x, v_influence))`` for all x.

Precondition:
    ``value_function(x).size() <= max_num_values`` for all x.

Raises:
    RuntimeError if influence_value_offset = ∞.

Raises:
    RuntimeError if influence_value_offset ≤ 0.)""";
        } ctor;
        // Symbol: drake::solvers::MinimumValueLowerBoundConstraint::influence_value
        struct /* influence_value */ {
          // Source: drake/solvers/minimum_value_constraint.h
          const char* doc = R"""(Getter for the influence value.)""";
        } influence_value;
        // Symbol: drake::solvers::MinimumValueLowerBoundConstraint::max_num_values
        struct /* max_num_values */ {
          // Source: drake/solvers/minimum_value_constraint.h
          const char* doc =
R"""(Getter for maximum number of values expected from value_function.)""";
        } max_num_values;
        // Symbol: drake::solvers::MinimumValueLowerBoundConstraint::minimum_value_lower
        struct /* minimum_value_lower */ {
          // Source: drake/solvers/minimum_value_constraint.h
          const char* doc =
R"""(Getter for the lower bound on the minimum value.)""";
        } minimum_value_lower;
        // Symbol: drake::solvers::MinimumValueLowerBoundConstraint::set_penalty_function
        struct /* set_penalty_function */ {
          // Source: drake/solvers/minimum_value_constraint.h
          const char* doc = R"""(Setter for the penalty function.)""";
        } set_penalty_function;
      } MinimumValueLowerBoundConstraint;
      // Symbol: drake::solvers::MinimumValuePenaltyFunction
      struct /* MinimumValuePenaltyFunction */ {
        // Source: drake/solvers/minimum_value_constraint.h
        const char* doc =
R"""(Computes the penalty function φ(x) and its derivatives dφ(x)/dx. Valid
penalty functions must meet the following criteria:

1.     φ(x) ≥ 0 ∀ x ∈ ℝ.
2. dφ(x)/dx ≤ 0 ∀ x ∈ ℝ.
3.     φ(x) = 0 ∀ x ≥ 0.
4. dφ(x)/dx < 0 ∀ x < 0.

If ``dpenalty_dx`` is nullptr, the function should only compute φ(x).)""";
      } MinimumValuePenaltyFunction;
      // Symbol: drake::solvers::MinimumValueUpperBoundConstraint
      struct /* MinimumValueUpperBoundConstraint */ {
        // Source: drake/solvers/minimum_value_constraint.h
        const char* doc =
R"""(Constrain min(v) <= ub where v=f(x). Namely at least one element of
the vector ``v`` returned by the user-provided function f(x) to be no
larger than a specified value ``ub``.

The formulation of the constraint is


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    SmoothUnderMax( φ((vᵢ - v_influence)/(v_influence - ub)) / φ(-1) ) ≥ 1

.. raw:: html

    </details>

where vᵢ is the i-th value returned by the user-provided function,
``ub`` is the upper bound for the min(v). (Note that ``ub`` is NOT the
upper bound of ``v``). v_influence is the "influence value" (the value
below which an element influences the constraint or, conversely, the
value above which an element is ignored), φ is a
solvers∷MinimumValuePenaltyFunction. SmoothUnderMax(x) is a smooth,
under approximation of max(v) (i.e. SmoothUnderMax(v) <= max(v) for
all v). We require that ub < v_influence. The input scaling (vᵢ -
v_influence)/(v_influence - ub) ensures that at the boundary of the
feasible set (when vᵢ == ub), we evaluate the penalty function at -1,
where it is required to have a non-zero gradient. The user-provided
function may return a vector with up to ``max_num_values`` elements.
If it returns a vector with fewer than ``max_num_values`` elements,
the remaining elements are assumed to be greater than the "influence
value".)""";
        // Symbol: drake::solvers::MinimumValueUpperBoundConstraint::MinimumValueUpperBoundConstraint
        struct /* ctor */ {
          // Source: drake/solvers/minimum_value_constraint.h
          const char* doc =
R"""(Constructs a MinimumValueUpperBoundConstraint. min(v) <= ub

Parameter ``num_vars``:
    The number of inputs to ``value_function``

Parameter ``minimum_value_upper``:
    The upper bound on the minimum allowed value for all elements of
    the vector returned by ``value_function``, namely
    min(value_function(x)) <= minimum_value_upper

Parameter ``influence_value_offset``:
    The difference between the influence value, v_influence, and
    minimum_value_upper. This value must be finite and strictly
    positive, as it is used to scale the values returned by
    ``value_function``. Larger values may increase the possibility of
    finding a solution to the constraint. With a small v_influence,
    the value_function will ignore the entries with value less than
    v_influence. While it is possible that by changing x, that value
    (that currently been ignored) can decrease to below ub with a
    different x, by using a small v_influence, the gradient of that
    entry is never considered if the entry is ignored. We strongly
    suggest using a larger ``v_influence`` compared to the one used in
    MinimumValueConstraint when constraining min(v) >= lb.

Parameter ``max_num_values``:
    The maximum number of elements in the vector returned by
    ``value_function``.

Parameter ``value_function``:
    User-provided function that takes a ``num_vars``-element vector
    and the influence distance as inputs and returns a vector with up
    to ``max_num_values`` elements. The function can omit from the
    return vector any elements larger than the provided influence
    distance.

Parameter ``value_function_double``:
    Optional user-provide function that computes the same values as
    ``value_function`` but for double rather than AutoDiffXd. If
    omitted, ``value_function`` will be called (and the gradients
    discarded) when this constraint is evaluated for doubles.

Precondition:
    ``value_function_double(ExtractValue(x), v_influence) ==
    ExtractValue(value_function(x, v_influence))`` for all x.

Precondition:
    ``value_function(x).size() <= max_num_values`` for all x.

Raises:
    RuntimeError if influence_value_offset = ∞.

Raises:
    RuntimeError if influence_value_offset ≤ 0.)""";
        } ctor;
        // Symbol: drake::solvers::MinimumValueUpperBoundConstraint::influence_value
        struct /* influence_value */ {
          // Source: drake/solvers/minimum_value_constraint.h
          const char* doc = R"""(Getter for the influence value.)""";
        } influence_value;
        // Symbol: drake::solvers::MinimumValueUpperBoundConstraint::max_num_values
        struct /* max_num_values */ {
          // Source: drake/solvers/minimum_value_constraint.h
          const char* doc =
R"""(Getter for maximum number of values expected from value_function.)""";
        } max_num_values;
        // Symbol: drake::solvers::MinimumValueUpperBoundConstraint::minimum_value_upper
        struct /* minimum_value_upper */ {
          // Source: drake/solvers/minimum_value_constraint.h
          const char* doc =
R"""(Getter for the upper bound on the minimum value.)""";
        } minimum_value_upper;
        // Symbol: drake::solvers::MinimumValueUpperBoundConstraint::set_penalty_function
        struct /* set_penalty_function */ {
          // Source: drake/solvers/minimum_value_constraint.h
          const char* doc = R"""(Setter for the penalty function.)""";
        } set_penalty_function;
      } MinimumValueUpperBoundConstraint;
      // Symbol: drake::solvers::MixedIntegerBranchAndBound
      struct /* MixedIntegerBranchAndBound */ {
        // Source: drake/solvers/branch_and_bound.h
        const char* doc =
R"""(Given a mixed-integer optimization problem (MIP) (or more accurately,
mixed binary problem), solve this problem through branch-and-bound
process. We will first replace all the binary variables with
continuous variables, and relax the integral constraint on the binary
variables z ∈ {0, 1} with continuous constraints 0 ≤ z ≤ 1. In the
subsequent steps, at each node of the tree, we will fix some binary
variables to either 0 or 1, and solve the rest of the variables.
Notice that we will create a new set of variables in the
branch-and-bound process, since we need to replace the binary
variables with continuous variables.)""";
        // Symbol: drake::solvers::MixedIntegerBranchAndBound::GetNewVariable
        struct /* GetNewVariable */ {
          // Source: drake/solvers/branch_and_bound.h
          const char* doc =
R"""(Given an old variable in the original mixed-integer program, return
the corresponding new variable in the branch-and-bound process.

Parameter ``old_variable``:
    A variable in the original mixed-integer program.

Returns ``new_variable``:
    The corresponding variable in the branch-and-bound procedure.

Precondition:
    old_variable is a variable in the mixed-integer program, passed in
    the constructor of this MixedIntegerBranchAndBound.

Raises:
    RuntimeError if the pre-condition fails.)""";
        } GetNewVariable;
        // Symbol: drake::solvers::MixedIntegerBranchAndBound::GetNewVariables
        struct /* GetNewVariables */ {
          // Source: drake/solvers/branch_and_bound.h
          const char* doc =
R"""(Given a matrix of old variables in the original mixed-integer program,
return a matrix of corresponding new variables in the branch-and-bound
process.

Parameter ``old_variables``:
    Variables in the original mixed-integer program.

Returns ``new_variables``:
    The corresponding variables in the branch-and-bound procedure.)""";
        } GetNewVariables;
        // Symbol: drake::solvers::MixedIntegerBranchAndBound::GetOptimalCost
        struct /* GetOptimalCost */ {
          // Source: drake/solvers/branch_and_bound.h
          const char* doc = R"""(Get the optimal cost.)""";
        } GetOptimalCost;
        // Symbol: drake::solvers::MixedIntegerBranchAndBound::GetSolution
        struct /* GetSolution */ {
          // Source: drake/solvers/branch_and_bound.h
          const char* doc_2args_mip_var_nth_best_solution =
R"""(Get the n'th best integral solution for a variable. The best solutions
are sorted in the ascending order based on their costs. Each solution
is found in a separate node in the branch-and-bound tree, so the
values of the binary variables are different in each solution.

Parameter ``mip_var``:
    A variable in the original MIP.

Parameter ``nth_best_solution``:
    The index of the best integral solution.

Precondition:
    ``mip_var`` is a variable in the original MIP.

Precondition:
    ``nth_best_solution`` is between 0 and solutions().size().

Raises:
    RuntimeError if the preconditions are not satisfied.)""";
          // Source: drake/solvers/branch_and_bound.h
          const char* doc_2args_constEigenMatrixBase_int =
R"""(Get the n'th best integral solution for some variables. The best
solutions are sorted in the ascending order based on their costs. Each
solution is found in a separate node in the branch-and-bound tree, so

Parameter ``mip_vars``:
    Variables in the original MIP.

Parameter ``nth_best_solution``:
    The index of the best integral solution.

Precondition:
    ``mip_vars`` are variables in the original MIP.

Precondition:
    ``nth_best_solution`` is between 0 and solutions().size().

Raises:
    RuntimeError if the preconditions are not satisfied.)""";
        } GetSolution;
        // Symbol: drake::solvers::MixedIntegerBranchAndBound::GetSubOptimalCost
        struct /* GetSubOptimalCost */ {
          // Source: drake/solvers/branch_and_bound.h
          const char* doc =
R"""(Get the n'th sub-optimal cost. The costs are sorted in the ascending
order. The sub-optimal costs do not include the optimal cost.

Parameter ``nth_suboptimal_cost``:
    The n'th sub-optimal cost.

Precondition:
    ``nth_suboptimal_cost`` is between 0 and solutions().size() - 1.

Raises:
    RuntimeError if the precondition is not satisfied.)""";
        } GetSubOptimalCost;
        // Symbol: drake::solvers::MixedIntegerBranchAndBound::IsLeafNodeFathomed
        struct /* IsLeafNodeFathomed */ {
          // Source: drake/solvers/branch_and_bound.h
          const char* doc =
R"""(If a leaf node is fathomed, then there is no need to branch on this
node any more. A leaf node is fathomed is any of the following
conditions are satisfied:

1. The optimization problem in the node is infeasible.
2. The optimal cost of the node is larger than the best upper bound.
3. The optimal solution to the node satisfies all the integral constraints.
4. All binary variables are fixed to either 0 or 1 in this node.

Parameter ``leaf_node``:
    A leaf node to check if it is fathomed.

Precondition:
    The node should be a leaf node.

Raises:
    RuntimeError if the precondition is not satisfied.)""";
        } IsLeafNodeFathomed;
        // Symbol: drake::solvers::MixedIntegerBranchAndBound::MixedIntegerBranchAndBound
        struct /* ctor */ {
          // Source: drake/solvers/branch_and_bound.h
          const char* doc =
R"""(Construct a branch-and-bound tree from a mixed-integer optimization
program.

Parameter ``prog``:
    A mixed-integer optimization program.

Parameter ``solver_id``:
    The ID of the solver for the optimization.)""";
        } ctor;
        // Symbol: drake::solvers::MixedIntegerBranchAndBound::NodeCallbackFun
        struct /* NodeCallbackFun */ {
          // Source: drake/solvers/branch_and_bound.h
          const char* doc =
R"""(The function signature for user defined node callback function.)""";
        } NodeCallbackFun;
        // Symbol: drake::solvers::MixedIntegerBranchAndBound::NodeSelectFun
        struct /* NodeSelectFun */ {
          // Source: drake/solvers/branch_and_bound.h
          const char* doc =
R"""(The function signature for the user defined method to pick a branching
node or a branching variable.)""";
        } NodeSelectFun;
        // Symbol: drake::solvers::MixedIntegerBranchAndBound::NodeSelectionMethod
        struct /* NodeSelectionMethod */ {
          // Source: drake/solvers/branch_and_bound.h
          const char* doc =
R"""(Different methods to pick a branching node.)""";
          // Symbol: drake::solvers::MixedIntegerBranchAndBound::NodeSelectionMethod::kDepthFirst
          struct /* kDepthFirst */ {
            // Source: drake/solvers/branch_and_bound.h
            const char* doc =
R"""(Pick the node with the most binary variables fixed.)""";
          } kDepthFirst;
          // Symbol: drake::solvers::MixedIntegerBranchAndBound::NodeSelectionMethod::kMinLowerBound
          struct /* kMinLowerBound */ {
            // Source: drake/solvers/branch_and_bound.h
            const char* doc =
R"""(Pick the node with the smallest optimal cost.)""";
          } kMinLowerBound;
          // Symbol: drake::solvers::MixedIntegerBranchAndBound::NodeSelectionMethod::kUserDefined
          struct /* kUserDefined */ {
            // Source: drake/solvers/branch_and_bound.h
            const char* doc = R"""(User defined.)""";
          } kUserDefined;
        } NodeSelectionMethod;
        // Symbol: drake::solvers::MixedIntegerBranchAndBound::Options
        struct /* Options */ {
          // Source: drake/solvers/branch_and_bound.h
          const char* doc =
R"""(Configuration settings for the MixedIntegerBranchAndBound constructor.)""";
          // Symbol: drake::solvers::MixedIntegerBranchAndBound::Options::Options
          struct /* ctor */ {
            // Source: drake/solvers/branch_and_bound.h
            const char* doc = R"""()""";
          } ctor;
          // Symbol: drake::solvers::MixedIntegerBranchAndBound::Options::Serialize
          struct /* Serialize */ {
            // Source: drake/solvers/branch_and_bound.h
            const char* doc =
R"""(Passes this object to an Archive. Refer to yaml_serialization "YAML
Serialization" for background.)""";
          } Serialize;
          // Symbol: drake::solvers::MixedIntegerBranchAndBound::Options::max_explored_nodes
          struct /* max_explored_nodes */ {
            // Source: drake/solvers/branch_and_bound.h
            const char* doc =
R"""(The maximal number of explored nodes in the tree. The branch and bound
process will terminate if the tree has explored this number of nodes.
max_explored_nodes <= 0 means that we don't put an upper bound on the
number of explored nodes.)""";
          } max_explored_nodes;
          auto Serialize__fields() const {
            return std::array{
              std::make_pair("max_explored_nodes", max_explored_nodes.doc),
            };
          }
        } Options;
        // Symbol: drake::solvers::MixedIntegerBranchAndBound::SetNodeSelectionMethod
        struct /* SetNodeSelectionMethod */ {
          // Source: drake/solvers/branch_and_bound.h
          const char* doc =
R"""(The user can choose the method to pick a node for branching. We
provide options such as "depth first" or "min lower bound".

Parameter ``node_selection_method``:
    The option to pick a node. If the option is
    NodeSelectionMethod∷kUserDefined, then the user should also
    provide the method to pick a node through
    SetUserDefinedNodeSelectionFunction.)""";
        } SetNodeSelectionMethod;
        // Symbol: drake::solvers::MixedIntegerBranchAndBound::SetSearchIntegralSolutionByRounding
        struct /* SetSearchIntegralSolutionByRounding */ {
          // Source: drake/solvers/branch_and_bound.h
          const char* doc =
R"""(Set the flag to true if the user wants to search an integral solution
in each node, after the optimization problem in that node is solved.
The program can search for an integral solution based on the solution
to the optimization program in the node, by rounding the binary
variables to the nearest integer value, and solve for the continuous
variables. If a solution is obtained in this new program, then this
solution is an integral solution to the mixed-integer program.)""";
        } SetSearchIntegralSolutionByRounding;
        // Symbol: drake::solvers::MixedIntegerBranchAndBound::SetUserDefinedNodeCallbackFunction
        struct /* SetUserDefinedNodeCallbackFunction */ {
          // Source: drake/solvers/branch_and_bound.h
          const char* doc =
R"""(The user can set a defined callback function in each node. This
function is called after the optimization is solved in each node.)""";
        } SetUserDefinedNodeCallbackFunction;
        // Symbol: drake::solvers::MixedIntegerBranchAndBound::SetUserDefinedNodeSelectionFunction
        struct /* SetUserDefinedNodeSelectionFunction */ {
          // Source: drake/solvers/branch_and_bound.h
          const char* doc =
R"""(Set the user-defined method to pick the branching node. This method is
used if the user calls
SetNodeSelectionMethod(NodeSelectionMethod∷kUserDefined).

For example, if the user has defined a function LeftMostNode that
would return the left-most unfathomed node in the tree, then the user
could do


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    MixedIntegerBranchAndBoundNode* LeftMostNodeInSubTree(
        const MixedIntegerBranchAndBound& branch_and_bound,
        const MixedIntegerBranchAndBoundNode& subtree_root) {
      // Starting from the subtree root, find the left most leaf node that is
    not fathomed.
      blah
    }
    
    MixedIntegerBranchAndBound bnb(...);
    bnb.SetNodeSelectionMethod(
        MixedIntegerBranchAndBound∷NodeSelectionMethod∷kUserDefined);
    // Use a lambda function as the NodeSelectionFun
    bnb->SetUserDefinedNodeSelectionFunction([](
        const MixedIntegerBranchAndBound& branch_and_bound) {
      return LeftMostNodeInSubTree(branch_and_bound,
    *(branch_and_bound.root()));

.. raw:: html

    </details>

A more detailed example can be found in
solvers/test/branch_and_bound_test.cc in
TestSetUserDefinedNodeSelectionFunction.

Note:
    The user defined function should pick an un-fathomed leaf node for
    branching.

Raises:
    RuntimeError if the node is not a leaf node, or it is fathomed.)""";
        } SetUserDefinedNodeSelectionFunction;
        // Symbol: drake::solvers::MixedIntegerBranchAndBound::SetUserDefinedVariableSelectionFunction
        struct /* SetUserDefinedVariableSelectionFunction */ {
          // Source: drake/solvers/branch_and_bound.h
          const char* doc =
R"""(Set the user-defined method to pick the branching variable. This
method is used if the user calls
SetVariableSelectionMethod(VariableSelectionMethod∷kUserDefined).

For example, if the user has defined a function FirstVariable, that
would return the first un-fixed binary variable in this branch as


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    SymbolicVariable* FirstVariable(const MixedIntegerBranchAndBoundNode& node)
    {
      return node.remaining_binary_variables().begin();
    }

.. raw:: html

    </details>

The user can then set the branch-and-bound to use this function to
select the branching variable as


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    MixedIntegerBranchAndBound bnb(...);
    bnb.SetVariableSelectionMethod(
        MixedIntegerBranchAndBound:VariableSelectionMethod∷kUserDefined);
    // Set VariableSelectFun by using a function pointer.
    bnb.SetUserDefinedVariableSelectionFunction(FirstVariable);

.. raw:: html

    </details>)""";
        } SetUserDefinedVariableSelectionFunction;
        // Symbol: drake::solvers::MixedIntegerBranchAndBound::SetVariableSelectionMethod
        struct /* SetVariableSelectionMethod */ {
          // Source: drake/solvers/branch_and_bound.h
          const char* doc =
R"""(The user can choose the method to pick a variable for branching in
each node. We provide options such as "most ambivalent" or "least
ambivalent".

Parameter ``variable_selection_method``:
    The option to pick a variable. If the option is
    VariableSelectionMethod∷kUserDefined, then the user should also
    provide the method to pick a variable through
    SetUserDefinedVariableSelectionFunction(...).)""";
        } SetVariableSelectionMethod;
        // Symbol: drake::solvers::MixedIntegerBranchAndBound::Solve
        struct /* Solve */ {
          // Source: drake/solvers/branch_and_bound.h
          const char* doc =
R"""(Solve the mixed-integer problem (MIP) through a branch and bound
process.

Returns ``solution_result``:
    If solution_result=SolutionResult∷kSolutionFound, then the best
    solutions are stored inside solutions(). The user can access the
    value of each variable(s) through GetSolution(...). If
    solution_result=SolutionResult∷kInfeasibleConstraints, then the
    mixed-integer problem is primal infeasible. If
    solution_result=SolutionResult∷kUnbounded, then the mixed-integer
    problem is primal unbounded.)""";
        } Solve;
        // Symbol: drake::solvers::MixedIntegerBranchAndBound::VariableSelectFun
        struct /* VariableSelectFun */ {
          // Source: drake/solvers/branch_and_bound.h
          const char* doc = R"""()""";
        } VariableSelectFun;
        // Symbol: drake::solvers::MixedIntegerBranchAndBound::VariableSelectionMethod
        struct /* VariableSelectionMethod */ {
          // Source: drake/solvers/branch_and_bound.h
          const char* doc =
R"""(Different methods to pick a branching variable.)""";
          // Symbol: drake::solvers::MixedIntegerBranchAndBound::VariableSelectionMethod::kLeastAmbivalent
          struct /* kLeastAmbivalent */ {
            // Source: drake/solvers/branch_and_bound.h
            const char* doc =
R"""(Pick the variable whose value is closest to 0 or 1.)""";
          } kLeastAmbivalent;
          // Symbol: drake::solvers::MixedIntegerBranchAndBound::VariableSelectionMethod::kMostAmbivalent
          struct /* kMostAmbivalent */ {
            // Source: drake/solvers/branch_and_bound.h
            const char* doc =
R"""(Pick the variable whose value is closest to 0.5)""";
          } kMostAmbivalent;
          // Symbol: drake::solvers::MixedIntegerBranchAndBound::VariableSelectionMethod::kUserDefined
          struct /* kUserDefined */ {
            // Source: drake/solvers/branch_and_bound.h
            const char* doc = R"""(User defined.)""";
          } kUserDefined;
        } VariableSelectionMethod;
        // Symbol: drake::solvers::MixedIntegerBranchAndBound::absolute_gap_tol
        struct /* absolute_gap_tol */ {
          // Source: drake/solvers/branch_and_bound.h
          const char* doc = R"""(Getter for the absolute gap tolerance.)""";
        } absolute_gap_tol;
        // Symbol: drake::solvers::MixedIntegerBranchAndBound::best_lower_bound
        struct /* best_lower_bound */ {
          // Source: drake/solvers/branch_and_bound.h
          const char* doc = R"""(Getter for the best lower bound.)""";
        } best_lower_bound;
        // Symbol: drake::solvers::MixedIntegerBranchAndBound::best_upper_bound
        struct /* best_upper_bound */ {
          // Source: drake/solvers/branch_and_bound.h
          const char* doc = R"""(Getter for the best upper bound.)""";
        } best_upper_bound;
        // Symbol: drake::solvers::MixedIntegerBranchAndBound::relative_gap_tol
        struct /* relative_gap_tol */ {
          // Source: drake/solvers/branch_and_bound.h
          const char* doc = R"""(Geeter for the relative gap tolerance.)""";
        } relative_gap_tol;
        // Symbol: drake::solvers::MixedIntegerBranchAndBound::root
        struct /* root */ {
          // Source: drake/solvers/branch_and_bound.h
          const char* doc =
R"""(Getter for the root node. Note that this is aliased for the lifetime
of this object.)""";
        } root;
        // Symbol: drake::solvers::MixedIntegerBranchAndBound::set_absolute_gap_tol
        struct /* set_absolute_gap_tol */ {
          // Source: drake/solvers/branch_and_bound.h
          const char* doc =
R"""(Setter for the absolute gap tolerance. The branch-and-bound will
terminate if its difference between its best upper bound and best
lower bound is below this gap tolerance.)""";
        } set_absolute_gap_tol;
        // Symbol: drake::solvers::MixedIntegerBranchAndBound::set_relative_gap_tol
        struct /* set_relative_gap_tol */ {
          // Source: drake/solvers/branch_and_bound.h
          const char* doc =
R"""(Setter for the relative gap tolerance. The branch-and-bound will
terminate if (best_upper_bound() - best_lower_bound()) /
abs(best_lower_bound()) is smaller than this tolerance.)""";
        } set_relative_gap_tol;
        // Symbol: drake::solvers::MixedIntegerBranchAndBound::solutions
        struct /* solutions */ {
          // Source: drake/solvers/branch_and_bound.h
          const char* doc =
R"""(Getter for the solutions. Returns a list of solutions, together with
the costs evaluated at the solutions. The solutions are sorted in the
ascending order based on the cost.)""";
        } solutions;
      } MixedIntegerBranchAndBound;
      // Symbol: drake::solvers::MixedIntegerBranchAndBoundNode
      struct /* MixedIntegerBranchAndBoundNode */ {
        // Source: drake/solvers/branch_and_bound.h
        const char* doc =
R"""(A node in the branch-and-bound (bnb) tree. The whole branch-and-bound
tree solves the mixed-integer problem min f(x) (1) s.t g(x) ≤ 0 z ∈
{0, 1} where the binary variables z are a subset of the decision
variables x. In this node, we will fix some binary variables to either
0 and 1, and relax the rest of the binary variables to continuous
variables between 0 and 1. Namely we will solve the following problem
with all variables being continuous min f(x) (2) s.t g(x) ≤ 0 z_fixed
= b_fixed 0 ≤ z_relaxed ≤ 1 where z_fixed, z_relaxed is a partition of
the original binary variables z. z_fixed is the fixed binary
variables, z_relaxed is the relaxed binary variables. b_fixed is a
vector containing the assigned values of the fixed binary variables
z_fixed, b_fixed only contains value either 0 or 1.

Each node is created from its parent node, by fixing one binary
variable to either 0 or 1.)""";
        // Symbol: drake::solvers::MixedIntegerBranchAndBoundNode::Branch
        struct /* Branch */ {
          // Source: drake/solvers/branch_and_bound.h
          const char* doc =
R"""(Branches on ``binary_variable``, and creates two child nodes. In the
left child node, the binary variable is fixed to 0. In the right node,
the binary variable is fixed to 1. Solves the optimization program in
each child node.

Parameter ``binary_variable``:
    This binary variable is fixed to either 0 or 1 in the child node.

Precondition:
    binary_variable is in remaining_binary_variables_;

Raises:
    RuntimeError if the preconditions are not met.)""";
        } Branch;
        // Symbol: drake::solvers::MixedIntegerBranchAndBoundNode::ConstructRootNode
        struct /* ConstructRootNode */ {
          // Source: drake/solvers/branch_and_bound.h
          const char* doc =
R"""(Construct the root node from an optimization program. For the
mixed-integer optimization program min f(x) (1) s.t g(x) ≤ 0 z ∈ {0,
1} we will construct a root node for this mixed-integer program. In
the root node, it enforces all the costs and constraints in the
original program, except the binary constraint z ∈ {0, 1}. Instead, it
enforces the relaxed constraint to 0 ≤ z ≤ 1. So the root node
contains the program min f(x) (2) s.t g(x) ≤ 0 0 ≤ z ≤ 1 This
optimization program is solved during the node construction.

Parameter ``prog``:
    The mixed-integer optimization program (1) in the documentation
    above.

Parameter ``solver_id``:
    The ID of the solver for the optimization program.

Returns ``(node``:
    , map_old_vars_to_new_vars) node is the root node of the tree,
    that contains the optimization program (2) in the documentation
    above. This root node has no parent. We also need to recreate new
    decision variables in the root node, from the original
    optimization program (1), since the binary variables will be
    converted to continuous variables in (2). We thus return the map
    from the old variables to the new variables.

Precondition:
    prog should contain binary variables.

Precondition:
    solver_id can be either Gurobi or Scs.

Raises:
    RuntimeError if the preconditions are not met.)""";
        } ConstructRootNode;
        // Symbol: drake::solvers::MixedIntegerBranchAndBoundNode::IsLeaf
        struct /* IsLeaf */ {
          // Source: drake/solvers/branch_and_bound.h
          const char* doc =
R"""(Determine if a node is a leaf or not. A leaf node has no child nodes.)""";
        } IsLeaf;
        // Symbol: drake::solvers::MixedIntegerBranchAndBoundNode::IsRoot
        struct /* IsRoot */ {
          // Source: drake/solvers/branch_and_bound.h
          const char* doc =
R"""(Returns true if a node is the root. A root node has no parent.)""";
        } IsRoot;
        // Symbol: drake::solvers::MixedIntegerBranchAndBoundNode::MixedIntegerBranchAndBoundNode
        struct /* ctor */ {
          // Source: drake/solvers/branch_and_bound.h
          const char* doc = R"""()""";
        } ctor;
        // Symbol: drake::solvers::MixedIntegerBranchAndBoundNode::NumExploredNodesInSubtree
        struct /* NumExploredNodesInSubtree */ {
          // Source: drake/solvers/branch_and_bound.h
          const char* doc =
R"""(Returns the total number of explored nodes in the subtree (including
this node if it has been explored).)""";
        } NumExploredNodesInSubtree;
        // Symbol: drake::solvers::MixedIntegerBranchAndBoundNode::fixed_binary_value
        struct /* fixed_binary_value */ {
          // Source: drake/solvers/branch_and_bound.h
          const char* doc =
R"""(Getter for the value of the binary variable, which was not fixed in
the parent node, but is fixed to either 0 or 1 in this node.)""";
        } fixed_binary_value;
        // Symbol: drake::solvers::MixedIntegerBranchAndBoundNode::fixed_binary_variable
        struct /* fixed_binary_variable */ {
          // Source: drake/solvers/branch_and_bound.h
          const char* doc =
R"""(Getter for the binary variable, whose value was not fixed in the
parent node, but is fixed to either 0 or 1 in this node.)""";
        } fixed_binary_variable;
        // Symbol: drake::solvers::MixedIntegerBranchAndBoundNode::is_explored
        struct /* is_explored */ {
          // Source: drake/solvers/branch_and_bound.h
          const char* doc =
R"""(If the mathematical program in this node has been solved and the
result is stored inside this node, then we say this node has been
explored.)""";
        } is_explored;
        // Symbol: drake::solvers::MixedIntegerBranchAndBoundNode::left_child
        struct /* left_child */ {
          // Source: drake/solvers/branch_and_bound.h
          const char* doc = R"""(Getter for the left child.)""";
        } left_child;
        // Symbol: drake::solvers::MixedIntegerBranchAndBoundNode::mutable_left_child
        struct /* mutable_left_child */ {
          // Source: drake/solvers/branch_and_bound.h
          const char* doc = R"""(Getter for the mutable left child.)""";
        } mutable_left_child;
        // Symbol: drake::solvers::MixedIntegerBranchAndBoundNode::mutable_parent
        struct /* mutable_parent */ {
          // Source: drake/solvers/branch_and_bound.h
          const char* doc = R"""(Getter for the mutable parent node.)""";
        } mutable_parent;
        // Symbol: drake::solvers::MixedIntegerBranchAndBoundNode::mutable_right_child
        struct /* mutable_right_child */ {
          // Source: drake/solvers/branch_and_bound.h
          const char* doc = R"""(Getter for the mutable right child.)""";
        } mutable_right_child;
        // Symbol: drake::solvers::MixedIntegerBranchAndBoundNode::optimal_solution_is_integral
        struct /* optimal_solution_is_integral */ {
          // Source: drake/solvers/branch_and_bound.h
          const char* doc =
R"""(Getter for optimal_solution_is_integral.

Precondition:
    The optimization problem is solved successfully.

Raises:
    RuntimeError if the precondition is not satisfied.)""";
        } optimal_solution_is_integral;
        // Symbol: drake::solvers::MixedIntegerBranchAndBoundNode::parent
        struct /* parent */ {
          // Source: drake/solvers/branch_and_bound.h
          const char* doc = R"""(Getter for the parent node.)""";
        } parent;
        // Symbol: drake::solvers::MixedIntegerBranchAndBoundNode::prog
        struct /* prog */ {
          // Source: drake/solvers/branch_and_bound.h
          const char* doc = R"""(Getter for the mathematical program.)""";
        } prog;
        // Symbol: drake::solvers::MixedIntegerBranchAndBoundNode::prog_result
        struct /* prog_result */ {
          // Source: drake/solvers/branch_and_bound.h
          const char* doc =
R"""(Getter for the mathematical program result.)""";
        } prog_result;
        // Symbol: drake::solvers::MixedIntegerBranchAndBoundNode::remaining_binary_variables
        struct /* remaining_binary_variables */ {
          // Source: drake/solvers/branch_and_bound.h
          const char* doc =
R"""(Getter for the remaining binary variables in this node.)""";
        } remaining_binary_variables;
        // Symbol: drake::solvers::MixedIntegerBranchAndBoundNode::right_child
        struct /* right_child */ {
          // Source: drake/solvers/branch_and_bound.h
          const char* doc = R"""(Getter for the right child.)""";
        } right_child;
        // Symbol: drake::solvers::MixedIntegerBranchAndBoundNode::solution_result
        struct /* solution_result */ {
          // Source: drake/solvers/branch_and_bound.h
          const char* doc =
R"""(Getter for the solution result when solving the optimization program.)""";
        } solution_result;
        // Symbol: drake::solvers::MixedIntegerBranchAndBoundNode::solver_id
        struct /* solver_id */ {
          // Source: drake/solvers/branch_and_bound.h
          const char* doc = R"""(Getter for solver id.)""";
        } solver_id;
      } MixedIntegerBranchAndBoundNode;
      // Symbol: drake::solvers::MixedIntegerRotationConstraintGenerator
      struct /* MixedIntegerRotationConstraintGenerator */ {
        // Source: drake/solvers/mixed_integer_rotation_constraint.h
        const char* doc =
R"""(We relax the non-convex SO(3) constraint on rotation matrix R to
mixed-integer linear constraints. The formulation of these constraints
are described in Global Inverse Kinematics via Mixed-integer Convex
Optimization by Hongkai Dai, Gregory Izatt and Russ Tedrake, ISRR,
2017

The SO(3) constraint on a rotation matrix R = [r₁, r₂, r₃], rᵢ∈ℝ³ is


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    rᵢᵀrᵢ = 1    (1)
    rᵢᵀrⱼ = 0    (2)
    r₁ x r₂ = r₃ (3)

.. raw:: html

    </details>

To relax SO(3) constraint on rotation matrix R, we divide the range
[-1, 1] (the range of each entry in R) into smaller intervals [φ(i),
φ(i+1)], and then relax the SO(3) constraint within each interval. We
provide 3 approaches for relaxation 1. By replacing each bilinear
product in constraint (1), (2) and (3) with a new variable, in the
McCormick envelope of the bilinear product w = x * y. 2. By
considering the intersection region between axis-aligned boxes, and
the surface of a unit sphere in 3D. 3. By combining the two approaches
above. This will result in a tighter relaxation.

These three approaches give different relaxation of SO(3) constraint
(the feasible sets for each relaxation are different), and different
computation speed. The users can switch between the approaches to find
the best fit for their problem.

Note:
    If you have several rotation matrices that all need to be relaxed
    through mixed-integer constraint, then you can create a single
    MixedIntegerRotationConstraintGenerator object, and add the
    mixed-integer constraint to each rotation matrix, by calling
    AddToProgram() function repeatedly.)""";
        // Symbol: drake::solvers::MixedIntegerRotationConstraintGenerator::AddToProgram
        struct /* AddToProgram */ {
          // Source: drake/solvers/mixed_integer_rotation_constraint.h
          const char* doc =
R"""(Add the mixed-integer linear constraints to the optimization program,
as a relaxation of SO(3) constraint on the rotation matrix ``R``.

Parameter ``R``:
    The rotation matrix on which the SO(3) constraint is imposed.

Parameter ``prog``:
    The optimization program to which the mixed-integer constraints
    (and additional variables) are added.)""";
        } AddToProgram;
        // Symbol: drake::solvers::MixedIntegerRotationConstraintGenerator::Approach
        struct /* Approach */ {
          // Source: drake/solvers/mixed_integer_rotation_constraint.h
          const char* doc = R"""()""";
          // Symbol: drake::solvers::MixedIntegerRotationConstraintGenerator::Approach::kBilinearMcCormick
          struct /* kBilinearMcCormick */ {
            // Source: drake/solvers/mixed_integer_rotation_constraint.h
            const char* doc =
R"""(Relax SO(3) constraint by considering the McCormick envelope on the
bilinear product.)""";
          } kBilinearMcCormick;
          // Symbol: drake::solvers::MixedIntegerRotationConstraintGenerator::Approach::kBoth
          struct /* kBoth */ {
            // Source: drake/solvers/mixed_integer_rotation_constraint.h
            const char* doc =
R"""(Relax SO(3) constraint by considering both the intersection between
boxes and the unit sphere surface, and the McCormick envelope on the
bilinear product.)""";
          } kBoth;
          // Symbol: drake::solvers::MixedIntegerRotationConstraintGenerator::Approach::kBoxSphereIntersection
          struct /* kBoxSphereIntersection */ {
            // Source: drake/solvers/mixed_integer_rotation_constraint.h
            const char* doc =
R"""(Relax SO(3) constraint by considering the intersection between boxes
and the unit sphere surface.)""";
          } kBoxSphereIntersection;
        } Approach;
        // Symbol: drake::solvers::MixedIntegerRotationConstraintGenerator::MixedIntegerRotationConstraintGenerator
        struct /* ctor */ {
          // Source: drake/solvers/mixed_integer_rotation_constraint.h
          const char* doc =
R"""(Constructor

Parameter ``approach``:
    Refer to MixedIntegerRotationConstraintGenerator∷Approach for the
    details.

Parameter ``num_intervals_per_half_axis``:
    We will cut the range [-1, 1] evenly to 2 *
    ``num_intervals_per_half_axis`` small intervals. The number of
    binary variables will depend on the number of intervals.

Parameter ``interval_binning``:
    The binning scheme we use to add SOS2 constraint with binary
    variables. If interval_binning = kLinear, then we will add 9 * 2 *
    ``num_intervals_per_half_axis binary`` variables; if
    interval_binning = kLogarithmic, then we will add 9 * (1 +
    log₂(num_intervals_per_half_axis)) binary variables. Refer to
    AddLogarithmicSos2Constraint and AddSos2Constraint for more
    details.)""";
        } ctor;
        // Symbol: drake::solvers::MixedIntegerRotationConstraintGenerator::ReturnType
        struct /* ReturnType */ {
          // Source: drake/solvers/mixed_integer_rotation_constraint.h
          const char* doc = R"""()""";
          // Symbol: drake::solvers::MixedIntegerRotationConstraintGenerator::ReturnType::B_
          struct /* B_ */ {
            // Source: drake/solvers/mixed_integer_rotation_constraint.h
            const char* doc =
R"""(B_ contains the new binary variables added to the program. B_`i][j]
represents in which interval R(i, j) lies. If we use linear binning,
then B_[i][j] is of length 2 * num_intervals_per_half_axis_. B_[i][j
<k>`_ = 1 => φ(k) ≤ R(i, j) ≤ φ(k + 1) B_`i][j <k>`_ = 0 => R(i, j) ≥
φ(k + 1) or R(i, j) ≤ φ(k) If we use logarithmic binning, then
B_[i][j] is of length 1 + log₂(num_intervals_per_half_axis_). If
B_[i][j] represents integer k in reflected Gray code, then R(i, j) is
in the interval [φ(k), φ(k+1)].)""";
          } B_;
          // Symbol: drake::solvers::MixedIntegerRotationConstraintGenerator::ReturnType::lambda_
          struct /* lambda_ */ {
            // Source: drake/solvers/mixed_integer_rotation_constraint.h
            const char* doc =
R"""(λ contains part of the new continuous variables added to the program.
λ_[i][j] is of length 2 * num_intervals_per_half_axis_ + 1, such that
R(i, j) = φᵀ * λ_[i][j]. Notice that λ_[i][j] satisfies the special
ordered set of type 2 (SOS2) constraint. Namely at most two entries in
λ_[i][j] can be strictly positive, and these two entries have to be
consecutive. Mathematically


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    ∑ₖ λ_[i][j](k) = 1
    λ_[i][j](k) ≥ 0 ∀ k
    ∃ m s.t λ_[i][j](n) = 0 if n ≠ m and n ≠ m+1

.. raw:: html

    </details>)""";
          } lambda_;
        } ReturnType;
        // Symbol: drake::solvers::MixedIntegerRotationConstraintGenerator::approach
        struct /* approach */ {
          // Source: drake/solvers/mixed_integer_rotation_constraint.h
          const char* doc = R"""()""";
        } approach;
        // Symbol: drake::solvers::MixedIntegerRotationConstraintGenerator::interval_binning
        struct /* interval_binning */ {
          // Source: drake/solvers/mixed_integer_rotation_constraint.h
          const char* doc = R"""()""";
        } interval_binning;
        // Symbol: drake::solvers::MixedIntegerRotationConstraintGenerator::num_intervals_per_half_axis
        struct /* num_intervals_per_half_axis */ {
          // Source: drake/solvers/mixed_integer_rotation_constraint.h
          const char* doc = R"""()""";
        } num_intervals_per_half_axis;
        // Symbol: drake::solvers::MixedIntegerRotationConstraintGenerator::phi
        struct /* phi */ {
          // Source: drake/solvers/mixed_integer_rotation_constraint.h
          const char* doc = R"""(Getter for φ.)""";
        } phi;
        // Symbol: drake::solvers::MixedIntegerRotationConstraintGenerator::phi_nonnegative
        struct /* phi_nonnegative */ {
          // Source: drake/solvers/mixed_integer_rotation_constraint.h
          const char* doc =
R"""(Getter for φ₊, the non-negative part of φ.)""";
        } phi_nonnegative;
      } MixedIntegerRotationConstraintGenerator;
      // Symbol: drake::solvers::MobyLcpSolver
      struct /* MobyLcpSolver */ {
        // Source: drake/solvers/moby_lcp_solver.h
        const char* doc =
R"""(A class for solving Linear Complementarity Problems (LCPs). Solving a
LCP requires finding a solution to the problem:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    Mz + q = w
    z ≥ 0
    w ≥ 0
    zᵀw = 0

.. raw:: html

    </details>

(where M ∈ ℝⁿˣⁿ and q ∈ ℝⁿ are problem inputs and z ∈ ℝⁿ and w ∈ ℝⁿ
are unknown vectors) or correctly reporting that such a solution does
not exist. In spite of their linear structure, solving LCPs is NP-Hard
[Cottle 1992]. However, some LCPs are significantly easier to solve.
For instance, it can be seen that the LCP is solvable in worst-case
polynomial time for the case of symmetric positive-semi-definite M by
formulating it as the following convex quadratic program:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    minimize:   f(z) = zᵀw = zᵀ(Mz + q)
    subject to: z ≥ 0
                Mz + q ≥ 0

.. raw:: html

    </details>

Note that this quadratic program's (QP) objective function at the
minimum z cannot be less than zero, and the LCP is only solved if the
objective function at the minimum is equal to zero. Since the seminal
result of Karmarkar, it has been known that convex QPs are solvable in
polynomial time [Karmarkar 1984].

The difficulty of solving an LCP is characterized by the properties of
its particular matrix, namely the class of matrices it belongs to.
Classes include, for example, Q, Q₀, P, P₀, copositive, and Z
matrices. [Cottle 1992] and [Murty 1998] (see pp. 224-230 in the
latter) describe relevant matrix classes in more detail.

* [Cottle 1992]     R. Cottle, J.-S. Pang, and R. Stone. The Linear
                    Complementarity Problem. Academic Press, 1992.
* [Karmarkar 1984]  N. Karmarkar. A New Polynomial-Time Algorithm for
                    Linear Programming. Combinatorica, 4(4), pp. 373-395.
* [Murty 1988]      K. Murty. Linear Complementarity, Linear and Nonlinear
                    Programming. Heldermann Verlag, 1988.)""";
        // Symbol: drake::solvers::MobyLcpSolver::ComputeZeroTolerance
        struct /* ComputeZeroTolerance */ {
          // Source: drake/solvers/moby_lcp_solver.h
          const char* doc =
R"""(Calculates the zero tolerance that the solver would compute if the
user does not specify a tolerance.)""";
        } ComputeZeroTolerance;
        // Symbol: drake::solvers::MobyLcpSolver::MobyLcpSolver
        struct /* ctor */ {
          // Source: drake/solvers/moby_lcp_solver.h
          const char* doc = R"""()""";
        } ctor;
        // Symbol: drake::solvers::MobyLcpSolver::ProgramAttributesSatisfied
        struct /* ProgramAttributesSatisfied */ {
          // Source: drake/solvers/moby_lcp_solver.h
          const char* doc = R"""()""";
        } ProgramAttributesSatisfied;
        // Symbol: drake::solvers::MobyLcpSolver::SolveLcpFast
        struct /* SolveLcpFast */ {
          // Source: drake/solvers/moby_lcp_solver.h
          const char* doc =
R"""(Fast pivoting algorithm for LCPs of the form M = PAPᵀ, q = Pb, where b
∈ ℝᵐ, P ∈ ℝⁿˣᵐ, and A ∈ ℝᵐˣᵐ (where A is positive definite).
Therefore, q is in the range of P and M is positive semi-definite. An
LCP of this form is also guaranteed to have a solution [Cottle 1992].

This particular implementation focuses on the case where the solution
requires few nonzero nonbasic variables, meaning that few z variables
need be nonzero to find a solution to Mz + q = w. This algorithm,
which is based off of Dantzig's Principle Pivoting Method I [Cottle
1992] is described in [Drumwright 2015]. This algorithm is able to use
"warm" starting- a solution to a "nearby" LCP can be used to find the
solution to a given LCP more quickly.

Although this solver is theoretically guaranteed to give a solution to
the LCPs described above, accumulated floating point error from
pivoting operations could cause the solver to fail. Additionally, the
solver can be applied with some success to problems outside of its
guaranteed matrix class. For these reasons, the solver returns a flag
indicating success/failure.

Parameter ``M``:
    the LCP matrix.

Parameter ``q``:
    the LCP vector.

Parameter ``z``:
    the solution to the LCP on return (if the solver succeeds). If the
    length of z is equal to the length of q, the solver will attempt
    to use z's value as a starting solution. If the solver fails
    (returns ``False``), `z` will be set to the zero vector.

Parameter ``zero_tol``:
    The tolerance for testing against zero. If the tolerance is
    negative (default) the solver will determine a generally
    reasonable tolerance.

Raises:
    RuntimeError if M is non-square or M's dimensions do not equal q's
    dimension.

Returns:
    ``True`` if the solver succeeded and ``False`` otherwise.

* [Cottle 1992]      R. Cottle, J.-S. Pang, and R. Stone. The Linear
                     Complementarity Problem. Academic Press, 1992.
* [Drumwright 2015]  E. Drumwright. Rapidly computable viscous friction
                     and no-slip rigid contact models. arXiv:
                     1504.00719v1. 2015.)""";
        } SolveLcpFast;
        // Symbol: drake::solvers::MobyLcpSolver::SolveLcpFastRegularized
        struct /* SolveLcpFastRegularized */ {
          // Source: drake/solvers/moby_lcp_solver.h
          const char* doc =
R"""(Regularized version of the fast pivoting algorithm for LCPs of the
form M = PAPᵀ, q = Pb, where b ∈ ℝᵐ, P ∈ ℝⁿˣᵐ, and A ∈ ℝᵐˣᵐ (where A
is positive definite). Therefore, q is in the range of P and M is
positive semi-definite. Please see SolveLcpFast() for more
documentation about the particular algorithm.

This implementation wraps that algorithm with a Tikhonov-type
regularization approach. Specifically, this implementation repeatedly
attempts to solve the LCP:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    (M + Iα)z + q = w
    z ≥ 0
    w ≥ 0
    zᵀw = 0

.. raw:: html

    </details>

where I is the identity matrix and α ≪ 1, using geometrically
increasing values of α, until the LCP is solved. Cottle et al.
describe how, for sufficiently large α, the LCP will always be
solvable [Cottle 1992], p. 493.

Although this solver is theoretically guaranteed to give a solution to
the LCPs described above, accumulated floating point error from
pivoting operations could cause the solver to fail. Additionally, the
solver can be applied with some success to problems outside of its
guaranteed matrix class. For these reasons, the solver returns a flag
indicating success/failure.

Parameter ``M``:
    the LCP matrix.

Parameter ``q``:
    the LCP vector.

Parameter ``z``:
    the solution to the LCP on return (if the solver succeeds). If the
    length of z is equal to the length of q, the solver will attempt
    to use z's value as a starting solution.

Parameter ``min_exp``:
    The minimum exponent for computing α over [10ᵝ, 10ᵞ] in steps of
    10ᵟ, where β is the minimum exponent, γ is the maximum exponent,
    and δ is the stepping exponent.

Parameter ``step_exp``:
    The stepping exponent for computing α over [10ᵝ, 10ᵞ] in steps of
    10ᵟ, where β is the minimum exponent, γ is the maximum exponent,
    and δ is the stepping exponent.

Parameter ``max_exp``:
    The maximum exponent for computing α over [10ᵝ, 10ᵞ] in steps of
    10ᵟ, where β is the minimum exponent, γ is the maximum exponent,
    and δ is the stepping exponent.

Parameter ``zero_tol``:
    The tolerance for testing against zero. If the tolerance is
    negative (default) the solver will determine a generally
    reasonable tolerance.

Raises:
    RuntimeError if M is non-square or M's dimensions do not equal q's
    dimension.

Returns:
    ``True`` if the solver succeeded and ``False`` if the solver did
    not find a solution for α = 10ᵞ.

See also:
    SolveLcpFast()

* [Cottle, 1992]     R. Cottle, J.-S. Pang, and R. Stone. The Linear
                     Complementarity Problem. Academic Press, 1992.)""";
        } SolveLcpFastRegularized;
        // Symbol: drake::solvers::MobyLcpSolver::SolveLcpLemke
        struct /* SolveLcpLemke */ {
          // Source: drake/solvers/moby_lcp_solver.h
          const char* doc =
R"""(Lemke's Algorithm for solving LCPs in the matrix class E, which
contains all strictly semimonotone matrices, all P-matrices, and all
strictly copositive matrices. Lemke's Algorithm is described in
[Cottle 1992], Section 4.4. This implementation was adapted from the
LEMKE Library [LEMKE] for Matlab; this particular implementation fixes
a bug in LEMKE that could occur when multiple indices passed the
minimum ratio test.

Although this solver is theoretically guaranteed to give a solution to
the LCPs described above, accumulated floating point error from
pivoting operations could cause the solver to fail. Additionally, the
solver can be applied with some success to problems outside of its
guaranteed matrix classes. For these reasons, the solver returns a
flag indicating success/failure.

Parameter ``M``:
    the LCP matrix.

Parameter ``q``:
    the LCP vector.

Parameter ``z``:
    the solution to the LCP on return (if the solver succeeds). If the
    length of z is equal to the length of q, the solver will attempt
    to use z's value as a starting solution. **This warmstarting is
    generally not recommended**: it has a predisposition to lead to a
    failing pivoting sequence. If the solver fails (returns
    ``False``), `z` will be set to the zero vector.

Parameter ``zero_tol``:
    The tolerance for testing against zero. If the tolerance is
    negative (default) the solver will determine a generally
    reasonable tolerance.

Parameter ``piv_tol``:
    The tolerance for testing against zero, specifically used for the
    purpose of finding variables for pivoting. If the tolerance is
    negative (default) the solver will determine a generally
    reasonable tolerance.

Returns:
    ``True`` if the solver **believes** it has computed a solution
    (which it determines by the ability to "pivot out" the
    "artificial" variable (see [Cottle 1992]) and ``False`` otherwise.

Warning:
    The caller should verify that the algorithm has solved the LCP to
    the desired tolerances on returns indicating success.

Raises:
    RuntimeError if M is not square or the dimensions of M do not
    match the length of q.

* [Cottle 1992]      R. Cottle, J.-S. Pang, and R. Stone. The Linear
                     Complementarity Problem. Academic Press, 1992.
* [LEMKE]          P. Fackler and M. Miranda. LEMKE.
                   http://people.sc.fsu.edu/~burkardt/m\_src/lemke/lemke.m)""";
        } SolveLcpLemke;
        // Symbol: drake::solvers::MobyLcpSolver::SolveLcpLemkeRegularized
        struct /* SolveLcpLemkeRegularized */ {
          // Source: drake/solvers/moby_lcp_solver.h
          const char* doc =
R"""(Lemke's Algorithm for solving LCPs in the matrix class E, which
contains all strictly semimonotone matrices, all P-matrices, and all
strictly copositive matrices. Lemke's Algorithm is described in
[Cottle 1992], Section 4.4.

This implementation wraps that algorithm with a Tikhonov-type
regularization approach. Specifically, this implementation repeatedly
attempts to solve the LCP:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    (M + Iα)z + q = w
    z ≥ 0
    w ≥ 0
    zᵀw = 0

.. raw:: html

    </details>

where I is the identity matrix and α ≪ 1, using geometrically
increasing values of α, until the LCP is solved. See
SolveLcpFastRegularized() for description of the regularization
process and the function parameters, which are identical. See
SolveLcpLemke() for a description of Lemke's Algorithm. See
SolveLcpFastRegularized() for a description of all calling parameters
other than ``z``, which apply equally well to this function.

Parameter ``z``:
    the solution to the LCP on return (if the solver succeeds). If the
    length of z is equal to the length of q, the solver will attempt
    to use z's value as a starting solution. **This warmstarting is
    generally not recommended**: it has a predisposition to lead to a
    failing pivoting sequence.

See also:
    SolveLcpFastRegularized()

See also:
    SolveLcpLemke()

* [Cottle 1992]      R. Cottle, J.-S. Pang, and R. Stone. The Linear
                     Complementarity Problem. Academic Press, 1992.)""";
        } SolveLcpLemkeRegularized;
        // Symbol: drake::solvers::MobyLcpSolver::get_num_pivots
        struct /* get_num_pivots */ {
          // Source: drake/solvers/moby_lcp_solver.h
          const char* doc =
R"""(Returns the number of pivoting operations made by the last LCP solve.)""";
        } get_num_pivots;
        // Symbol: drake::solvers::MobyLcpSolver::id
        struct /* id */ {
          // Source: drake/solvers/moby_lcp_solver.h
          const char* doc = R"""()""";
        } id;
        // Symbol: drake::solvers::MobyLcpSolver::is_available
        struct /* is_available */ {
          // Source: drake/solvers/moby_lcp_solver.h
          const char* doc = R"""()""";
        } is_available;
        // Symbol: drake::solvers::MobyLcpSolver::is_enabled
        struct /* is_enabled */ {
          // Source: drake/solvers/moby_lcp_solver.h
          const char* doc = R"""()""";
        } is_enabled;
        // Symbol: drake::solvers::MobyLcpSolver::reset_num_pivots
        struct /* reset_num_pivots */ {
          // Source: drake/solvers/moby_lcp_solver.h
          const char* doc =
R"""(Resets the number of pivoting operations made by the last LCP solver
to zero.)""";
        } reset_num_pivots;
      } MobyLcpSolver;
      // Symbol: drake::solvers::MosekSolver
      struct /* MosekSolver */ {
        // Source: drake/solvers/mosek_solver.h
        const char* doc =
R"""(An implementation of SolverInterface for the commercially-licensed
MOSEK (TM) solver (https://www.mosek.com/).

Drake downloads and builds MOSEK™ automatically, but to enable it you
must set the location of your license file as described in the
documentation at https://drake.mit.edu/bazel.html#mosek.

The MOSEKLM_LICENSE_FILE environment variable controls whether or not
SolverInterface∷enabled() returns true. Iff it is set to any non-empty
value, then the solver is enabled; otherwise, the solver is not
enabled.

Note:
    MOSEK™ only cares about the initial guess of integer variables.
    The initial guess of continuous variables are not passed to
    MOSEK™. If all the integer variables are set to some integer
    values, then MOSEK™ will be forced to compute the remaining
    continuous variable values as the initial guess. (MOSEK™ might
    change the values of the integer/binary variables in the
    subsequent iterations.) If the specified integer solution is
    infeasible or incomplete, MOSEK™ will simply ignore it. For more
    details, check
    https://docs.mosek.com/11.1/capi/tutorial-mio-shared.html?highlight=initial

MOSEK™ supports many solver parameters. You can refer to the full list
of parameters in
https://docs.mosek.com/11.1/capi/param-groups.html#doc-param-groups.
On top of these parameters, we also provide the following additional
parameters

- "writedata"
   set to a file name so that MOSEK™ solver will write the
   optimization model to this file. check
   https://docs.mosek.com/11.1/capi/solver-io.html#saving-a-problem-to-a-file
   for more details. The supported file extensions are listed in
   https://docs.mosek.com/11.1/capi/supported-file-formats.html#doc-shared-file-formats.
   Set this parameter to "" if you don't want to write to a file. Default is
   not to write to a file.)""";
        // Symbol: drake::solvers::MosekSolver::AcquireLicense
        struct /* AcquireLicense */ {
          // Source: drake/solvers/mosek_solver.h
          const char* doc =
R"""(This acquires a MOSEK™ license environment shared among all
MosekSolver instances; the environment will stay valid as long as at
least one shared_ptr returned by this function is alive. Call this
ONLY if you must use different MathematicalProgram instances at
different instances in time, and repeatedly acquiring the license is
costly (e.g., requires contacting a license server).

Returns:
    A shared pointer to a license environment that will stay valid as
    long as any shared_ptr returned by this function is alive. If
    MOSEK™ is not available in your build, this will return a null
    (empty) shared_ptr.

Raises:
    RuntimeError if MOSEK™ is available but a license cannot be
    obtained.)""";
        } AcquireLicense;
        // Symbol: drake::solvers::MosekSolver::Details
        struct /* Details */ {
          // Source: drake/solvers/mosek_solver.h
          const char* doc =
R"""(Type of details stored in MathematicalProgramResult.)""";
        } Details;
        // Symbol: drake::solvers::MosekSolver::MosekSolver
        struct /* ctor */ {
          // Source: drake/solvers/mosek_solver.h
          const char* doc = R"""()""";
        } ctor;
        // Symbol: drake::solvers::MosekSolver::ProgramAttributesSatisfied
        struct /* ProgramAttributesSatisfied */ {
          // Source: drake/solvers/mosek_solver.h
          const char* doc = R"""()""";
        } ProgramAttributesSatisfied;
        // Symbol: drake::solvers::MosekSolver::UnsatisfiedProgramAttributes
        struct /* UnsatisfiedProgramAttributes */ {
          // Source: drake/solvers/mosek_solver.h
          const char* doc = R"""()""";
        } UnsatisfiedProgramAttributes;
        // Symbol: drake::solvers::MosekSolver::id
        struct /* id */ {
          // Source: drake/solvers/mosek_solver.h
          const char* doc = R"""()""";
        } id;
        // Symbol: drake::solvers::MosekSolver::is_available
        struct /* is_available */ {
          // Source: drake/solvers/mosek_solver.h
          const char* doc = R"""()""";
        } is_available;
        // Symbol: drake::solvers::MosekSolver::is_enabled
        struct /* is_enabled */ {
          // Source: drake/solvers/mosek_solver.h
          const char* doc =
R"""(Returns true iff the environment variable MOSEKLM_LICENSE_FILE has
been set to a non-empty value.)""";
        } is_enabled;
      } MosekSolver;
      // Symbol: drake::solvers::MosekSolverDetails
      struct /* MosekSolverDetails */ {
        // Source: drake/solvers/mosek_solver.h
        const char* doc =
R"""(The MOSEK™ solver details after calling Solve() function. The user can
call MathematicalProgramResult∷get_solver_details<MosekSolver>() to
obtain the details.)""";
        // Symbol: drake::solvers::MosekSolverDetails::optimizer_time
        struct /* optimizer_time */ {
          // Source: drake/solvers/mosek_solver.h
          const char* doc =
R"""(The MOSEK™ optimization time. Please refer to MSK_DINF_OPTIMIZER_TIME
in
https://docs.mosek.com/11.1/capi/constants.html?highlight=msk_dinf_optimizer_time)""";
        } optimizer_time;
        // Symbol: drake::solvers::MosekSolverDetails::rescode
        struct /* rescode */ {
          // Source: drake/solvers/mosek_solver.h
          const char* doc =
R"""(The response code returned from MOSEK™ solver. Check
https://docs.mosek.com/11.1/capi/response-codes.html for the meaning
on the response code.)""";
        } rescode;
        // Symbol: drake::solvers::MosekSolverDetails::solution_status
        struct /* solution_status */ {
          // Source: drake/solvers/mosek_solver.h
          const char* doc =
R"""(The solution status after solving the problem. Check
https://docs.mosek.com/11.1/capi/accessing-solution.html and
https://docs.mosek.com/11.1/capi/constants.html#mosek.solsta for the
meaning on the solution status.)""";
        } solution_status;
      } MosekSolverDetails;
      // Symbol: drake::solvers::NewRotationMatrixVars
      struct /* NewRotationMatrixVars */ {
        // Source: drake/solvers/rotation_constraint.h
        const char* doc =
R"""(Allocates a 3x3 matrix of decision variables with the trivial bounding
box constraint ensuring all elements are [-1,1], and the linear
constraint imposing -1 <= trace(R) <= 3.)""";
      } NewRotationMatrixVars;
      // Symbol: drake::solvers::NewSymmetricVariableNames
      struct /* NewSymmetricVariableNames */ {
        // Source: drake/solvers/mathematical_program.h
        const char* doc = R"""()""";
      } NewSymmetricVariableNames;
      // Symbol: drake::solvers::NewVariableNames
      struct /* NewVariableNames */ {
        // Source: drake/solvers/mathematical_program.h
        const char* doc = R"""()""";
        // Symbol: drake::solvers::NewVariableNames::type
        struct /* type */ {
          // Source: drake/solvers/mathematical_program.h
          const char* doc = R"""()""";
        } type;
      } NewVariableNames;
      // Symbol: drake::solvers::NloptSolver
      struct /* NloptSolver */ {
        // Source: drake/solvers/nlopt_solver.h
        const char* doc = R"""()""";
        // Symbol: drake::solvers::NloptSolver::AlgorithmName
        struct /* AlgorithmName */ {
          // Source: drake/solvers/nlopt_solver.h
          const char* doc =
R"""(The key name for the string-valued algorithm.)""";
        } AlgorithmName;
        // Symbol: drake::solvers::NloptSolver::ConstraintToleranceName
        struct /* ConstraintToleranceName */ {
          // Source: drake/solvers/nlopt_solver.h
          const char* doc =
R"""(The key name for the double-valued constraint tolerance.)""";
        } ConstraintToleranceName;
        // Symbol: drake::solvers::NloptSolver::Details
        struct /* Details */ {
          // Source: drake/solvers/nlopt_solver.h
          const char* doc =
R"""(Type of details stored in MathematicalProgramResult.)""";
        } Details;
        // Symbol: drake::solvers::NloptSolver::MaxEvalName
        struct /* MaxEvalName */ {
          // Source: drake/solvers/nlopt_solver.h
          const char* doc =
R"""(The key name for int-valued maximum number of evaluations.)""";
        } MaxEvalName;
        // Symbol: drake::solvers::NloptSolver::MaxTimeName
        struct /* MaxTimeName */ {
          // Source: drake/solvers/nlopt_solver.h
          const char* doc =
R"""(The key name for the maximum runtime. By default, there is no maximum
runtime. A nonpositive value will be interpreted as no maximum
runtime.)""";
        } MaxTimeName;
        // Symbol: drake::solvers::NloptSolver::NloptSolver
        struct /* ctor */ {
          // Source: drake/solvers/nlopt_solver.h
          const char* doc = R"""()""";
        } ctor;
        // Symbol: drake::solvers::NloptSolver::ProgramAttributesSatisfied
        struct /* ProgramAttributesSatisfied */ {
          // Source: drake/solvers/nlopt_solver.h
          const char* doc = R"""()""";
        } ProgramAttributesSatisfied;
        // Symbol: drake::solvers::NloptSolver::XAbsoluteToleranceName
        struct /* XAbsoluteToleranceName */ {
          // Source: drake/solvers/nlopt_solver.h
          const char* doc =
R"""(The key name for double-valued x absolute tolerance.)""";
        } XAbsoluteToleranceName;
        // Symbol: drake::solvers::NloptSolver::XRelativeToleranceName
        struct /* XRelativeToleranceName */ {
          // Source: drake/solvers/nlopt_solver.h
          const char* doc =
R"""(The key name for double-valued x relative tolerance.)""";
        } XRelativeToleranceName;
        // Symbol: drake::solvers::NloptSolver::id
        struct /* id */ {
          // Source: drake/solvers/nlopt_solver.h
          const char* doc = R"""()""";
        } id;
        // Symbol: drake::solvers::NloptSolver::is_available
        struct /* is_available */ {
          // Source: drake/solvers/nlopt_solver.h
          const char* doc = R"""()""";
        } is_available;
        // Symbol: drake::solvers::NloptSolver::is_enabled
        struct /* is_enabled */ {
          // Source: drake/solvers/nlopt_solver.h
          const char* doc = R"""()""";
        } is_enabled;
      } NloptSolver;
      // Symbol: drake::solvers::NloptSolverDetails
      struct /* NloptSolverDetails */ {
        // Source: drake/solvers/nlopt_solver.h
        const char* doc =
R"""(The NLopt solver details after calling Solve() function. The user can
call MathematicalProgramResult∷get_solver_details<NloptSolver>() to
obtain the details.)""";
        // Symbol: drake::solvers::NloptSolverDetails::status
        struct /* status */ {
          // Source: drake/solvers/nlopt_solver.h
          const char* doc =
R"""(The return status of NLopt solver. Please refer to
https://nlopt.readthedocs.io/en/latest/NLopt_Reference/#return-values.)""";
        } status;
      } NloptSolverDetails;
      // Symbol: drake::solvers::OsqpSolver
      struct /* OsqpSolver */ {
        // Source: drake/solvers/osqp_solver.h
        const char* doc =
R"""(A wrapper to call `OSQP <https://osqp.org/>`_ using Drake's
MathematicalProgram.

For details about OSQP's available options, refer to the `OSQP manual
<https://osqp.org/docs/interfaces/solver_settings.html>`_. Drake uses
OSQP's default values for all options except following: - Drake
defaults to ``polish=true`` (upstream default is ``False``). - Drake
defaults to ``adaptive_rho_interval=ADAPTIVE_RHO_FIXED`` to make the
output deterministic (upstream default is ``0``, which uses
non-deterministic timing measurements to establish the interval). N.B.
Generally the interval should be an integer multiple of
``check_termination``, so if you choose to override either option you
should probably override both at once.

At the end of OsqpSolver∷Solve() function, we always return the value
of the primal and dual variables inside the OSQP solver, regardless of
the solver status (whether it is optimal, infeasible, unbounded, etc),
except when the problem has an invalid input. Users should always
check the solver status before interpreting the returned primal and
dual variables.)""";
        // Symbol: drake::solvers::OsqpSolver::Details
        struct /* Details */ {
          // Source: drake/solvers/osqp_solver.h
          const char* doc =
R"""(Type of details stored in MathematicalProgramResult.)""";
        } Details;
        // Symbol: drake::solvers::OsqpSolver::OsqpSolver
        struct /* ctor */ {
          // Source: drake/solvers/osqp_solver.h
          const char* doc = R"""()""";
        } ctor;
        // Symbol: drake::solvers::OsqpSolver::ProgramAttributesSatisfied
        struct /* ProgramAttributesSatisfied */ {
          // Source: drake/solvers/osqp_solver.h
          const char* doc = R"""()""";
        } ProgramAttributesSatisfied;
        // Symbol: drake::solvers::OsqpSolver::UnsatisfiedProgramAttributes
        struct /* UnsatisfiedProgramAttributes */ {
          // Source: drake/solvers/osqp_solver.h
          const char* doc = R"""()""";
        } UnsatisfiedProgramAttributes;
        // Symbol: drake::solvers::OsqpSolver::id
        struct /* id */ {
          // Source: drake/solvers/osqp_solver.h
          const char* doc = R"""()""";
        } id;
        // Symbol: drake::solvers::OsqpSolver::is_available
        struct /* is_available */ {
          // Source: drake/solvers/osqp_solver.h
          const char* doc = R"""()""";
        } is_available;
        // Symbol: drake::solvers::OsqpSolver::is_enabled
        struct /* is_enabled */ {
          // Source: drake/solvers/osqp_solver.h
          const char* doc = R"""()""";
        } is_enabled;
      } OsqpSolver;
      // Symbol: drake::solvers::OsqpSolverDetails
      struct /* OsqpSolverDetails */ {
        // Source: drake/solvers/osqp_solver.h
        const char* doc =
R"""(The OSQP solver details after calling Solve() function. The user can
call MathematicalProgramResult∷get_solver_details<OsqpSolver>() to
obtain the details.)""";
        // Symbol: drake::solvers::OsqpSolverDetails::dual_res
        struct /* dual_res */ {
          // Source: drake/solvers/osqp_solver.h
          const char* doc = R"""(Norm of dual residue.)""";
        } dual_res;
        // Symbol: drake::solvers::OsqpSolverDetails::iter
        struct /* iter */ {
          // Source: drake/solvers/osqp_solver.h
          const char* doc = R"""(Number of iterations taken.)""";
        } iter;
        // Symbol: drake::solvers::OsqpSolverDetails::polish_time
        struct /* polish_time */ {
          // Source: drake/solvers/osqp_solver.h
          const char* doc = R"""(Time taken for polish phase (seconds).)""";
        } polish_time;
        // Symbol: drake::solvers::OsqpSolverDetails::primal_res
        struct /* primal_res */ {
          // Source: drake/solvers/osqp_solver.h
          const char* doc = R"""(Norm of primal residue.)""";
        } primal_res;
        // Symbol: drake::solvers::OsqpSolverDetails::rho_updates
        struct /* rho_updates */ {
          // Source: drake/solvers/osqp_solver.h
          const char* doc = R"""(Number of rho updates.)""";
        } rho_updates;
        // Symbol: drake::solvers::OsqpSolverDetails::run_time
        struct /* run_time */ {
          // Source: drake/solvers/osqp_solver.h
          const char* doc = R"""(Total OSQP time (seconds).)""";
        } run_time;
        // Symbol: drake::solvers::OsqpSolverDetails::setup_time
        struct /* setup_time */ {
          // Source: drake/solvers/osqp_solver.h
          const char* doc = R"""(Time taken for setup phase (seconds).)""";
        } setup_time;
        // Symbol: drake::solvers::OsqpSolverDetails::solve_time
        struct /* solve_time */ {
          // Source: drake/solvers/osqp_solver.h
          const char* doc = R"""(Time taken for solve phase (seconds).)""";
        } solve_time;
        // Symbol: drake::solvers::OsqpSolverDetails::status_val
        struct /* status_val */ {
          // Source: drake/solvers/osqp_solver.h
          const char* doc =
R"""(Status of the solver at termination. Please refer to
https://github.com/oxfordcontrol/osqp/blob/master/include/constants.h)""";
        } status_val;
        // Symbol: drake::solvers::OsqpSolverDetails::y
        struct /* y */ {
          // Source: drake/solvers/osqp_solver.h
          const char* doc =
R"""(y contains the solution for the Lagrangian multiplier associated with
l <= Ax <= u. The Lagrangian multiplier is set only when OSQP solves
the problem. Notice that the order of the linear constraints are
linear inequality first, and then linear equality constraints.)""";
        } y;
      } OsqpSolverDetails;
      // Symbol: drake::solvers::PerspectiveQuadraticCost
      struct /* PerspectiveQuadraticCost */ {
        // Source: drake/solvers/cost.h
        const char* doc =
R"""(If :math:`z = Ax + b,` implements a cost of the form:

.. math:: (z_1^2 + z_2^2 + ... + z_{n-1}^2) / z_0.

Note that this cost is convex when we additionally constrain z_0 > 0.
It is treated as a generic nonlinear objective by most solvers.

Costs of this form are sometimes referred to as "quadratic over
linear".)""";
        // Symbol: drake::solvers::PerspectiveQuadraticCost::A
        struct /* A */ {
          // Source: drake/solvers/cost.h
          const char* doc = R"""()""";
        } A;
        // Symbol: drake::solvers::PerspectiveQuadraticCost::DoDisplay
        struct /* DoDisplay */ {
          // Source: drake/solvers/cost.h
          const char* doc = R"""()""";
        } DoDisplay;
        // Symbol: drake::solvers::PerspectiveQuadraticCost::DoEval
        struct /* DoEval */ {
          // Source: drake/solvers/cost.h
          const char* doc = R"""()""";
        } DoEval;
        // Symbol: drake::solvers::PerspectiveQuadraticCost::DoToLatex
        struct /* DoToLatex */ {
          // Source: drake/solvers/cost.h
          const char* doc = R"""()""";
        } DoToLatex;
        // Symbol: drake::solvers::PerspectiveQuadraticCost::PerspectiveQuadraticCost
        struct /* ctor */ {
          // Source: drake/solvers/cost.h
          const char* doc =
R"""(Construct a cost of the form (z_1^2 + z_2^2 + ... + z_{n-1}^2) / z_0
where z = Ax + b.

Parameter ``A``:
    Linear term.

Parameter ``b``:
    Constant term.)""";
        } ctor;
        // Symbol: drake::solvers::PerspectiveQuadraticCost::UpdateCoefficients
        struct /* UpdateCoefficients */ {
          // Source: drake/solvers/cost.h
          const char* doc =
R"""(Updates the coefficients of the cost. Note that the number of
variables (columns of A) cannot change.

Parameter ``new_A``:
    New linear term.

Parameter ``new_b``:
    New constant term.)""";
        } UpdateCoefficients;
        // Symbol: drake::solvers::PerspectiveQuadraticCost::b
        struct /* b */ {
          // Source: drake/solvers/cost.h
          const char* doc = R"""()""";
        } b;
        // Symbol: drake::solvers::PerspectiveQuadraticCost::update_A_entry
        struct /* update_A_entry */ {
          // Source: drake/solvers/cost.h
          const char* doc =
R"""(Updates A(i, j) = val.

Raises:
    if i or j are invalid indices.)""";
        } update_A_entry;
        // Symbol: drake::solvers::PerspectiveQuadraticCost::update_b_entry
        struct /* update_b_entry */ {
          // Source: drake/solvers/cost.h
          const char* doc =
R"""(Updates b(i) = val.

Raises:
    if i is an invalid index.)""";
        } update_b_entry;
      } PerspectiveQuadraticCost;
      // Symbol: drake::solvers::PolynomialConstraint
      struct /* PolynomialConstraint */ {
        // Source: drake/solvers/constraint.h
        const char* doc =
R"""(A constraint on the values of multivariate polynomials.


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    lb[i] ≤ P[i](x, y...) ≤ ub[i],

.. raw:: html

    </details>

where each P[i] is a multivariate polynomial in x, y...

The Polynomial class uses a different variable naming scheme; thus the
caller must provide a list of Polynomial∷VarType variables that
correspond to the members of the MathematicalProgram∷Binding (the
individual scalar elements of the given VariableList).)""";
        // Symbol: drake::solvers::PolynomialConstraint::PolynomialConstraint
        struct /* ctor */ {
          // Source: drake/solvers/constraint.h
          const char* doc =
R"""(Constructs a polynomial constraint

Parameter ``polynomials``:
    Polynomial vector, a ``num_constraints`` x 1 vector.

Parameter ``poly_vars``:
    Polynomial variables, a ``num_vars`` x 1 vector.

Parameter ``lb``:
    Lower bounds, a ``num_constraints`` x 1 vector.

Parameter ``ub``:
    Upper bounds, a ``num_constraints`` x 1 vector.)""";
        } ctor;
        // Symbol: drake::solvers::PolynomialConstraint::poly_vars
        struct /* poly_vars */ {
          // Source: drake/solvers/constraint.h
          const char* doc = R"""()""";
        } poly_vars;
        // Symbol: drake::solvers::PolynomialConstraint::polynomials
        struct /* polynomials */ {
          // Source: drake/solvers/constraint.h
          const char* doc = R"""()""";
        } polynomials;
      } PolynomialConstraint;
      // Symbol: drake::solvers::PolynomialCost
      struct /* PolynomialCost */ {
        // Source: drake/solvers/cost.h
        const char* doc =
R"""(Implements a cost of the form P(x, y...) where P is a multivariate
polynomial in x, y, ...

The Polynomial class uses a different variable naming scheme; thus the
caller must provide a list of Polynomial∷VarType variables that
correspond to the members of the Binding<> (the individual scalar
elements of the given VariableList).)""";
        // Symbol: drake::solvers::PolynomialCost::PolynomialCost
        struct /* ctor */ {
          // Source: drake/solvers/cost.h
          const char* doc =
R"""(Constructs a polynomial cost

Parameter ``polynomials``:
    Polynomial vector, a 1 x 1 vector.

Parameter ``poly_vars``:
    Polynomial variables, a ``num_vars`` x 1 vector.)""";
        } ctor;
        // Symbol: drake::solvers::PolynomialCost::poly_vars
        struct /* poly_vars */ {
          // Source: drake/solvers/cost.h
          const char* doc = R"""()""";
        } poly_vars;
        // Symbol: drake::solvers::PolynomialCost::polynomials
        struct /* polynomials */ {
          // Source: drake/solvers/cost.h
          const char* doc = R"""()""";
        } polynomials;
      } PolynomialCost;
      // Symbol: drake::solvers::PolynomialEvaluator
      struct /* PolynomialEvaluator */ {
        // Source: drake/solvers/evaluator_base.h
        const char* doc =
R"""(Implements an evaluator of the form P(x, y...) where P is a
multivariate polynomial in x, y, ...

The Polynomial class uses a different variable naming scheme; thus the
caller must provide a list of Polynomial∷VarType variables that
correspond to the members of the Binding<> (the individual scalar
elements of the given VariableList).)""";
        // Symbol: drake::solvers::PolynomialEvaluator::PolynomialEvaluator
        struct /* ctor */ {
          // Source: drake/solvers/evaluator_base.h
          const char* doc =
R"""(Constructs a polynomial evaluator given a set of polynomials and the
corresponding variables.

Parameter ``polynomials``:
    Polynomial vector, a ``num_outputs`` x 1 vector.

Parameter ``poly_vars``:
    Polynomial variables, a ``num_vars`` x 1 vector.)""";
        } ctor;
        // Symbol: drake::solvers::PolynomialEvaluator::poly_vars
        struct /* poly_vars */ {
          // Source: drake/solvers/evaluator_base.h
          const char* doc = R"""()""";
        } poly_vars;
        // Symbol: drake::solvers::PolynomialEvaluator::polynomials
        struct /* polynomials */ {
          // Source: drake/solvers/evaluator_base.h
          const char* doc = R"""()""";
        } polynomials;
      } PolynomialEvaluator;
      // Symbol: drake::solvers::PositiveSemidefiniteConstraint
      struct /* PositiveSemidefiniteConstraint */ {
        // Source: drake/solvers/constraint.h
        const char* doc =
R"""(Implements a positive semidefinite constraint on a symmetric matrix S

.. math:: \text{
    S is p.s.d
}

namely, all eigen values of S are non-negative.

Note:
    if the matix S has 1 row, then it is better to impose a linear
    inequality constraints; if it has 2 rows, then it is better to
    impose a rotated Lorentz cone constraint, since a 2 x 2 matrix S
    being p.s.d is equivalent to the constraint [S(0, 0), S(1, 1),
    S(0, 1)] in the rotated Lorentz cone.)""";
        // Symbol: drake::solvers::PositiveSemidefiniteConstraint::DoEval
        struct /* DoEval */ {
          // Source: drake/solvers/constraint.h
          const char* doc_was_unable_to_choose_unambiguous_names =
R"""(Evaluate the eigen values of the symmetric matrix.

Parameter ``x``:
    The stacked columns of the symmetric matrix.)""";
        } DoEval;
        // Symbol: drake::solvers::PositiveSemidefiniteConstraint::DoToLatex
        struct /* DoToLatex */ {
          // Source: drake/solvers/constraint.h
          const char* doc = R"""()""";
        } DoToLatex;
        // Symbol: drake::solvers::PositiveSemidefiniteConstraint::PositiveSemidefiniteConstraint
        struct /* ctor */ {
          // Source: drake/solvers/constraint.h
          const char* doc =
R"""(Impose the constraint that a symmetric matrix with size ``rows`` x
``rows`` is positive semidefinite.

See also:
    MathematicalProgram∷AddPositiveSemidefiniteConstraint() for how to
    use this constraint on some decision variables. We currently use
    this constraint as a place holder in MathematicalProgram, to
    indicate the positive semidefiniteness of some decision variables.

Parameter ``rows``:
    The number of rows (and columns) of the symmetric matrix.

Note:
    ``rows`` should be a positive integer. If `rows`==1 or `rows`==2,
    then consider imposing a linear inequality or rotated Lorentz cone
    constraint respectively.

Example:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    // Create a MathematicalProgram object.
    auto prog = MathematicalProgram();
    
    // Add a 3 x 3 symmetric matrix S to optimization program as new decision
    // variables.
    auto S = prog.NewSymmetricContinuousVariables<3>("S");
    
    // Impose a positive semidefinite constraint on S.
    std∷shared_ptr<PositiveSemidefiniteConstraint> psd_constraint =
        prog.AddPositiveSemidefiniteConstraint(S);
    
    /////////////////////////////////////////////////////////////
    // Add more constraints to make the program more interesting,
    // but this is not needed.
    
    // Add the constraint that S(1, 0) = 1.
    prog.AddBoundingBoxConstraint(1, 1, S(1, 0));
    
    // Minimize S(0, 0) + S(1, 1) + S(2, 2).
    prog.AddLinearCost(Eigen∷RowVector3d(1, 1, 1), {S.diagonal()});
    
    /////////////////////////////////////////////////////////////
    
    // Now solve the program.
    auto result = Solve(prog);
    
    // Retrieve the solution of matrix S.
    auto S_value = GetSolution(S, result);
    
    // Compute the eigen values of the solution, to see if they are
    // all non-negative.
    Vector6d S_stacked;
    S_stacked << S_value.col(0), S_value.col(1), S_value.col(2);
    
    Eigen∷VectorXd S_eigen_values;
    psd_constraint->Eval(S_stacked, S_eigen_values);
    
    std∷cout<<"S solution is: " << S << std∷endl;
    std∷cout<<"The eigen value of S is " << S_eigen_values << std∷endl;

.. raw:: html

    </details>)""";
        } ctor;
        // Symbol: drake::solvers::PositiveSemidefiniteConstraint::WarnOnSmallMatrixSize
        struct /* WarnOnSmallMatrixSize */ {
          // Source: drake/solvers/constraint.h
          const char* doc =
R"""(Throw a warning if the matrix size is either 1x1 or 2x2.)""";
        } WarnOnSmallMatrixSize;
        // Symbol: drake::solvers::PositiveSemidefiniteConstraint::matrix_rows
        struct /* matrix_rows */ {
          // Source: drake/solvers/constraint.h
          const char* doc = R"""()""";
        } matrix_rows;
      } PositiveSemidefiniteConstraint;
      // Symbol: drake::solvers::ProgramAttribute
      struct /* ProgramAttribute */ {
        // Source: drake/solvers/program_attribute.h
        const char* doc = R"""()""";
        // Symbol: drake::solvers::ProgramAttribute::kBinaryVariable
        struct /* kBinaryVariable */ {
          // Source: drake/solvers/program_attribute.h
          const char* doc = R"""(Variable taking binary value {0, 1}.)""";
        } kBinaryVariable;
        // Symbol: drake::solvers::ProgramAttribute::kCallback
        struct /* kCallback */ {
          // Source: drake/solvers/program_attribute.h
          const char* doc =
R"""(Supports callback during solving the problem.)""";
        } kCallback;
        // Symbol: drake::solvers::ProgramAttribute::kExponentialConeConstraint
        struct /* kExponentialConeConstraint */ {
          // Source: drake/solvers/program_attribute.h
          const char* doc = R"""(An exponential cone constraint.)""";
        } kExponentialConeConstraint;
        // Symbol: drake::solvers::ProgramAttribute::kGenericConstraint
        struct /* kGenericConstraint */ {
          // Source: drake/solvers/program_attribute.h
          const char* doc =
R"""(A generic constraint, doesn't belong to any specific)""";
        } kGenericConstraint;
        // Symbol: drake::solvers::ProgramAttribute::kGenericCost
        struct /* kGenericCost */ {
          // Source: drake/solvers/program_attribute.h
          const char* doc =
R"""(A generic cost, doesn't belong to any specific cost type)""";
        } kGenericCost;
        // Symbol: drake::solvers::ProgramAttribute::kL2NormCost
        struct /* kL2NormCost */ {
          // Source: drake/solvers/program_attribute.h
          const char* doc = R"""(An L2 norm |Ax+b|)""";
        } kL2NormCost;
        // Symbol: drake::solvers::ProgramAttribute::kLinearComplementarityConstraint
        struct /* kLinearComplementarityConstraint */ {
          // Source: drake/solvers/program_attribute.h
          const char* doc = R"""(A linear complementarity constraint in)""";
        } kLinearComplementarityConstraint;
        // Symbol: drake::solvers::ProgramAttribute::kLinearConstraint
        struct /* kLinearConstraint */ {
          // Source: drake/solvers/program_attribute.h
          const char* doc = R"""(A constraint on a linear function.)""";
        } kLinearConstraint;
        // Symbol: drake::solvers::ProgramAttribute::kLinearCost
        struct /* kLinearCost */ {
          // Source: drake/solvers/program_attribute.h
          const char* doc = R"""(A linear function as the cost.)""";
        } kLinearCost;
        // Symbol: drake::solvers::ProgramAttribute::kLinearEqualityConstraint
        struct /* kLinearEqualityConstraint */ {
          // Source: drake/solvers/program_attribute.h
          const char* doc =
R"""(An equality constraint on a linear function.)""";
        } kLinearEqualityConstraint;
        // Symbol: drake::solvers::ProgramAttribute::kLorentzConeConstraint
        struct /* kLorentzConeConstraint */ {
          // Source: drake/solvers/program_attribute.h
          const char* doc = R"""(A Lorentz cone constraint.)""";
        } kLorentzConeConstraint;
        // Symbol: drake::solvers::ProgramAttribute::kPositiveSemidefiniteConstraint
        struct /* kPositiveSemidefiniteConstraint */ {
          // Source: drake/solvers/program_attribute.h
          const char* doc = R"""(A positive semidefinite constraint.)""";
        } kPositiveSemidefiniteConstraint;
        // Symbol: drake::solvers::ProgramAttribute::kQuadraticConstraint
        struct /* kQuadraticConstraint */ {
          // Source: drake/solvers/program_attribute.h
          const char* doc = R"""(A constraint on a quadratic function.)""";
        } kQuadraticConstraint;
        // Symbol: drake::solvers::ProgramAttribute::kQuadraticCost
        struct /* kQuadraticCost */ {
          // Source: drake/solvers/program_attribute.h
          const char* doc = R"""(A quadratic function as the cost.)""";
        } kQuadraticCost;
        // Symbol: drake::solvers::ProgramAttribute::kRotatedLorentzConeConstraint
        struct /* kRotatedLorentzConeConstraint */ {
          // Source: drake/solvers/program_attribute.h
          const char* doc = R"""(A rotated Lorentz cone constraint.)""";
        } kRotatedLorentzConeConstraint;
      } ProgramAttribute;
      // Symbol: drake::solvers::ProgramAttributes
      struct /* ProgramAttributes */ {
        // Source: drake/solvers/program_attribute.h
        const char* doc = R"""()""";
      } ProgramAttributes;
      // Symbol: drake::solvers::ProgramType
      struct /* ProgramType */ {
        // Source: drake/solvers/program_attribute.h
        const char* doc =
R"""(A coarse categorization of the optimization problem based on the type
of constraints/costs/variables. Notice that Drake chooses the solver
based on a finer category; for example we have a specific solver for
equality-constrained convex QP.)""";
        // Symbol: drake::solvers::ProgramType::kCGP
        struct /* kCGP */ {
          // Source: drake/solvers/program_attribute.h
          const char* doc =
R"""(Conic Geometric Programming, this is a superset that unifies GP and
SDP. Refer to http://people.lids.mit.edu/pari/cgp_preprint.pdf for
more details.)""";
        } kCGP;
        // Symbol: drake::solvers::ProgramType::kGP
        struct /* kGP */ {
          // Source: drake/solvers/program_attribute.h
          const char* doc =
R"""(Geometric Programming, with a linear cost and exponential cone
constraints.)""";
        } kGP;
        // Symbol: drake::solvers::ProgramType::kLCP
        struct /* kLCP */ {
          // Source: drake/solvers/program_attribute.h
          const char* doc =
R"""(Linear Complementarity Programs. Programs with linear complementary
constraints and no cost.)""";
        } kLCP;
        // Symbol: drake::solvers::ProgramType::kLP
        struct /* kLP */ {
          // Source: drake/solvers/program_attribute.h
          const char* doc =
R"""(Linear Programming, with a linear cost and linear constraints.)""";
        } kLP;
        // Symbol: drake::solvers::ProgramType::kMILP
        struct /* kMILP */ {
          // Source: drake/solvers/program_attribute.h
          const char* doc =
R"""(Mixed-integer Linear Programming. LP with some variables taking binary
values.)""";
        } kMILP;
        // Symbol: drake::solvers::ProgramType::kMIQP
        struct /* kMIQP */ {
          // Source: drake/solvers/program_attribute.h
          const char* doc =
R"""(Mixed-integer Quadratic Programming. QP with some variables taking
binary values.)""";
        } kMIQP;
        // Symbol: drake::solvers::ProgramType::kMISDP
        struct /* kMISDP */ {
          // Source: drake/solvers/program_attribute.h
          const char* doc =
R"""(Mixed-integer Semidefinite Programming. SDP with some variables taking
binary values.)""";
        } kMISDP;
        // Symbol: drake::solvers::ProgramType::kMISOCP
        struct /* kMISOCP */ {
          // Source: drake/solvers/program_attribute.h
          const char* doc =
R"""(Mixed-integer Second-order Cone Programming. SOCP with some variables
taking binary values.)""";
        } kMISOCP;
        // Symbol: drake::solvers::ProgramType::kNLP
        struct /* kNLP */ {
          // Source: drake/solvers/program_attribute.h
          const char* doc =
R"""(nonlinear programming. Programs with generic costs or constraints.)""";
        } kNLP;
        // Symbol: drake::solvers::ProgramType::kQP
        struct /* kQP */ {
          // Source: drake/solvers/program_attribute.h
          const char* doc =
R"""(Quadratic Programming, with a convex quadratic cost and linear
constraints.)""";
        } kQP;
        // Symbol: drake::solvers::ProgramType::kQuadraticCostConicConstraint
        struct /* kQuadraticCostConicConstraint */ {
          // Source: drake/solvers/program_attribute.h
          const char* doc =
R"""(convex quadratic cost with nonlinear conic constraints.)""";
        } kQuadraticCostConicConstraint;
        // Symbol: drake::solvers::ProgramType::kSDP
        struct /* kSDP */ {
          // Source: drake/solvers/program_attribute.h
          const char* doc =
R"""(Semidefinite Programming, with a linear cost and positive semidefinite
matrix constraints.)""";
        } kSDP;
        // Symbol: drake::solvers::ProgramType::kSOCP
        struct /* kSOCP */ {
          // Source: drake/solvers/program_attribute.h
          const char* doc =
R"""(Second-order Cone Programming, with a linear cost and second-order
cone constraints.)""";
        } kSOCP;
        // Symbol: drake::solvers::ProgramType::kUnknown
        struct /* kUnknown */ {
          // Source: drake/solvers/program_attribute.h
          const char* doc =
R"""(Does not fall into any of the types above.)""";
        } kUnknown;
      } ProgramType;
      // Symbol: drake::solvers::ProjectedGradientDescentSolver
      struct /* ProjectedGradientDescentSolver */ {
        // Source: drake/solvers/projected_gradient_descent_solver.h
        const char* doc =
R"""(Solves a nonlinear program via the projected gradient descent
algorithm. The gradient is determined by differentiation of the costs,
or the user can supply a custom gradient function. The projection step
is itself an optimization problem in general, but the user can supply
a custom projection function. The user can also specify a specific
solver interface to be used to solve the projection problem.

The solver terminates if - the projection step fails to find a
feasible solution, - the 2-norm distance between subsequent iterates
is less than the user-specified tolerance (see
ProjectedGradientDescentSolver∷ConvergenceTolOptionName
"ConvergenceTolOptionName"), or - a maximum number of iterations have
been run (see ProjectedGradientDescentSolver∷MaxIterationsOptionName
"MaxIterationsOptionName").

Warning:
    This feature is considered to be **experimental** and may change
    or be removed at any time, without any deprecation notice ahead of
    time.)""";
        // Symbol: drake::solvers::ProjectedGradientDescentSolver::BacktrackingAlpha0OptionName
        struct /* BacktrackingAlpha0OptionName */ {
          // Source: drake/solvers/projected_gradient_descent_solver.h
          const char* doc =
R"""(Returns:
    string key for SolverOptions to set the value of alpha_0 to use
    for the backtracking line search. Must be positive.)""";
        } BacktrackingAlpha0OptionName;
        // Symbol: drake::solvers::ProjectedGradientDescentSolver::BacktrackingCOptionName
        struct /* BacktrackingCOptionName */ {
          // Source: drake/solvers/projected_gradient_descent_solver.h
          const char* doc =
R"""(Returns:
    string key for SolverOptions to set the value of c to use for the
    backtracking line search. Must be between 0 and 1.)""";
        } BacktrackingCOptionName;
        // Symbol: drake::solvers::ProjectedGradientDescentSolver::BacktrackingTauOptionName
        struct /* BacktrackingTauOptionName */ {
          // Source: drake/solvers/projected_gradient_descent_solver.h
          const char* doc =
R"""(Returns:
    string key for SolverOptions to set the value of tau to use for
    the backtracking line search. Must be between 0 and 1.)""";
        } BacktrackingTauOptionName;
        // Symbol: drake::solvers::ProjectedGradientDescentSolver::ConvergenceTolOptionName
        struct /* ConvergenceTolOptionName */ {
          // Source: drake/solvers/projected_gradient_descent_solver.h
          const char* doc =
R"""(Returns:
    string key for SolverOptions to set the threshold used to
    determine convergence. It must be positive.)""";
        } ConvergenceTolOptionName;
        // Symbol: drake::solvers::ProjectedGradientDescentSolver::MaxIterationsOptionName
        struct /* MaxIterationsOptionName */ {
          // Source: drake/solvers/projected_gradient_descent_solver.h
          const char* doc =
R"""(Returns:
    string key for SolverOptions to set the maximum number of
    iterations. It must be a positive integer.)""";
        } MaxIterationsOptionName;
        // Symbol: drake::solvers::ProjectedGradientDescentSolver::ProgramAttributesSatisfied
        struct /* ProgramAttributesSatisfied */ {
          // Source: drake/solvers/projected_gradient_descent_solver.h
          const char* doc = R"""()""";
        } ProgramAttributesSatisfied;
        // Symbol: drake::solvers::ProjectedGradientDescentSolver::ProjectedGradientDescentSolver
        struct /* ctor */ {
          // Source: drake/solvers/projected_gradient_descent_solver.h
          const char* doc = R"""()""";
        } ctor;
        // Symbol: drake::solvers::ProjectedGradientDescentSolver::SetCustomGradientFunction
        struct /* SetCustomGradientFunction */ {
          // Source: drake/solvers/projected_gradient_descent_solver.h
          const char* doc =
R"""(Specify a custom gradient function. Otherwise, this solver will
differentiate through the costs in the MathematicalProgram it's used
to solve.)""";
        } SetCustomGradientFunction;
        // Symbol: drake::solvers::ProjectedGradientDescentSolver::SetCustomProjectionFunction
        struct /* SetCustomProjectionFunction */ {
          // Source: drake/solvers/projected_gradient_descent_solver.h
          const char* doc =
R"""(Specify a custom projection function. Otherwise, this solver will
attempt to solve the L2 projection onto the feasible set of the
MathematicalProgram it's used to solve. The projection function should
return a boolean value indicating success or failure. It should take
in two arguments: the point we are trying to stay close to, and an
output argument where the projected value will be placed.)""";
        } SetCustomProjectionFunction;
        // Symbol: drake::solvers::ProjectedGradientDescentSolver::SetProjectionSolverInterface
        struct /* SetProjectionSolverInterface */ {
          // Source: drake/solvers/projected_gradient_descent_solver.h
          const char* doc =
R"""(Specify a solver interface to be used when solving the L2 projection
onto the feasible set of the MathematicalProgram it's being used to
solve.)""";
        } SetProjectionSolverInterface;
        // Symbol: drake::solvers::ProjectedGradientDescentSolver::id
        struct /* id */ {
          // Source: drake/solvers/projected_gradient_descent_solver.h
          const char* doc = R"""()""";
        } id;
        // Symbol: drake::solvers::ProjectedGradientDescentSolver::is_available
        struct /* is_available */ {
          // Source: drake/solvers/projected_gradient_descent_solver.h
          const char* doc = R"""()""";
        } is_available;
        // Symbol: drake::solvers::ProjectedGradientDescentSolver::is_enabled
        struct /* is_enabled */ {
          // Source: drake/solvers/projected_gradient_descent_solver.h
          const char* doc = R"""()""";
        } is_enabled;
      } ProjectedGradientDescentSolver;
      // Symbol: drake::solvers::QuadraticConstraint
      struct /* QuadraticConstraint */ {
        // Source: drake/solvers/constraint.h
        const char* doc =
R"""(lb ≤ .5 xᵀQx + bᵀx ≤ ub Without loss of generality, the class stores a
symmetric matrix Q. For a non-symmetric matrix Q₀, we can define Q =
(Q₀ + Q₀ᵀ) / 2, since xᵀQ₀x = xᵀQ₀ᵀx = xᵀ*(Q₀+Q₀ᵀ)/2 *x. The first
equality holds because the transpose of a scalar is the scalar itself.
Hence we can always convert a non-symmetric matrix Q₀ to a symmetric
matrix Q.)""";
        // Symbol: drake::solvers::QuadraticConstraint::HessianType
        struct /* HessianType */ {
          // Source: drake/solvers/constraint.h
          const char* doc =
R"""(Whether the Hessian matrix is positive semidefinite, negative
semidefinite, indefinite or a zero-matrix.)""";
          // Symbol: drake::solvers::QuadraticConstraint::HessianType::kIndefinite
          struct /* kIndefinite */ {
            // Source: drake/solvers/constraint.h
            const char* doc = R"""()""";
          } kIndefinite;
          // Symbol: drake::solvers::QuadraticConstraint::HessianType::kNegativeSemidefinite
          struct /* kNegativeSemidefinite */ {
            // Source: drake/solvers/constraint.h
            const char* doc = R"""()""";
          } kNegativeSemidefinite;
          // Symbol: drake::solvers::QuadraticConstraint::HessianType::kPositiveSemidefinite
          struct /* kPositiveSemidefinite */ {
            // Source: drake/solvers/constraint.h
            const char* doc = R"""()""";
          } kPositiveSemidefinite;
          // Symbol: drake::solvers::QuadraticConstraint::HessianType::kZero
          struct /* kZero */ {
            // Source: drake/solvers/constraint.h
            const char* doc = R"""()""";
          } kZero;
        } HessianType;
        // Symbol: drake::solvers::QuadraticConstraint::Q
        struct /* Q */ {
          // Source: drake/solvers/constraint.h
          const char* doc =
R"""(The symmetric matrix Q, being the Hessian of this constraint.)""";
        } Q;
        // Symbol: drake::solvers::QuadraticConstraint::QuadraticConstraint
        struct /* ctor */ {
          // Source: drake/solvers/constraint.h
          const char* doc =
R"""(Construct a quadratic constraint.

Template parameter ``DerivedQ``:
    The type for Q.

Template parameter ``Derivedb``:
    The type for b.

Parameter ``Q0``:
    The square matrix. Notice that Q₀ does not have to be symmetric.

Parameter ``b``:
    The linear coefficient.

Parameter ``lb``:
    The lower bound.

Parameter ``ub``:
    The upper bound.

Parameter ``hessian_type``:
    (optional) Indicates the type of Hessian matrix Q0. If
    hessian_type is not std∷nullopt, then the user guarantees the type
    of Q0. If hessian_type=std∷nullopt, then QuadraticConstraint will
    check the type of Q0. To speed up the constructor, set
    hessian_type != std∷nullopt if you can. If this type is set
    incorrectly, then the downstream code (for example the solver)
    will malfunction.

Raises:
    RuntimeError if Q0 isn't a square matrix, or b.rows() !=
    Q0.rows().)""";
        } ctor;
        // Symbol: drake::solvers::QuadraticConstraint::UpdateCoefficients
        struct /* UpdateCoefficients */ {
          // Source: drake/solvers/constraint.h
          const char* doc =
R"""(Updates the quadratic and linear term of the constraint. The new
matrices need to have the same dimension as before.

Parameter ``new_Q``:
    new quadratic term

Parameter ``new_b``:
    new linear term

Parameter ``hessian_type``:
    (optional) Indicates the type of Hessian matrix Q0. If
    hessian_type is not std∷nullopt, then the user guarantees the type
    of Q0. If hessian_type=std∷nullopt, then QuadraticConstraint will
    check the type of Q0. To speed up the constructor, set
    hessian_type != std∷nullopt if you can.)""";
        } UpdateCoefficients;
        // Symbol: drake::solvers::QuadraticConstraint::b
        struct /* b */ {
          // Source: drake/solvers/constraint.h
          const char* doc = R"""()""";
        } b;
        // Symbol: drake::solvers::QuadraticConstraint::hessian_type
        struct /* hessian_type */ {
          // Source: drake/solvers/constraint.h
          const char* doc = R"""()""";
        } hessian_type;
        // Symbol: drake::solvers::QuadraticConstraint::is_convex
        struct /* is_convex */ {
          // Source: drake/solvers/constraint.h
          const char* doc =
R"""(Returns if this quadratic constraint is convex.)""";
        } is_convex;
      } QuadraticConstraint;
      // Symbol: drake::solvers::QuadraticCost
      struct /* QuadraticCost */ {
        // Source: drake/solvers/cost.h
        const char* doc =
R"""(Implements a cost of the form

.. math:: .5 x'Qx + b'x + c

.)""";
        // Symbol: drake::solvers::QuadraticCost::Q
        struct /* Q */ {
          // Source: drake/solvers/cost.h
          const char* doc =
R"""(Returns the symmetric matrix Q, as the Hessian of the cost.)""";
        } Q;
        // Symbol: drake::solvers::QuadraticCost::QuadraticCost
        struct /* ctor */ {
          // Source: drake/solvers/cost.h
          const char* doc =
R"""(Constructs a cost of the form

.. math:: .5 x'Qx + b'x + c

.

Parameter ``Q``:
    Quadratic term.

Parameter ``b``:
    Linear term.

Parameter ``c``:
    (optional) Constant term.

Parameter ``is_hessian_psd``:
    (optional) Indicates if the Hessian matrix Q is positive
    semidefinite (psd) or not. If set to true, then the user
    guarantees that Q is psd; if set to false, then the user
    guarantees that Q is not psd. If set to std∷nullopt, then the
    constructor will check if Q is psd or not. The default is
    std∷nullopt. To speed up the constructor, set is_hessian_psd to
    either true or false.)""";
        } ctor;
        // Symbol: drake::solvers::QuadraticCost::UpdateCoefficients
        struct /* UpdateCoefficients */ {
          // Source: drake/solvers/cost.h
          const char* doc =
R"""(Updates the quadratic and linear term of the constraint. The new
matrices need to have the same dimension as before.

Parameter ``new_Q``:
    New quadratic term.

Parameter ``new_b``:
    New linear term.

Parameter ``new_c``:
    (optional) New constant term.

Parameter ``is_hessian_psd``:
    (optional) Indicates if the Hessian matrix Q is positive
    semidefinite (psd) or not. If set to true, then the user
    guarantees that Q is psd; if set to false, then the user
    guarantees that Q is not psd. If set to std∷nullopt, then this
    function will check if Q is psd or not. The default is
    std∷nullopt. To speed up the computation, set is_hessian_psd to
    either true or false.)""";
        } UpdateCoefficients;
        // Symbol: drake::solvers::QuadraticCost::UpdateHessianEntry
        struct /* UpdateHessianEntry */ {
          // Source: drake/solvers/cost.h
          const char* doc =
R"""(Updates both Q(i, j) and Q(j, i) to val

Parameter ``is_hessian_psd``:
    If this is ``nullopt``, the new Hessian is checked (possibly
    expensively) for PSD-ness. If this is set true/false, the cost's
    convexity is updated to that value without checking (it is the
    user's responsibility to make sure the flag is set correctly).

Note:
    If you have multiple entries in the Hessian matrix to update, and
    you don't specify is_hessian_psd, then it is much faster to call
    UpdateCoefficients(new_A, new_b) where new_A contains all the
    updated entries.)""";
        } UpdateHessianEntry;
        // Symbol: drake::solvers::QuadraticCost::b
        struct /* b */ {
          // Source: drake/solvers/cost.h
          const char* doc = R"""()""";
        } b;
        // Symbol: drake::solvers::QuadraticCost::c
        struct /* c */ {
          // Source: drake/solvers/cost.h
          const char* doc = R"""()""";
        } c;
        // Symbol: drake::solvers::QuadraticCost::is_convex
        struct /* is_convex */ {
          // Source: drake/solvers/cost.h
          const char* doc =
R"""(Returns true if this cost is convex. A quadratic cost if convex if and
only if its Hessian matrix Q is positive semidefinite.)""";
        } is_convex;
        // Symbol: drake::solvers::QuadraticCost::update_constant_term
        struct /* update_constant_term */ {
          // Source: drake/solvers/cost.h
          const char* doc = R"""(Updates the constant term to ``new_c``.)""";
        } update_constant_term;
        // Symbol: drake::solvers::QuadraticCost::update_linear_coefficient_entry
        struct /* update_linear_coefficient_entry */ {
          // Source: drake/solvers/cost.h
          const char* doc = R"""(Updates b(i)=val.)""";
        } update_linear_coefficient_entry;
      } QuadraticCost;
      // Symbol: drake::solvers::QuadraticallySmoothedHingeLoss
      struct /* QuadraticallySmoothedHingeLoss */ {
        // Source: drake/solvers/minimum_value_constraint.h
        const char* doc =
R"""(A linear hinge loss, smoothed with a quadratic loss near the origin.
The formulation is in equation (6) of [1]. The penalty is <pre
class="unicode-art"> ⎧ 0 if x ≥ 0 φ(x) = ⎨ x²/2 if -1 < x < 0 ⎩ -0.5 -
x if x ≤ -1. </pre> [1] "Loss Functions for Preference Levels:
Regression with Discrete Ordered Labels." by Jason Rennie and Nathan
Srebro, Proceedings of IJCAI multidisciplinary workshop on Advances in
Preference Handling.)""";
      } QuadraticallySmoothedHingeLoss;
      // Symbol: drake::solvers::RemoveFreeVariableMethod
      struct /* RemoveFreeVariableMethod */ {
        // Source: drake/solvers/sdpa_free_format.h
        const char* doc =
R"""(SDPA format doesn't accept free variables, namely the problem it
solves is in this form P1

max tr(C * X) s.t tr(Aᵢ*X) = aᵢ X ≽ 0.

Notice that the decision variable X has to be in the proper cone X ≽
0, and it doesn't accept free variable (without the conic constraint).
On the other hand, most real-world applications require free
variables, namely problems in this form P2

max tr(C * X) + dᵀs s.t tr(Aᵢ*X) + bᵢᵀs = aᵢ X ≽ 0 s is free.

In order to remove the free variables, we consider three approaches.
1. Replace a free variable s with two variables s = p - q, p ≥ 0, q ≥
0. 2. First write the dual of the problem P2 as D2

min aᵀy s.t ∑ᵢ yᵢAᵢ - C = Z Z ≽ 0 Bᵀ * y = d,

where bᵢᵀ is the i'th row of B. The last constraint Bᵀ * y = d means y
= ŷ + Nt, where Bᵀ * ŷ = d, and N is the null space of Bᵀ. Hence, D2
is equivalent to the following problem, D3

min aᵀNt + aᵀŷ s.t ∑ᵢ tᵢFᵢ - (C -∑ᵢ ŷᵢAᵢ) = Z Z ≽ 0,

where Fᵢ = ∑ⱼ NⱼᵢAⱼ. D3 is the dual of the following primal problem P3
without free variables

max tr((C-∑ᵢ ŷᵢAᵢ)*X̂) + aᵀŷ s.t tr(FᵢX̂) = (Nᵀa)(i) X̂ ≽ 0.

Then (X, s) = (X̂, B⁻¹(a - tr(Aᵢ X̂))) is the solution to the original
problem P2. 3. Add a slack variable t, with the Lorentz cone
constraint t ≥ sqrt(sᵀs).)""";
        // Symbol: drake::solvers::RemoveFreeVariableMethod::kLorentzConeSlack
        struct /* kLorentzConeSlack */ {
          // Source: drake/solvers/sdpa_free_format.h
          const char* doc =
R"""(Approach 3, add a slack variable t with the lorentz cone constraint t
≥ sqrt(sᵀs).)""";
        } kLorentzConeSlack;
        // Symbol: drake::solvers::RemoveFreeVariableMethod::kNullspace
        struct /* kNullspace */ {
          // Source: drake/solvers/sdpa_free_format.h
          const char* doc =
R"""(Approach 2, reformulate the dual problem by considering the nullspace
of the linear constraint in the dual.)""";
        } kNullspace;
        // Symbol: drake::solvers::RemoveFreeVariableMethod::kTwoSlackVariables
        struct /* kTwoSlackVariables */ {
          // Source: drake/solvers/sdpa_free_format.h
          const char* doc =
R"""(Approach 1, replace a free variable s as s = y⁺ - y⁻, y⁺ ≥ 0, y⁻ ≥ 0.)""";
        } kTwoSlackVariables;
      } RemoveFreeVariableMethod;
      // Symbol: drake::solvers::RollPitchYawLimitOptions
      struct /* RollPitchYawLimitOptions */ {
        // Source: drake/solvers/rotation_constraint.h
        const char* doc = R"""()""";
        // Symbol: drake::solvers::RollPitchYawLimitOptions::kNoLimits
        struct /* kNoLimits */ {
          // Source: drake/solvers/rotation_constraint.h
          const char* doc = R"""()""";
        } kNoLimits;
        // Symbol: drake::solvers::RollPitchYawLimitOptions::kPitch_0_to_PI
        struct /* kPitch_0_to_PI */ {
          // Source: drake/solvers/rotation_constraint.h
          const char* doc = R"""()""";
        } kPitch_0_to_PI;
        // Symbol: drake::solvers::RollPitchYawLimitOptions::kPitch_0_to_PI_2
        struct /* kPitch_0_to_PI_2 */ {
          // Source: drake/solvers/rotation_constraint.h
          const char* doc = R"""()""";
        } kPitch_0_to_PI_2;
        // Symbol: drake::solvers::RollPitchYawLimitOptions::kPitch_NegPI_2_to_PI_2
        struct /* kPitch_NegPI_2_to_PI_2 */ {
          // Source: drake/solvers/rotation_constraint.h
          const char* doc = R"""()""";
        } kPitch_NegPI_2_to_PI_2;
        // Symbol: drake::solvers::RollPitchYawLimitOptions::kRPYError
        struct /* kRPYError */ {
          // Source: drake/solvers/rotation_constraint.h
          const char* doc = R"""(Do not use, to avoid & vs. && typos.)""";
        } kRPYError;
        // Symbol: drake::solvers::RollPitchYawLimitOptions::kRoll_0_to_PI
        struct /* kRoll_0_to_PI */ {
          // Source: drake/solvers/rotation_constraint.h
          const char* doc = R"""()""";
        } kRoll_0_to_PI;
        // Symbol: drake::solvers::RollPitchYawLimitOptions::kRoll_0_to_PI_2
        struct /* kRoll_0_to_PI_2 */ {
          // Source: drake/solvers/rotation_constraint.h
          const char* doc = R"""()""";
        } kRoll_0_to_PI_2;
        // Symbol: drake::solvers::RollPitchYawLimitOptions::kRoll_NegPI_2_to_PI_2
        struct /* kRoll_NegPI_2_to_PI_2 */ {
          // Source: drake/solvers/rotation_constraint.h
          const char* doc = R"""()""";
        } kRoll_NegPI_2_to_PI_2;
        // Symbol: drake::solvers::RollPitchYawLimitOptions::kYaw_0_to_PI
        struct /* kYaw_0_to_PI */ {
          // Source: drake/solvers/rotation_constraint.h
          const char* doc = R"""()""";
        } kYaw_0_to_PI;
        // Symbol: drake::solvers::RollPitchYawLimitOptions::kYaw_0_to_PI_2
        struct /* kYaw_0_to_PI_2 */ {
          // Source: drake/solvers/rotation_constraint.h
          const char* doc = R"""()""";
        } kYaw_0_to_PI_2;
        // Symbol: drake::solvers::RollPitchYawLimitOptions::kYaw_NegPI_2_to_PI_2
        struct /* kYaw_NegPI_2_to_PI_2 */ {
          // Source: drake/solvers/rotation_constraint.h
          const char* doc = R"""()""";
        } kYaw_NegPI_2_to_PI_2;
      } RollPitchYawLimitOptions;
      // Symbol: drake::solvers::RollPitchYawLimits
      struct /* RollPitchYawLimits */ {
        // Source: drake/solvers/rotation_constraint.h
        const char* doc = R"""()""";
      } RollPitchYawLimits;
      // Symbol: drake::solvers::RotatedLorentzConeConstraint
      struct /* RotatedLorentzConeConstraint */ {
        // Source: drake/solvers/constraint.h
        const char* doc =
R"""(Constraining that the linear expression :math:`z=Ax+b` lies within
rotated Lorentz cone. A vector z ∈ ℝ ⁿ lies within rotated Lorentz
cone, if

.. math:: z_0 \ge 0\
z_1 \ge 0\
z_0  z_1 \ge z_2^2 + z_3^2 + ... + z_{n-1}^2

where A ∈ ℝ ⁿˣᵐ, b ∈ ℝ ⁿ are given matrices.

For more information and visualization, please refer to
https://docs.mosek.com/modeling-cookbook/cqo.html (Fig 3.1))""";
        // Symbol: drake::solvers::RotatedLorentzConeConstraint::A
        struct /* A */ {
          // Source: drake/solvers/constraint.h
          const char* doc = R"""(Getter for A.)""";
        } A;
        // Symbol: drake::solvers::RotatedLorentzConeConstraint::A_dense
        struct /* A_dense */ {
          // Source: drake/solvers/constraint.h
          const char* doc = R"""(Getter for dense version of A.)""";
        } A_dense;
        // Symbol: drake::solvers::RotatedLorentzConeConstraint::RotatedLorentzConeConstraint
        struct /* ctor */ {
          // Source: drake/solvers/constraint.h
          const char* doc =
R"""(Raises:
    RuntimeError if A.rows() < 3.)""";
        } ctor;
        // Symbol: drake::solvers::RotatedLorentzConeConstraint::UpdateCoefficients
        struct /* UpdateCoefficients */ {
          // Source: drake/solvers/constraint.h
          const char* doc =
R"""(Updates the coefficients, the updated constraint is z=new_A * x +
new_b in the rotated Lorentz cone.

Raises:
    RuntimeError if the new_A.cols() != A.cols(), namely the variable
    size should not change.

Precondition:
    new_A.rows() >= 3 and new_A.rows() == new_b.rows().)""";
        } UpdateCoefficients;
        // Symbol: drake::solvers::RotatedLorentzConeConstraint::b
        struct /* b */ {
          // Source: drake/solvers/constraint.h
          const char* doc = R"""(Getter for b.)""";
        } b;
      } RotatedLorentzConeConstraint;
      // Symbol: drake::solvers::ScsSolver
      struct /* ScsSolver */ {
        // Source: drake/solvers/scs_solver.h
        const char* doc = R"""()""";
        // Symbol: drake::solvers::ScsSolver::Details
        struct /* Details */ {
          // Source: drake/solvers/scs_solver.h
          const char* doc =
R"""(Type of details stored in MathematicalProgramResult.)""";
        } Details;
        // Symbol: drake::solvers::ScsSolver::ProgramAttributesSatisfied
        struct /* ProgramAttributesSatisfied */ {
          // Source: drake/solvers/scs_solver.h
          const char* doc = R"""()""";
        } ProgramAttributesSatisfied;
        // Symbol: drake::solvers::ScsSolver::ScsSolver
        struct /* ctor */ {
          // Source: drake/solvers/scs_solver.h
          const char* doc = R"""()""";
        } ctor;
        // Symbol: drake::solvers::ScsSolver::UnsatisfiedProgramAttributes
        struct /* UnsatisfiedProgramAttributes */ {
          // Source: drake/solvers/scs_solver.h
          const char* doc = R"""()""";
        } UnsatisfiedProgramAttributes;
        // Symbol: drake::solvers::ScsSolver::id
        struct /* id */ {
          // Source: drake/solvers/scs_solver.h
          const char* doc = R"""()""";
        } id;
        // Symbol: drake::solvers::ScsSolver::is_available
        struct /* is_available */ {
          // Source: drake/solvers/scs_solver.h
          const char* doc = R"""()""";
        } is_available;
        // Symbol: drake::solvers::ScsSolver::is_enabled
        struct /* is_enabled */ {
          // Source: drake/solvers/scs_solver.h
          const char* doc = R"""()""";
        } is_enabled;
      } ScsSolver;
      // Symbol: drake::solvers::ScsSolverDetails
      struct /* ScsSolverDetails */ {
        // Source: drake/solvers/scs_solver.h
        const char* doc =
R"""(The SCS solver details after calling Solve() function. The user can
call MathematicalProgramResult∷get_solver_details<ScsSolver>() to
obtain the details.)""";
        // Symbol: drake::solvers::ScsSolverDetails::dual_objective
        struct /* dual_objective */ {
          // Source: drake/solvers/scs_solver.h
          const char* doc =
R"""(Dual objective value at termination. Equal to SCS_INFO.dobj)""";
        } dual_objective;
        // Symbol: drake::solvers::ScsSolverDetails::duality_gap
        struct /* duality_gap */ {
          // Source: drake/solvers/scs_solver.h
          const char* doc = R"""(duality gap. Equal to SCS_INFO.gap.)""";
        } duality_gap;
        // Symbol: drake::solvers::ScsSolverDetails::iter
        struct /* iter */ {
          // Source: drake/solvers/scs_solver.h
          const char* doc =
R"""(These are the information returned by SCS at termination, please refer
to "SCS_INFO" struct in
https://github.com/cvxgrp/scs/blob/master/include/scs.h Number of
iterations taken at termination. Equal to SCS_INFO.iter)""";
        } iter;
        // Symbol: drake::solvers::ScsSolverDetails::primal_objective
        struct /* primal_objective */ {
          // Source: drake/solvers/scs_solver.h
          const char* doc =
R"""(Primal objective value at termination. Equal to SCS_INFO.pobj)""";
        } primal_objective;
        // Symbol: drake::solvers::ScsSolverDetails::primal_residue
        struct /* primal_residue */ {
          // Source: drake/solvers/scs_solver.h
          const char* doc =
R"""(Primal equality residue. Equal to SCS_INFO.res_pri)""";
        } primal_residue;
        // Symbol: drake::solvers::ScsSolverDetails::residue_infeasibility
        struct /* residue_infeasibility */ {
          // Source: drake/solvers/scs_solver.h
          const char* doc =
R"""(infeasibility certificate residue. Equal to SCS_INFO.res_infeas)""";
        } residue_infeasibility;
        // Symbol: drake::solvers::ScsSolverDetails::residue_unbounded_a
        struct /* residue_unbounded_a */ {
          // Source: drake/solvers/scs_solver.h
          const char* doc =
R"""(unbounded certificate residue. Equal to SCS_INFO.res_unbdd_a)""";
        } residue_unbounded_a;
        // Symbol: drake::solvers::ScsSolverDetails::residue_unbounded_p
        struct /* residue_unbounded_p */ {
          // Source: drake/solvers/scs_solver.h
          const char* doc =
R"""(unbounded certificate residue. Equal to SCS_INFO.res_unbdd_p)""";
        } residue_unbounded_p;
        // Symbol: drake::solvers::ScsSolverDetails::s
        struct /* s */ {
          // Source: drake/solvers/scs_solver.h
          const char* doc =
R"""(The primal equality constraint slack, namely Ax + s = b where x is the
primal variable.)""";
        } s;
        // Symbol: drake::solvers::ScsSolverDetails::scs_setup_time
        struct /* scs_setup_time */ {
          // Source: drake/solvers/scs_solver.h
          const char* doc =
R"""(Time taken for SCS to setup in milliseconds. Equal to
SCS_INFO.setup_time.)""";
        } scs_setup_time;
        // Symbol: drake::solvers::ScsSolverDetails::scs_solve_time
        struct /* scs_solve_time */ {
          // Source: drake/solvers/scs_solver.h
          const char* doc =
R"""(Time taken for SCS to solve in millisecond. Equal to
SCS_INFO.solve_time.)""";
        } scs_solve_time;
        // Symbol: drake::solvers::ScsSolverDetails::scs_status
        struct /* scs_status */ {
          // Source: drake/solvers/scs_solver.h
          const char* doc =
R"""(The status of the solver at termination. Please refer to
https://github.com/cvxgrp/scs/blob/master/include/glbopts.h Note that
the SCS code on github master might be slightly more up-to-date than
the version used in Drake.)""";
        } scs_status;
        // Symbol: drake::solvers::ScsSolverDetails::y
        struct /* y */ {
          // Source: drake/solvers/scs_solver.h
          const char* doc =
R"""(The dual variable values at termination.)""";
        } y;
      } ScsSolverDetails;
      // Symbol: drake::solvers::SemidefiniteRelaxationOptions
      struct /* SemidefiniteRelaxationOptions */ {
        // Source: drake/solvers/semidefinite_relaxation.h
        const char* doc =
R"""(Configuration options for the MakeSemidefiniteRelaxation. Throughout
these options, we refer to the variables of the original optimization
program as y, and the semidefinite variable of the associate
relaxation as X.

X has the structure X = [Y, y] [yᵀ, one])""";
        // Symbol: drake::solvers::SemidefiniteRelaxationOptions::add_implied_linear_constraints
        struct /* add_implied_linear_constraints */ {
          // Source: drake/solvers/semidefinite_relaxation.h
          const char* doc =
R"""(Given a program with the linear equality constraints Ay = b, sets
whether to add the implied linear constraints [A, -b]X = 0 to the
semidefinite relaxation.)""";
        } add_implied_linear_constraints;
        // Symbol: drake::solvers::SemidefiniteRelaxationOptions::add_implied_linear_equality_constraints
        struct /* add_implied_linear_equality_constraints */ {
          // Source: drake/solvers/semidefinite_relaxation.h
          const char* doc =
R"""(Given a program with the linear constraints Ay ≤ b, sets whether to
add the implied linear constraints [A,-b]X[A,-b]ᵀ ≤ 0 to the
semidefinite relaxation.)""";
        } add_implied_linear_equality_constraints;
        // Symbol: drake::solvers::SemidefiniteRelaxationOptions::set_to_strongest
        struct /* set_to_strongest */ {
          // Source: drake/solvers/semidefinite_relaxation.h
          const char* doc =
R"""(Configure the semidefinite relaxation options to provide the strongest
possible semidefinite relaxation that we currently support. This in
general will give the tightest convex relaxation we support, but the
longest solve times.)""";
        } set_to_strongest;
        // Symbol: drake::solvers::SemidefiniteRelaxationOptions::set_to_weakest
        struct /* set_to_weakest */ {
          // Source: drake/solvers/semidefinite_relaxation.h
          const char* doc =
R"""(Configure the semidefinite relaxation options to provide the weakest
semidefinite relaxation that we currently support. This in general
will create the loosest convex relaxation we support, but the shortest
solve times. This is equivalent to the standard Shor Relaxation (see
Quadratic Optimization Problems by NZ Shor or Semidefinite Programming
by Vandenberghe and Boyd).)""";
        } set_to_weakest;
      } SemidefiniteRelaxationOptions;
      // Symbol: drake::solvers::SnoptSolver
      struct /* SnoptSolver */ {
        // Source: drake/solvers/snopt_solver.h
        const char* doc =
R"""(An implementation of SolverInterface for the commercially-licensed
SNOPT solver (https://ccom.ucsd.edu/~optimizers/solvers/snopt/).

Builds of Drake from source do not compile SNOPT by default, so
therefore SolverInterface∷available() will return false. You must
opt-in to build SNOPT per the documentation at
https://drake.mit.edu/bazel.html#snopt.

`Drake's pre-compiled binary releases
<https://drake.mit.edu/installation.html>`_ do incorporate SNOPT, so
therefore SolverInterface∷available() will return true. Thanks to
Philip E. Gill and Elizabeth Wong for their kind support.

There is no license configuration required to use SNOPT, but you may
set the environtment variable ``DRAKE_SNOPT_SOLVER_ENABLED`` to "0" to
force-disable SNOPT, in which case SolverInterface∷enabled() will
return false.)""";
        // Symbol: drake::solvers::SnoptSolver::Details
        struct /* Details */ {
          // Source: drake/solvers/snopt_solver.h
          const char* doc =
R"""(Type of details stored in MathematicalProgramResult.)""";
        } Details;
        // Symbol: drake::solvers::SnoptSolver::ProgramAttributesSatisfied
        struct /* ProgramAttributesSatisfied */ {
          // Source: drake/solvers/snopt_solver.h
          const char* doc = R"""()""";
        } ProgramAttributesSatisfied;
        // Symbol: drake::solvers::SnoptSolver::SnoptSolver
        struct /* ctor */ {
          // Source: drake/solvers/snopt_solver.h
          const char* doc = R"""()""";
        } ctor;
        // Symbol: drake::solvers::SnoptSolver::id
        struct /* id */ {
          // Source: drake/solvers/snopt_solver.h
          const char* doc = R"""()""";
        } id;
        // Symbol: drake::solvers::SnoptSolver::is_available
        struct /* is_available */ {
          // Source: drake/solvers/snopt_solver.h
          const char* doc = R"""()""";
        } is_available;
        // Symbol: drake::solvers::SnoptSolver::is_bounded_lp_broken
        struct /* is_bounded_lp_broken */ {
          // Source: drake/solvers/snopt_solver.h
          const char* doc =
R"""(For some reason, SNOPT 7.4 fails to detect a simple LP being
unbounded.)""";
        } is_bounded_lp_broken;
        // Symbol: drake::solvers::SnoptSolver::is_enabled
        struct /* is_enabled */ {
          // Source: drake/solvers/snopt_solver.h
          const char* doc =
R"""(Returns true iff the environment variable DRAKE_SNOPT_SOLVER_ENABLED
is unset or set to anything other than "0".)""";
        } is_enabled;
      } SnoptSolver;
      // Symbol: drake::solvers::SnoptSolverDetails
      struct /* SnoptSolverDetails */ {
        // Source: drake/solvers/snopt_solver.h
        const char* doc =
R"""(The SNOPT solver details after calling Solve() function. The user can
call MathematicalProgramResult∷get_solver_details<SnoptSolver>() to
obtain the details.)""";
        // Symbol: drake::solvers::SnoptSolverDetails::F
        struct /* F */ {
          // Source: drake/solvers/snopt_solver.h
          const char* doc =
R"""(The final value of the vector of problem functions F(x).)""";
        } F;
        // Symbol: drake::solvers::SnoptSolverDetails::Fmul
        struct /* Fmul */ {
          // Source: drake/solvers/snopt_solver.h
          const char* doc =
R"""(The final value of the dual variables (Lagrange multipliers) for the
general constraints F_lower <= F(x) <= F_upper.)""";
        } Fmul;
        // Symbol: drake::solvers::SnoptSolverDetails::info
        struct /* info */ {
          // Source: drake/solvers/snopt_solver.h
          const char* doc =
R"""(The snopt INFO field. Please refer to section 8.6 in "User's Guide for
SNOPT Version 7: Software for Large-Scale Nonlinear Programming"
(https://web.stanford.edu/group/SOL/guides/sndoc7.pdf) by Philip E.
Gill to interpret the INFO field.)""";
        } info;
        // Symbol: drake::solvers::SnoptSolverDetails::solve_time
        struct /* solve_time */ {
          // Source: drake/solvers/snopt_solver.h
          const char* doc =
R"""(The duration of the snopt solve in seconds.)""";
        } solve_time;
        // Symbol: drake::solvers::SnoptSolverDetails::xmul
        struct /* xmul */ {
          // Source: drake/solvers/snopt_solver.h
          const char* doc =
R"""(The final value of the dual variables for the bound constraint x_lower
<= x <= x_upper.)""";
        } xmul;
      } SnoptSolverDetails;
      // Symbol: drake::solvers::SolutionResult
      struct /* SolutionResult */ {
        // Source: drake/solvers/solution_result.h
        const char* doc = R"""()""";
        // Symbol: drake::solvers::SolutionResult::kDualInfeasible
        struct /* kDualInfeasible */ {
          // Source: drake/solvers/solution_result.h
          const char* doc =
R"""(Dual problem is infeasible. In this case we cannot infer the status of
the primal problem.)""";
        } kDualInfeasible;
        // Symbol: drake::solvers::SolutionResult::kInfeasibleConstraints
        struct /* kInfeasibleConstraints */ {
          // Source: drake/solvers/solution_result.h
          const char* doc = R"""(The primal is infeasible.)""";
        } kInfeasibleConstraints;
        // Symbol: drake::solvers::SolutionResult::kInfeasibleOrUnbounded
        struct /* kInfeasibleOrUnbounded */ {
          // Source: drake/solvers/solution_result.h
          const char* doc =
R"""(The primal is either infeasible or unbounded.)""";
        } kInfeasibleOrUnbounded;
        // Symbol: drake::solvers::SolutionResult::kInvalidInput
        struct /* kInvalidInput */ {
          // Source: drake/solvers/solution_result.h
          const char* doc = R"""(Invalid input.)""";
        } kInvalidInput;
        // Symbol: drake::solvers::SolutionResult::kIterationLimit
        struct /* kIterationLimit */ {
          // Source: drake/solvers/solution_result.h
          const char* doc = R"""(Reaches the iteration limits.)""";
        } kIterationLimit;
        // Symbol: drake::solvers::SolutionResult::kSolutionFound
        struct /* kSolutionFound */ {
          // Source: drake/solvers/solution_result.h
          const char* doc = R"""(Found the optimal solution.)""";
        } kSolutionFound;
        // Symbol: drake::solvers::SolutionResult::kSolutionResultNotSet
        struct /* kSolutionResultNotSet */ {
          // Source: drake/solvers/solution_result.h
          const char* doc =
R"""(The initial (invalid) solution result. This value should be
overwritten by the solver during Solve().)""";
        } kSolutionResultNotSet;
        // Symbol: drake::solvers::SolutionResult::kSolverSpecificError
        struct /* kSolverSpecificError */ {
          // Source: drake/solvers/solution_result.h
          const char* doc =
R"""(Solver-specific error. (Try
MathematicalProgramResult∷get_solver_details() or enabling verbose
solver output.))""";
        } kSolverSpecificError;
        // Symbol: drake::solvers::SolutionResult::kUnbounded
        struct /* kUnbounded */ {
          // Source: drake/solvers/solution_result.h
          const char* doc = R"""(The primal is unbounded.)""";
        } kUnbounded;
      } SolutionResult;
      // Symbol: drake::solvers::Solve
      struct /* Solve */ {
        // Source: drake/solvers/solve.h
        const char* doc_3args =
R"""(Solves an optimization program, with optional initial guess and solver
options. This function first chooses the best solver depending on the
availability of the solver and the program formulation; it then
constructs that solver and call the Solve function of that solver. The
optimization result is stored in the return argument.

Parameter ``prog``:
    Contains the formulation of the program, and possibly solver
    options.

Parameter ``initial_guess``:
    The initial guess for the decision variables. If an
    ``initial_guess`` is provided, then the solver uses
    ``initial_guess`` and ignores the initial guess stored in
    ``prog``.

Parameter ``solver_options``:
    The options in addition to those stored in ``prog``. For each
    option entry (like print out), there are 4 ways to set that
    option, and the priority given to the solver options is as follows
    (from lowest / least, to highest / most): 1. common option set on
    the MathematicalProgram itself 2. common option passed as an
    argument to Solve 3. solver-specific option set on the
    MathematicalProgram itself 4. solver-specific option passed as an
    argument to Solve

Returns:
    result The result of solving the program through the solver.)""";
        // Source: drake/solvers/solve.h
        const char* doc_2args =
R"""(Solves an optimization program with a given initial guess.)""";
      } Solve;
      // Symbol: drake::solvers::SolveInParallel
      struct /* SolveInParallel */ {
        // Source: drake/solvers/solve.h
        const char* doc_6args_progs_initial_guesses_solver_options_solver_ids_parallelism_dynamic_schedule =
R"""(Solves progs[i] into result[i], optionally using initial_guess[i] and
solver_options[i] if given, by invoking the solver at solver_ids[i] if
provided. If solver_ids[i] is nullopt then the best available solver
is selected for progs[i] depending on the availability of the solver
and the problem formulation. If solver_ids == nullptr then this is
done for every progs[i].

Uses at most parallelism cores, with static scheduling by default.

Parameter ``dynamic_schedule``:
    If dynamic_schedule is false then static scheduling is used and so
    each core will solve approximately 1/parallelism of the programs.
    This is most efficient when all the programs take approximately
    the same amount of time to solve. If dynamic_schedule is true,
    then dynamic scheduling is used and all the programs are queued
    into a single pool and each core will take the next program off
    the queue when it becomes available. This is best when each
    program takes a dramatically different amount of time to solve.

Note:
    When using a proprietary solver (e.g. Mosek) your organization may
    have limited license seats. It is recommended that the number of
    parallel solves does not exceed the total number of license seats.

Note:
    Only programs which are thread safe are solved concurrently.
    Programs that are not thread safe will be solved sequentially in a
    thread safe manner.

Raises:
    RuntimeError if initial_guess and solver_options are provided and
    not the same size as progs.

Raises:
    RuntimeError if any of the progs are nullptr.

Raises:
    RuntimeError if any of the programs cannot be solved.)""";
        // Source: drake/solvers/solve.h
        const char* doc_6args_progs_initial_guesses_solver_options_solver_id_parallelism_dynamic_schedule =
R"""(Provides the same functionality as SolveInParallel, but allows for
specifying a single solver id and solver option that is used when
solving all programs.

Raises:
    RuntimeError if the provided solver cannot solve all of progs.

Raises:
    RuntimeError if initial_guesses are provided and not the same size
    as progs.

Raises:
    RuntimeError if any of the progs are nullptr.)""";
      } SolveInParallel;
      // Symbol: drake::solvers::SolverBase
      struct /* SolverBase */ {
        // Source: drake/solvers/solver_base.h
        const char* doc =
R"""(Abstract base class used by implementations of individual solvers.)""";
        // Symbol: drake::solvers::SolverBase::AreProgramAttributesSatisfied
        struct /* AreProgramAttributesSatisfied */ {
          // Source: drake/solvers/solver_base.h
          const char* doc = R"""()""";
        } AreProgramAttributesSatisfied;
        // Symbol: drake::solvers::SolverBase::DoSolve
        struct /* DoSolve */ {
          // Source: drake/solvers/solver_base.h
          const char* doc =
R"""(Hook for subclasses to implement Solve. Prior to the SolverBase's call
to this method, the solver's availability and capabilities vs the
program attributes have already been checked, and the result's
set_solver_id() and set_decision_variable_index() have already been
set. The options and initial guess are already merged, i.e., the
DoSolve implementation should ignore prog's solver options and prog's
initial guess.)""";
        } DoSolve;
        // Symbol: drake::solvers::SolverBase::DoSolve2
        struct /* DoSolve2 */ {
          // Source: drake/solvers/solver_base.h
          const char* doc =
R"""((Internal use only) Like DoSolve() but using SpecificOptions instead
of SolverOptions. The "2" here means "version 2", to help disabiguate
various DoSolve overloads.

By default, DoSolve() will delegate to DoSolve2() and DoSolve2() will
throw. Subclasses should override exactly one of the various DoSolver
methods. Solvers wrapped inside Drake should override DoSolve2();
solvers wrapped in downstream projects must override DoSolve() since
this function is marked as internal use only.)""";
        } DoSolve2;
        // Symbol: drake::solvers::SolverBase::ExplainUnsatisfiedProgramAttributes
        struct /* ExplainUnsatisfiedProgramAttributes */ {
          // Source: drake/solvers/solver_base.h
          const char* doc = R"""()""";
        } ExplainUnsatisfiedProgramAttributes;
        // Symbol: drake::solvers::SolverBase::Solve
        struct /* Solve */ {
          // Source: drake/solvers/solver_base.h
          const char* doc =
R"""(Like SolverInterface∷Solve(), but the result is a return value instead
of an output argument.)""";
        } Solve;
        // Symbol: drake::solvers::SolverBase::SolverBase
        struct /* ctor */ {
          // Source: drake/solvers/solver_base.h
          const char* doc =
R"""(Constructs a SolverBase with the given default implementations of the
solver_id(), available(), enabled(), AreProgramAttributesSatisfied(),
and ExplainUnsatisfiedProgramAttributes() methods. Typically, the
subclass will simply pass the address of its static method, e.g.,
``&available``, for these functors. Any of the functors can be
nullptr, in which case the subclass must override the matching virtual
method instead, except for ``explain_unsatisfied`` which already has a
default implementation.)""";
        } ctor;
        // Symbol: drake::solvers::SolverBase::available
        struct /* available */ {
          // Source: drake/solvers/solver_base.h
          const char* doc = R"""()""";
        } available;
        // Symbol: drake::solvers::SolverBase::enabled
        struct /* enabled */ {
          // Source: drake/solvers/solver_base.h
          const char* doc = R"""()""";
        } enabled;
        // Symbol: drake::solvers::SolverBase::solver_id
        struct /* solver_id */ {
          // Source: drake/solvers/solver_base.h
          const char* doc = R"""()""";
        } solver_id;
      } SolverBase;
      // Symbol: drake::solvers::SolverId
      struct /* SolverId */ {
        // Source: drake/solvers/solver_id.h
        const char* doc =
R"""(Identifies a SolverInterface implementation.

A moved-from instance is guaranteed to be empty and will not compare
equal to any non-empty ID.)""";
        // Symbol: drake::solvers::SolverId::SolverId
        struct /* ctor */ {
          // Source: drake/solvers/solver_id.h
          const char* doc =
R"""(Constructs a specific, known solver type. Internally, a hidden integer
is allocated and assigned to this instance; all instances that share
an integer (including copies of this instance) are considered equal.
The solver names are not enforced to be unique, though we recommend
that they remain so in practice.

For best performance, choose a name that is 15 characters or less, so
that it fits within the libstdc++ "small string" optimization ("SSO").)""";
        } ctor;
        // Symbol: drake::solvers::SolverId::name
        struct /* name */ {
          // Source: drake/solvers/solver_id.h
          const char* doc = R"""()""";
        } name;
      } SolverId;
      // Symbol: drake::solvers::SolverInterface
      struct /* SolverInterface */ {
        // Source: drake/solvers/solver_interface.h
        const char* doc =
R"""(Interface used by implementations of individual solvers.)""";
        // Symbol: drake::solvers::SolverInterface::AreProgramAttributesSatisfied
        struct /* AreProgramAttributesSatisfied */ {
          // Source: drake/solvers/solver_interface.h
          const char* doc =
R"""(Returns true iff the program's attributes are compatible with this
solver's capabilities.)""";
        } AreProgramAttributesSatisfied;
        // Symbol: drake::solvers::SolverInterface::ExplainUnsatisfiedProgramAttributes
        struct /* ExplainUnsatisfiedProgramAttributes */ {
          // Source: drake/solvers/solver_interface.h
          const char* doc =
R"""(Describes the reasons (if any) why the program is incompatible with
this solver's capabilities. If AreProgramAttributesSatisfied would
return true for the program, then this function returns the empty
string.)""";
        } ExplainUnsatisfiedProgramAttributes;
        // Symbol: drake::solvers::SolverInterface::Solve
        struct /* Solve */ {
          // Source: drake/solvers/solver_interface.h
          const char* doc =
R"""(Solves an optimization program with optional initial guess and solver
options. Note that these initial guess and solver options are not
written to ``prog``. If the ``prog`` has set an initial guess, and
``initial_guess`` is set, then ``initial_guess`` takes priority. If
the ``prog`` has set an option for a solver, and ``solver_options``
contains a different value for the same option on the same solver,
then ``solver_options`` takes priority. Derived implementations of
this interface may elect to throw RuntimeError for badly formed
programs.)""";
        } Solve;
        // Symbol: drake::solvers::SolverInterface::SolverInterface
        struct /* ctor */ {
          // Source: drake/solvers/solver_interface.h
          const char* doc = R"""()""";
        } ctor;
        // Symbol: drake::solvers::SolverInterface::available
        struct /* available */ {
          // Source: drake/solvers/solver_interface.h
          const char* doc =
R"""(Returns true iff support for this solver has been compiled into Drake.
When this method returns false, the Solve method will throw.

Most solver implementations will always return true, but certain
solvers may have been excluded at compile-time due to licensing
restrictions, or to narrow Drake's dependency footprint. In Drake's
default build, only commercially-licensed solvers might return false.

Contrast this with enabled(), which reflects whether a solver has been
configured for use at runtime (not compile-time).

For details on linking commercial solvers, refer to the solvers' class
overview documentation, e.g., SnoptSolver, MosekSolver, GurobiSolver.)""";
        } available;
        // Symbol: drake::solvers::SolverInterface::enabled
        struct /* enabled */ {
          // Source: drake/solvers/solver_interface.h
          const char* doc =
R"""(Returns true iff this solver is properly configured for use at
runtime. When this method returns false, the Solve method will throw.

Most solver implementation will always return true, but certain
solvers require additional configuration before they may be used,
e.g., setting an environment variable to specify a license file or
license server. In Drake's default build, only commercially-licensed
solvers might return false.

Contrast this with available(), which reflects whether a solver has
been incorporated into Drake at compile-time (and has nothing to do
with the runtime configuration). A solver where available() returns
false may still return true for enabled() if it is properly
configured.

The mechanism to configure a particular solver implementation is
specific to the solver in question, but typically uses an environment
variable. For details on configuring commercial solvers, refer to the
solvers' class overview documentation, e.g., SnoptSolver, MosekSolver,
GurobiSolver.)""";
        } enabled;
        // Symbol: drake::solvers::SolverInterface::solver_id
        struct /* solver_id */ {
          // Source: drake/solvers/solver_interface.h
          const char* doc = R"""(Returns the identifier of this solver.)""";
        } solver_id;
      } SolverInterface;
      // Symbol: drake::solvers::SolverOptions
      struct /* SolverOptions */ {
        // Source: drake/solvers/solver_options.h
        const char* doc =
R"""(Stores options for multiple solvers. This interface does not do any
verification of solver parameters. It does not even verify that the
specified solver exists. Use this only when you have particular
knowledge of what solver is being invoked, and exactly what tuning is
required.

Supported solver names/options:

"SNOPT" -- Parameter names and values as specified in SNOPT User's
Guide section 7.7 "Description of the optional parameters", used as
described in section 7.5 for snSet(). The SNOPT user guide can be
obtained from https://web.stanford.edu/group/SOL/guides/sndoc7.pdf

"IPOPT" -- Parameter names and values as specified in IPOPT users
guide section "Options Reference"
https://coin-or.github.io/Ipopt/OPTIONS.html

"NLOPT" -- Parameter names and values are specified in
https://nlopt.readthedocs.io/en/latest/NLopt_C-plus-plus_Reference/
(in the Stopping criteria section). Besides these parameters, the user
can specify "algorithm" using a string of the algorithm name. The
complete set of algorithms is listed in "nlopt_algorithm_to_string()"
function in github.com/stevengj/nlopt/blob/master/src/api/general.c.
If you would like to use certain algorithm, for example
NLOPT_LD_SLSQP, call ``SetOption(NloptSolver∷id(),
NloptSolver∷AlgorithmName(), "LD_SLSQP");``

"GUROBI" -- Parameter name and values as specified in Gurobi Reference
Manual
https://docs.gurobi.com/projects/optimizer/en/12.0/reference/parameters.html

"SCS" -- Parameter name and values as specified in the struct
SCS_SETTINGS in SCS header file
https://github.com/cvxgrp/scs/blob/master/include/scs.h Note that the
SCS code on github master might be more up-to-date than the version
used in Drake.

"MOSEK™" -- Parameter name and values as specified in Mosek Reference
https://docs.mosek.com/11.1/capi/parameters.html

"OSQP" -- Parameter name and values as specified in OSQP Reference
https://osqp.org/docs/interfaces/solver_settings.html#solver-settings

"Clarabel" -- Parameter name and values as specified in Clarabel
https://oxfordcontrol.github.io/ClarabelDocs/stable/api_settings/ Note
that ``direct_solve_method`` is not supported in Drake yet. Clarabel's
boolean options should be passed as integers (0 or 1).

"CSDP" -- Parameter name and values as specified at
https://manpages.ubuntu.com/manpages/focal/en/man1/csdp-randgraph.1.html)""";
        // Symbol: drake::solvers::SolverOptions::Merge
        struct /* Merge */ {
          // Source: drake/solvers/solver_options.h
          const char* doc =
R"""(Merges the other solver options into this. If ``other`` and ``this``
option both define the same option for the same solver, we ignore the
one from ``other`` and keep the one from ``this``.)""";
        } Merge;
        // Symbol: drake::solvers::SolverOptions::OptionValue
        struct /* OptionValue */ {
          // Source: drake/solvers/solver_options.h
          const char* doc =
R"""(The values stored in SolverOptions can be double, int, or string. In
the future, we might re-order or add more allowed types without any
deprecation period, so be sure to use std∷visit or std∷get<T> to
retrieve the variant's value in a future-proof way.)""";
        } OptionValue;
        // Symbol: drake::solvers::SolverOptions::Serialize
        struct /* Serialize */ {
          // Source: drake/solvers/solver_options.h
          const char* doc =
R"""(Passes this object to an Archive. Refer to yaml_serialization "YAML
Serialization" for background.)""";
        } Serialize;
        // Symbol: drake::solvers::SolverOptions::SetOption
        struct /* SetOption */ {
          // Source: drake/solvers/solver_options.h
          const char* doc_3args =
R"""(Sets a solver option for a specific solver. If the solver doesn't
support the option, it will throw an exception during the Solve (not
when setting the option here).)""";
          // Source: drake/solvers/solver_options.h
          const char* doc_2args =
R"""(Sets a common option for all solvers supporting that option (for
example, printing the progress in each iteration). If the solver
doesn't support the option, the option is ignored.)""";
        } SetOption;
        // Symbol: drake::solvers::SolverOptions::operator!=
        struct /* operator_ne */ {
          // Source: drake/solvers/solver_options.h
          const char* doc = R"""()""";
        } operator_ne;
        // Symbol: drake::solvers::SolverOptions::options
        struct /* options */ {
          // Source: drake/solvers/solver_options.h
          const char* doc =
R"""(The options are indexed first by the solver name and second by the
key. In the case of Drake's common options, the solver name is
"Drake".)""";
        } options;
        // Symbol: drake::solvers::SolverOptions::to_string
        struct /* to_string */ {
          // Source: drake/solvers/solver_options.h
          const char* doc = R"""()""";
        } to_string;
        auto Serialize__fields() const {
          return std::array{
            std::make_pair("options", options.doc),
          };
        }
      } SolverOptions;
      // Symbol: drake::solvers::SolverType
      struct /* SolverType */ {
        // Source: drake/solvers/solver_type.h
        const char* doc =
R"""(This type only exists for backwards compatibility, and should not be
used in new code.)""";
        // Symbol: drake::solvers::SolverType::kClp
        struct /* kClp */ {
          // Source: drake/solvers/solver_type.h
          const char* doc = R"""()""";
        } kClp;
        // Symbol: drake::solvers::SolverType::kCsdp
        struct /* kCsdp */ {
          // Source: drake/solvers/solver_type.h
          const char* doc = R"""()""";
        } kCsdp;
        // Symbol: drake::solvers::SolverType::kEqualityConstrainedQP
        struct /* kEqualityConstrainedQP */ {
          // Source: drake/solvers/solver_type.h
          const char* doc = R"""()""";
        } kEqualityConstrainedQP;
        // Symbol: drake::solvers::SolverType::kGurobi
        struct /* kGurobi */ {
          // Source: drake/solvers/solver_type.h
          const char* doc = R"""()""";
        } kGurobi;
        // Symbol: drake::solvers::SolverType::kIpopt
        struct /* kIpopt */ {
          // Source: drake/solvers/solver_type.h
          const char* doc = R"""()""";
        } kIpopt;
        // Symbol: drake::solvers::SolverType::kLinearSystem
        struct /* kLinearSystem */ {
          // Source: drake/solvers/solver_type.h
          const char* doc = R"""()""";
        } kLinearSystem;
        // Symbol: drake::solvers::SolverType::kMobyLCP
        struct /* kMobyLCP */ {
          // Source: drake/solvers/solver_type.h
          const char* doc = R"""()""";
        } kMobyLCP;
        // Symbol: drake::solvers::SolverType::kMosek
        struct /* kMosek */ {
          // Source: drake/solvers/solver_type.h
          const char* doc = R"""()""";
        } kMosek;
        // Symbol: drake::solvers::SolverType::kNlopt
        struct /* kNlopt */ {
          // Source: drake/solvers/solver_type.h
          const char* doc = R"""()""";
        } kNlopt;
        // Symbol: drake::solvers::SolverType::kOsqp
        struct /* kOsqp */ {
          // Source: drake/solvers/solver_type.h
          const char* doc = R"""()""";
        } kOsqp;
        // Symbol: drake::solvers::SolverType::kScs
        struct /* kScs */ {
          // Source: drake/solvers/solver_type.h
          const char* doc = R"""()""";
        } kScs;
        // Symbol: drake::solvers::SolverType::kSnopt
        struct /* kSnopt */ {
          // Source: drake/solvers/solver_type.h
          const char* doc = R"""()""";
        } kSnopt;
        // Symbol: drake::solvers::SolverType::kUnrevisedLemke
        struct /* kUnrevisedLemke */ {
          // Source: drake/solvers/solver_type.h
          const char* doc = R"""()""";
        } kUnrevisedLemke;
      } SolverType;
      // Symbol: drake::solvers::SolverTypeConverter
      struct /* SolverTypeConverter */ {
        // Source: drake/solvers/solver_type_converter.h
        const char* doc =
R"""(Converts between SolverType and SolverId. This class only exists for
backwards compatibility, and should not be used in new code.)""";
        // Symbol: drake::solvers::SolverTypeConverter::IdToType
        struct /* IdToType */ {
          // Source: drake/solvers/solver_type_converter.h
          const char* doc =
R"""(Converts the given ID to its matching type, iff the type matches one
of SolverType's known values.)""";
        } IdToType;
        // Symbol: drake::solvers::SolverTypeConverter::SolverTypeConverter
        struct /* ctor */ {
          // Source: drake/solvers/solver_type_converter.h
          const char* doc = R"""()""";
        } ctor;
        // Symbol: drake::solvers::SolverTypeConverter::TypeToId
        struct /* TypeToId */ {
          // Source: drake/solvers/solver_type_converter.h
          const char* doc =
R"""(Converts the given type to its matching ID.)""";
        } TypeToId;
      } SolverTypeConverter;
      // Symbol: drake::solvers::UnrevisedLemkeSolver
      struct /* UnrevisedLemkeSolver */ {
        // Source: drake/solvers/unrevised_lemke_solver.h
        const char* doc =
R"""(A class for the Unrevised Implementation of Lemke Algorithm's for
solving Linear Complementarity Problems (LCPs). See MobyLcpSolver for
a description of LCPs. This code makes extensive use of the following
document: [Dai 2018] Dai, H. and Drumwright, E. Computing the
Principal Pivoting Transform for Solving Linear Complementarity
Problems with Lemke's Algorithm. (2018, located in
doc/pivot_column.pdf).)""";
        // Symbol: drake::solvers::UnrevisedLemkeSolver::ComputeZeroTolerance
        struct /* ComputeZeroTolerance */ {
          // Source: drake/solvers/unrevised_lemke_solver.h
          const char* doc =
R"""(Calculates the zero tolerance that the solver would compute if the
user does not specify a tolerance.)""";
        } ComputeZeroTolerance;
        // Symbol: drake::solvers::UnrevisedLemkeSolver::IsSolution
        struct /* IsSolution */ {
          // Source: drake/solvers/unrevised_lemke_solver.h
          const char* doc =
R"""(Checks whether a given candidate solution to the LCP Mz + q = w, z ≥
0, w ≥ 0, zᵀw = 0 is satisfied to a given tolerance. If the tolerance
is non-positive, this method computes a reasonable tolerance using M.)""";
        } IsSolution;
        // Symbol: drake::solvers::UnrevisedLemkeSolver::ProgramAttributesSatisfied
        struct /* ProgramAttributesSatisfied */ {
          // Source: drake/solvers/unrevised_lemke_solver.h
          const char* doc = R"""()""";
        } ProgramAttributesSatisfied;
        // Symbol: drake::solvers::UnrevisedLemkeSolver::SolveLcpLemke
        struct /* SolveLcpLemke */ {
          // Source: drake/solvers/unrevised_lemke_solver.h
          const char* doc =
R"""(Lemke's Algorithm for solving LCPs in the matrix class E, which
contains all strictly semimonotone matrices, all P-matrices, and all
strictly copositive matrices. The solver can be applied with
occasional success to problems outside of its guaranteed matrix
classes. Lemke's Algorithm is described in [Cottle 1992], Section 4.4.

The solver will denote failure on return if it exceeds a problem-size
dependent number of iterations.

Parameter ``M``:
    the LCP matrix.

Parameter ``q``:
    the LCP vector.

Parameter ``z``:
    the solution to the LCP on return (if the solver succeeds). If the
    length of z is equal to the length of q, the solver will attempt
    to use the basis from the last solution. This strategy can prove
    exceptionally fast if solutions differ little between successive
    calls. If the solver fails (returns ``False``), `z` will be set to
    the zero vector on return.

Parameter ``num_pivots``:
    the number of pivots used, on return.

Parameter ``zero_tol``:
    The tolerance for testing against zero. If the tolerance is
    negative (default) the solver will determine a generally
    reasonable tolerance.

Returns:
    ``True`` if the solver computes a solution to floating point
    tolerances (i.e., if IsSolution() returns ``True`` on the problem)
    and ``False`` otherwise.

Raises:
    RuntimeError if M is not square or the dimensions of M do not
    match the length of q.

* [Cottle 1992]      R. Cottle, J.-S. Pang, and R. Stone. The Linear
                     Complementarity Problem. Academic Press, 1992.)""";
        } SolveLcpLemke;
        // Symbol: drake::solvers::UnrevisedLemkeSolver::UnrevisedLemkeSolver<T>
        struct /* ctor */ {
          // Source: drake/solvers/unrevised_lemke_solver.h
          const char* doc = R"""()""";
        } ctor;
        // Symbol: drake::solvers::UnrevisedLemkeSolver::id
        struct /* id */ {
          // Source: drake/solvers/unrevised_lemke_solver.h
          const char* doc = R"""()""";
        } id;
        // Symbol: drake::solvers::UnrevisedLemkeSolver::is_available
        struct /* is_available */ {
          // Source: drake/solvers/unrevised_lemke_solver.h
          const char* doc = R"""()""";
        } is_available;
        // Symbol: drake::solvers::UnrevisedLemkeSolver::is_enabled
        struct /* is_enabled */ {
          // Source: drake/solvers/unrevised_lemke_solver.h
          const char* doc = R"""()""";
        } is_enabled;
      } UnrevisedLemkeSolver;
      // Symbol: drake::solvers::UnrevisedLemkeSolverId
      struct /* UnrevisedLemkeSolverId */ {
        // Source: drake/solvers/unrevised_lemke_solver.h
        const char* doc =
R"""(Non-template class for UnrevisedLemkeSolver<T> constants.)""";
        // Symbol: drake::solvers::UnrevisedLemkeSolverId::UnrevisedLemkeSolverId
        struct /* ctor */ {
          // Source: drake/solvers/unrevised_lemke_solver.h
          const char* doc = R"""()""";
        } ctor;
        // Symbol: drake::solvers::UnrevisedLemkeSolverId::id
        struct /* id */ {
          // Source: drake/solvers/unrevised_lemke_solver.h
          const char* doc =
R"""(Returns:
    same as SolverInterface∷solver_id())""";
        } id;
      } UnrevisedLemkeSolverId;
      // Symbol: drake::solvers::VariableRefList
      struct /* VariableRefList */ {
        // Source: drake/solvers/decision_variable.h
        const char* doc = R"""()""";
      } VariableRefList;
      // Symbol: drake::solvers::VectorXDecisionVariable
      struct /* VectorXDecisionVariable */ {
        // Source: drake/solvers/decision_variable.h
        const char* doc = R"""()""";
      } VectorXDecisionVariable;
      // Symbol: drake::solvers::VectorXIndeterminate
      struct /* VectorXIndeterminate */ {
        // Source: drake/solvers/indeterminate.h
        const char* doc =
R"""(VectorXIndeterminate is used as an alias for
Eigen∷Matrix<symbolic∷Variable, Eigen∷Dynamic, 1>.

See also:
    MatrixIndeterminate<int, int>)""";
      } VectorXIndeterminate;
      // Symbol: drake::solvers::VisualizationCallback
      struct /* VisualizationCallback */ {
        // Source: drake/solvers/evaluator_base.h
        const char* doc =
R"""(Defines a simple evaluator with no outputs that takes a callback
function pointer. This is intended for debugging / visualization of
intermediate results during an optimization (for solvers that support
it).)""";
        // Symbol: drake::solvers::VisualizationCallback::CallbackFunction
        struct /* CallbackFunction */ {
          // Source: drake/solvers/evaluator_base.h
          const char* doc = R"""()""";
        } CallbackFunction;
        // Symbol: drake::solvers::VisualizationCallback::EvalCallback
        struct /* EvalCallback */ {
          // Source: drake/solvers/evaluator_base.h
          const char* doc = R"""()""";
        } EvalCallback;
        // Symbol: drake::solvers::VisualizationCallback::VisualizationCallback
        struct /* ctor */ {
          // Source: drake/solvers/evaluator_base.h
          const char* doc = R"""()""";
        } ctor;
      } VisualizationCallback;
      // Symbol: drake::solvers::operator!=
      struct /* operator_ne */ {
        // Source: drake/solvers/solver_id.h
        const char* doc = R"""()""";
      } operator_ne;
      // Symbol: drake::solvers::to_string
      struct /* to_string */ {
        // Source: drake/solvers/common_solver_option.h
        const char* doc =
R"""(Returns the short, unadorned name of the option, e.g.,
``kPrintFileName``.)""";
      } to_string;
    } solvers;
  } drake;
} pydrake_doc_solvers;

#if defined(__GNUG__)
#pragma GCC diagnostic pop
#endif
