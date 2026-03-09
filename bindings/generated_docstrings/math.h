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

// #include "drake/math/autodiff.h"
// #include "drake/math/autodiff_gradient.h"
// #include "drake/math/barycentric.h"
// #include "drake/math/binomial_coefficient.h"
// #include "drake/math/bspline_basis.h"
// #include "drake/math/compute_numerical_gradient.h"
// #include "drake/math/continuous_algebraic_riccati_equation.h"
// #include "drake/math/continuous_lyapunov_equation.h"
// #include "drake/math/convert_time_derivative.h"
// #include "drake/math/cross_product.h"
// #include "drake/math/differentiable_norm.h"
// #include "drake/math/discrete_algebraic_riccati_equation.h"
// #include "drake/math/discrete_lyapunov_equation.h"
// #include "drake/math/eigen_sparse_triplet.h"
// #include "drake/math/evenly_distributed_pts_on_sphere.h"
// #include "drake/math/fast_pose_composition_functions.h"
// #include "drake/math/fourth_order_tensor.h"
// #include "drake/math/gradient.h"
// #include "drake/math/gradient_util.h"
// #include "drake/math/gray_code.h"
// #include "drake/math/hopf_coordinate.h"
// #include "drake/math/jacobian.h"
// #include "drake/math/knot_vector_type.h"
// #include "drake/math/linear_solve.h"
// #include "drake/math/matrix_util.h"
// #include "drake/math/normalize_vector.h"
// #include "drake/math/quadratic_form.h"
// #include "drake/math/quaternion.h"
// #include "drake/math/random_rotation.h"
// #include "drake/math/rigid_transform.h"
// #include "drake/math/roll_pitch_yaw.h"
// #include "drake/math/rotation_conversion_gradient.h"
// #include "drake/math/rotation_matrix.h"
// #include "drake/math/soft_min_max.h"
// #include "drake/math/unit_vector.h"
// #include "drake/math/wrap_to.h"

// Symbol: pydrake_doc_math
constexpr struct /* pydrake_doc_math */ {
  // Symbol: drake
  struct /* drake */ {
    // Symbol: drake::math
    struct /* math */ {
      // Symbol: drake::math::AreAutoDiffVecXdEqual
      struct /* AreAutoDiffVecXdEqual */ {
        // Source: drake/math/autodiff_gradient.h
        const char* doc =
R"""(Determines if a and b are equal. a equals to b if they have the same
value and gradients. TODO(hongkai.dai) implement and use
std∷equal_to<> for comparing Eigen vector of AutoDiffXd.)""";
      } AreAutoDiffVecXdEqual;
      // Symbol: drake::math::AreQuaternionsEqualForOrientation
      struct /* AreQuaternionsEqualForOrientation */ {
        // Source: drake/math/quaternion.h
        const char* doc =
R"""(This function tests whether two quaternions represent the same
orientation. This function converts each quaternion to its canonical
form and tests whether the absolute value of the difference in
corresponding elements of these canonical quaternions is within
tolerance.

Parameter ``quat1``:
    Quaternion [w, x, y, z] that relates two right-handed orthogonal
    unitary bases e.g., Ax, Ay, Az (A) to Bx, By, Bz (B). Note: quat
    is analogous to the rotation matrix R_AB.

Parameter ``quat2``:
    Quaternion with a description analogous to quat1.

Parameter ``tolerance``:
    Nonnegative real scalar defining the allowable difference in the
    orientation described by quat1 and quat2.

Returns:
    ``True`` if quat1 and quat2 represent the same orientation (to
    within tolerance), otherwise ``False``.)""";
      } AreQuaternionsEqualForOrientation;
      // Symbol: drake::math::BalanceQuadraticForms
      struct /* BalanceQuadraticForms */ {
        // Source: drake/math/quadratic_form.h
        const char* doc =
R"""(Given two quadratic forms, x'Sx > 0 and x'Px, (with P symmetric and
full rank), finds a change of variables x = Ty, which simultaneously
diagonalizes both forms (as inspired by "balanced truncation" in
model-order reduction [1]). In this note, we use abs(M) to indicate
the elementwise absolute value.

Adapting from [1], we observe that there is a family of coordinate
systems that can simultaneously diagonalize T'ST and T'PT. Using D to
denote a diagonal matrix, we call the result S-normal if T'ST = I and
abs(T'PT) = D⁻², call it P-normal if T'ST = D² and abs(T'PT) = I, and
call it "balanced" if T'ST = D and abs(T'PT) = D⁻¹. Note that if P >
0, then T'PT = D⁻¹.

We find x=Ty such that T'ST = D and abs(T'PT) = D⁻¹, where D is
diagonal. The recipe is: - Factorize S = LL', and choose R=L⁻¹. - Take
svd(RPR') = UΣV', and note that U=V for positive definite matrices,
and V is U up to a sign flip of the singular vectors for all symmetric
matrices. - Choose T = R'U Σ^{-1/4}, where the matrix exponent can be
taken elementwise because Σ is diagonal. This gives T'ST = Σ^{-1/2}
(by using U'U=I), and abs(T'PT) = Σ^{1/2}. If P > 0, then T'PT =
Σ^{1/2}.

Note that the numerical "balancing" can address the absolute scaling
of the quadratic forms, but not the relative scaling. To understand
this, consider the scalar case: we have two quadratic functions, sx²
and px², with s>0, p>0. We'd like to choose x=Ty so that sT²y² and
pT²y² are "balanced" (we'd like them both to be close to y²). We'll
choose T=p^{-1/4}s^{-1/4}, which gives sx² = sqrt(s/p)y², and px² =
sqrt(p/s)y². For instance if s=1e8 and p=1e8, then t=1e-4 and st^2 =
pt^2 = 1. But if s=10, p=1e7, then t=0.01, and st^2 = 1e-3, pt^2 =
1e3.

In the matrix case, the absolute scaling is important -- it ensures
that the two quadratic forms have the same matrix condition number and
makes them as close as possible to 1. Besides absolute scaling, in the
matrix case the balancing transform diagonalizes both quadratic forms.

[1] B. Moore, “Principal component analysis in linear systems:
Controllability, observability, and model reduction,” IEEE Trans.
Automat. Contr., vol. 26, no. 1, pp. 17–32, Feb. 1981.)""";
      } BalanceQuadraticForms;
      // Symbol: drake::math::BarycentricMesh
      struct /* BarycentricMesh */ {
        // Source: drake/math/barycentric.h
        const char* doc =
R"""(Represents a multi-linear function (from vector inputs to vector
outputs) by interpolating between points on a mesh using (triangular)
barycentric interpolation.

For a technical description of barycentric interpolation, see e.g.
Remi Munos and Andrew Moore, "Barycentric Interpolators for Continuous
Space and Time Reinforcement Learning", NIPS 1998)""";
        // Symbol: drake::math::BarycentricMesh::BarycentricMesh<T>
        struct /* ctor */ {
          // Source: drake/math/barycentric.h
          const char* doc = R"""(Constructs the mesh.)""";
        } ctor;
        // Symbol: drake::math::BarycentricMesh::Coordinates
        struct /* Coordinates */ {
          // Source: drake/math/barycentric.h
          const char* doc =
R"""(The mesh is represented by a std∷set (to ensure uniqueness and provide
logarithmic lookups) of coordinates in each input dimension. Note: The
values are type double, not T (We do not plan to take gradients, etc
w/ respect to them).)""";
        } Coordinates;
        // Symbol: drake::math::BarycentricMesh::Eval
        struct /* Eval */ {
          // Source: drake/math/barycentric.h
          const char* doc_3args =
R"""(Evaluates the function at the ``input`` values, by interpolating
between the values at ``mesh_values``. Inputs that are outside the
bounding box of the input_grid are interpolated as though they were
projected (elementwise) to the closest face of the defined mesh.

Note that the dimension of the output vector is completely defined by
the mesh_values argument. This class does not maintain any information
related to the size of the output.

Parameter ``mesh_values``:
    is a num_outputs by get_num_mesh_points() matrix containing the
    points to interpolate between. The order of the columns must be
    consistent with the mesh indices curated by this class, as exposed
    by get_mesh_point().

Parameter ``input``:
    must be a vector of length get_num_inputs().

Parameter ``output``:
    is the interpolated vector of length num_outputs)""";
          // Source: drake/math/barycentric.h
          const char* doc_2args =
R"""(Returns the function evaluated at ``input``.)""";
        } Eval;
        // Symbol: drake::math::BarycentricMesh::EvalBarycentricWeights
        struct /* EvalBarycentricWeights */ {
          // Source: drake/math/barycentric.h
          const char* doc =
R"""(Writes the mesh indices used for interpolation to ``mesh_indices``,
and the interpolating coefficients to ``weights``. Inputs that are
outside the bounding box of the input_grid are interpolated as though
they were projected (elementwise) to the closest face of the defined
mesh.

Parameter ``input``:
    must be a vector of length get_num_inputs().

Parameter ``mesh_indices``:
    is a pointer to a vector of length get_num_interpolants().

Parameter ``weights``:
    is a vector of coefficients (which sum to 1) of length
    get_num_interpolants().)""";
        } EvalBarycentricWeights;
        // Symbol: drake::math::BarycentricMesh::EvalWithMixedScalars
        struct /* EvalWithMixedScalars */ {
          // Source: drake/math/barycentric.h
          const char* doc_3args =
R"""(Performs Eval, but with the possibility of the values on the mesh
having a different scalar type than the values defining the mesh
(symbolic∷Expression containing decision variables for an optimization
problem is an important example)

Template parameter ``ValueT``:
    defines the scalar type of the mesh_values and the output.

See also:
    Eval)""";
          // Source: drake/math/barycentric.h
          const char* doc_2args =
R"""(Returns the function evaluated at ``input``.)""";
        } EvalWithMixedScalars;
        // Symbol: drake::math::BarycentricMesh::MeshGrid
        struct /* MeshGrid */ {
          // Source: drake/math/barycentric.h
          const char* doc = R"""()""";
        } MeshGrid;
        // Symbol: drake::math::BarycentricMesh::MeshValuesFrom
        struct /* MeshValuesFrom */ {
          // Source: drake/math/barycentric.h
          const char* doc =
R"""(Evaluates ``vector_func`` at all input mesh points and extracts the
mesh value matrix that should be used to approximate the function with
this barycentric interpolation.


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    MatrixXd mesh_values = bary.MeshValuesFrom(
        [](const auto& x) { return Vector1d(std∷sin(x[0])); });

.. raw:: html

    </details>)""";
        } MeshValuesFrom;
        // Symbol: drake::math::BarycentricMesh::get_all_mesh_points
        struct /* get_all_mesh_points */ {
          // Source: drake/math/barycentric.h
          const char* doc =
R"""(Returns a matrix with all of the mesh points, one per column.)""";
        } get_all_mesh_points;
        // Symbol: drake::math::BarycentricMesh::get_input_grid
        struct /* get_input_grid */ {
          // Source: drake/math/barycentric.h
          const char* doc = R"""()""";
        } get_input_grid;
        // Symbol: drake::math::BarycentricMesh::get_input_size
        struct /* get_input_size */ {
          // Source: drake/math/barycentric.h
          const char* doc = R"""()""";
        } get_input_size;
        // Symbol: drake::math::BarycentricMesh::get_mesh_point
        struct /* get_mesh_point */ {
          // Source: drake/math/barycentric.h
          const char* doc_2args =
R"""(Writes the position of a mesh point in the input space referenced by
its scalar index to ``point``.

Parameter ``index``:
    must be in [0, get_num_mesh_points).

Parameter ``point``:
    is set to the num_inputs-by-1 location of the mesh point.)""";
          // Source: drake/math/barycentric.h
          const char* doc_1args =
R"""(Returns the position of a mesh point in the input space referenced by
its scalar index to ``point``.

Parameter ``index``:
    must be in [0, get_num_mesh_points).)""";
        } get_mesh_point;
        // Symbol: drake::math::BarycentricMesh::get_num_interpolants
        struct /* get_num_interpolants */ {
          // Source: drake/math/barycentric.h
          const char* doc = R"""()""";
        } get_num_interpolants;
        // Symbol: drake::math::BarycentricMesh::get_num_mesh_points
        struct /* get_num_mesh_points */ {
          // Source: drake/math/barycentric.h
          const char* doc = R"""()""";
        } get_num_mesh_points;
      } BarycentricMesh;
      // Symbol: drake::math::BinomialCoefficient
      struct /* BinomialCoefficient */ {
        // Source: drake/math/binomial_coefficient.h
        const char* doc =
R"""(Computes the binomial coefficient ``n``-choose-``k`` efficiently using
a dynamic programming recursion.
https://en.wikipedia.org/wiki/Binomial_coefficient

Precondition:
    k >= 0)""";
      } BinomialCoefficient;
      // Symbol: drake::math::BsplineBasis
      struct /* BsplineBasis */ {
        // Source: drake/math/bspline_basis.h
        const char* doc =
R"""(Given a set of non-descending breakpoints t₀ ≤ t₁ ≤ ⋅⋅⋅ ≤ tₘ, a
B-spline basis of order k is a set of n + 1 (where n = m - k)
piecewise polynomials of degree k - 1 defined over those breakpoints.
The elements of this set are called "B-splines". The vector (t₀, t₁,
..., tₘ)' is referred to as the "knot vector" of the basis and its
elements are referred to as "knots".

At a breakpoint with multiplicity p (i.e. a breakpoint that appears p
times in the knot vector), B-splines are guaranteed to have Cᵏ⁻ᵖ⁻¹
continuity.

A B-spline curve using a B-spline basis B, is a parametric curve
mapping parameter values in [tₖ₋₁, tₙ₊₁] to a vector space V. For t ∈
[tₖ₋₁, tₙ₊₁] the value of the curve is given by the linear combination
of n + 1 control points, pᵢ ∈ V, with the elements of B evaluated at
t.

For more information on B-splines and their uses, see (for example)
Patrikalakis et al. [1].

[1] https://web.mit.edu/hyperbook/Patrikalakis-Maekawa-Cho/node15.html)""";
        // Symbol: drake::math::BsplineBasis::BsplineBasis<T>
        struct /* ctor */ {
          // Source: drake/math/bspline_basis.h
          const char* doc_2args =
R"""(Constructs a B-spline basis with the specified ``order`` and
``knots``.

Precondition:
    ``knots`` is sorted in non-descending order.

Raises:
    RuntimeError if knots.size() < 2 * order.)""";
          // Source: drake/math/bspline_basis.h
          const char* doc_5args =
R"""(Constructs a B-spline basis with the specified ``order``,
`num_basis_functions`, ``initial_parameter_value``,
`final_parameter_value`, and an auto-generated knot vector of the
specified ``type``.

Raises:
    RuntimeError if num_basis_functions < order

Precondition:
    initial_parameter_value ≤ final_parameter_value)""";
        } ctor;
        // Symbol: drake::math::BsplineBasis::ComputeActiveBasisFunctionIndices
        struct /* ComputeActiveBasisFunctionIndices */ {
          // Source: drake/math/bspline_basis.h
          const char* doc_1args_parameter_interval =
R"""(Returns the indices of the basis functions which may evaluate to
non-zero values for some parameter value in ``parameter_interval``;
all other basis functions are strictly zero over
``parameter_interval``.

Precondition:
    parameter_interval[0] ≤ parameter_interval[1]

Precondition:
    parameter_interval[0] ≥ initial_parameter_value()

Precondition:
    parameter_interval[1] ≤ final_parameter_value())""";
          // Source: drake/math/bspline_basis.h
          const char* doc_1args_parameter_value =
R"""(Returns the indices of the basis functions which may evaluate to
non-zero values for ``parameter_value``; all other basis functions are
strictly zero at this point.

Precondition:
    parameter_value ≥ initial_parameter_value()

Precondition:
    parameter_value ≤ final_parameter_value())""";
        } ComputeActiveBasisFunctionIndices;
        // Symbol: drake::math::BsplineBasis::EvaluateBasisFunctionI
        struct /* EvaluateBasisFunctionI */ {
          // Source: drake/math/bspline_basis.h
          const char* doc =
R"""(Returns the value of the ``i``-th basis function evaluated at
``parameter_value``.)""";
        } EvaluateBasisFunctionI;
        // Symbol: drake::math::BsplineBasis::EvaluateCurve
        struct /* EvaluateCurve */ {
          // Source: drake/math/bspline_basis.h
          const char* doc =
R"""(Evaluates the B-spline curve defined by ``this`` and
``control_points`` at the given ``parameter_value``.

Parameter ``control_points``:
    Control points of the B-spline curve.

Parameter ``parameter_value``:
    Parameter value at which to evaluate the B-spline curve defined by
    ``this`` and ``control_points``.

Precondition:
    control_points.size() == num_basis_functions()

Precondition:
    parameter_value ≥ initial_parameter_value()

Precondition:
    parameter_value ≤ final_parameter_value())""";
        } EvaluateCurve;
        // Symbol: drake::math::BsplineBasis::EvaluateLinearInControlPoints
        struct /* EvaluateLinearInControlPoints */ {
          // Source: drake/math/bspline_basis.h
          const char* doc =
R"""(Returns the vector, M, such that


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    EvaluateCurve(control_points, parameter_value) = control_points * M

.. raw:: html

    </details>

where T_control_points==VectorX<T> (so control_points is a matrix).
This is useful for writing linear constraints on the control points.

Precondition:
    parameter_value ≥ initial_parameter_value()

Precondition:
    parameter_value ≤ final_parameter_value())""";
        } EvaluateLinearInControlPoints;
        // Symbol: drake::math::BsplineBasis::FindContainingInterval
        struct /* FindContainingInterval */ {
          // Source: drake/math/bspline_basis.h
          const char* doc =
R"""(For a ``parameter_value`` = t, the interval that contains it is the
pair of knot values [tᵢ, tᵢ₊₁] for the greatest i such that tᵢ ≤ t and
tᵢ < final_parameter_value(). This function returns that value of i.

Precondition:
    parameter_value ≥ initial_parameter_value()

Precondition:
    parameter_value ≤ final_parameter_value())""";
        } FindContainingInterval;
        // Symbol: drake::math::BsplineBasis::Serialize
        struct /* Serialize */ {
          // Source: drake/math/bspline_basis.h
          const char* doc =
R"""(Passes this object to an Archive. Refer to yaml_serialization "YAML
Serialization" for background. This method is only available when T =
double.)""";
        } Serialize;
        // Symbol: drake::math::BsplineBasis::degree
        struct /* degree */ {
          // Source: drake/math/bspline_basis.h
          const char* doc =
R"""(The degree of the piecewise polynomials comprising this B-spline basis
(k - 1 in the class description).)""";
        } degree;
        // Symbol: drake::math::BsplineBasis::final_parameter_value
        struct /* final_parameter_value */ {
          // Source: drake/math/bspline_basis.h
          const char* doc =
R"""(The maximum allowable parameter value for B-spline curves using this
basis (tₙ₊₁ in the class description).)""";
        } final_parameter_value;
        // Symbol: drake::math::BsplineBasis::initial_parameter_value
        struct /* initial_parameter_value */ {
          // Source: drake/math/bspline_basis.h
          const char* doc =
R"""(The minimum allowable parameter value for B-spline curves using this
basis (tₖ₋₁ in the class description).)""";
        } initial_parameter_value;
        // Symbol: drake::math::BsplineBasis::knots
        struct /* knots */ {
          // Source: drake/math/bspline_basis.h
          const char* doc =
R"""(The knot vector of this B-spline basis (the vector (t₀, t₁, ..., tₘ)'
in the class description).)""";
        } knots;
        // Symbol: drake::math::BsplineBasis::num_basis_functions
        struct /* num_basis_functions */ {
          // Source: drake/math/bspline_basis.h
          const char* doc =
R"""(The number of basis functions in this B-spline basis (n + 1 in the
class description).)""";
        } num_basis_functions;
        // Symbol: drake::math::BsplineBasis::operator!=
        struct /* operator_ne */ {
          // Source: drake/math/bspline_basis.h
          const char* doc = R"""()""";
        } operator_ne;
        // Symbol: drake::math::BsplineBasis::order
        struct /* order */ {
          // Source: drake/math/bspline_basis.h
          const char* doc =
R"""(The order of this B-spline basis (k in the class description).)""";
        } order;
      } BsplineBasis;
      // Symbol: drake::math::CalculateAngularVelocityExpressedInBFromQuaternionDt
      struct /* CalculateAngularVelocityExpressedInBFromQuaternionDt */ {
        // Source: drake/math/quaternion.h
        const char* doc =
R"""(This function calculates angular velocity from a quaternion and its
time- derivative. Algorithm from [Kane, 1983] Section 1.13, Pages
58-59.

- [Kane, 1983] "Spacecraft Dynamics," McGraw-Hill Book Co., New York, 1983.
  (with P. W. Likins and D. A. Levinson).  Available for free .pdf download:
  https://ecommons.cornell.edu/handle/1813/637

Parameter ``quat_AB``:
    Quaternion [w, x, y, z] that relates two right-handed orthogonal
    unitary bases e.g., Ax, Ay, Az (A) to Bx, By, Bz (B). Note:
    quat_AB is analogous to the rotation matrix R_AB.

Parameter ``quatDt``:
    Time-derivative of ``quat_AB``, i.e. [ẇ, ẋ, ẏ, ż].

Returns ``w_AB_B``:
    B's angular velocity in A, expressed in B.)""";
      } CalculateAngularVelocityExpressedInBFromQuaternionDt;
      // Symbol: drake::math::CalculateQuaternionDtConstraintViolation
      struct /* CalculateQuaternionDtConstraintViolation */ {
        // Source: drake/math/quaternion.h
        const char* doc =
R"""(This function calculates how well a quaternion and its time-derivative
satisfy the quaternion time-derivative constraint specified in [Kane,
1983] Section 1.13, equations 12-13, page 59. For a quaternion [w, x,
y, z], the quaternion must satisfy: w^2 + x^2 + y^2 + z^2 = 1, hence
its time-derivative must satisfy: 2*(w*ẇ + x*ẋ + y*ẏ + z*ż) = 0.

- [Kane, 1983] "Spacecraft Dynamics," McGraw-Hill Book Co., New York, 1983.
  (with P. W. Likins and D. A. Levinson).  Available for free .pdf download:
  https://ecommons.cornell.edu/handle/1813/637

Parameter ``quat``:
    Quaternion [w, x, y, z] that relates two right-handed orthogonal
    unitary bases e.g., Ax, Ay, Az (A) to Bx, By, Bz (B). Note: A
    quaternion like quat_AB is analogous to the rotation matrix R_AB.

Parameter ``quatDt``:
    Time-derivative of ``quat``, i.e., [ẇ, ẋ, ẏ, ż].

Returns ``quaternionDt_constraint_violation``:
    The amount the time- derivative of the quaternion constraint has
    been violated, which may be positive or negative (0 means the
    constraint is perfectly satisfied).)""";
      } CalculateQuaternionDtConstraintViolation;
      // Symbol: drake::math::CalculateQuaternionDtFromAngularVelocityExpressedInB
      struct /* CalculateQuaternionDtFromAngularVelocityExpressedInB */ {
        // Source: drake/math/quaternion.h
        const char* doc =
R"""(This function calculates a quaternion's time-derivative from its
quaternion and angular velocity. Algorithm from [Kane, 1983] Section
1.13, Pages 58-59.

- [Kane, 1983] "Spacecraft Dynamics," McGraw-Hill Book Co., New York, 1983.
  (With P. W. Likins and D. A. Levinson).  Available for free .pdf download:
  https://ecommons.cornell.edu/handle/1813/637

Parameter ``quat_AB``:
    Quaternion [w, x, y, z] that relates two right-handed orthogonal
    unitary bases e.g., Ax, Ay, Az (A) to Bx, By, Bz (B). Note:
    quat_AB is analogous to the rotation matrix R_AB.

Parameter ``w_AB_B``:
    B's angular velocity in A, expressed in B.

Returns ``quatDt``:
    Time-derivative of quat_AB, i.e., [ẇ, ẋ, ẏ, ż].)""";
      } CalculateQuaternionDtFromAngularVelocityExpressedInB;
      // Symbol: drake::math::CalculateReflectedGrayCodes
      struct /* CalculateReflectedGrayCodes */ {
        // Source: drake/math/gray_code.h
        const char* doc =
R"""(Returns a matrix whose i'th row is the Gray code for integer i.

Template parameter ``NumDigits``:
    The number of digits in the Gray code.

Parameter ``num_digits``:
    The number of digits in the Gray code.

Returns:
    M. M is a matrix of size 2ᵏ x k, where ``k`` is ``num_digits``.
    M.row(i) is the Gray code for integer i.)""";
      } CalculateReflectedGrayCodes;
      // Symbol: drake::math::ClosestQuaternion
      struct /* ClosestQuaternion */ {
        // Source: drake/math/quaternion.h
        const char* doc =
R"""(Returns a unit quaternion that represents the same orientation as
``quat2``, and has the "shortest" geodesic distance on the unit sphere
to ``quat1``.)""";
      } ClosestQuaternion;
      // Symbol: drake::math::ComputeNumericalGradient
      struct /* ComputeNumericalGradient */ {
        // Source: drake/math/compute_numerical_gradient.h
        const char* doc =
R"""(Compute the gradient of a function f(x) through numerical difference.

Parameter ``calc_func``:
    calc_func(x, &y) computes the value of f(x), and stores the value
    in y. ``calc_func`` is responsible for properly resizing the
    output ``y`` when it consists of an Eigen vector of Eigen∷Dynamic
    size.

Parameter ``x``:
    The point at which the numerical gradient is computed.

Parameter ``option``:
    The options for computing numerical gradient.

Template parameter ``DerivedX``:
    an Eigen column vector.

Template parameter ``DerivedY``:
    an Eigen column vector.

Template parameter ``DerivedCalcX``:
    The type of x in the calc_func. Must be an Eigen column vector. It
    is possible to have DerivedCalcX being different from DerivedX,
    for example, ``calc_func`` could be solvers∷EvaluatorBase(const
    Eigen∷Ref<const Eigen∷VectorXd>&, Eigen∷VectorXd*), but ``x``
    could be of type Eigen∷VectorXd. TODO(hongkai.dai): understand why
    the default template DerivedCalcX = DerivedX doesn't compile when
    I instantiate ComputeNumericalGradient<DerivedX,
    DerivedY>(calc_func, x);

Returns ``gradient``:
    a matrix of size x.rows() x y.rows(). gradient(i, j) is ∂f(i) /
    ∂x(j)

Examples:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    {cc}
    // Create a std∷function from a lambda expression.
    std∷function<void (const Eigen∷Vector2d&, Vector3d*)> foo = [](const
    Eigen∷Vector2d& x, Vector3d*y) { (*y)(0) = x(0); (*y)(1) = x(0) * x(1);
    (*y)(2) = x(0) * std∷sin(x(1));};
    Eigen∷Vector3d x_eval(1, 2, 3);
    auto J = ComputeNumericalGradient(foo, x_eval);
    // Note that if we pass in a lambda to ComputeNumericalGradient, then
    // ComputeNumericalGradient has to instantiate the template types explicitly,
    // as in this example. The issue of template deduction with std∷function is
    // explained in
    //
    https://stackoverflow.com/questions/48529410/template-arguments-deduction-failed-passing-func-pointer-to-stdfunction
    auto bar = [](const Eigen∷Vector2d& x, Eigen∷Vector2d* y) {*y = x; };
    auto J2 = ComputeNumericalGradient<Eigen∷Vector2d,
    Eigen∷Vector2d, Eigen∷Vector2d>(bar, Eigen∷Vector2d(2, 3));

.. raw:: html

    </details>)""";
      } ComputeNumericalGradient;
      // Symbol: drake::math::ContinuousAlgebraicRiccatiEquation
      struct /* ContinuousAlgebraicRiccatiEquation */ {
        // Source: drake/math/continuous_algebraic_riccati_equation.h
        const char* doc_4args_A_B_Q_R =
R"""(Computes the unique stabilizing solution S to the continuous-time
algebraic Riccati equation:

.. math:: S A + A' S - S B R^{-1} B' S + Q = 0

Raises:
    RuntimeError if the Hamiltanoian matrix


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    ⌈A   BR⁻¹Bᵀ⌉
    ⌊Q      −Aᵀ⌋

.. raw:: html

    </details>

is not invertible.

Raises:
    RuntimeError if R is not positive definite.

Note:
    the pair (A, B) should be stabilizable, and (Q, A) should be
    detectable. For more information, please refer to page 526-527 of
    Linear Systems by Thomas Kailath.

Based on the Matrix Sign Function method outlined in this paper:
http://www.engr.iupui.edu/~skoskie/ECE684/Riccati_algorithms.pdf)""";
        // Source: drake/math/continuous_algebraic_riccati_equation.h
        const char* doc_4args_A_B_Q_R_cholesky =
R"""(This is functionally the same as ContinuousAlgebraicRiccatiEquation(A,
B, Q, R). The Cholesky decomposition of R is passed in instead of R.)""";
      } ContinuousAlgebraicRiccatiEquation;
      // Symbol: drake::math::ConvertTimeDerivativeToOtherFrame
      struct /* ConvertTimeDerivativeToOtherFrame */ {
        // Source: drake/math/convert_time_derivative.h
        const char* doc =
R"""(Given ᴮd/dt(v) (the time derivative in frame B of an arbitrary 3D
vector v) and given ᴬωᴮ (frame B's angular velocity in another frame
A), this method computes ᴬd/dt(v) (the time derivative in frame A of
v) by: ᴬd/dt(v) = ᴮd/dt(v) + ᴬωᴮ x v

This mathematical operation is known as the "Transport Theorem" or the
"Golden Rule for Vector Differentiation" [Mitiguy 2016, §7.3]. It was
discovered by Euler in 1758. Its explicit notation with superscript
frames was invented by Thomas Kane in 1950. Its use as the defining
property of angular velocity was by Mitiguy in 1993.

In source code and comments, we use the following monogram notations:
DtA_v = ᴬd/dt(v) denotes the time derivative in frame A of the vector
v. DtA_v_E = [ᴬd/dt(v)]_E denotes the time derivative in frame A of
vector v, with the resulting new vector quantity expressed in a frame
E.

In source code, this mathematical operation is performed with all
vectors expressed in the same frame E as [ᴬd/dt(v)]ₑ = [ᴮd/dt(v)]ₑ +
[ᴬωᴮ]ₑ x [v]ₑ which in monogram notation is:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    DtA_v_E = DtB_v_E + w_AB_E x v_E

.. raw:: html

    </details>

[Mitiguy 2016] Mitiguy, P., 2016. Advanced Dynamics & Motion
Simulation.)""";
      } ConvertTimeDerivativeToOtherFrame;
      // Symbol: drake::math::DecomposePSDmatrixIntoXtransposeTimesX
      struct /* DecomposePSDmatrixIntoXtransposeTimesX */ {
        // Source: drake/math/quadratic_form.h
        const char* doc =
R"""(For a symmetric positive semidefinite matrix Y, decompose it into XᵀX,
where the number of rows in X equals to the rank of Y. Notice that
this decomposition is not unique. For any orthonormal matrix U, s.t
UᵀU = identity, X_prime = UX also satisfies X_primeᵀX_prime = Y. Here
we only return one valid decomposition.

Parameter ``Y``:
    A symmetric positive semidefinite matrix.

Parameter ``zero_tol``:
    We will need to check if some value (for example, the absolute
    value of Y's eigenvalues) is smaller than zero_tol. If it is, then
    we deem that value as 0.

Parameter ``return_empty_if_not_psd``:
    If true, then return an empty matrix of size 0-by-Y.cols() if Y is
    not PSD (either the decomposition fails or the resulting
    eigenvalues are less that ``zero_tol)``. If false (the default),
    then throw an exception if Y is not PSD. This option is
    particularly useful because it is brittle/expensive to test the
    exact success criteria before calling this function.

Returns ``X``:
    . The matrix X satisfies XᵀX = Y and X.rows() = rank(Y).

Precondition:
1. Y is positive semidefinite or return_empty_if_not_psd = true.
     2. zero_tol is non-negative.

    $Raises:

RuntimeError when the pre-conditions are not satisfied.

Note:
    We only use the lower triangular part of Y.)""";
      } DecomposePSDmatrixIntoXtransposeTimesX;
      // Symbol: drake::math::DecomposePositiveQuadraticForm
      struct /* DecomposePositiveQuadraticForm */ {
        // Source: drake/math/quadratic_form.h
        const char* doc =
R"""(Rewrite a quadratic form xᵀQx + bᵀx + c to (Rx+d)ᵀ(Rx+d) where


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    RᵀR = Q
    Rᵀd = b / 2
    dᵀd = c

.. raw:: html

    </details>

This decomposition requires the matrix


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    ⌈Q     b/2⌉
    ⌊bᵀ/2    c⌋

.. raw:: html

    </details>

to be positive semidefinite.

We return R and d with the minimal number of rows, namely the rows of
R and d equal to the rank of the matrix


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    ⌈Q     b/2⌉
    ⌊bᵀ/2    c⌋

.. raw:: html

    </details>

Notice that R might have more rows than Q, For example, the quadratic
expression x² + 2x + 5 =(x+1)² + 2², it can be decomposed as


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    ⎛⌈1⌉ * x + ⌈1⌉⎞ᵀ * ⎛⌈1⌉ * x + ⌈1⌉⎞
    ⎝⌊0⌋       ⌊2⌋⎠    ⎝⌊0⌋       ⌊2⌋⎠

.. raw:: html

    </details>

Here R has 2 rows while Q only has 1 row.

On the other hand the quadratic expression x² + 2x + 1 can be
decomposed as (x+1) * (x+1), where R has 1 row, same as Q.

Also notice that this decomposition is not unique. For example, with
any permutation matrix P, we can define


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    R₁ = P*R
    d₁ = P*d

.. raw:: html

    </details>

Then (R₁*x+d₁)ᵀ(R₁*x+d₁) gives the same quadratic form.

Parameter ``Q``:
    The square matrix.

Parameter ``b``:
    The vector containing the linear coefficients.

Parameter ``c``:
    The constant term.

Parameter ``tol``:
    We will determine if this quadratic form is always non-negative,
    by checking the Eigen values of the matrix [Q b/2] [bᵀ/2 c] are
    all greater than -tol. $*Default:* is 0.

Returns ``(R``:
    , d). R and d have the same number of rows. R.cols() == x.rows().
    R.rows() equals to the rank of the matrix


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    [Q    b/2]
       [bᵀ/2   c]

.. raw:: html

    </details>

Precondition:
1. The quadratic form is always non-negative, namely the matrix
        


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    [Q    b/2]
            [bᵀ/2   c]

.. raw:: html

    </details>

is positive semidefinite. 2. ``Q`` and ``b`` are of the correct size.
3. ``tol`` is non-negative.

Raises:
    RuntimeError if the precondition is not satisfied.)""";
      } DecomposePositiveQuadraticForm;
      // Symbol: drake::math::DifferentiableNorm
      struct /* DifferentiableNorm */ {
        // Source: drake/math/differentiable_norm.h
        const char* doc =
R"""(The 2-norm function |x| is not differentiable at x=0 (its gradient is
x/|x|, which has a division-by-zero problem). On the other hand, x=0
happens very often. Hence we return a subgradient x/(|x| + ε) when x
is almost 0, and returns the original gradient, x/|x|, otherwise.)""";
      } DifferentiableNorm;
      // Symbol: drake::math::DiscardGradient
      struct /* DiscardGradient */ {
        // Source: drake/math/autodiff.h
        const char* doc =
R"""(``B = DiscardGradient(A)`` enables casting from a matrix of
AutoDiffScalars to AutoDiffScalar∷Scalar type, explicitly throwing
away any gradient information. For a matrix of type, e.g.
``MatrixX<AutoDiffXd> A``, the comparable operation ``B =
A.cast<double>()`` should (and does) fail to compile. Use
``DiscardGradient(A)`` if you want to force the cast (and explicitly
declare that information is lost).

When called with a matrix that is already of type ``double``, this
function returns a *reference* to the argument without any copying.
This efficiently avoids extra copying, but be careful about reference
lifetimes!

See ExtractValue() for a note on similar Drake functions.

See also:
    ExtractValue(), DiscardZeroGradient())""";
      } DiscardGradient;
      // Symbol: drake::math::DiscardZeroGradient
      struct /* DiscardZeroGradient */ {
        // Source: drake/math/autodiff_gradient.h
        const char* doc =
R"""(``B = DiscardZeroGradient(A, precision)`` enables casting from a
matrix of AutoDiffScalars to AutoDiffScalar∷Scalar type, but first
checking that the gradient matrix is empty or zero. For a matrix of
type, e.g. ``MatrixX<AutoDiffXd> A``, the comparable operation ``B =
A.cast<double>()`` should (and does) fail to compile. Use
``DiscardZeroGradient(A)`` if you want to force the cast (and the
check).

When called with a matrix that is already of type ``double``, this
function returns a *reference* to the argument without any copying.
This efficiently avoids extra copying, but be careful about reference
lifetimes!

See ExtractValue() for a note on similar Drake functions.

Parameter ``precision``:
    is passed to Eigen's isZero(precision) to evaluate whether the
    gradients are zero.

Raises:
    RuntimeError if the gradients were not empty nor zero.

See also:
    DiscardGradient())""";
      } DiscardZeroGradient;
      // Symbol: drake::math::DiscreteAlgebraicRiccatiEquation
      struct /* DiscreteAlgebraicRiccatiEquation */ {
        // Source: drake/math/discrete_algebraic_riccati_equation.h
        const char* doc_4args =
R"""(Computes the unique stabilizing solution X to the discrete-time
algebraic Riccati equation:

AᵀXA − X − AᵀXB(BᵀXB + R)⁻¹BᵀXA + Q = 0

Raises:
    RuntimeError if Q is not symmetric positive semidefinite.

Raises:
    RuntimeError if R is not symmetric positive definite.

Raises:
    RuntimeError if (A, B) isn't a stabilizable pair.

Raises:
    RuntimeError if (A, C) isn't a detectable pair where Q = CᵀC.)""";
        // Source: drake/math/discrete_algebraic_riccati_equation.h
        const char* doc_5args =
R"""(Computes the unique stabilizing solution X to the discrete-time
algebraic Riccati equation:

AᵀXA − X − (AᵀXB + N)(BᵀXB + R)⁻¹(BᵀXA + Nᵀ) + Q = 0

This is equivalent to solving the original DARE:

A₂ᵀXA₂ − X − A₂ᵀXB(BᵀXB + R)⁻¹BᵀXA₂ + Q₂ = 0

where A₂ and Q₂ are a change of variables:

A₂ = A − BR⁻¹Nᵀ and Q₂ = Q − NR⁻¹Nᵀ

This overload of the DARE is useful for finding the control law uₖ
that minimizes the following cost function subject to xₖ₊₁ = Axₖ +
Buₖ.


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    ∞ [xₖ]ᵀ[Q  N][xₖ]
    J = Σ [uₖ] [Nᵀ R][uₖ] ΔT
    k=0

.. raw:: html

    </details>

This is a more general form of the following. The linear-quadratic
regulator is the feedback control law uₖ that minimizes the following
cost function subject to xₖ₊₁ = Axₖ + Buₖ:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    ∞
    J = Σ (xₖᵀQxₖ + uₖᵀRuₖ) ΔT
    k=0

.. raw:: html

    </details>

This can be refactored as:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    ∞ [xₖ]ᵀ[Q 0][xₖ]
    J = Σ [uₖ] [0 R][uₖ] ΔT
    k=0

.. raw:: html

    </details>

Raises:
    RuntimeError if Q₂ is not symmetric positive semidefinite.

Raises:
    RuntimeError if R is not symmetric positive definite.

Raises:
    RuntimeError if (A₂, B) isn't a stabilizable pair.

Raises:
    RuntimeError if (A₂, C) isn't a detectable pair where Q₂ = CᵀC.)""";
      } DiscreteAlgebraicRiccatiEquation;
      // Symbol: drake::math::EigenToStdVector
      struct /* EigenToStdVector */ {
        // Source: drake/math/matrix_util.h
        const char* doc =
R"""(Converts a MatrixX<T> into a std∷vector<MatrixX<T>>, taking each
column of the m-by-n matrix ``mat`` into an m-by-1 element of the
returned std∷vector.)""";
      } EigenToStdVector;
      // Symbol: drake::math::ExtractGradient
      struct /* ExtractGradient */ {
        // Source: drake/math/autodiff_gradient.h
        const char* doc =
R"""(Returns the ``derivatives()`` portion from a matrix of AutoDiffScalar
entries. (Each entry contains a value and derivatives.)

Parameter ``auto_diff_matrix``:
    An object whose Eigen type represents a matrix of AutoDiffScalar
    entries.

Parameter ``num_derivatives``:
    (Optional) The number of derivatives to return in case the input
    matrix has none, which we interpret as ``num_derivatives`` zeroes.
    If ``num_derivatives`` is supplied and the input matrix has
    derivatives, the sizes must match.

Returns ``gradient_matrix``:
    An Eigen∷Matrix with number of rows equal to the total size (rows
    x cols) of the input matrix and number of columns equal to the
    number of derivatives. Each output row corresponds to one entry of
    the input matrix, using the input matrix storage order. For
    example, in the typical case of a ColMajor ``auto_diff_matrix``,
    we have ``auto_diff_matrix(r, c).derivatives() ==
    gradient_matrix.row(r + c * auto_diff_matrix.rows())``.

Template parameter ``Derived``:
    An Eigen type representing a matrix with AutoDiffScalar entries.
    The type will be inferred from the type of the
    ``auto_diff_matrix`` parameter at the call site.

Raises:
    RuntimeError if the input matrix has elements with inconsistent,
    non-zero numbers of derivatives.

Raises:
    RuntimeError if ``num_derivatives`` is specified but the input
    matrix has a different, non-zero number of derivatives.)""";
      } ExtractGradient;
      // Symbol: drake::math::ExtractPrincipalSubmatrix
      struct /* ExtractPrincipalSubmatrix */ {
        // Source: drake/math/matrix_util.h
        const char* doc =
R"""(Extracts the principal submatrix from the ordered set of indices. The
indices must be in monotonically increasing order and non-empty. This
method makes no assumptions about the symmetry of the matrix, nor that
the matrix is square. However, all indices must be valid for both rows
and columns.)""";
      } ExtractPrincipalSubmatrix;
      // Symbol: drake::math::ExtractValue
      struct /* ExtractValue */ {
        // Source: drake/math/autodiff.h
        const char* doc =
R"""(Returns the ``value()`` portion from a matrix of AutoDiffScalar
entries. (Each entry contains a value and some derivatives.)

Parameter ``auto_diff_matrix``:
    An object whose Eigen type represents a matrix of AutoDiffScalar
    entries.

Returns ``value``:
    An Eigen∷Matrix of the same dimensions as the input matrix, but
    containing only the value portion of each entry, without the
    derivatives.

Template parameter ``Derived``:
    An Eigen type representing a matrix with AutoDiffScalar entries.
    The type will be inferred from the type of the
    ``auto_diff_matrix`` parameter at the call site.

Note:
    Drake provides several similar functions: DiscardGradient() is
    specialized so that it can be applied to a ``Matrix<T>`` where T
    could be an AutoDiffScalar or an ordinary double, in which case it
    returns the original matrix at no cost. DiscardZeroGradient() is
    similar but requires that the discarded gradient was zero.
    drake∷ExtractDoubleOrThrow() has many specializations, including
    one for ``Matrix<AutoDiffScalar>`` that behaves identically to
    ExtractValue().

See also:
    DiscardGradient(), drake∷ExtractDoubleOrThrow())""";
      } ExtractValue;
      // Symbol: drake::math::GeneratePythonCsc
      struct /* GeneratePythonCsc */ {
        // Source: drake/math/matrix_util.h
        const char* doc =
R"""(Returns the python statement to construct scipy.sparse matrix. The
generated code will call sparse.csc_matrix() directly (please make
sure you have imported the module through ``from scipy import
sparse``), and will end with a newline.

Parameter ``mat``:
    The Eigen matrix to be generated to python code.

Parameter ``name``:
    The name of the python variable for the sparse matrix.

Returns:
    the generated python code)""";
      } GeneratePythonCsc;
      // Symbol: drake::math::GetDerivativeSize
      struct /* GetDerivativeSize */ {
        // Source: drake/math/autodiff_gradient.h
        const char* doc =
R"""(Given a matrix of AutoDiffScalars, returns the size of the
derivatives.

Raises:
    runtime_error if some entry has different (non-zero) number of
    derivatives as the others.)""";
      } GetDerivativeSize;
      // Symbol: drake::math::GetLinearSolver
      struct /* GetLinearSolver */ {
        // Source: drake/math/linear_solve.h
        const char* doc =
R"""(Get the linear solver for a matrix A. If A has scalar type of double
or symbolic∷Expressions, then the returned linear solver will have the
same scalar type. If A has scalar type of Eigen∷AutoDiffScalar, then
the returned linear solver will have scalar type of double. See
get_linear_solver for more details.)""";
      } GetLinearSolver;
      // Symbol: drake::math::GetSubMatrixGradientArray
      struct /* GetSubMatrixGradientArray */ {
        // Source: drake/math/gradient_util.h
        const char* doc = R"""()""";
        // Symbol: drake::math::GetSubMatrixGradientArray::type
        struct /* type */ {
          // Source: drake/math/gradient_util.h
          const char* doc = R"""()""";
        } type;
      } GetSubMatrixGradientArray;
      // Symbol: drake::math::GetSubMatrixGradientSingleElement
      struct /* GetSubMatrixGradientSingleElement */ {
        // Source: drake/math/gradient_util.h
        const char* doc = R"""()""";
        // Symbol: drake::math::GetSubMatrixGradientSingleElement::type
        struct /* type */ {
          // Source: drake/math/gradient_util.h
          const char* doc = R"""()""";
        } type;
      } GetSubMatrixGradientSingleElement;
      // Symbol: drake::math::Gradient
      struct /* Gradient */ {
        // Source: drake/math/gradient.h
        const char* doc =
R"""(Recursively defined template specifying a matrix type of the correct
size for a gradient of a matrix function with respect to ``nq``
variables, of any order.)""";
        // Symbol: drake::math::Gradient::type
        struct /* type */ {
          // Source: drake/math/gradient.h
          const char* doc = R"""()""";
        } type;
      } Gradient;
      // Symbol: drake::math::GrayCodeToInteger
      struct /* GrayCodeToInteger */ {
        // Source: drake/math/gray_code.h
        const char* doc =
R"""(Converts the Gray code to an integer. For example (0, 0) -> 0 (0, 1)
-> 1 (1, 1) -> 2 (1, 0) -> 3

Parameter ``gray_code``:
    The N-digit Gray code, where N is gray_code.rows()

Returns:
    The integer represented by the Gray code ``gray_code``.)""";
      } GrayCodeToInteger;
      // Symbol: drake::math::GrayCodesMatrix
      struct /* GrayCodesMatrix */ {
        // Source: drake/math/gray_code.h
        const char* doc =
R"""(GrayCodesMatrix∷type returns an Eigen matrix of integers. The size of
this matrix is determined by the number of digits in the Gray code.)""";
        // Symbol: drake::math::GrayCodesMatrix::type
        struct /* type */ {
          // Source: drake/math/gray_code.h
          const char* doc = R"""()""";
        } type;
      } GrayCodesMatrix;
      // Symbol: drake::math::HopfCoordinateToQuaternion
      struct /* HopfCoordinateToQuaternion */ {
        // Source: drake/math/hopf_coordinate.h
        const char* doc =
R"""(Transforms Hopf coordinates to a quaternion w, x, y, z as w =
cos(θ/2)cos(ψ/2) x = cos(θ/2)sin(ψ/2) y = sin(θ/2)cos(φ+ψ/2) z =
sin(θ/2)sin(φ+ψ/2) The user can refer to equation 5 of Generating
Uniform Incremental Grids on SO(3) Using the Hopf Fibration by Anna
Yershova, Steven LaValle and Julie Mitchell, 2008

Parameter ``theta``:
    The θ angle.

Parameter ``phi``:
    The φ angle.

Parameter ``psi``:
    The ψ angle.)""";
      } HopfCoordinateToQuaternion;
      // Symbol: drake::math::InitializeAutoDiff
      struct /* InitializeAutoDiff */ {
        // Source: drake/math/autodiff.h
        const char* doc_just_value =
R"""(Initializes a single AutoDiff matrix given the corresponding value
matrix.

Creates an AutoDiff matrix that matches ``value`` in size with
derivative of compile time size ``nq`` and runtime size
``num_derivatives``. Sets its values to be equal to ``value``, and for
each element i of ``auto_diff_matrix``, sets derivative number
``deriv_num_start`` + i to one (all other derivatives set to zero).

When ``value`` is a matrix (rather than just a vector) note in
particular that the return value will use the same storage order
(ColMajor vs RowMajor) and that the derivative numbers count up using
the *storage order* of ``value(i)``.

Parameter ``value``:
    'regular' matrix of values

Parameter ``num_derivatives``:
    (Optional) size of the derivatives vector

*Default:* total size of the value matrix
    $Parameter ``deriv_num_start``:

(Optional) starting index into derivative vector (i.e. element
deriv_num_start in derivative vector corresponds to matrix(0, 0)).

*Default:* 0
    $Returns ``auto_diff_matrix``:

The result as described above.)""";
        // Source: drake/math/autodiff_gradient.h
        const char* doc_value_and_gradient =
R"""(Returns an AutoDiff matrix given a matrix of values and a gradient
matrix.

Parameter ``value``:
    The value matrix. Will be accessed with a single index.

Parameter ``gradient``:
    The gradient matrix. The number of rows must match the total size
    (nrow x ncol) of the value matrix. Derivatives of value(j) should
    be stored in row j of the gradient matrix.

Returns ``auto_diff_matrix``:
    The matrix of AutoDiffScalars. Will have the same dimensions and
    storage order as the value matrix.)""";
      } InitializeAutoDiff;
      // Symbol: drake::math::InitializeAutoDiffTuple
      struct /* InitializeAutoDiffTuple */ {
        // Source: drake/math/autodiff.h
        const char* doc =
R"""(Given a series of Eigen matrices, creates a tuple of corresponding
AutoDiff matrices with values equal to the input matrices and properly
initialized derivative vectors.

The size of the derivative vector of each element of the matrices in
the output tuple will be the same, and will equal the sum of the
number of elements of the matrices in ``args``. If all of the matrices
in ``args`` have fixed size, then the derivative vectors will also
have fixed size (being the sum of the sizes at compile time of all of
the input arguments), otherwise the derivative vectors will have
dynamic size. The 0th element of the derivative vectors will
correspond to the derivative with respect to the 0th element of the
first argument. Subsequent derivative vector elements correspond first
to subsequent elements of the first input argument (traversed in the
same order as InitializeAutoDiff() for that matrix), and so on for
subsequent arguments.

Parameter ``args``:
    a series of Eigen matrices

Returns:
    a tuple of properly initialized AutoDiff matrices corresponding to
    ``args``)""";
      } InitializeAutoDiffTuple;
      // Symbol: drake::math::IsBothQuaternionAndQuaternionDtOK
      struct /* IsBothQuaternionAndQuaternionDtOK */ {
        // Source: drake/math/quaternion.h
        const char* doc =
R"""(This function tests if a quaternion satisfies the time-derivative
constraint specified in [Kane, 1983] Section 1.13, equation 13, page
59. A quaternion [w, x, y, z] must satisfy w^2 + x^2 + y^2 + z^2 = 1,
hence its time-derivative must satisfy 2*(w*ẇ + x*ẋ + y*ẏ + z*ż) =
0. Note: To accurately test whether the time-derivative quaternion
constraint is satisfied, the quaternion constraint is also tested to
be accurate.

- [Kane, 1983] "Spacecraft Dynamics," McGraw-Hill Book Co., New York, 1983.
  (with P. W. Likins and D. A. Levinson).  Available for free .pdf download:
  https://ecommons.cornell.edu/handle/1813/637

Parameter ``quat``:
    Quaternion [w, x, y, z] that relates two right-handed orthogonal
    unitary bases e.g., Ax, Ay, Az (A) to Bx, By, Bz (B). Note: A
    quaternion like quat_AB is analogous to the rotation matrix R_AB.

Parameter ``quatDt``:
    Time-derivative of ``quat``, i.e., [ẇ, ẋ, ẏ, ż].

Parameter ``tolerance``:
    Tolerance for quaternion constraints.

Returns:
    ``True`` if both of the two previous constraints are within
    tolerance.)""";
      } IsBothQuaternionAndQuaternionDtOK;
      // Symbol: drake::math::IsPositiveDefinite
      struct /* IsPositiveDefinite */ {
        // Source: drake/math/matrix_util.h
        const char* doc =
R"""(Checks if a matrix is symmetric (with tolerance ``symmetry_tolerance``
-- see IsSymmetric) and has all eigenvalues greater than
``eigenvalue_tolerance``. ``eigenvalue_tolerance`` must be >= 0 --
where 0 implies positive semi-definite (but is of course subject to
all of the pitfalls of floating point).

To consider the numerical robustness of the eigenvalue estimation, we
specifically check that min_eigenvalue >= eigenvalue_tolerance *
max(1, max_abs_eigenvalue).)""";
      } IsPositiveDefinite;
      // Symbol: drake::math::IsQuaternionAndQuaternionDtEqualAngularVelocityExpressedInB
      struct /* IsQuaternionAndQuaternionDtEqualAngularVelocityExpressedInB */ {
        // Source: drake/math/quaternion.h
        const char* doc =
R"""(This function tests if a quaternion and a quaternion's time derivative
can calculate and match an angular velocity to within a tolerance.
Note: This function first tests if the quaternion [w, x, y, z]
satisfies w^2 + x^2 + y^2 + z^2 = 1 (to within tolerance) and if its
time-derivative satisfies w*ẇ + x*ẋ + y*ẏ + z*ż = 0 (to within
tolerance). Lastly, it tests if each element of the angular velocity
calculated from quat and quatDt is within tolerance of w_B (described
below).

Parameter ``quat``:
    Quaternion [w, x, y, z] that relates two right-handed orthogonal
    unitary bases e.g., Ax, Ay, Az (A) to Bx, By, Bz (B). Note: A
    quaternion like quat_AB is analogous to the rotation matrix R_AB.

Parameter ``quatDt``:
    Time-derivative of ``quat``, i.e., [ẇ, ẋ, ẏ, ż].

Parameter ``w_B``:
    Rigid body B's angular velocity in frame A, expressed in B.

Parameter ``tolerance``:
    Tolerance for quaternion constraints.

Returns:
    ``True`` if all three of the previous constraints are within
    tolerance.)""";
      } IsQuaternionAndQuaternionDtEqualAngularVelocityExpressedInB;
      // Symbol: drake::math::IsQuaternionValid
      struct /* IsQuaternionValid */ {
        // Source: drake/math/quaternion.h
        const char* doc =
R"""(This function tests if a quaternion satisfies the quaternion
constraint specified in [Kane, 1983] Section 1.3, equation 4, page 12,
i.e., a quaternion [w, x, y, z] must satisfy: w^2 + x^2 + y^2 + z^2 =
1.

- [Kane, 1983] "Spacecraft Dynamics," McGraw-Hill Book Co., New York, 1983.
  (with P. W. Likins and D. A. Levinson).  Available for free .pdf download:
  https://ecommons.cornell.edu/handle/1813/637

Parameter ``quat``:
    Quaternion [w, x, y, z] that relates two right-handed orthogonal
    unitary bases e.g., Ax, Ay, Az (A) to Bx, By, Bz (B). Note: A
    quaternion like quat_AB is analogous to the rotation matrix R_AB.

Parameter ``tolerance``:
    Tolerance for quaternion constraint, i.e., how much is w^2 + x^2 +
    y^2 + z^2 allowed to differ from 1.

Returns:
    ``True`` if the quaternion constraint is satisfied within
    tolerance.)""";
      } IsQuaternionValid;
      // Symbol: drake::math::IsSymmetric
      struct /* IsSymmetric */ {
        // Source: drake/math/matrix_util.h
        const char* doc_1args =
R"""(Determines if a matrix is symmetric. If std∷equal_to<>()(matrix(i, j),
matrix(j, i)) is true for all i, j, then the matrix is symmetric.)""";
        // Source: drake/math/matrix_util.h
        const char* doc_2args =
R"""(Determines if a matrix is symmetric based on whether the difference
between matrix(i, j) and matrix(j, i) is smaller than ``precision``
for all i, j. The precision is absolute. Matrix with nan or inf
entries is not allowed.)""";
      } IsSymmetric;
      // Symbol: drake::math::KnotVectorType
      struct /* KnotVectorType */ {
        // Source: drake/math/knot_vector_type.h
        const char* doc =
R"""(Enum representing types of knot vectors. "Uniform" refers to the
spacing between the knots. "Clamped" indicates that the first and last
knots have multiplicity equal to the order of the spline.

Reference:
http://web.mit.edu/hyperbook/Patrikalakis-Maekawa-Cho/node17.html)""";
        // Symbol: drake::math::KnotVectorType::kClampedUniform
        struct /* kClampedUniform */ {
          // Source: drake/math/knot_vector_type.h
          const char* doc = R"""()""";
        } kClampedUniform;
        // Symbol: drake::math::KnotVectorType::kUniform
        struct /* kUniform */ {
          // Source: drake/math/knot_vector_type.h
          const char* doc = R"""()""";
        } kUniform;
      } KnotVectorType;
      // Symbol: drake::math::LinearSolver
      struct /* LinearSolver */ {
        // Source: drake/math/linear_solve.h
        const char* doc =
R"""(Solves a linear system of equations A*x=b. Depending on the scalar
types of A and b, the scalar type of x is summarized in this table. |
b ＼ A | double | ADS | Expr | |--------|--------|-----|----- | |
double | double | ADS | NA | | ADS | ADS | ADS | NA | | Expr | NA | NA
| Expr |

where ADS stands for Eigen∷AutoDiffScalar, and Expr stands for
symbolic∷Expression.

Using LinearSolver class is as fast as using Eigen's linear solver
directly when neither A nor b contains AutoDiffScalar. When either A
or b contains AutoDiffScalar, using LinearSolver is much faster than
using Eigen's autodiffable linear solver (for example
Eigen∷LDLT<Eigen∷Matrix<Eigen∷AutoDiffScalar, 3, 3>>).

Here is the example code


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    {cc}
    Eigen∷Matrix<AutoDiffXd, 2, 2> A;
    A(0, 0).value() = 1;
    A(0, 0).derivatives() = Eigen∷Vector3d(1, 2, 3);
    A(0, 1).value() = 2;
    A(0, 1).derivatives() = Eigen∷Vector3d(2, 3, 4);
    A(1, 0).value() = 2;
    A(1, 0).derivatives() = Eigen∷Vector3d(3, 4, 5);
    A(1, 1).value() = 5;
    A(1, 1).derivatives() = Eigen∷Vector3d(4, 5, 6);
    LinearSolver<Eigen∷LLT, Eigen∷Matrix<AutoDiffXd, 2, 2>> solver(A);
    Eigen∷Matrix<AutoDiffXd, 2, 1> b;
    b(0).value() = 2;
    b(0).derivatives() = Eigen∷Vector3d(1, 2, 3);
    b(1).value() = 3;
    b(1).derivatives() = Eigen∷Vector3d(4, 5, 6);
    Eigen∷Matrix<AutoDiffXd, 2, 1> x = solver.Solve(b);

.. raw:: html

    </details>)""";
        // Symbol: drake::math::LinearSolver::LinearSolver<LinearSolverType, DerivedA>
        struct /* ctor */ {
          // Source: drake/math/linear_solve.h
          const char* doc =
R"""(Default constructor. Constructs an empty linear solver.)""";
        } ctor;
        // Symbol: drake::math::LinearSolver::Solve
        struct /* Solve */ {
          // Source: drake/math/linear_solve.h
          const char* doc =
R"""(Solves system A*x = b. Return type is as described in the table above.)""";
        } Solve;
        // Symbol: drake::math::LinearSolver::SolverType
        struct /* SolverType */ {
          // Source: drake/math/linear_solve.h
          const char* doc = R"""()""";
        } SolverType;
        // Symbol: drake::math::LinearSolver::eigen_linear_solver
        struct /* eigen_linear_solver */ {
          // Source: drake/math/linear_solve.h
          const char* doc =
R"""(Getter for the Eigen linear solver. The scalar type in the Eigen
linear solver depends on the scalar type in A matrix, as shown in this
table | A | double | ADS | Expr |
|--------------|--------|--------|----- | |linear_solver | double |
double | Expr |

where ADS stands for Eigen∷AutoDiffScalar, Expr stands for
symbolic∷Expression.

Note that when A contains autodiffscalar, we only use the double
version of Eigen linear solver. By using implicit-function theorem
with the double-valued Eigen linear solver, we can compute the
gradient of the solution much faster than directly autodiffing the
Eigen linear solver.)""";
        } eigen_linear_solver;
      } LinearSolver;
      // Symbol: drake::math::MatGradMult
      struct /* MatGradMult */ {
        // Source: drake/math/gradient_util.h
        const char* doc = R"""()""";
        // Symbol: drake::math::MatGradMult::type
        struct /* type */ {
          // Source: drake/math/gradient_util.h
          const char* doc = R"""()""";
        } type;
      } MatGradMult;
      // Symbol: drake::math::MatGradMultMat
      struct /* MatGradMultMat */ {
        // Source: drake/math/gradient_util.h
        const char* doc = R"""()""";
        // Symbol: drake::math::MatGradMultMat::type
        struct /* type */ {
          // Source: drake/math/gradient_util.h
          const char* doc = R"""()""";
        } type;
      } MatGradMultMat;
      // Symbol: drake::math::NormalizeVector
      struct /* NormalizeVector */ {
        // Source: drake/math/normalize_vector.h
        const char* doc =
R"""(Computes the normalized vector, optionally with its gradient and
second derivative.

Parameter ``x``:
    An N x 1 vector to be normalized. Must not be zero.

Parameter ``x_norm``:
    The normalized vector (N x 1).

Parameter ``dx_norm``:
    If non-null, returned as an N x N matrix, where dx_norm(i,j) = D
    x_norm(i)/D x(j).

Parameter ``ddx_norm``:
    If non-null, and dx_norm is non-null, returned as an N^2 x N
    matrix, where ddx_norm.col(j) = D dx_norm/D x(j), with dx_norm
    stacked columnwise.

(D x / D y above means partial derivative of x with respect to y.))""";
      } NormalizeVector;
      // Symbol: drake::math::NumericalGradientMethod
      struct /* NumericalGradientMethod */ {
        // Source: drake/math/compute_numerical_gradient.h
        const char* doc = R"""()""";
        // Symbol: drake::math::NumericalGradientMethod::kBackward
        struct /* kBackward */ {
          // Source: drake/math/compute_numerical_gradient.h
          const char* doc =
R"""(Compute the gradient as (f(x) - f(x - Δx)) / Δx, with Δx > 0)""";
        } kBackward;
        // Symbol: drake::math::NumericalGradientMethod::kCentral
        struct /* kCentral */ {
          // Source: drake/math/compute_numerical_gradient.h
          const char* doc =
R"""(Compute the gradient as (f(x + Δx) - f(x - Δx)) / (2Δx), with Δx > 0)""";
        } kCentral;
        // Symbol: drake::math::NumericalGradientMethod::kForward
        struct /* kForward */ {
          // Source: drake/math/compute_numerical_gradient.h
          const char* doc =
R"""(Compute the gradient as (f(x + Δx) - f(x)) / Δx, with Δx > 0)""";
        } kForward;
      } NumericalGradientMethod;
      // Symbol: drake::math::NumericalGradientOption
      struct /* NumericalGradientOption */ {
        // Source: drake/math/compute_numerical_gradient.h
        const char* doc = R"""()""";
        // Symbol: drake::math::NumericalGradientOption::NumericalGradientOption
        struct /* ctor */ {
          // Source: drake/math/compute_numerical_gradient.h
          const char* doc =
R"""(Parameter ``function_accuracy``:
    The accuracy of evaluating function f(x). For double-valued
    functions (with magnitude around 1), the accuracy is usually about
    1E-15.)""";
        } ctor;
        // Symbol: drake::math::NumericalGradientOption::method
        struct /* method */ {
          // Source: drake/math/compute_numerical_gradient.h
          const char* doc = R"""()""";
        } method;
        // Symbol: drake::math::NumericalGradientOption::perturbation_size
        struct /* perturbation_size */ {
          // Source: drake/math/compute_numerical_gradient.h
          const char* doc = R"""()""";
        } perturbation_size;
      } NumericalGradientOption;
      // Symbol: drake::math::ProjectMatToRotMatWithAxis
      struct /* ProjectMatToRotMatWithAxis */ {
        // Source: drake/math/rotation_matrix.h
        const char* doc =
R"""(Projects an approximate 3 x 3 rotation matrix M onto an orthonormal
matrix R so that R is a rotation matrix associated with a angle-axis
rotation by an angle θ about a vector direction ``axis``, with
``angle_lb <= θ <= angle_ub``.

Parameter ``M``:
    the matrix to be projected.

Parameter ``axis``:
    vector direction associated with angle-axis rotation for R. axis
    can be a non-unit vector, but cannot be the zero vector.

Parameter ``angle_lb``:
    the lower bound of the rotation angle θ.

Parameter ``angle_ub``:
    the upper bound of the rotation angle θ.

Returns:
    Rotation angle θ of the projected matrix, angle_lb <= θ <=
    angle_ub

Raises:
    RuntimeError if axis is the zero vector or if angle_lb > angle_ub.

Note:
    This method is useful for reconstructing a rotation matrix for a
    revolute joint with joint limits.

Note:
    This can be formulated as an optimization problem


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    min_θ trace((R - M)ᵀ*(R - M))
      subject to R = I + sinθ * A + (1 - cosθ) * A²   (1)
                 angle_lb <= θ <= angle_ub

.. raw:: html

    </details>

where A is the cross product matrix of a = axis / axis.norm() = [a₁,
a₂, a₃]


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    A = [ 0  -a₃  a₂]
        [ a₃  0  -a₁]
        [-a₂  a₁  0 ]

.. raw:: html

    </details>

Equation (1) is the Rodriguez Formula that computes the rotation
matrix R from the angle-axis rotation with angle θ and vector
direction ``axis``. For details, see
http://mathworld.wolfram.com/RodriguesRotationFormula.html The
objective function can be simplified as


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    max_θ trace(Rᵀ * M + Mᵀ * R)

.. raw:: html

    </details>

By substituting the matrix ``R`` with the angle-axis representation,
the optimization problem is formulated as


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    max_θ sinθ * trace(Aᵀ*M) - cosθ * trace(Mᵀ * A²)
       subject to angle_lb <= θ <= angle_ub

.. raw:: html

    </details>

By introducing α = atan2(-trace(Mᵀ * A²), trace(Aᵀ*M)), we can compute
the optimal θ as


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    θ = π/2 + 2kπ - α, if angle_lb <= π/2 + 2kπ - α <= angle_ub, k ∈ integers
    else
       θ = angle_lb, if sin(angle_lb + α) >= sin(angle_ub + α)
       θ = angle_ub, if sin(angle_lb + α) <  sin(angle_ub + α)

.. raw:: html

    </details>

See also:
    GlobalInverseKinematics for an usage of this function.)""";
      } ProjectMatToRotMatWithAxis;
      // Symbol: drake::math::QuaternionToCanonicalForm
      struct /* QuaternionToCanonicalForm */ {
        // Source: drake/math/quaternion.h
        const char* doc =
R"""(This function returns a quaternion in its "canonical form" meaning
that it returns a quaternion [w, x, y, z] with a non-negative w. For
example, if passed a quaternion [-0.3, +0.4, +0.5, +0.707], the
function returns the quaternion's canonical form [+0.3, -0.4, -0.5,
-0.707].

Parameter ``quat``:
    Quaternion [w, x, y, z] that relates two right-handed orthogonal
    unitary bases e.g., Ax, Ay, Az (A) to Bx, By, Bz (B). Note: quat
    is analogous to the rotation matrix R_AB.

Returns:
    Canonical form of quat, which means that either the original quat
    is returned or a quaternion representing the same orientation but
    with negated [w, x, y, z], to ensure a positive w in returned
    quaternion.)""";
      } QuaternionToCanonicalForm;
      // Symbol: drake::math::QuaternionToHopfCoordinate
      struct /* QuaternionToHopfCoordinate */ {
        // Source: drake/math/hopf_coordinate.h
        const char* doc =
R"""(Convert a unit-length quaternion (w, x, y, z) (with the requirement w
>= 0) to Hopf coordinate as ψ = 2*atan2(x, w) φ = mod(atan2(z, y) -
ψ/2, 2pi) θ = 2*atan2(√(y²+z²), √(w²+x²)) ψ is in the range of [-pi,
pi]. φ is in the range of [0, 2pi]. θ is in the range of [0, pi]. If
the given quaternion has w < 0, then we reverse the signs of (w, x, y,
z), and compute the Hopf coordinate of (-w, -x, -y, -z).

Parameter ``quaternion``:
    A unit length quaternion.

Returns:
    hopf_coordinate (θ, φ, ψ) as an Eigen vector.)""";
      } QuaternionToHopfCoordinate;
      // Symbol: drake::math::RealContinuousLyapunovEquation
      struct /* RealContinuousLyapunovEquation */ {
        // Source: drake/math/continuous_lyapunov_equation.h
        const char* doc =
R"""(Parameter ``A``:
    A user defined real square matrix.

Parameter ``Q``:
    A user defined real symmetric matrix.

Precondition:
    Q is a symmetric matrix.

Computes a unique solution X to the continuous Lyapunov equation:
``AᵀX + XA + Q = 0``, where A is real and square, and Q is real,
symmetric and of equal size as A.

Raises:
    RuntimeError if A or Q are not square matrices or do not have the
    same size.

Limitations: Given the Eigenvalues of A as λ₁, ..., λₙ, there exists a
unique solution if and only if λᵢ + λ̅ⱼ ≠ 0 ∀ i,j, where λ̅ⱼ is the
complex conjugate of λⱼ.

Raises:
    RuntimeError if the solution is not unique.

There are no further limitations on the eigenvalues of A. Further, if
all λᵢ have negative real parts, and if Q is positive semi-definite,
then X is also positive semi-definite [1]. Therefore, if one searches
for a Lyapunov function V(z) = zᵀXz for the stable linear system ż =
Az, then the solution of the Lyapunov Equation ``AᵀX + XA + Q = 0``
only returns a valid Lyapunov function if Q is positive semi-definite.

The implementation is based on SLICOT routine SB03MD [2]. Note the
transformation Q = -C. The complexity of this routine is O(n³). If A
is larger than 2-by-2, then a Schur factorization is performed.

Raises:
    RuntimeError if Schur factorization failed.

A tolerance of ε is used to check if a double variable is equal to
zero, where the default value for ε is 1e-10. It has been used to
check (1) if λᵢ + λ̅ⱼ = 0, ∀ i,j; (2) if A is a 1-by-1 zero matrix;
(3) if A's trace or determinant is 0 when A is a 2-by-2 matrix.

[1] Bartels, R.H. and G.W. Stewart, "Solution of the Matrix Equation
AX + XB = C," Comm. of the ACM, Vol. 15, No. 9, 1972.

[2] http://slicot.org/objects/software/shared/doc/SB03MD.html)""";
      } RealContinuousLyapunovEquation;
      // Symbol: drake::math::RealDiscreteLyapunovEquation
      struct /* RealDiscreteLyapunovEquation */ {
        // Source: drake/math/discrete_lyapunov_equation.h
        const char* doc =
R"""(Parameter ``A``:
    A user defined real square matrix.

Parameter ``Q``:
    A user defined real symmetric matrix.

Precondition:
    Q is a symmetric matrix.

Computes the unique solution X to the discrete Lyapunov equation:
``AᵀXA - X + Q = 0``, where A is real and square, and Q is real,
symmetric and of equal size as A.

Raises:
    RuntimeError if A or Q are not square matrices or do not have the
    same size.

Limitations: Given the Eigenvalues of A as λ₁, ..., λₙ, there exists a
unique solution if and only if λᵢ * λⱼ ≠ 1 ∀ i,j and λᵢ ≠ ±1, ∀ i [1].

Raises:
    RuntimeError if the solution is not unique.[3]

There are no further limitations on the eigenvalues of A. Further, if
|λᵢ|<1, ∀ i, and if Q is positive semi-definite, then X is also
positive semi-definite [2]. Therefore, if one searches for a Lyapunov
function V(z) = zᵀXz for the stable linear system zₙ₊₁ = Azₙ, then the
solution of the Lyapunov Equation ``AᵀXA - X + Q = 0`` only returns a
valid Lyapunov function if Q is positive semi-definite.

The implementation is based on SLICOT routine SB03MD [2]. Note the
transformation Q = -C. The complexity of this routine is O(n³). If A
is larger than 2-by-2, then a Schur factorization is performed.

Raises:
    RuntimeError if Schur factorization fails.

A tolerance of ε is used to check if a double variable is equal to
zero, where the default value for ε is 1e-10. It has been used to
check (1) if λᵢ = ±1 ∀ i; (2) if λᵢ * λⱼ = 1, i ≠ j.

[1] Barraud, A.Y., "A numerical algorithm to solve AᵀXA - X = Q,"
IEEE® Trans. Auto. Contr., AC-22, pp. 883-885, 1977.

[2] http://slicot.org/objects/software/shared/doc/SB03MD.html

[3] https://www.mathworks.com/help/control/ref/dlyap.html)""";
      } RealDiscreteLyapunovEquation;
      // Symbol: drake::math::RigidTransform
      struct /* RigidTransform */ {
        // Source: drake/math/rigid_transform.h
        const char* doc =
R"""(This class represents a proper rigid transform between two frames
which can be regarded in two ways. A rigid transform describes the
"pose" between two frames A and B (i.e., the relative orientation and
position of A to B). Alternately, it can be regarded as a
distance-preserving operator that can rotate and/or translate a rigid
body without changing its shape or size (rigid) and without
mirroring/reflecting the body (proper), e.g., it can add one position
vector to another and express the result in a particular basis as
``p_AoQ_A = X_AB * p_BoQ_B`` (Q is any point). In many ways, this
rigid transform class is conceptually similar to using a homogeneous
matrix as a linear operator. See operator* documentation for an
exception.

The class stores a RotationMatrix that relates right-handed orthogonal
unit vectors Ax, Ay, Az fixed in frame A to right-handed orthogonal
unit vectors Bx, By, Bz fixed in frame B. The class also stores a
position vector from Ao (the origin of frame A) to Bo (the origin of
frame B). The position vector is expressed in frame A. The monogram
notation for the transform relating frame A to B is ``X_AB``. The
monogram notation for the rotation matrix relating A to B is ``R_AB``.
The monogram notation for the position vector from Ao to Bo is
``p_AoBo_A``. See multibody_quantities for monogram notation for
dynamics.

Note:
    This class does not store the frames associated with the transform
    and cannot enforce correct usage of this class. For example, it
    makes sense to multiply RigidTransforms as ``X_AB * X_BC``, but
    not ``X_AB * X_CB``.

Note:
    This class is not a 4x4 transformation matrix -- even though its
    operator*() methods act mostly like 4x4 matrix multiplication.
    Instead, this class contains a 3x3 rotation matrix class and a 3x1
    position vector. To convert this to a 3x4 matrix, use
    GetAsMatrix34(). To convert this to a 4x4 matrix, use
    GetAsMatrix4(). To convert this to an Eigen∷Isometry, use
    GetAsIsometry().

Note:
    An isometry is sometimes regarded as synonymous with rigid
    transform. The RigidTransform class has important advantages over
    Eigen∷Isometry. - RigidTransform is built on an underlying
    rigorous 3x3 RotationMatrix class that has significant
    functionality for 3D orientation. - In Debug builds,
    RigidTransform requires a valid 3x3 rotation matrix and a valid
    (non-NAN) position vector. Eigen∷Isometry does not. -
    RigidTransform catches bugs that are undetected by Eigen∷Isometry.
    - RigidTransform has additional functionality and ease-of-use,
    resulting in shorter, easier to write, and easier to read code. -
    The name Isometry is unfamiliar to many roboticists and
    dynamicists and for them Isometry.linear() is (for example) a
    counter-intuitive method name to return a rotation matrix.

Note:
    One of the constructors in this class provides an implicit
    conversion from an Eigen Translation to RigidTransform.

Authors:
    Paul Mitiguy (2018) Original author.

Authors:
    Drake team (see https://drake.mit.edu/credits).)""";
        // Symbol: drake::math::RigidTransform::ApplyAxialRotation
        struct /* ApplyAxialRotation */ {
          // Source: drake/math/rigid_transform.h
          const char* doc =
R"""((Internal use only) Given an axial rotation transform arX_BC (just an
axial rotation and no translation), uses it to efficiently re-express
a given vector. This takes only 6 floating point operations.

Parameter ``arX_BC``:
    the axial transform to be applied.

Parameter ``p_C``:
    a position vector expressed in frame C, to be re-expressed in
    frame B since there is zero translation.

Returns ``p_B``:
    the input position vector p_C, now re-expressed in frame B.

Template parameter ``axis``:
    0, 1, or 2 corresponding to +x, +y, or +z rotation axis.

Precondition:
    arX_BC is an special_xform_def "Axial rotation transform".)""";
        } ApplyAxialRotation;
        // Symbol: drake::math::RigidTransform::GetAsIsometry3
        struct /* GetAsIsometry3 */ {
          // Source: drake/math/rigid_transform.h
          const char* doc =
R"""(Returns the isometry in ℜ³ that is equivalent to a RigidTransform.)""";
        } GetAsIsometry3;
        // Symbol: drake::math::RigidTransform::GetAsMatrix34
        struct /* GetAsMatrix34 */ {
          // Source: drake/math/rigid_transform.h
          const char* doc =
R"""(Returns the 3x4 matrix associated with this RigidTransform, i.e.,
X_AB.


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    ┌                ┐
     │ R_AB  p_AoBo_A │
     └                ┘

.. raw:: html

    </details>)""";
        } GetAsMatrix34;
        // Symbol: drake::math::RigidTransform::GetAsMatrix4
        struct /* GetAsMatrix4 */ {
          // Source: drake/math/rigid_transform.h
          const char* doc =
R"""(Returns the 4x4 matrix associated with this RigidTransform, i.e.,
X_AB.


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    ┌                ┐
     │ R_AB  p_AoBo_A │
     │                │
     │   0      1     │
     └                ┘

.. raw:: html

    </details>)""";
        } GetAsMatrix4;
        // Symbol: drake::math::RigidTransform::GetMaximumAbsoluteDifference
        struct /* GetMaximumAbsoluteDifference */ {
          // Source: drake/math/rigid_transform.h
          const char* doc =
R"""(Computes the infinity norm of ``this`` - `other` (i.e., the maximum
absolute value of the difference between the elements of ``this`` and
``other``).

Parameter ``other``:
    RigidTransform to subtract from ``this``.

Returns:
    ‖`this` - `other`‖∞)""";
        } GetMaximumAbsoluteDifference;
        // Symbol: drake::math::RigidTransform::GetMaximumAbsoluteTranslationDifference
        struct /* GetMaximumAbsoluteTranslationDifference */ {
          // Source: drake/math/rigid_transform.h
          const char* doc =
R"""(Returns the maximum absolute value of the difference in the position
vectors (translation) in ``this`` and ``other``. In other words,
returns the infinity norm of the difference in the position vectors.

Parameter ``other``:
    RigidTransform whose position vector is subtracted from the
    position vector in ``this``.)""";
        } GetMaximumAbsoluteTranslationDifference;
        // Symbol: drake::math::RigidTransform::Identity
        struct /* Identity */ {
          // Source: drake/math/rigid_transform.h
          const char* doc =
R"""(Returns the identity RigidTransform (corresponds to coincident
frames).

Returns:
    the RigidTransform that corresponds to aligning the two frames so
    unit vectors Ax = Bx, Ay = By, Az = Bz and point Ao is coincident
    with Bo. Hence, the returned RigidTransform contains a 3x3
    identity matrix and a zero position vector.)""";
        } Identity;
        // Symbol: drake::math::RigidTransform::InvertAndCompose
        struct /* InvertAndCompose */ {
          // Source: drake/math/rigid_transform.h
          const char* doc =
R"""(Calculates the product of ``this`` inverted and another
RigidTransform. If you consider ``this`` to be the transform X_AB, and
``other`` to be X_AC, then this method returns X_BC = X_AB⁻¹ * X_AC.
For T==double, this method can be *much* faster than inverting first
and then performing the composition, because it can take advantage of
the special structure of a rigid transform to avoid unnecessary memory
and floating point operations. On some platforms it can use SIMD
instructions for further speedups.

Parameter ``other``:
    RigidTransform that post-multiplies ``this`` inverted.

Returns ``X_BC``:
    where X_BC = this⁻¹ * other.

Note:
    It is possible (albeit improbable) to create an invalid rigid
    transform by accumulating round-off error with a large number of
    multiplies.)""";
        } InvertAndCompose;
        // Symbol: drake::math::RigidTransform::IsExactlyEqualTo
        struct /* IsExactlyEqualTo */ {
          // Source: drake/math/rigid_transform.h
          const char* doc =
R"""(Returns true if ``this`` is exactly equal to ``other``.

Parameter ``other``:
    RigidTransform to compare to ``this``.

Returns:
    ``True`` if each element of ``this`` is exactly equal to the
    corresponding element of ``other``.)""";
        } IsExactlyEqualTo;
        // Symbol: drake::math::RigidTransform::IsExactlyIdentity
        struct /* IsExactlyIdentity */ {
          // Source: drake/math/rigid_transform.h
          const char* doc =
R"""(Returns ``True`` if ``this`` is exactly the identity RigidTransform.

See also:
    IsNearlyIdentity().)""";
        } IsExactlyIdentity;
        // Symbol: drake::math::RigidTransform::IsNearlyEqualTo
        struct /* IsNearlyEqualTo */ {
          // Source: drake/math/rigid_transform.h
          const char* doc =
R"""(Compares each element of ``this`` to the corresponding element of
``other`` to check if they are the same to within a specified
``tolerance``.

Parameter ``other``:
    RigidTransform to compare to ``this``.

Parameter ``tolerance``:
    maximum allowable absolute difference between the elements in
    ``this`` and ``other``.

Returns:
    ``True`` if ``‖this.matrix() - other.matrix()‖∞ <= tolerance``.

Note:
    Consider scaling tolerance with the largest of magA and magB,
    where magA and magB denoted the magnitudes of ``this`` position
    vector and ``other`` position vectors, respectively.)""";
        } IsNearlyEqualTo;
        // Symbol: drake::math::RigidTransform::IsNearlyIdentity
        struct /* IsNearlyIdentity */ {
          // Source: drake/math/rigid_transform.h
          const char* doc =
R"""(Returns true if ``this`` is within tolerance of the identity
RigidTransform.

Parameter ``translation_tolerance``:
    a non-negative number. One way to choose ``translation_tolerance``
    is to multiply a characteristic length (e.g., the magnitude of a
    characteristic position vector) by an epsilon (e.g.,
    RotationMatrix∷get_internal_tolerance_for_orthonormality()).

Returns:
    ``True`` if the RotationMatrix portion of ``this`` satisfies
    RotationMatrix∷IsNearlyIdentity() and if the position vector
    portion of ``this`` is equal to zero vector within
    ``translation_tolerance``.

See also:
    IsExactlyIdentity().)""";
        } IsNearlyIdentity;
        // Symbol: drake::math::RigidTransform::MakeAxialRotation
        struct /* MakeAxialRotation */ {
          // Source: drake/math/rigid_transform.h
          const char* doc =
R"""((Internal use only) Creates an axial rotation transform arX_AB
consisting of only an axial rotation of ``theta`` radians about x, y,
or z and no translation. Of the 12 entries in the transform matrix,
only 4 are active; the rest will be set to 0 or 1. This structure can
be exploited for efficient updating and operating with this transform.

Parameter ``theta``:
    the rotation angle.

Returns ``arX_BC``:
    the axial transform (also known as X_BC(theta)).

Template parameter ``axis``:
    0, 1, or 2 corresponding to +x, +y, or +z rotation axis.

See also:
    special_xform_def "Specialized transforms".)""";
        } MakeAxialRotation;
        // Symbol: drake::math::RigidTransform::MakeUnchecked
        struct /* MakeUnchecked */ {
          // Source: drake/math/rigid_transform.h
          const char* doc =
R"""((Advanced) Constructs a RigidTransform from a 3x4 matrix, without any
validity checks nor orthogonalization.

Parameter ``pose``:
    3x4 matrix that contains a 3x3 rotation matrix ``R_AB`` and also a
    3x1 position vector ``p_AoBo_A`` (the position vector from frame
    A's origin to frame B's origin, expressed in frame A).


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    ┌                ┐
     │ R_AB  p_AoBo_A │
     └                ┘

.. raw:: html

    </details>)""";
        } MakeUnchecked;
        // Symbol: drake::math::RigidTransform::PostMultiplyByAxialRotation
        struct /* PostMultiplyByAxialRotation */ {
          // Source: drake/math/rigid_transform.h
          const char* doc =
R"""((Internal use only) With ``this`` a general transform X_AB, and given
an axial rotation transform arX_BC, efficiently calculates ``X_AC =
X_AB * arX_BC``. This requires only 18 floating point operations.

Parameter ``arX_BC``:
    An axial rotation transform about the indicated ``axis``.

Parameter ``X_AC``:
    Preallocated space for the result, which will be a general
    transform. Must not overlap with ``this`` in memory.

Template parameter ``axis``:
    0, 1, or 2 corresponding to +x, +y, or +z rotation axis.

Precondition:
    arX_BC is an special_xform_def "Axial rotation transform"

Precondition:
    X_AC does not overlap with ``this`` in memory.)""";
        } PostMultiplyByAxialRotation;
        // Symbol: drake::math::RigidTransform::PostMultiplyByAxialTranslation
        struct /* PostMultiplyByAxialTranslation */ {
          // Source: drake/math/rigid_transform.h
          const char* doc =
R"""((Internal use only) Composes ``this`` general transform X_AB with a
given axial translation transform atX_BC to efficiently calculate
``X_AC = X_AB * atX_BC``. This requires only 6 floating point
operations.

Parameter ``atX_BC``:
    An axial translation transform about the indicated ``axis``.

Parameter ``X_AC``:
    Preallocated space for the result, which will be a general
    transform.

Template parameter ``axis``:
    0, 1, or 2 corresponding to +x, +y, or +z rotation axis.

Precondition:
    atX_BC is an special_xform_def "Axial translation transform".)""";
        } PostMultiplyByAxialTranslation;
        // Symbol: drake::math::RigidTransform::PostMultiplyByRotation
        struct /* PostMultiplyByRotation */ {
          // Source: drake/math/rigid_transform.h
          const char* doc =
R"""((Internal use only) Composes ``this`` general transform X_AB with a
given rotation-only transform rX_BC to efficiently calculate ``X_AC =
X_AB * rX_BC``. This requires 45 floating point operations.

Parameter ``rX_BC``:
    the rotation-only transform.

Parameter ``X_AC``:
    Preallocated space for the result.

Precondition:
    rX_BC is a special_xform_def "Rotation transform".)""";
        } PostMultiplyByRotation;
        // Symbol: drake::math::RigidTransform::PostMultiplyByTranslation
        struct /* PostMultiplyByTranslation */ {
          // Source: drake/math/rigid_transform.h
          const char* doc =
R"""((Internal use only) Composes ``this`` general transform X_AB with a
given translation-only transform tX_BC to efficiently calculate ``X_AC
= X_AB * tX_BC``. This requires only 18 floating point operations.

Parameter ``tX_BC``:
    the translation-only transform.

Parameter ``X_AC``:
    preallocated space for the result.

Precondition:
    tX_BC is a special_xform_def "Translation transform".)""";
        } PostMultiplyByTranslation;
        // Symbol: drake::math::RigidTransform::PreMultiplyByAxialRotation
        struct /* PreMultiplyByAxialRotation */ {
          // Source: drake/math/rigid_transform.h
          const char* doc =
R"""((Internal use only) With ``this`` a general transform X_BC, and given
an axial rotation transform arX_AB, efficiently calculates ``X_AC =
arX_AB * X_BC``. This requires only 24 floating point operations.

Parameter ``arX_AB``:
    An axial rotation transform about the indicated ``axis``.

Parameter ``X_AC``:
    Preallocated space for the result, which will be a general
    transform. Must not overlap with arX_BC or ``this`` in memory.

Template parameter ``axis``:
    0, 1, or 2 corresponding to +x, +y, or +z rotation axis.

Precondition:
    arX_AB is an special_xform_def "Axial rotation transform"

Precondition:
    X_AC does not overlap with arX_AB or ``this`` in memory.)""";
        } PreMultiplyByAxialRotation;
        // Symbol: drake::math::RigidTransform::PreMultiplyByAxialTranslation
        struct /* PreMultiplyByAxialTranslation */ {
          // Source: drake/math/rigid_transform.h
          const char* doc =
R"""((Internal use only) With ``this`` a general transform X_BC, and given
an axial translation transform atX_AB, efficiently calculates ``X_AC =
atX_AB * X_BC``. This requires just 1 floating point operation.

Parameter ``atX_AB``:
    An axial translation transform along the indicated ``axis``.

Parameter ``X_AC``:
    Preallocated space for the result, which will be a general
    transform.

Template parameter ``axis``:
    0, 1, or 2 corresponding to +x, +y, or +z translation axis.

Precondition:
    atX_AB is an special_xform_def "Axial translation transform")""";
        } PreMultiplyByAxialTranslation;
        // Symbol: drake::math::RigidTransform::RigidTransform<type-parameter-0-0>
        struct /* ctor */ {
          // Source: drake/math/rigid_transform.h
          const char* doc_0args =
R"""(Constructs the RigidTransform that corresponds to aligning the two
frames so unit vectors Ax = Bx, Ay = By, Az = Bz and point Ao is
coincident with Bo. Hence, the constructed RigidTransform contains an
identity RotationMatrix and a zero position vector.)""";
          // Source: drake/math/rigid_transform.h
          const char* doc_2args_R_p =
R"""(Constructs a RigidTransform from a rotation matrix and a position
vector.

Parameter ``R``:
    rotation matrix relating frames A and B (e.g., ``R_AB``).

Parameter ``p``:
    position vector from frame A's origin to frame B's origin,
    expressed in frame A. In monogram notation p is denoted
    ``p_AoBo_A``.)""";
          // Source: drake/math/rigid_transform.h
          const char* doc_2args_rpy_p =
R"""(Constructs a RigidTransform from a RollPitchYaw and a position vector.

Parameter ``rpy``:
    a RollPitchYaw which is a Space-fixed (extrinsic) X-Y-Z rotation
    with "roll-pitch-yaw" angles ``[r, p, y]`` or equivalently a Body-
    fixed (intrinsic) Z-Y-X rotation with "yaw-pitch-roll" angles
    ``[y, p, r]``.

See also:
    RotationMatrix∷RotationMatrix(const RollPitchYaw<T>&)

Parameter ``p``:
    position vector from frame A's origin to frame B's origin,
    expressed in frame A. In monogram notation p is denoted
    ``p_AoBo_A``.)""";
          // Source: drake/math/rigid_transform.h
          const char* doc_2args_quaternion_p =
R"""(Constructs a RigidTransform from a Quaternion and a position vector.

Parameter ``quaternion``:
    a non-zero, finite quaternion which may or may not have unit
    length [i.e., ``quaternion.norm()`` does not have to be 1].

Parameter ``p``:
    position vector from frame A's origin to frame B's origin,
    expressed in frame A. In monogram notation p is denoted
    ``p_AoBo_A``.

Raises:
    RuntimeError in debug builds if the rotation matrix that is built
    from ``quaternion`` is invalid.

See also:
    RotationMatrix∷RotationMatrix(const Eigen∷Quaternion<T>&))""";
          // Source: drake/math/rigid_transform.h
          const char* doc_2args_theta_lambda_p =
R"""(Constructs a RigidTransform from an AngleAxis and a position vector.

Parameter ``theta_lambda``:
    an Eigen∷AngleAxis whose associated axis (vector direction herein
    called ``lambda``) is non-zero and finite, but which may or may
    not have unit length [i.e., ``lambda.norm()`` does not have to be
    1].

Parameter ``p``:
    position vector from frame A's origin to frame B's origin,
    expressed in frame A. In monogram notation p is denoted
    ``p_AoBo_A``.

Raises:
    RuntimeError in debug builds if the rotation matrix that is built
    from ``theta_lambda`` is invalid.

See also:
    RotationMatrix∷RotationMatrix(const Eigen∷AngleAxis<T>&))""";
          // Source: drake/math/rigid_transform.h
          const char* doc_1args_R =
R"""(Constructs a RigidTransform with a given RotationMatrix and a zero
position vector.

Parameter ``R``:
    rotation matrix relating frames A and B (e.g., ``R_AB``).)""";
          // Source: drake/math/rigid_transform.h
          const char* doc_1args_p =
R"""(Constructs a RigidTransform that contains an identity RotationMatrix
and a given position vector ``p``.

Parameter ``p``:
    position vector from frame A's origin to frame B's origin,
    expressed in frame A. In monogram notation p is denoted
    ``p_AoBo_A``.)""";
          // Source: drake/math/rigid_transform.h
          const char* doc_1args_translation =
R"""(Constructs a RigidTransform that contains an identity RotationMatrix
and the position vector underlying the given ``translation``.

Parameter ``translation``:
    translation-only transform that stores p_AoQ_A, the position
    vector from frame A's origin to a point Q, expressed in frame A.

Note:
    The constructed RigidTransform ``X_AAq`` relates frame A to a
    frame Aq whose basis unit vectors are aligned with Ax, Ay, Az and
    whose origin position is located at point Q.

Note:
    This constructor provides an implicit conversion from Translation
    to RigidTransform.)""";
          // Source: drake/math/rigid_transform.h
          const char* doc_1args_pose =
R"""(Constructs a RigidTransform from an Eigen Isometry3.

Parameter ``pose``:
    Isometry3 that contains an allegedly valid rotation matrix
    ``R_AB`` and also contains a position vector ``p_AoBo_A`` from
    frame A's origin to frame B's origin. ``p_AoBo_A`` must be
    expressed in frame A.

Raises:
    RuntimeError in debug builds if R_AB is not a proper orthonormal
    3x3 rotation matrix.

Note:
    No attempt is made to orthogonalize the 3x3 rotation matrix part
    of ``pose``. As needed, use
    RotationMatrix∷ProjectToRotationMatrix().)""";
          // Source: drake/math/rigid_transform.h
          const char* doc_1args_constEigenMatrixBase =
R"""(Constructs a RigidTransform from an appropriate Eigen **expression**.

Parameter ``pose``:
    Generic Eigen matrix **expression**.

Raises:
    RuntimeError if the Eigen **expression** in pose does not resolve
    to a Vector3 or 3x4 matrix or 4x4 matrix or if the rotational part
    of ``pose`` is not a proper orthonormal 3x3 rotation matrix or if
    ``pose`` is a 4x4 matrix whose final row is not ``[0, 0, 0, 1]``.

Note:
    No attempt is made to orthogonalize the 3x3 rotation matrix part
    of ``pose``. As needed, use
    RotationMatrix∷ProjectToRotationMatrix().

Note:
    This constructor prevents ambiguity that would otherwise exist for
    a RigidTransform constructor whose argument is an Eigen
    **expression**.


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    const Vector3<double> position(4, 5, 6);
    const RigidTransform<double> X1(3 * position);
    ----------------------------------------------
    const RotationMatrix<double> R(RollPitchYaw<double>(1, 2, 3));
    Eigen∷Matrix<double, 3, 4> pose34;
    pose34 << R.matrix(), position;
    const RigidTransform<double> X2(1.0 * pose34);
    ----------------------------------------------
    Eigen∷Matrix<double, 4, 4> pose4;
    pose4 << R.matrix(), position,
             0, 0, 0, 1;
    const RigidTransform<double> X3(pose4 * pose4);

.. raw:: html

    </details>)""";
        } ctor;
        // Symbol: drake::math::RigidTransform::SetFromIsometry3
        struct /* SetFromIsometry3 */ {
          // Source: drake/math/rigid_transform.h
          const char* doc =
R"""(Sets ``this`` RigidTransform from an Eigen Isometry3.

Parameter ``pose``:
    Isometry3 that contains an allegedly valid rotation matrix
    ``R_AB`` and also contains a position vector ``p_AoBo_A`` from
    frame A's origin to frame B's origin. ``p_AoBo_A`` must be
    expressed in frame A.

Raises:
    RuntimeError in debug builds if R_AB is not a proper orthonormal
    3x3 rotation matrix.

Note:
    No attempt is made to orthogonalize the 3x3 rotation matrix part
    of ``pose``. As needed, use
    RotationMatrix∷ProjectToRotationMatrix().)""";
        } SetFromIsometry3;
        // Symbol: drake::math::RigidTransform::SetIdentity
        struct /* SetIdentity */ {
          // Source: drake/math/rigid_transform.h
          const char* doc =
R"""(Sets ``this`` RigidTransform so it corresponds to aligning the two
frames so unit vectors Ax = Bx, Ay = By, Az = Bz and point Ao is
coincident with Bo. Hence, ``this`` RigidTransform contains a 3x3
identity matrix and a zero position vector.)""";
        } SetIdentity;
        // Symbol: drake::math::RigidTransform::UpdateAxialRotation
        struct /* UpdateAxialRotation */ {
          // Source: drake/math/rigid_transform.h
          const char* doc_2args =
R"""((Internal use only) Given a new rotation angle θ, updates the axial
rotation transform arX_BC to represent the new rotation angle. We
expect that arX_BC was already such a transform (about the given x, y,
or z axis). Only the 4 active elements are modified; the other 8
remain unchanged.

Parameter ``theta``:
    the new rotation angle in radians.

Parameter ``arX_BC``:
    the axial rotation transform matrix to be updated.

Template parameter ``axis``:
    0, 1, or 2 corresponding to +x, +y, or +z rotation axis.

See also:
    the overloaded signature if you already have sin(θ) and cos(θ).

Precondition:
    arX_BC is an special_xform_def "Axial rotation transform".)""";
          // Source: drake/math/rigid_transform.h
          const char* doc_3args =
R"""((Internal use only) Given sin(θ) and cos(θ), where θ is a new rotation
angle, updates the axial rotation transform arX_BC to represent the
new rotation angle. We expect that arX_BC was already such a transform
(about the given x, y, or z axis). Only the 4 active elements are
modified; the other 8 remain unchanged.

Parameter ``sin_theta``:
    sin(θ), where θ is the new rotation angle.

Parameter ``cos_theta``:
    cos(θ), where θ is the new rotation angle.

Parameter ``arX_BC``:
    the axial rotation transform matrix to be updated.

Template parameter ``axis``:
    0, 1, or 2 corresponding to +x, +y, or +z rotation axis.

See also:
    the overloaded signature if you just have the angle θ.

Precondition:
    arX_BC is an special_xform_def "Axial rotation transform".

Precondition:
    ``sin_theta`` and ``cos_theta`` are sine and cosine of the same
    angle.)""";
        } UpdateAxialRotation;
        // Symbol: drake::math::RigidTransform::UpdateAxialTranslation
        struct /* UpdateAxialTranslation */ {
          // Source: drake/math/rigid_transform.h
          const char* doc =
R"""((Internal use only) Given a new ``distance``, updates the axial
translation transform atX_BC to represent the new translation by that
amount. We expect that atX_BC was already such a transform (about the
given x, y, or z axis). Only the 1 active element is modified; the
other 11 remain unchanged. No floating point operations are needed.

Parameter ``distance``:
    the component of p_BC along the ``axis`` direction.

Parameter ``atX_BC``:
    the axial translation transform matrix to be updated.

Template parameter ``axis``:
    0, 1, or 2 corresponding to +x, +y, or +z translation axis.

Precondition:
    atX_BC is an special_xform_def "Axial translation transform".)""";
        } UpdateAxialTranslation;
        // Symbol: drake::math::RigidTransform::cast
        struct /* cast */ {
          // Source: drake/math/rigid_transform.h
          const char* doc =
R"""(Creates a RigidTransform templatized on a scalar type U from a
RigidTransform templatized on scalar type T. For example,


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    RigidTransform<double> source = RigidTransform<double>∷Identity();
    RigidTransform<AutoDiffXd> foo = source.cast<AutoDiffXd>();

.. raw:: html

    </details>

Template parameter ``U``:
    Scalar type on which the returned RigidTransform is templated.

Note:
    ``RigidTransform<From>∷cast<To>()`` creates a new
    ``RigidTransform<To>`` from a ``RigidTransform<From>`` but only if
    type ``To`` is constructible from type ``From``. This cast method
    works in accordance with Eigen's cast method for Eigen's objects
    that underlie this RigidTransform. For example, Eigen currently
    allows cast from type double to AutoDiffXd, but not vice-versa.)""";
        } cast;
        // Symbol: drake::math::RigidTransform::inverse
        struct /* inverse */ {
          // Source: drake/math/rigid_transform.h
          const char* doc =
R"""(Returns X_BA = X_AB⁻¹, the inverse of ``this`` RigidTransform.

Note:
    The inverse of RigidTransform X_AB is X_BA, which contains the
    rotation matrix R_BA = R_AB⁻¹ = R_ABᵀ and the position vector
    ``p_BoAo_B_`` (position from B's origin Bo to A's origin Ao,
    expressed in frame B).

Note:
    : The square-root of a RigidTransform's condition number is
    roughly the magnitude of the position vector. The accuracy of the
    calculation for the inverse of a RigidTransform drops off with the
    sqrt condition number.)""";
        } inverse;
        // Symbol: drake::math::RigidTransform::operator*
        struct /* operator_mul */ {
          // Source: drake/math/rigid_transform.h
          const char* doc_1args_other =
R"""(Multiplies ``this`` RigidTransform ``X_AB`` by the ``other``
RigidTransform ``X_BC`` and returns the RigidTransform ``X_AC = X_AB *
X_BC``.)""";
          // Source: drake/math/rigid_transform.h
          const char* doc_1args_X_BBq =
R"""(Multiplies ``this`` RigidTransform ``X_AB`` by the translation-only
transform ``X_BBq`` and returns the RigidTransform ``X_ABq = X_AB *
X_BBq``.

Note:
    The rotation matrix in the returned RigidTransform ``X_ABq`` is
    equal to the rotation matrix in ``X_AB``. `X_ABq` and ``X_AB``
    only differ by origin location.)""";
          // Source: drake/math/rigid_transform.h
          const char* doc_1args_p_BoQ_B =
R"""(Multiplies ``this`` RigidTransform ``X_AB`` by the position vector
``p_BoQ_B`` which is from Bo (B's origin) to an arbitrary point Q.

Parameter ``p_BoQ_B``:
    position vector from Bo to Q, expressed in frame B.

Returns ``p_AoQ_A``:
    position vector from Ao to Q, expressed in frame A.)""";
          // Source: drake/math/rigid_transform.h
          const char* doc_1args_vec_B =
R"""(Multiplies ``this`` RigidTransform ``X_AB`` by the 4-element vector
``vec_B``, equivalent to ``X_AB.GetAsMatrix4() * vec_B``.

Parameter ``vec_B``:
    4-element vector whose first 3 elements are the position vector
    p_BoQ_B from Bo (frame B's origin) to an arbitrary point Q,
    expressed in frame B and whose 4ᵗʰ element is 1 𝐨𝐫 whose first 3
    elements are a vector (maybe unrelated to Bo or Q) and whose 4ᵗʰ
    element is 0.

Returns ``vec_A``:
    4-element vector whose first 3 elements are the position vector
    p_AoQ_A from Ao (frame A's origin) to Q, expressed in frame A and
    whose 4ᵗʰ element is 1 𝐨𝐫 whose first 3 elements are a vector
    (maybe unrelated to Bo and Q) and whose 4ᵗʰ element is 0.

Raises:
    RuntimeError if the 4ᵗʰ element of vec_B is not 0 or 1.)""";
          // Source: drake/math/rigid_transform.h
          const char* doc_1args_constEigenMatrixBase =
R"""(Multiplies ``this`` RigidTransform ``X_AB`` by the n position vectors
``p_BoQ1_B`` ... `p_BoQn_B`, where ``p_BoQi_B`` is the iᵗʰ position
vector from Bo (frame B's origin) to an arbitrary point Qi, expressed
in frame B.

Parameter ``p_BoQ_B``:
    ``3 x n`` matrix with n position vectors ``p_BoQi_B`` or an
    expression that resolves to a ``3 x n`` matrix of position
    vectors.

Returns ``p_AoQ_A``:
    ``3 x n`` matrix with n position vectors ``p_AoQi_A``, i.e., n
    position vectors from Ao (frame A's origin) to Qi, expressed in
    frame A. Specifically, this operator* is defined so that ``X_AB *
    p_BoQ_B`` returns ``p_AoQ_A = p_AoBo_A + R_AB * p_BoQ_B``, where
    ``p_AoBo_A`` is the position vector from Ao to Bo expressed in A
    and ``R_AB`` is the rotation matrix relating the orientation of
    frames A and B.

Note:
    As needed, use parentheses. This operator* is not associative. To
    see this, let ``p = p_AoBo_A``, `q = p_BoQ_B` and note (X_AB * q)
    * 7 = (p + R_AB * q) * 7 ≠ X_AB * (q * 7) = p + R_AB * (q * 7).


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    const RollPitchYaw<double> rpy(0.1, 0.2, 0.3);
    const RigidTransform<double> X_AB(rpy, Vector3d(1, 2, 3));
    Eigen∷Matrix<double, 3, 2> p_BoQ_B;
    p_BoQ_B.col(0) = Vector3d(4, 5, 6);
    p_BoQ_B.col(1) = Vector3d(9, 8, 7);
    const Eigen∷Matrix<double, 3, 2> p_AoQ_A = X_AB * p_BoQ_B;

.. raw:: html

    </details>)""";
        } operator_mul;
        // Symbol: drake::math::RigidTransform::operator*=
        struct /* operator_imul */ {
          // Source: drake/math/rigid_transform.h
          const char* doc =
R"""(In-place multiply of ``this`` RigidTransform ``X_AB`` by ``other``
RigidTransform ``X_BC``.

Parameter ``other``:
    RigidTransform that post-multiplies ``this``.

Returns:
    ``this`` RigidTransform which has been multiplied by ``other``. On
    return, ``this = X_AC``, where ``X_AC = X_AB * X_BC``.)""";
        } operator_imul;
        // Symbol: drake::math::RigidTransform::rotation
        struct /* rotation */ {
          // Source: drake/math/rigid_transform.h
          const char* doc =
R"""(Returns R_AB, the rotation matrix portion of ``this`` RigidTransform.

Returns ``R_AB``:
    the rotation matrix portion of ``this`` RigidTransform.)""";
        } rotation;
        // Symbol: drake::math::RigidTransform::set
        struct /* set */ {
          // Source: drake/math/rigid_transform.h
          const char* doc =
R"""(Sets ``this`` RigidTransform from a RotationMatrix and a position
vector.

Parameter ``R``:
    rotation matrix relating frames A and B (e.g., ``R_AB``).

Parameter ``p``:
    position vector from frame A's origin to frame B's origin,
    expressed in frame A. In monogram notation p is denoted
    ``p_AoBo_A``.)""";
        } set;
        // Symbol: drake::math::RigidTransform::set_rotation
        struct /* set_rotation */ {
          // Source: drake/math/rigid_transform.h
          const char* doc_1args_R =
R"""(Sets the RotationMatrix portion of ``this`` RigidTransform.

Parameter ``R``:
    rotation matrix relating frames A and B (e.g., ``R_AB``).)""";
          // Source: drake/math/rigid_transform.h
          const char* doc_1args_rpy =
R"""(Sets the rotation part of ``this`` RigidTransform from a RollPitchYaw.

Parameter ``rpy``:
    "roll-pitch-yaw" angles.

See also:
    RotationMatrix∷RotationMatrix(const RollPitchYaw<T>&) which
    describes the parameter, preconditions, etc.)""";
          // Source: drake/math/rigid_transform.h
          const char* doc_1args_quaternion =
R"""(Sets the rotation part of ``this`` RigidTransform from a Quaternion.

Parameter ``quaternion``:
    a quaternion which may or may not have unit length.

See also:
    RotationMatrix∷RotationMatrix(const Eigen∷Quaternion<T>&) which
    describes the parameter, preconditions, exception conditions, etc.)""";
          // Source: drake/math/rigid_transform.h
          const char* doc_1args_theta_lambda =
R"""(Sets the rotation part of ``this`` RigidTransform from an AngleAxis.

Parameter ``theta_lambda``:
    an angle ``theta`` (in radians) and vector ``lambda``.

See also:
    RotationMatrix∷RotationMatrix(const Eigen∷AngleAxis<T>&) which
    describes the parameter, preconditions, exception conditions, etc.)""";
        } set_rotation;
        // Symbol: drake::math::RigidTransform::set_translation
        struct /* set_translation */ {
          // Source: drake/math/rigid_transform.h
          const char* doc =
R"""(Sets the position vector portion of ``this`` RigidTransform.

Parameter ``p``:
    position vector from Ao (frame A's origin) to Bo (frame B's
    origin) expressed in frame A. In monogram notation p is denoted
    p_AoBo_A.)""";
        } set_translation;
        // Symbol: drake::math::RigidTransform::translation
        struct /* translation */ {
          // Source: drake/math/rigid_transform.h
          const char* doc =
R"""(Returns ``p_AoBo_A``, the position vector portion of ``this``
RigidTransform, i.e., position vector from Ao (frame A's origin) to Bo
(frame B's origin).)""";
        } translation;
      } RigidTransform;
      // Symbol: drake::math::RigidTransformd
      struct /* RigidTransformd */ {
        // Source: drake/math/rigid_transform.h
        const char* doc =
R"""(Abbreviation (alias/typedef) for a RigidTransform double scalar type.)""";
      } RigidTransformd;
      // Symbol: drake::math::RollPitchYaw
      struct /* RollPitchYaw */ {
        // Source: drake/math/roll_pitch_yaw.h
        const char* doc =
R"""(This class represents the orientation between two arbitrary frames A
and D associated with a Space-fixed (extrinsic) X-Y-Z rotation by
"roll-pitch-yaw" angles ``[r, p, y]``, which is equivalent to a
Body-fixed (intrinsic) Z-Y-X rotation by "yaw-pitch-roll" angles ``[y,
p, r]``. The rotation matrix ``R_AD`` associated with this
roll-pitch-yaw ``[r, p, y]`` rotation sequence is equal to the matrix
multiplication shown below.


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    ⎡cos(y) -sin(y)  0⎤   ⎡ cos(p)  0  sin(p)⎤   ⎡1      0        0 ⎤
    R_AD = ⎢sin(y)  cos(y)  0⎥ * ⎢     0   1      0 ⎥ * ⎢0  cos(r)  -sin(r)⎥
           ⎣    0       0   1⎦   ⎣-sin(p)  0  cos(p)⎦   ⎣0  sin(r)   cos(r)⎦
         =       R_AB          *        R_BC          *        R_CD

.. raw:: html

    </details>

Note:
    In this discussion, A is the Space frame and D is the Body frame.
    One way to visualize this rotation sequence is by introducing
    intermediate frames B and C (useful constructs to understand this
    rotation sequence). Initially, the frames are aligned so ``Di = Ci
    = Bi = Ai (i = x, y, z)`` where Dx, Dy, Dz and Ax, Ay, Az are
    orthogonal unit vectors fixed in frames D and A respectively.
    Similarly for Bx, By, Bz and Cx, Cy, Cz in frame B, C. Then D is
    subjected to successive right-handed rotations relative to A.

* 1st rotation R_CD: Frame D rotates relative to frames C, B, A by a
roll angle ``r`` about ``Dx = Cx``.  Note: D and C are no longer aligned.

* 2nd rotation R_BC: Frames D, C (collectively -- as if welded together)
rotate relative to frame B, A by a pitch angle ``p`` about ``Cy = By``.
Note: C and B are no longer aligned.

* 3rd rotation R_AB: Frames D, C, B (collectively -- as if welded)
rotate relative to frame A by a yaw angle ``y`` about ``Bz = Az``.
Note: B and A are no longer aligned.
The monogram notation for the rotation matrix relating A to D is ``R_AD``.

See also:
    multibody_quantities for monogram notation for dynamics and
    orientation_discussion "a discussion on rotation matrices".

Note:
    This class does not store the frames associated with this rotation
    sequence.)""";
        // Symbol: drake::math::RollPitchYaw::CalcAngularVelocityInChildFromRpyDt
        struct /* CalcAngularVelocityInChildFromRpyDt */ {
          // Source: drake/math/roll_pitch_yaw.h
          const char* doc =
R"""(Calculates angular velocity from ``this`` RollPitchYaw whose
roll-pitch-yaw angles ``[r; p; y]`` relate the orientation of two
generic frames A and D.

Parameter ``rpyDt``:
    Time-derivative of ``[r; p; y]``, i.e., ``[ṙ; ṗ; ẏ]``.

Returns:
    w_AD_D, frame D's angular velocity in frame A, expressed in
    "child" frame D. In other words, returns [ω0; ω1; ω2]ᴅ, where
    ``w_AD_D = ω0 Dx + ω1 Dy + ω2 Dz``, and where [ω0; ω1; ω2]ᴅ is
    calculated via the 3x3 matrix Nc⁻¹ (the inverse of the matrix Nc
    documented in CalcMatrixRelatingRpyDtToAngularVelocityInChild()).


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    ⌈ ω0 ⌉         ⌈ ṙ ⌉            ⌈ 1      0        -sin(p)    ⌉
    | ω1 |  = Nc⁻¹ | ṗ |     Nc⁻¹ = | 0   cos(r)   sin(r)*cos(p) |
    ⌊ ω2 ⌋ᴅ        ⌊ ẏ ⌋            ⌊ 0  -sin(r)   cos(r)*cos(p) ⌋

.. raw:: html

    </details>)""";
        } CalcAngularVelocityInChildFromRpyDt;
        // Symbol: drake::math::RollPitchYaw::CalcAngularVelocityInParentFromRpyDt
        struct /* CalcAngularVelocityInParentFromRpyDt */ {
          // Source: drake/math/roll_pitch_yaw.h
          const char* doc =
R"""(Calculates angular velocity from ``this`` RollPitchYaw whose
roll-pitch-yaw angles ``[r; p; y]`` relate the orientation of two
generic frames A and D.

Parameter ``rpyDt``:
    Time-derivative of ``[r; p; y]``, i.e., ``[ṙ; ṗ; ẏ]``.

Returns:
    w_AD_A, frame D's angular velocity in frame A, expressed in
    "parent" frame A. In other words, returns [ωx; ωy; ωz]ᴀ, where
    ``w_AD_A = ωx Ax + ωy Ay + ωz Az``, and where [ωx; ωy; ωz]ᴀ is
    calculated via the the 3x3 matrix Np⁻¹ (the inverse of the matrix
    Np documented in
    CalcMatrixRelatingRpyDtToAngularVelocityInParent()).


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    ⌈ ωx ⌉         ⌈ ṙ ⌉            ⌈ cos(y)*cos(p)  -sin(y)  0 ⌉
    | ωy |  = Np⁻¹ | ṗ |     Np⁻¹ = | sin(y)*cos(p)   cos(y)  0 |
    ⌊ ωz ⌋ᴀ        ⌊ ẏ ⌋            ⌊   -sin(p)         0     1 ⌋

.. raw:: html

    </details>)""";
        } CalcAngularVelocityInParentFromRpyDt;
        // Symbol: drake::math::RollPitchYaw::CalcMatrixRelatingRpyDtToAngularVelocityInChild
        struct /* CalcMatrixRelatingRpyDtToAngularVelocityInChild */ {
          // Source: drake/math/roll_pitch_yaw.h
          const char* doc =
R"""(For ``this`` RollPitchYaw with roll-pitch-yaw angles [r; p; y] which
relate the orientation of two generic frames A and D, returns the 3x3
matrix Nc relating ṙ, ṗ, ẏ to ω0, ω1, ω2, where frame D's angular
velocity in A, expressed in "child" D is ``w_AD_D = ω0 Dx + ω1 Dy + ω2
Dz``. Hence, Nc contains partial derivatives of [ṙ, ṗ, ẏ] with
respect to [ω0; ω1; ω2]ᴅ.


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    ⌈ ṙ ⌉      ⌈ ω0 ⌉          ⌈ 1  sin(r)*tan(p)  cos(r)*tan(p) ⌉
    | ṗ | = Nc | ω1 |     Nc = | 0      cos(r)        −sin(r)    |
    ⌊ ẏ ⌋      ⌊ ω2 ⌋ᴅ         ⌊ 0  sin(r)/cos(p)  cos(r)/cos(p) ⌋

.. raw:: html

    </details>

Raises:
    RuntimeError if ``cos(p) ≈ 0`` (`p` is near gimbal-lock).

Note:
    This method has a divide-by-zero error (singularity) when the
    cosine of the pitch angle ``p`` is zero [i.e., ``cos(p) = 0`].
    This problem (called "gimbal lock") occurs when `p = n π + π /
    2``, where n is any integer. There are associated precision
    problems (inaccuracies) in the neighborhood of these pitch angles,
    i.e., when ``cos(p) ≈ 0``.

See also:
    CalcMatrixRelatingRpyDtToAngularVelocityInParent())""";
        } CalcMatrixRelatingRpyDtToAngularVelocityInChild;
        // Symbol: drake::math::RollPitchYaw::CalcMatrixRelatingRpyDtToAngularVelocityInParent
        struct /* CalcMatrixRelatingRpyDtToAngularVelocityInParent */ {
          // Source: drake/math/roll_pitch_yaw.h
          const char* doc =
R"""(For ``this`` RollPitchYaw with roll-pitch-yaw angles [r; p; y] which
relate the orientation of two generic frames A and D, returns the 3x3
matrix Np relating ṙ, ṗ, ẏ to ωx, ωy, ωz, where frame D's angular
velocity in A, expressed in "parent" A is ``w_AD_A = ωx Ax + ωy Ay +
ωz Az``. Hence, Np contains partial derivatives of [ṙ, ṗ, ẏ] with
respect to [ωx; ωy; ωz]ᴀ.


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    ⌈ ṙ ⌉      ⌈ ωx ⌉          ⌈ cos(y)/cos(p)  sin(y)/cos(p)   0 ⌉
    | ṗ | = Np | ωy |     Np = |   −sin(y)          cos(y)      0 |
    ⌊ ẏ ⌋      ⌊ ωz ⌋ᴀ         ⌊ cos(y)*tan(p)   sin(y)*tan(p)  1 ⌋

.. raw:: html

    </details>

Raises:
    RuntimeError if ``cos(p) ≈ 0`` (`p` is near gimbal-lock).

Note:
    This method has a divide-by-zero error (singularity) when the
    cosine of the pitch angle ``p`` is zero [i.e., ``cos(p) = 0`].
    This problem (called "gimbal lock") occurs when `p = n π + π /
    2``, where n is any integer. There are associated precision
    problems (inaccuracies) in the neighborhood of these pitch angles,
    i.e., when ``cos(p) ≈ 0``.

See also:
    CalcMatrixRelatingRpyDtToAngularVelocityInChild())""";
        } CalcMatrixRelatingRpyDtToAngularVelocityInParent;
        // Symbol: drake::math::RollPitchYaw::CalcRotationMatrixDt
        struct /* CalcRotationMatrixDt */ {
          // Source: drake/math/roll_pitch_yaw.h
          const char* doc =
R"""(Forms Ṙ, the ordinary derivative of the RotationMatrix ``R`` with
respect to an independent variable ``t`` (`t` usually denotes time)
and ``R`` is the RotationMatrix formed by ``this`` RollPitchYaw. The
roll-pitch-yaw angles r, p, y are regarded as functions of ``t``
[i.e., r(t), p(t), y(t)].

Parameter ``rpyDt``:
    Ordinary derivative of rpy with respect to an independent variable
    ``t`` (`t` usually denotes time, but not necessarily).

Returns:
    Ṙ, the ordinary derivative of ``R`` with respect to ``t``,
    calculated as Ṙ = ∂R/∂r * ṙ + ∂R/∂p * ṗ + ∂R/∂y * ẏ. In other
    words, the returned (i, j) element is ∂Rij/∂r * ṙ + ∂Rij/∂p * ṗ
    + ∂Rij/∂y * ẏ.)""";
        } CalcRotationMatrixDt;
        // Symbol: drake::math::RollPitchYaw::CalcRpyDDtFromAngularAccelInChild
        struct /* CalcRpyDDtFromAngularAccelInChild */ {
          // Source: drake/math/roll_pitch_yaw.h
          const char* doc =
R"""(Uses angular acceleration to compute the 2ⁿᵈ time-derivative of
``this`` RollPitchYaw whose angles ``[r; p; y]`` orient two generic
frames A and D.

Parameter ``rpyDt``:
    time-derivative of ``[r; p; y]``, i.e., ``[ṙ; ṗ; ẏ]``.

Parameter ``alpha_AD_D``:
    frame D's angular acceleration in frame A, expressed in frame D.

Returns:
    ``[r̈, p̈, ÿ]``, the 2ⁿᵈ time-derivative of ``this``
    RollPitchYaw.

Raises:
    RuntimeError if ``cos(p) ≈ 0`` (`p` is near gimbal-lock).

Note:
    This method has a divide-by-zero error (singularity) when the
    cosine of the pitch angle ``p`` is zero [i.e., ``cos(p) = 0`].
    This problem (called "gimbal lock") occurs when `p = n π + π /
    2``, where n is any integer. There are associated precision
    problems (inaccuracies) in the neighborhood of these pitch angles,
    i.e., when ``cos(p) ≈ 0``.)""";
        } CalcRpyDDtFromAngularAccelInChild;
        // Symbol: drake::math::RollPitchYaw::CalcRpyDDtFromRpyDtAndAngularAccelInParent
        struct /* CalcRpyDDtFromRpyDtAndAngularAccelInParent */ {
          // Source: drake/math/roll_pitch_yaw.h
          const char* doc =
R"""(Uses angular acceleration to compute the 2ⁿᵈ time-derivative of
``this`` RollPitchYaw whose angles ``[r; p; y]`` orient two generic
frames A and D.

Parameter ``rpyDt``:
    time-derivative of ``[r; p; y]``, i.e., ``[ṙ; ṗ; ẏ]``.

Parameter ``alpha_AD_A``:
    frame D's angular acceleration in frame A, expressed in frame A.

Returns:
    ``[r̈, p̈, ÿ]``, the 2ⁿᵈ time-derivative of ``this``
    RollPitchYaw.

Raises:
    RuntimeError if ``cos(p) ≈ 0`` (`p` is near gimbal-lock).

Note:
    This method has a divide-by-zero error (singularity) when the
    cosine of the pitch angle ``p`` is zero [i.e., ``cos(p) = 0`].
    This problem (called "gimbal lock") occurs when `p = n π + π /
    2``, where n is any integer. There are associated precision
    problems (inaccuracies) in the neighborhood of these pitch angles,
    i.e., when ``cos(p) ≈ 0``.)""";
        } CalcRpyDDtFromRpyDtAndAngularAccelInParent;
        // Symbol: drake::math::RollPitchYaw::CalcRpyDtFromAngularVelocityInChild
        struct /* CalcRpyDtFromAngularVelocityInChild */ {
          // Source: drake/math/roll_pitch_yaw.h
          const char* doc =
R"""(Uses angular velocity to compute the 1ˢᵗ time-derivative of ``this``
RollPitchYaw whose angles ``[r; p; y]`` orient two generic frames A
and D.

Parameter ``w_AD_D``:
    frame D's angular velocity in frame A, expressed in D.

Returns:
    ``[ṙ; ṗ; ẏ]``, the 1ˢᵗ time-derivative of ``this``
    RollPitchYaw.

Raises:
    RuntimeError if ``cos(p) ≈ 0`` (`p` is near gimbal-lock).

Note:
    Enhanced documentation for this method and its gimbal-lock
    (divide- by-zero error) is in
    CalcMatrixRelatingRpyDtToAngularVelocityInChild().

See also:
    CalcRpyDtFromAngularVelocityInParent())""";
        } CalcRpyDtFromAngularVelocityInChild;
        // Symbol: drake::math::RollPitchYaw::CalcRpyDtFromAngularVelocityInParent
        struct /* CalcRpyDtFromAngularVelocityInParent */ {
          // Source: drake/math/roll_pitch_yaw.h
          const char* doc =
R"""(Uses angular velocity to compute the 1ˢᵗ time-derivative of ``this``
RollPitchYaw whose angles ``[r; p; y]`` orient two generic frames A
and D.

Parameter ``w_AD_A``:
    frame D's angular velocity in frame A, expressed in A.

Returns:
    ``[ṙ; ṗ; ẏ]``, the 1ˢᵗ time-derivative of ``this``
    RollPitchYaw.

Raises:
    RuntimeError if ``cos(p) ≈ 0`` (`p` is near gimbal-lock).

Note:
    Enhanced documentation for this method and its gimbal-lock
    (divide- by-zero error) is in
    CalcMatrixRelatingRpyDtToAngularVelocityInParent().

See also:
    CalcRpyDtFromAngularVelocityInChild())""";
        } CalcRpyDtFromAngularVelocityInParent;
        // Symbol: drake::math::RollPitchYaw::DoesCosPitchAngleViolateGimbalLockTolerance
        struct /* DoesCosPitchAngleViolateGimbalLockTolerance */ {
          // Source: drake/math/roll_pitch_yaw.h
          const char* doc =
R"""(Returns true if the pitch-angle ``p`` is close to gimbal-lock, which
means ``cos(p) ≈ 0`` or ``p ≈ (n*π + π/2)`` where ``n = 0, ±1, ±2,
...``. More specifically, returns true if ``abs(cos_pitch_angle)`` is
less than an internally-defined tolerance of gimbal-lock.

Parameter ``cos_pitch_angle``:
    cosine of the pitch angle, i.e., ``cos(p)``.

Note:
    Pitch-angles close to gimbal-lock can cause problems with
    numerical precision and numerical integration.)""";
        } DoesCosPitchAngleViolateGimbalLockTolerance;
        // Symbol: drake::math::RollPitchYaw::DoesPitchAngleViolateGimbalLockTolerance
        struct /* DoesPitchAngleViolateGimbalLockTolerance */ {
          // Source: drake/math/roll_pitch_yaw.h
          const char* doc =
R"""(Returns true if the pitch-angle ``p`` is within an internally-defined
tolerance of gimbal-lock. In other words, this method returns true if
``p ≈ (n*π + π/2)`` where ``n = 0, ±1, ±2, ...``.

Note:
    To improve efficiency when cos(pitch_angle()) is already
    calculated, instead use the function
    DoesCosPitchAngleViolateGimbalLockTolerance().

See also:
    DoesCosPitchAngleViolateGimbalLockTolerance())""";
        } DoesPitchAngleViolateGimbalLockTolerance;
        // Symbol: drake::math::RollPitchYaw::GimbalLockPitchAngleTolerance
        struct /* GimbalLockPitchAngleTolerance */ {
          // Source: drake/math/roll_pitch_yaw.h
          const char* doc =
R"""(Returns the internally-defined allowable closeness (in radians) of the
pitch angle ``p`` to gimbal-lock, i.e., the allowable proximity of
``p`` to ``(n*π + π/2)`` where ``n = 0, ±1, ±2, ...``.)""";
        } GimbalLockPitchAngleTolerance;
        // Symbol: drake::math::RollPitchYaw::IsNearlyEqualTo
        struct /* IsNearlyEqualTo */ {
          // Source: drake/math/roll_pitch_yaw.h
          const char* doc =
R"""(Compares each element of ``this`` to the corresponding element of
``other`` to check if they are the same to within a specified
``tolerance``.

Parameter ``other``:
    RollPitchYaw to compare to ``this``.

Parameter ``tolerance``:
    maximum allowable absolute difference between the matrix elements
    in ``this`` and ``other``.

Returns:
    ``True`` if ``‖this - other‖∞ <= tolerance``.)""";
        } IsNearlyEqualTo;
        // Symbol: drake::math::RollPitchYaw::IsNearlySameOrientation
        struct /* IsNearlySameOrientation */ {
          // Source: drake/math/roll_pitch_yaw.h
          const char* doc =
R"""(Compares each element of the RotationMatrix R1 produced by ``this`` to
the corresponding element of the RotationMatrix R2 produced by
``other`` to check if they are the same to within a specified
``tolerance``.

Parameter ``other``:
    RollPitchYaw to compare to ``this``.

Parameter ``tolerance``:
    maximum allowable absolute difference between R1, R2.

Returns:
    ``True`` if ``‖R1 - R2‖∞ <= tolerance``.)""";
        } IsNearlySameOrientation;
        // Symbol: drake::math::RollPitchYaw::IsRollPitchYawInCanonicalRange
        struct /* IsRollPitchYawInCanonicalRange */ {
          // Source: drake/math/roll_pitch_yaw.h
          const char* doc =
R"""(Returns true if roll-pitch-yaw angles ``[r, p, y]`` are in the range
``-π <= r <= π``, `-π/2 <= p <= π/2`, ``-π <= y <= π``.)""";
        } IsRollPitchYawInCanonicalRange;
        // Symbol: drake::math::RollPitchYaw::IsValid
        struct /* IsValid */ {
          // Source: drake/math/roll_pitch_yaw.h
          const char* doc =
R"""(Returns true if ``rpy`` contains valid roll, pitch, yaw angles.

Parameter ``rpy``:
    allegedly valid roll, pitch, yaw angles.

Note:
    an angle is invalid if it is NaN or infinite.)""";
        } IsValid;
        // Symbol: drake::math::RollPitchYaw::RollPitchYaw<T>
        struct /* ctor */ {
          // Source: drake/math/roll_pitch_yaw.h
          const char* doc_1args_rpy =
R"""(Constructs a RollPitchYaw from a 3x1 array of angles.

Parameter ``rpy``:
    3x1 array with roll, pitch, yaw angles (units of radians).

Raises:
    RuntimeError in debug builds if !IsValid(rpy).)""";
          // Source: drake/math/roll_pitch_yaw.h
          const char* doc_3args_roll_pitch_yaw =
R"""(Constructs a RollPitchYaw from roll, pitch, yaw angles (radian units).

Parameter ``roll``:
    x-directed angle in SpaceXYZ rotation sequence.

Parameter ``pitch``:
    y-directed angle in SpaceXYZ rotation sequence.

Parameter ``yaw``:
    z-directed angle in SpaceXYZ rotation sequence.

Raises:
    RuntimeError in debug builds if !IsValid(Vector3<T>(roll, pitch,
    yaw)).)""";
          // Source: drake/math/roll_pitch_yaw.h
          const char* doc_1args_R =
R"""(Uses a RotationMatrix to construct a RollPitchYaw with roll-pitch-yaw
angles ``[r, p, y]`` in the range ``-π <= r <= π``, `-π/2 <= p <=
π/2`, ``-π <= y <= π``.

Parameter ``R``:
    a RotationMatrix.

Note:
    This new high-accuracy algorithm avoids numerical round-off issues
    encountered by some algorithms when pitch is within 1E-6 of π/2 or
    -π/2.)""";
          // Source: drake/math/roll_pitch_yaw.h
          const char* doc_1args_quaternion =
R"""(Uses a Quaternion to construct a RollPitchYaw with roll-pitch-yaw
angles ``[r, p, y]`` in the range ``-π <= r <= π``, `-π/2 <= p <=
π/2`, ``-π <= y <= π``.

Parameter ``quaternion``:
    unit Quaternion.

Note:
    This new high-accuracy algorithm avoids numerical round-off issues
    encountered by some algorithms when pitch is within 1E-6 of π/2 or
    -π/2.

Raises:
    RuntimeError in debug builds if !IsValid(rpy).)""";
        } ctor;
        // Symbol: drake::math::RollPitchYaw::SetFromQuaternion
        struct /* SetFromQuaternion */ {
          // Source: drake/math/roll_pitch_yaw.h
          const char* doc =
R"""(Uses a Quaternion to set ``this`` RollPitchYaw with roll-pitch-yaw
angles ``[r, p, y]`` in the range ``-π <= r <= π``, `-π/2 <= p <=
π/2`, ``-π <= y <= π``.

Parameter ``quaternion``:
    unit Quaternion.

Note:
    This new high-accuracy algorithm avoids numerical round-off issues
    encountered by some algorithms when pitch is within 1E-6 of π/2 or
    -π/2.

Raises:
    RuntimeError in debug builds if !IsValid(rpy).)""";
        } SetFromQuaternion;
        // Symbol: drake::math::RollPitchYaw::SetFromRotationMatrix
        struct /* SetFromRotationMatrix */ {
          // Source: drake/math/roll_pitch_yaw.h
          const char* doc =
R"""(Uses a high-accuracy efficient algorithm to set the roll-pitch-yaw
angles ``[r, p, y]`` that underlie ``this`` @RollPitchYaw, even when
the pitch angle p is very near a singularity (when p is within 1E-6 of
π/2 or -π/2). After calling this method, the underlying roll-pitch-yaw
``[r, p, y]`` has range ``-π <= r <= π``, `-π/2 <= p <= π/2`, ``-π <=
y <= π``.

Parameter ``R``:
    a RotationMatrix.

Note:
    This high-accuracy algorithm was invented at TRI in October 2016
    and avoids numerical round-off issues encountered by some
    algorithms when pitch is within 1E-6 of π/2 or -π/2.)""";
        } SetFromRotationMatrix;
        // Symbol: drake::math::RollPitchYaw::ToMatrix3ViaRotationMatrix
        struct /* ToMatrix3ViaRotationMatrix */ {
          // Source: drake/math/roll_pitch_yaw.h
          const char* doc =
R"""(Returns the 3x3 matrix representation of the RotationMatrix that
corresponds to ``this`` RollPitchYaw. This is a convenient "sugar"
method that is exactly equivalent to RotationMatrix(rpy).ToMatrix3().)""";
        } ToMatrix3ViaRotationMatrix;
        // Symbol: drake::math::RollPitchYaw::ToQuaternion
        struct /* ToQuaternion */ {
          // Source: drake/math/roll_pitch_yaw.h
          const char* doc =
R"""(Returns a quaternion representation of ``this`` RollPitchYaw.)""";
        } ToQuaternion;
        // Symbol: drake::math::RollPitchYaw::ToRotationMatrix
        struct /* ToRotationMatrix */ {
          // Source: drake/math/roll_pitch_yaw.h
          const char* doc =
R"""(Returns the RotationMatrix representation of ``this`` RollPitchYaw.)""";
        } ToRotationMatrix;
        // Symbol: drake::math::RollPitchYaw::pitch_angle
        struct /* pitch_angle */ {
          // Source: drake/math/roll_pitch_yaw.h
          const char* doc =
R"""(Returns the pitch-angle underlying ``this`` RollPitchYaw.)""";
        } pitch_angle;
        // Symbol: drake::math::RollPitchYaw::roll_angle
        struct /* roll_angle */ {
          // Source: drake/math/roll_pitch_yaw.h
          const char* doc =
R"""(Returns the roll-angle underlying ``this`` RollPitchYaw.)""";
        } roll_angle;
        // Symbol: drake::math::RollPitchYaw::set
        struct /* set */ {
          // Source: drake/math/roll_pitch_yaw.h
          const char* doc_1args =
R"""(Sets ``this`` RollPitchYaw from a 3x1 array of angles.

Parameter ``rpy``:
    3x1 array with roll, pitch, yaw angles (units of radians).

Raises:
    RuntimeError in debug builds if !IsValid(rpy).)""";
          // Source: drake/math/roll_pitch_yaw.h
          const char* doc_3args =
R"""(Sets ``this`` RollPitchYaw from roll, pitch, yaw angles (units of
radians).

Parameter ``roll``:
    x-directed angle in SpaceXYZ rotation sequence.

Parameter ``pitch``:
    y-directed angle in SpaceXYZ rotation sequence.

Parameter ``yaw``:
    z-directed angle in SpaceXYZ rotation sequence.

Raises:
    RuntimeError in debug builds if !IsValid(Vector3<T>(roll, pitch,
    yaw)).)""";
        } set;
        // Symbol: drake::math::RollPitchYaw::set_pitch_angle
        struct /* set_pitch_angle */ {
          // Source: drake/math/roll_pitch_yaw.h
          const char* doc =
R"""(Set the pitch-angle underlying ``this`` RollPitchYaw.

Parameter ``p``:
    pitch angle (in units of radians).)""";
        } set_pitch_angle;
        // Symbol: drake::math::RollPitchYaw::set_roll_angle
        struct /* set_roll_angle */ {
          // Source: drake/math/roll_pitch_yaw.h
          const char* doc =
R"""(Set the roll-angle underlying ``this`` RollPitchYaw.

Parameter ``r``:
    roll angle (in units of radians).)""";
        } set_roll_angle;
        // Symbol: drake::math::RollPitchYaw::set_yaw_angle
        struct /* set_yaw_angle */ {
          // Source: drake/math/roll_pitch_yaw.h
          const char* doc =
R"""(Set the yaw-angle underlying ``this`` RollPitchYaw.

Parameter ``y``:
    yaw angle (in units of radians).)""";
        } set_yaw_angle;
        // Symbol: drake::math::RollPitchYaw::vector
        struct /* vector */ {
          // Source: drake/math/roll_pitch_yaw.h
          const char* doc =
R"""(Returns the Vector3 underlying ``this`` RollPitchYaw.)""";
        } vector;
        // Symbol: drake::math::RollPitchYaw::yaw_angle
        struct /* yaw_angle */ {
          // Source: drake/math/roll_pitch_yaw.h
          const char* doc =
R"""(Returns the yaw-angle underlying ``this`` RollPitchYaw.)""";
        } yaw_angle;
      } RollPitchYaw;
      // Symbol: drake::math::RollPitchYawd
      struct /* RollPitchYawd */ {
        // Source: drake/math/roll_pitch_yaw.h
        const char* doc =
R"""(Abbreviation (alias/typedef) for a RollPitchYaw double scalar type.)""";
      } RollPitchYawd;
      // Symbol: drake::math::RotationMatrix
      struct /* RotationMatrix */ {
        // Source: drake/math/rotation_matrix.h
        const char* doc =
R"""(This class represents a 3x3 rotation matrix between two arbitrary
frames A and B and helps ensure users create valid rotation matrices.
This class relates right-handed orthogonal unit vectors Ax, Ay, Az
fixed in frame A to right-handed orthogonal unit vectors Bx, By, Bz
fixed in frame B. The monogram notation for the rotation matrix
relating A to B is ``R_AB``. An example that gives context to this
rotation matrix is ``v_A = R_AB * v_B``, where ``v_B`` denotes an
arbitrary vector v expressed in terms of Bx, By, Bz and ``v_A``
denotes vector v expressed in terms of Ax, Ay, Az. See
multibody_quantities for monogram notation for dynamics. See
orientation_discussion "a discussion on rotation matrices".

Note:
    This class does not store the frames associated with a rotation
    matrix nor does it enforce strict proper usage of this class with
    vectors.

Note:
    When assertions are enabled, several methods in this class perform
    a validity check and throw RuntimeError if the rotation matrix is
    invalid. When assertions are disabled, many of these validity
    checks are skipped (which helps improve speed). These validity
    tests are only performed for scalar types for which
    drake∷scalar_predicate<T>∷is_bool is ``True``. For instance,
    validity checks are not performed when T is symbolic∷Expression.

Authors:
    Paul Mitiguy (2018) Original author.

Authors:
    Drake team (see https://drake.mit.edu/credits).)""";
        // Symbol: drake::math::RotationMatrix::ApplyAxialRotation
        struct /* ApplyAxialRotation */ {
          // Source: drake/math/rotation_matrix.h
          const char* doc =
R"""((Internal use only) Given an axial rotation about a coordinate axis
(x, y, or z), uses it to efficiently re-express a vector. This takes
only 6 floating point operations.

Parameter ``aR_BC``:
    An axial rotation about the indicated axis.

Parameter ``v_C``:
    A vector expressed in frame C to be re-expressed in frame B.

Returns ``v_B``:
    The input vector re-expressed in frame B.

Template parameter ``axis``:
    0, 1, or 2 corresponding to +x, +y, or +z rotation axis.

Precondition:
    aR_BC is an axial_rotation_def "axial rotation matrix" about the
    given ``axis``.)""";
        } ApplyAxialRotation;
        // Symbol: drake::math::RotationMatrix::GetMaximumAbsoluteDifference
        struct /* GetMaximumAbsoluteDifference */ {
          // Source: drake/math/rotation_matrix.h
          const char* doc =
R"""(Computes the infinity norm of ``this`` - `other` (i.e., the maximum
absolute value of the difference between the elements of ``this`` and
``other``).

Parameter ``other``:
    RotationMatrix to subtract from ``this``.

Returns:
    ``‖this - other‖∞``)""";
        } GetMaximumAbsoluteDifference;
        // Symbol: drake::math::RotationMatrix::GetMeasureOfOrthonormality
        struct /* GetMeasureOfOrthonormality */ {
          // Source: drake/math/rotation_matrix.h
          const char* doc =
R"""(Returns how close the matrix R is to being a 3x3 orthonormal matrix by
computing ``‖R ⋅ Rᵀ - I‖∞`` (i.e., the maximum absolute value of the
difference between the elements of R ⋅ Rᵀ and the 3x3 identity
matrix).

Parameter ``R``:
    matrix being checked for orthonormality.

Returns:
    ``‖R ⋅ Rᵀ - I‖∞``)""";
        } GetMeasureOfOrthonormality;
        // Symbol: drake::math::RotationMatrix::Identity
        struct /* Identity */ {
          // Source: drake/math/rotation_matrix.h
          const char* doc = R"""()""";
        } Identity;
        // Symbol: drake::math::RotationMatrix::InvertAndCompose
        struct /* InvertAndCompose */ {
          // Source: drake/math/rotation_matrix.h
          const char* doc =
R"""(Calculates the product of ``this`` inverted and another
RotationMatrix. If you consider ``this`` to be the rotation matrix
R_AB, and ``other`` to be R_AC, then this method returns R_BC = R_AB⁻¹
* R_AC. For T==double, this method can be *much* faster than inverting
first and then performing the composition because it can take
advantage of the orthogonality of rotation matrices. On some platforms
it can use SIMD instructions for further speedups.

Parameter ``other``:
    RotationMatrix that post-multiplies ``this`` inverted.

Returns ``R_BC``:
    where R_BC = this⁻¹ * other.

Note:
    It is possible (albeit improbable) to create an invalid rotation
    matrix by accumulating round-off error with a large number of
    multiplies.)""";
        } InvertAndCompose;
        // Symbol: drake::math::RotationMatrix::IsAxialRotationOrThrow
        struct /* IsAxialRotationOrThrow */ {
          // Source: drake/math/rotation_matrix.h
          const char* doc =
R"""((Internal use only) Throws an exception if ``this`` RotationMatrix is
not an axial rotation about the indicated axis. See axial_rotation_def
"axial rotation matrix" for the conditions that must be satisfied for
a rotation matrix to be "axial". In addition, for numerical types T we
check here that the active elements are reasonable: there should be
two equal cos(θ) entries, ±sin(θ) entries, and sin²+cos²==1.
(Numerical tests are done to a fairly loose tolerance of 16ε to avoid
false negatives.)

Template parameter ``axis``:
    0, 1, or 2 corresponding to +x, +y, or +z rotation axis.

Note:
    This is intended for Debug-mode checks that the other methods in
    this section are being used properly.)""";
        } IsAxialRotationOrThrow;
        // Symbol: drake::math::RotationMatrix::IsExactlyEqualTo
        struct /* IsExactlyEqualTo */ {
          // Source: drake/math/rotation_matrix.h
          const char* doc =
R"""(Compares each element of ``this`` to the corresponding element of
``other`` to check if they are exactly the same.

Parameter ``other``:
    RotationMatrix to compare to ``this``.

Returns:
    true if each element of ``this`` is exactly equal to the
    corresponding element in ``other``.

See also:
    IsNearlyEqualTo().)""";
        } IsExactlyEqualTo;
        // Symbol: drake::math::RotationMatrix::IsExactlyIdentity
        struct /* IsExactlyIdentity */ {
          // Source: drake/math/rotation_matrix.h
          const char* doc =
R"""(Returns ``True`` if ``this`` is exactly equal to the identity matrix.

See also:
    IsNearlyIdentity().)""";
        } IsExactlyIdentity;
        // Symbol: drake::math::RotationMatrix::IsNearlyEqualTo
        struct /* IsNearlyEqualTo */ {
          // Source: drake/math/rotation_matrix.h
          const char* doc =
R"""(Compares each element of ``this`` to the corresponding element of
``other`` to check if they are the same to within a specified
``tolerance``.

Parameter ``other``:
    RotationMatrix to compare to ``this``.

Parameter ``tolerance``:
    maximum allowable absolute difference between the matrix elements
    in ``this`` and ``other``.

Returns:
    ``True`` if ``‖this - other‖∞ <= tolerance``.

See also:
    IsExactlyEqualTo().)""";
        } IsNearlyEqualTo;
        // Symbol: drake::math::RotationMatrix::IsNearlyIdentity
        struct /* IsNearlyIdentity */ {
          // Source: drake/math/rotation_matrix.h
          const char* doc =
R"""(Returns true if ``this`` is within tolerance of the identity
RigidTransform.

Parameter ``tolerance``:
    non-negative number that is generally the default value, namely
    RotationMatrix∷get_internal_tolerance_for_orthonormality().

See also:
    IsExactlyIdentity().)""";
        } IsNearlyIdentity;
        // Symbol: drake::math::RotationMatrix::IsOrthonormal
        struct /* IsOrthonormal */ {
          // Source: drake/math/rotation_matrix.h
          const char* doc =
R"""(Tests if a generic Matrix3 has orthonormal vectors to within the
threshold specified by ``tolerance``.

Parameter ``R``:
    an allegedly orthonormal rotation matrix.

Parameter ``tolerance``:
    maximum allowable absolute difference between R * Rᵀ and the
    identity matrix I, i.e., checks if ``‖R ⋅ Rᵀ - I‖∞ <= tolerance``.

Returns:
    ``True`` if R is an orthonormal matrix.)""";
        } IsOrthonormal;
        // Symbol: drake::math::RotationMatrix::IsValid
        struct /* IsValid */ {
          // Source: drake/math/rotation_matrix.h
          const char* doc_2args =
R"""(Tests if a generic Matrix3 seems to be a proper orthonormal rotation
matrix to within the threshold specified by ``tolerance``.

Parameter ``R``:
    an allegedly valid rotation matrix.

Parameter ``tolerance``:
    maximum allowable absolute difference of ``R * Rᵀ`` and the
    identity matrix I (i.e., checks if ``‖R ⋅ Rᵀ - I‖∞ <=
    tolerance``).

Returns:
    ``True`` if R is a valid rotation matrix.)""";
          // Source: drake/math/rotation_matrix.h
          const char* doc_1args =
R"""(Tests if a generic Matrix3 is a proper orthonormal rotation matrix to
within the threshold of get_internal_tolerance_for_orthonormality().

Parameter ``R``:
    an allegedly valid rotation matrix.

Returns:
    ``True`` if R is a valid rotation matrix.)""";
          // Source: drake/math/rotation_matrix.h
          const char* doc_0args =
R"""(Tests if ``this`` rotation matrix R is a proper orthonormal rotation
matrix to within the threshold of
get_internal_tolerance_for_orthonormality().

Returns:
    ``True`` if ``this`` is a valid rotation matrix.)""";
        } IsValid;
        // Symbol: drake::math::RotationMatrix::MakeAxialRotation
        struct /* MakeAxialRotation */ {
          // Source: drake/math/rotation_matrix.h
          const char* doc =
R"""((Internal use only) Creates an axial rotation aR_AB consisting of a
rotation of ``theta`` radians about x, y, or z. Of the 9 entries in
the rotation matrix, only 4 are active; the rest will be set to 0 or
1. This structure can be exploited for efficient updating and
operating with this rotation matrix.

Parameter ``theta``:
    the rotation angle.

Returns ``aR_BC``:
    the axial rotation (also known as R_BC(theta)).

Template parameter ``axis``:
    0, 1, or 2 corresponding to +x, +y, or +z rotation axis.

See also:
    axial_rotation_def "Axial rotations".)""";
        } MakeAxialRotation;
        // Symbol: drake::math::RotationMatrix::MakeClosestRotationToIdentityFromUnitZ
        struct /* MakeClosestRotationToIdentityFromUnitZ */ {
          // Source: drake/math/rotation_matrix.h
          const char* doc =
R"""(Creates a 3D right-handed orthonormal basis B from a given unit vector
u_A, returned as a rotation matrix R_AB. It consists of orthogonal
unit vectors [Bx, By, Bz] where Bz is u_A. The angle-axis
representation of the resulting rotation is the one with the minimum
rotation angle that rotates A to B. When u_A is not parallel or
antiparallel to [0, 0, 1], such rotation is unique.

Parameter ``u_A``:
    unit vector expressed in frame A that represents Bz.

Raises:
    RuntimeError if u_A is not a unit vector.

Returns ``R_AB``:
    the rotation matrix with properties as described above.)""";
        } MakeClosestRotationToIdentityFromUnitZ;
        // Symbol: drake::math::RotationMatrix::MakeFromOneUnitVector
        struct /* MakeFromOneUnitVector */ {
          // Source: drake/math/rotation_matrix.h
          const char* doc =
R"""((Advanced) Creates a right-handed orthonormal basis B from a given
unit vector u_A, returned as a rotation matrix R_AB.

Parameter ``u_A``:
    unit vector which is expressed in frame A and is either Bx or By
    or Bz (depending on the value of axis_index).

Parameter ``axis_index``:
    The index ∈ {0, 1, 2} of the unit vector associated with u_A, 0
    means u_A is Bx, 1 means u_A is By, and 2 means u_A is Bz.

Precondition:
    axis_index is 0 or 1 or 2.

Raises:
    RuntimeError if u_A is not a unit vector, i.e., |u_A| is not
    within a tolerance of 4ε ≈ 8.88E-16 to 1.0.

Note:
    This method is designed for speed and does not normalize u_A to
    ensure it is a unit vector. Alternatively, consider
    MakeFromOneVector().

Returns ``R_AB``:
    the rotation matrix whose properties are described in
    MakeFromOneVector().)""";
        } MakeFromOneUnitVector;
        // Symbol: drake::math::RotationMatrix::MakeFromOneVector
        struct /* MakeFromOneVector */ {
          // Source: drake/math/rotation_matrix.h
          const char* doc =
R"""(Creates a 3D right-handed orthonormal basis B from a given vector b_A,
returned as a rotation matrix R_AB. It consists of orthogonal unit
vectors u_A, v_A, w_A where u_A is the normalized b_A in the
axis_index column of R_AB and v_A has one element which is zero. If an
element of b_A is zero, then one element of w_A is 1 and the other two
elements are 0.

Parameter ``b_A``:
    vector expressed in frame A that when normalized as u_A =
    b_A.normalized() represents Bx, By, or Bz (depending on
    axis_index).

Parameter ``axis_index``:
    The index ∈ {0, 1, 2} of the unit vector associated with u_A, 0
    means u_A is Bx, 1 means u_A is By, and 2 means u_A is Bz.

Precondition:
    axis_index is 0 or 1 or 2.

Raises:
    RuntimeError if b_A cannot be made into a unit vector because b_A
    contains a NaN or infinity or |b_A| < 1.0E-10.

See also:
    MakeFromOneUnitVector() if b_A is known to already be unit length.

Returns ``R_AB``:
    the rotation matrix with properties as described above.)""";
        } MakeFromOneVector;
        // Symbol: drake::math::RotationMatrix::MakeFromOrthonormalColumns
        struct /* MakeFromOrthonormalColumns */ {
          // Source: drake/math/rotation_matrix.h
          const char* doc =
R"""((Advanced) Makes the RotationMatrix ``R_AB`` from right-handed
orthogonal unit vectors ``Bx``, `By`, ``Bz`` so the columns of
``R_AB`` are ``[Bx, By, Bz]``.

Parameter ``Bx``:
    first unit vector in right-handed orthogonal set.

Parameter ``By``:
    second unit vector in right-handed orthogonal set.

Parameter ``Bz``:
    third unit vector in right-handed orthogonal set.

Raises:
    RuntimeError in debug builds if ``R_AB`` fails IsValid(R_AB).

Note:
    In release builds, the caller can subsequently test if ``R_AB``
    is, in fact, a valid RotationMatrix by calling ``R_AB.IsValid()``.

Note:
    The rotation matrix ``R_AB`` relates two sets of right-handed
    orthogonal unit vectors, namely Ax, Ay, Az and Bx, By, Bz. The
    rows of ``R_AB`` are Ax, Ay, Az expressed in frame B
    (i.e.,``Ax_B``, `Ay_B`, ``Az_B``). The columns of ``R_AB`` are Bx,
    By, Bz expressed in frame A (i.e., ``Bx_A``, `By_A`, ``Bz_A``).)""";
        } MakeFromOrthonormalColumns;
        // Symbol: drake::math::RotationMatrix::MakeFromOrthonormalRows
        struct /* MakeFromOrthonormalRows */ {
          // Source: drake/math/rotation_matrix.h
          const char* doc =
R"""((Advanced) Makes the RotationMatrix ``R_AB`` from right-handed
orthogonal unit vectors ``Ax``, `Ay`, ``Az`` so the rows of ``R_AB``
are ``[Ax, Ay, Az]``.

Parameter ``Ax``:
    first unit vector in right-handed orthogonal set.

Parameter ``Ay``:
    second unit vector in right-handed orthogonal set.

Parameter ``Az``:
    third unit vector in right-handed orthogonal set.

Raises:
    RuntimeError in debug builds if ``R_AB`` fails IsValid(R_AB).

Note:
    In release builds, the caller can subsequently test if ``R_AB``
    is, in fact, a valid RotationMatrix by calling ``R_AB.IsValid()``.

Note:
    The rotation matrix ``R_AB`` relates two sets of right-handed
    orthogonal unit vectors, namely Ax, Ay, Az and Bx, By, Bz. The
    rows of ``R_AB`` are Ax, Ay, Az expressed in frame B
    (i.e.,``Ax_B``, `Ay_B`, ``Az_B``). The columns of ``R_AB`` are Bx,
    By, Bz expressed in frame A (i.e., ``Bx_A``, `By_A`, ``Bz_A``).)""";
        } MakeFromOrthonormalRows;
        // Symbol: drake::math::RotationMatrix::MakeUnchecked
        struct /* MakeUnchecked */ {
          // Source: drake/math/rotation_matrix.h
          const char* doc =
R"""((Advanced) Makes a RotationMatrix from a Matrix3. No check is
performed to test whether or not the parameter R is a valid rotation
matrix.)""";
        } MakeUnchecked;
        // Symbol: drake::math::RotationMatrix::MakeXRotation
        struct /* MakeXRotation */ {
          // Source: drake/math/rotation_matrix.h
          const char* doc =
R"""(Makes the RotationMatrix ``R_AB`` associated with rotating a frame B
relative to a frame A by an angle ``theta`` about unit vector ``Ax =
Bx``.

Parameter ``theta``:
    radian measure of rotation angle about Ax.

Note:
    Orientation is same as Eigen∷AngleAxis<T>(theta, Vector3d∷UnitX().

Note:
    ``R_AB`` relates two frames A and B having unit vectors Ax, Ay, Az
    and Bx, By, Bz. Initially, ``Bx = Ax``, `By = Ay`, ``Bz = Az``,
    then B undergoes a right-handed rotation relative to A by an angle
    ``theta`` about ``Ax = Bx``.


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    ⎡ 1       0                 0  ⎤
    R_AB = ⎢ 0   cos(theta)   -sin(theta) ⎥
           ⎣ 0   sin(theta)    cos(theta) ⎦

.. raw:: html

    </details>)""";
        } MakeXRotation;
        // Symbol: drake::math::RotationMatrix::MakeYRotation
        struct /* MakeYRotation */ {
          // Source: drake/math/rotation_matrix.h
          const char* doc =
R"""(Makes the RotationMatrix ``R_AB`` associated with rotating a frame B
relative to a frame A by an angle ``theta`` about unit vector ``Ay =
By``.

Parameter ``theta``:
    radian measure of rotation angle about Ay.

Note:
    Orientation is same as Eigen∷AngleAxis<T>(theta, Vector3d∷UnitY().

Note:
    ``R_AB`` relates two frames A and B having unit vectors Ax, Ay, Az
    and Bx, By, Bz. Initially, ``Bx = Ax``, `By = Ay`, ``Bz = Az``,
    then B undergoes a right-handed rotation relative to A by an angle
    ``theta`` about ``Ay = By``.


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    ⎡  cos(theta)   0   sin(theta) ⎤
    R_AB = ⎢          0    1           0  ⎥
           ⎣ -sin(theta)   0   cos(theta) ⎦

.. raw:: html

    </details>)""";
        } MakeYRotation;
        // Symbol: drake::math::RotationMatrix::MakeZRotation
        struct /* MakeZRotation */ {
          // Source: drake/math/rotation_matrix.h
          const char* doc =
R"""(Makes the RotationMatrix ``R_AB`` associated with rotating a frame B
relative to a frame A by an angle ``theta`` about unit vector ``Az =
Bz``.

Parameter ``theta``:
    radian measure of rotation angle about Az.

Note:
    Orientation is same as Eigen∷AngleAxis<T>(theta, Vector3d∷UnitZ().

Note:
    ``R_AB`` relates two frames A and B having unit vectors Ax, Ay, Az
    and Bx, By, Bz. Initially, ``Bx = Ax``, `By = Ay`, ``Bz = Az``,
    then B undergoes a right-handed rotation relative to A by an angle
    ``theta`` about ``Az = Bz``.


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    ⎡ cos(theta)  -sin(theta)   0 ⎤
    R_AB = ⎢ sin(theta)   cos(theta)   0 ⎥
           ⎣         0            0    1 ⎦

.. raw:: html

    </details>)""";
        } MakeZRotation;
        // Symbol: drake::math::RotationMatrix::PostMultiplyByAxialRotation
        struct /* PostMultiplyByAxialRotation */ {
          // Source: drake/math/rotation_matrix.h
          const char* doc =
R"""((Internal use only) With ``this`` a general rotation R_AB, and given
an axial rotation aR_BC, efficiently forms R_AC = R_AB * aR_BC. This
requires only 18 floating point operations.

Parameter ``aR_BC``:
    An axial rotation about the indicated axis.

Parameter ``R_AC``:
    The result, which will be a general rotation matrix. Must not
    overlap with ``this`` in memory.

Template parameter ``axis``:
    0, 1, or 2 corresponding to +x, +y, or +z rotation axis.

Precondition:
    aR_BC is an axial_rotation_def "axial rotation matrix" about the
    given ``axis``.

Precondition:
    R_AC does not overlap with ``this`` in memory.)""";
        } PostMultiplyByAxialRotation;
        // Symbol: drake::math::RotationMatrix::PreMultiplyByAxialRotation
        struct /* PreMultiplyByAxialRotation */ {
          // Source: drake/math/rotation_matrix.h
          const char* doc =
R"""((Internal use only) With ``this`` a general rotation R_BC, and given
an axial rotation aR_AB, efficiently forms R_AC = aR_AB * R_BC. This
requires only 18 floating point operations.

Parameter ``aR_AB``:
    An axial rotation about the indicated axis.

Parameter ``R_AC``:
    The result, which will be a general rotation matrix. Must not
    overlap with ``this`` in memory.

Template parameter ``axis``:
    0, 1, or 2 corresponding to +x, +y, or +z rotation axis.

Precondition:
    aR_AB is an axial_rotation_def "axial rotation matrix" about the
    given ``axis``.

Precondition:
    R_AC does not overlap with ``this`` in memory.)""";
        } PreMultiplyByAxialRotation;
        // Symbol: drake::math::RotationMatrix::ProjectToRotationMatrix
        struct /* ProjectToRotationMatrix */ {
          // Source: drake/math/rotation_matrix.h
          const char* doc =
R"""(Given an approximate rotation matrix M, finds the RotationMatrix R
closest to M. Closeness is measured with a matrix-2 norm (or
equivalently with a Frobenius norm). Hence, this method creates a
RotationMatrix R from a 3x3 matrix M by minimizing ``‖R - M‖₂`` (the
matrix-2 norm of (R-M)) subject to ``R * Rᵀ = I``, where I is the 3x3
identity matrix. For this problem, closeness can also be measured by
forming the orthonormal matrix R whose elements minimize the
double-summation ``∑ᵢ ∑ⱼ (R(i,j) - M(i,j))²`` where ``i = 1:3, j =
1:3``, subject to ``R * Rᵀ = I``. The square-root of this
double-summation is called the Frobenius norm.

Parameter ``M``:
    a 3x3 matrix.

Parameter ``quality_factor``:
    The quality of M as a rotation matrix. ``quality_factor`` = 1 is
    perfect (M = R). ``quality_factor`` = 1.25 means that when M
    multiplies a unit vector (magnitude 1), a vector of magnitude as
    large as 1.25 may result. ``quality_factor`` = 0.8 means that when
    M multiplies a unit vector, a vector of magnitude as small as 0.8
    may result. ``quality_factor`` = 0 means M is singular, so at
    least one of the bases related by matrix M does not span 3D space
    (when M multiples a unit vector, a vector of magnitude as small as
    0 may result).

Returns:
    proper orthonormal matrix R that is closest to M.

Raises:
    RuntimeError if R fails IsValid(R).

Note:
    William Kahan (UC Berkeley) and Hongkai Dai (Toyota Research
    Institute) proved that for this problem, the same R that minimizes
    the Frobenius norm also minimizes the matrix-2 norm (a.k.a an
    induced-2 norm), which is defined [Dahleh, Section 4.2] as the
    column matrix u which maximizes ``‖(R - M) u‖ / ‖u‖``, where ``u ≠
    0``. Since the matrix-2 norm of any matrix A is equal to the
    maximum singular value of A, minimizing the matrix-2 norm of (R -
    M) is equivalent to minimizing the maximum singular value of (R -
    M).

- [Dahleh] "Lectures on Dynamic Systems and Controls: Electrical
Engineering and Computer Science, Massachusetts Institute of Technology"
https://ocw.mit.edu/courses/electrical-engineering-and-computer-science/6-241j-dynamic-systems-and-control-spring-2011/readings/MIT6_241JS11_chap04.pdf)""";
        } ProjectToRotationMatrix;
        // Symbol: drake::math::RotationMatrix::RotationMatrix<type-parameter-0-0>
        struct /* ctor */ {
          // Source: drake/math/rotation_matrix.h
          const char* doc_0args =
R"""(Constructs a 3x3 identity RotationMatrix -- which corresponds to
aligning two frames (so that unit vectors Ax = Bx, Ay = By, Az = Bz).)""";
          // Source: drake/math/rotation_matrix.h
          const char* doc_1args_R =
R"""(Constructs a RotationMatrix from a Matrix3.

Parameter ``R``:
    an allegedly valid rotation matrix.

Raises:
    RuntimeError in debug builds if R fails IsValid(R).)""";
          // Source: drake/math/rotation_matrix.h
          const char* doc_1args_quaternion =
R"""(Constructs a RotationMatrix from an Eigen∷Quaternion.

Parameter ``quaternion``:
    a non-zero, finite quaternion which may or may not have unit
    length [i.e., ``quaternion.norm()`` does not have to be 1].

Raises:
    RuntimeError in debug builds if the rotation matrix R that is
    built from ``quaternion`` fails IsValid(R). For example, an
    exception is thrown if ``quaternion`` is zero or contains a NaN or
    infinity.

Note:
    This method has the effect of normalizing its ``quaternion``
    argument, without the inefficiency of the square-root associated
    with normalization.)""";
          // Source: drake/math/rotation_matrix.h
          const char* doc_1args_theta_lambda =
R"""(Constructs a RotationMatrix from an Eigen∷AngleAxis.

Parameter ``theta_lambda``:
    an Eigen∷AngleAxis whose associated axis (vector direction herein
    called ``lambda``) is non-zero and finite, but which may or may
    not have unit length [i.e., ``lambda.norm()`` does not have to be
    1].

Raises:
    RuntimeError in debug builds if the rotation matrix R that is
    built from ``theta_lambda`` fails IsValid(R). For example, an
    exception is thrown if ``lambda`` is zero or contains a NaN or
    infinity.)""";
          // Source: drake/math/rotation_matrix.h
          const char* doc_1args_rpy =
R"""(Constructs a RotationMatrix from an RollPitchYaw. In other words,
makes the RotationMatrix for a Space-fixed (extrinsic) X-Y-Z rotation
by "roll-pitch-yaw" angles ``[r, p, y]``, which is equivalent to a
Body-fixed (intrinsic) Z-Y-X rotation by "yaw-pitch-roll" angles ``[y,
p, r]``.

Parameter ``rpy``:
    a RollPitchYaw which is a Space-fixed (extrinsic) X-Y-Z rotation
    with "roll-pitch-yaw" angles ``[r, p, y]`` or equivalently a Body-
    fixed (intrinsic) Z-Y-X rotation with "yaw-pitch-roll" angles
    ``[y, p, r]``.

Returns ``R_AD``:
    , rotation matrix relating frame A to frame D.

Note:
    Denoting roll ``r``, pitch ``p``, yaw ``y``, this method returns a
    rotation matrix ``R_AD`` equal to the matrix multiplication shown
    below.


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    ⎡cos(y) -sin(y)  0⎤   ⎡ cos(p)  0  sin(p)⎤   ⎡1      0        0 ⎤
    R_AD = ⎢sin(y)  cos(y)  0⎥ * ⎢     0   1      0 ⎥ * ⎢0  cos(r)  -sin(r)⎥
           ⎣    0       0   1⎦   ⎣-sin(p)  0  cos(p)⎦   ⎣0  sin(r)   cos(r)⎦
         =       R_AB          *        R_BC          *        R_CD

.. raw:: html

    </details>

Note:
    In this discussion, A is the Space frame and D is the Body frame.
    One way to visualize this rotation sequence is by introducing
    intermediate frames B and C (useful constructs to understand this
    rotation sequence). Initially, the frames are aligned so ``Di = Ci
    = Bi = Ai (i = x, y, z)``. Then D is subjected to successive
    right-handed rotations relative to A.

* 1st rotation R_CD: Frame D rotates relative to frames C, B, A by a
roll angle ``r`` about ``Dx = Cx``.  Note: D and C are no longer aligned.

* 2nd rotation R_BC: Frames D, C (collectively -- as if welded together)
rotate relative to frame B, A by a pitch angle ``p`` about ``Cy = By``.
Note: C and B are no longer aligned.

* 3rd rotation R_AB: Frames D, C, B (collectively -- as if welded)
rotate relative to frame A by a roll angle ``y`` about ``Bz = Az``.
Note: B and A are no longer aligned.

Note:
    This method constructs a RotationMatrix from a RollPitchYaw.
    Vice-versa, there are high-accuracy RollPitchYaw
    constructor/methods that form a RollPitchYaw from a rotation
    matrix.)""";
          // Source: drake/math/rotation_matrix.h
          const char* doc_1args_internalDoNotInitializeMemberFields =
R"""((Internal use only) Constructs a RotationMatrix without initializing
the underlying 3x3 matrix. For use by RigidTransform and
RotationMatrix only.)""";
        } ctor;
        // Symbol: drake::math::RotationMatrix::ToAngleAxis
        struct /* ToAngleAxis */ {
          // Source: drake/math/rotation_matrix.h
          const char* doc =
R"""(Returns an AngleAxis ``theta_lambda`` containing an angle ``theta``
and unit vector (axis direction) ``lambda`` that represents ``this``
RotationMatrix.

Note:
    The orientation and RotationMatrix associated with ``theta *
    lambda`` is identical to that of ``(-theta) * (-lambda)``. The
    AngleAxis returned by this method chooses to have ``0 <= theta <=
    pi``.

Returns:
    an AngleAxis with ``0 <= theta <= pi`` and a unit vector
    ``lambda``.)""";
        } ToAngleAxis;
        // Symbol: drake::math::RotationMatrix::ToQuaternion
        struct /* ToQuaternion */ {
          // Source: drake/math/rotation_matrix.h
          const char* doc_0args =
R"""(Returns a quaternion q that represents ``this`` RotationMatrix. Since
the quaternion ``q`` and ``-q`` represent the same RotationMatrix,
this method chooses to return a canonical quaternion, i.e., with q(0)
>= 0.)""";
          // Source: drake/math/rotation_matrix.h
          const char* doc_1args =
R"""(Returns a unit quaternion q associated with the 3x3 matrix M. Since
the quaternion ``q`` and ``-q`` represent the same RotationMatrix,
this method chooses to return a canonical quaternion, i.e., with q(0)
>= 0.

Parameter ``M``:
    3x3 matrix to be made into a quaternion.

Returns:
    a unit quaternion q in canonical form, i.e., with q(0) >= 0.

Raises:
    RuntimeError in debug builds if the quaternion ``q`` returned by
    this method cannot construct a valid RotationMatrix. For example,
    if ``M`` contains NaNs, ``q`` will not be a valid quaternion.)""";
        } ToQuaternion;
        // Symbol: drake::math::RotationMatrix::ToQuaternionAsVector4
        struct /* ToQuaternionAsVector4 */ {
          // Source: drake/math/rotation_matrix.h
          const char* doc_0args =
R"""(Utility method to return the Vector4 associated with ToQuaternion().

See also:
    ToQuaternion().)""";
          // Source: drake/math/rotation_matrix.h
          const char* doc_1args =
R"""(Utility method to return the Vector4 associated with ToQuaternion(M).

Parameter ``M``:
    3x3 matrix to be made into a quaternion.

See also:
    ToQuaternion().)""";
        } ToQuaternionAsVector4;
        // Symbol: drake::math::RotationMatrix::ToRollPitchYaw
        struct /* ToRollPitchYaw */ {
          // Source: drake/math/rotation_matrix.h
          const char* doc =
R"""(Returns a RollPitchYaw that represents ``this`` RotationMatrix, with
roll-pitch-yaw angles ``[r, p, y]`` in the range ``-π <= r <= π``,
`-π/2 <= p <= π/2`, ``-π <= y <= π``.

Note:
    This new high-accuracy algorithm avoids numerical round-off issues
    encountered by some algorithms when pitch is within 1E-6 of π/2 or
    -π/2.)""";
        } ToRollPitchYaw;
        // Symbol: drake::math::RotationMatrix::UpdateAxialRotation
        struct /* UpdateAxialRotation */ {
          // Source: drake/math/rotation_matrix.h
          const char* doc_2args =
R"""((Internal use only) Given a new rotation angle θ, updates the axial
rotation aR_BC to represent the new rotation angle. We expect that
aR_BC was already such an axial rotation about the given x, y, or z
axis. Only the 4 active elements are modified; the other 5 remain
unchanged. (However, execution time is likely to be dominated by the
time to calculate sine and cosine.)

Parameter ``theta``:
    the new rotation angle in radians.

Parameter ``aR_BC``:
    the axial rotation matrix to be updated.

Template parameter ``axis``:
    0, 1, or 2 corresponding to +x, +y, or +z rotation axis.

See also:
    the overloaded signature if you already have sin(θ) & cos(θ).

Precondition:
    aR_BC is an axial_rotation_def "axial rotation matrix" about the
    given ``axis``.)""";
          // Source: drake/math/rotation_matrix.h
          const char* doc_3args =
R"""((Internal use only) Given sin(θ) and cos(θ), where θ is a new rotation
angle, updates the axial rotation aR_BC to represent the new rotation
angle. We expect that aR_BC was already such a rotation (about the
given x, y, or z axis). Only the 4 active elements are modified; the
other 5 remain unchanged.

Parameter ``sin_theta``:
    sin(θ), where θ is the new rotation angle.

Parameter ``cos_theta``:
    cos(θ), where θ is the new rotation angle.

Parameter ``aR_BC``:
    the axial rotation matrix to be updated.

Template parameter ``axis``:
    0, 1, or 2 corresponding to +x, +y, or +z rotation axis.

See also:
    the overloaded signature if you just have the angle θ.

Precondition:
    aR_BC is an axial_rotation_def "axial rotation matrix" about the
    given ``axis``.

Precondition:
    ``sin_theta`` and ``cos_theta`` are sine and cosine of the same
    angle.)""";
        } UpdateAxialRotation;
        // Symbol: drake::math::RotationMatrix::cast
        struct /* cast */ {
          // Source: drake/math/rotation_matrix.h
          const char* doc =
R"""(Creates a RotationMatrix templatized on a scalar type U from a
RotationMatrix templatized on scalar type T. For example,


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    RotationMatrix<double> source = RotationMatrix<double>∷Identity();
    RotationMatrix<AutoDiffXd> foo = source.cast<AutoDiffXd>();

.. raw:: html

    </details>

Template parameter ``U``:
    Scalar type on which the returned RotationMatrix is templated.

Note:
    ``RotationMatrix<From>∷cast<To>()`` creates a new
    ``RotationMatrix<To>`` from a ``RotationMatrix<From>`` but only if
    type ``To`` is constructible from type ``From``. This cast method
    works in accordance with Eigen's cast method for Eigen's Matrix3
    that underlies this RotationMatrix. For example, Eigen currently
    allows cast from type double to AutoDiffXd, but not vice-versa.)""";
        } cast;
        // Symbol: drake::math::RotationMatrix::col
        struct /* col */ {
          // Source: drake/math/rotation_matrix.h
          const char* doc =
R"""(Returns ``this`` rotation matrix's iᵗʰ column (i = 0, 1, 2). For
``this`` rotation matrix R_AB (which relates right-handed sets of
orthogonal unit vectors Ax, Ay, Az to Bx, By, Bz), - col(0) returns
Bx_A (Bx expressed in terms of Ax, Ay, Az). - col(1) returns By_A (By
expressed in terms of Ax, Ay, Az). - col(2) returns Bz_A (Bz expressed
in terms of Ax, Ay, Az).

Parameter ``index``:
    requested column index (0 <= index <= 2).

See also:
    row(), matrix()

Raises:
    In Debug builds, asserts (0 <= index <= 2).

Note:
    For efficiency and consistency with Eigen, this method returns the
    same quantity returned by Eigen's col() operator. The returned
    quantity can be assigned in various ways, e.g., as ``const auto&
    Bz_A = col(2);`` or ``Vector3<T> Bz_A = col(2);``)""";
        } col;
        // Symbol: drake::math::RotationMatrix::get_internal_tolerance_for_orthonormality
        struct /* get_internal_tolerance_for_orthonormality */ {
          // Source: drake/math/rotation_matrix.h
          const char* doc =
R"""(Returns an internal tolerance that checks rotation matrix
orthonormality.

Returns:
    internal tolerance (small multiplier of double-precision epsilon)
    used to check whether or not a rotation matrix is orthonormal.

Note:
    The tolerance is chosen by developers to ensure a reasonably valid
    (orthonormal) rotation matrix.

Note:
    To orthonormalize a 3x3 matrix, use ProjectToRotationMatrix().)""";
        } get_internal_tolerance_for_orthonormality;
        // Symbol: drake::math::RotationMatrix::inverse
        struct /* inverse */ {
          // Source: drake/math/rotation_matrix.h
          const char* doc = R"""()""";
        } inverse;
        // Symbol: drake::math::RotationMatrix::matrix
        struct /* matrix */ {
          // Source: drake/math/rotation_matrix.h
          const char* doc =
R"""(Returns the Matrix3 underlying a RotationMatrix.

See also:
    col(), row())""";
        } matrix;
        // Symbol: drake::math::RotationMatrix::operator*
        struct /* operator_mul */ {
          // Source: drake/math/rotation_matrix.h
          const char* doc_1args_other =
R"""(Calculates ``this`` rotation matrix ``R_AB`` multiplied by ``other``
rotation matrix ``R_BC``, returning the composition ``R_AB * R_BC``.

Parameter ``other``:
    RotationMatrix that post-multiplies ``this``.

Returns:
    rotation matrix that results from ``this`` multiplied by
    ``other``.

Note:
    It is possible (albeit improbable) to create an invalid rotation
    matrix by accumulating round-off error with a large number of
    multiplies.)""";
          // Source: drake/math/rotation_matrix.h
          const char* doc_1args_v_B =
R"""(Calculates ``this`` rotation matrix ``R_AB`` multiplied by an
arbitrary Vector3 expressed in the B frame.

Parameter ``v_B``:
    3x1 vector that post-multiplies ``this``.

Returns:
    3x1 vector ``v_A = R_AB * v_B``.)""";
          // Source: drake/math/rotation_matrix.h
          const char* doc_1args_constEigenMatrixBase =
R"""(Multiplies ``this`` RotationMatrix ``R_AB`` by the n vectors ``v1``,
... `vn`, where each vector has 3 elements and is expressed in frame
B.

Parameter ``v_B``:
    ``3 x n`` matrix whose n columns are regarded as arbitrary vectors
    ``v1``, ... `vn` expressed in frame B.

Returns ``v_A``:
    ``3 x n`` matrix whose n columns are vectors ``v1``, ... `vn`
    expressed in frame A.


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    const RollPitchYaw<double> rpy(0.1, 0.2, 0.3);
    const RotationMatrix<double> R_AB(rpy);
    Eigen∷Matrix<double, 3, 2> v_B;
    v_B.col(0) = Vector3d(4, 5, 6);
    v_B.col(1) = Vector3d(9, 8, 7);
    const Eigen∷Matrix<double, 3, 2> v_A = R_AB * v_B;

.. raw:: html

    </details>)""";
        } operator_mul;
        // Symbol: drake::math::RotationMatrix::operator*=
        struct /* operator_imul */ {
          // Source: drake/math/rotation_matrix.h
          const char* doc =
R"""(In-place multiply of ``this`` rotation matrix ``R_AB`` by ``other``
rotation matrix ``R_BC``. On return, ``this`` is set to equal ``R_AB *
R_BC``.

Parameter ``other``:
    RotationMatrix that post-multiplies ``this``.

Returns:
    ``this`` rotation matrix which has been multiplied by ``other``.

Note:
    It is possible (albeit improbable) to create an invalid rotation
    matrix by accumulating round-off error with a large number of
    multiplies.)""";
        } operator_imul;
        // Symbol: drake::math::RotationMatrix::row
        struct /* row */ {
          // Source: drake/math/rotation_matrix.h
          const char* doc =
R"""(Returns ``this`` rotation matrix's iᵗʰ row (i = 0, 1, 2). For ``this``
rotation matrix R_AB (which relates right-handed sets of orthogonal
unit vectors Ax, Ay, Az to Bx, By, Bz), - row(0) returns Ax_B (Ax
expressed in terms of Bx, By, Bz). - row(1) returns Ay_B (Ay expressed
in terms of Bx, By, Bz). - row(2) returns Az_B (Az expressed in terms
of Bx, By, Bz).

Parameter ``index``:
    requested row index (0 <= index <= 2).

See also:
    col(), matrix()

Raises:
    In Debug builds, asserts (0 <= index <= 2).

Note:
    For efficiency and consistency with Eigen, this method returns the
    same quantity returned by Eigen's row() operator. The returned
    quantity can be assigned in various ways, e.g., as ``const auto&
    Az_B = row(2);`` or ``RowVector3<T> Az_B = row(2);``)""";
        } row;
        // Symbol: drake::math::RotationMatrix::set
        struct /* set */ {
          // Source: drake/math/rotation_matrix.h
          const char* doc =
R"""(Sets ``this`` RotationMatrix from a Matrix3.

Parameter ``R``:
    an allegedly valid rotation matrix.

Raises:
    RuntimeError in debug builds if R fails IsValid(R).)""";
        } set;
        // Symbol: drake::math::RotationMatrix::transpose
        struct /* transpose */ {
          // Source: drake/math/rotation_matrix.h
          const char* doc = R"""()""";
        } transpose;
      } RotationMatrix;
      // Symbol: drake::math::RotationMatrixd
      struct /* RotationMatrixd */ {
        // Source: drake/math/rotation_matrix.h
        const char* doc =
R"""(Abbreviation (alias/typedef) for a RotationMatrix double scalar type.)""";
      } RotationMatrixd;
      // Symbol: drake::math::SoftOverMax
      struct /* SoftOverMax */ {
        // Source: drake/math/soft_min_max.h
        const char* doc =
R"""(Computes a smooth over approximation of max function, namely
SoftOverMax(x) >= max(x). Mathematically we compute this as (log (∑ᵢ
exp(αxᵢ))) / α.

Parameter ``x``:
    The vector for which we want to compute its soft max.

Parameter ``alpha``:
    α in the documentation above. Larger α makes the soft max more
    similar to max, with a sharper corner. Must be strictly positive
    and finite.

*Default:* is 1.
    $Raises:

RuntimeError if α <= 0.

Raises:
    RuntimeError if α is non-finite.)""";
      } SoftOverMax;
      // Symbol: drake::math::SoftOverMin
      struct /* SoftOverMin */ {
        // Source: drake/math/soft_min_max.h
        const char* doc =
R"""(Computes a smooth over approximation of min function, namely
SoftOverMin(x) >= min(x). Mathematically we compute this as ∑ᵢ
exp(-αxᵢ)*xᵢ / d, where d = ∑ⱼ exp(-αxⱼ)

Parameter ``x``:
    The vector for which we want to compute its soft min.

Parameter ``alpha``:
    α in the documentation above. Larger α makes the soft min more
    similar to min, with a sharper corner. Must be strictly positive
    and finite.

*Default:* is 1.
    $Raises:

RuntimeError if α <= 0.

Raises:
    RuntimeError if α is non-finite.)""";
      } SoftOverMin;
      // Symbol: drake::math::SoftUnderMax
      struct /* SoftUnderMax */ {
        // Source: drake/math/soft_min_max.h
        const char* doc =
R"""(Computes a smooth under approximation of max function, namely
SoftUnderMax(x) <= max(x). Mathematically we compute this as ∑ᵢ
exp(αxᵢ)*xᵢ / d, where d = ∑ⱼ exp(αxⱼ)

Parameter ``x``:
    The vector for which we want to compute its soft max.

Parameter ``alpha``:
    α in the documentation above. Larger α makes the soft max more
    similar to max, with a sharper corner. Must be strictly positive
    and finite.

*Default:* is 1.
    $Raises:

RuntimeError if α <= 0.

Raises:
    RuntimeError if α is non-finite.)""";
      } SoftUnderMax;
      // Symbol: drake::math::SoftUnderMin
      struct /* SoftUnderMin */ {
        // Source: drake/math/soft_min_max.h
        const char* doc =
R"""(Computes a smooth under approximation of min function, namely
SoftUnderMin(x) <= min(x). Mathematically we compute this as -(log (∑ᵢ
exp(-αxᵢ))) / α

Parameter ``x``:
    The vector for which we want to compute its soft min.

Parameter ``alpha``:
    α in the documentation above. Larger α makes the soft min more
    similar to min, with a sharper corner. Must be strictly positive
    and finite.

*Default:* is 1.
    $Raises:

RuntimeError if α <= 0.

Raises:
    RuntimeError if α is non-finite.)""";
      } SoftUnderMin;
      // Symbol: drake::math::SolveLinearSystem
      struct /* SolveLinearSystem */ {
        // Source: drake/math/linear_solve.h
        const char* doc_was_unable_to_choose_unambiguous_names =
R"""(Specialized when A and b are both double or symbolic∷Expression
matrices. See linear_solve_given_solver for more details. Note that
``A`` is unused, as we already compute its factorization in
``linear_solver``. But we keep it here for consistency with the
overloaded function, where A is a matrix of AutoDiffScalar.)""";
      } SolveLinearSystem;
      // Symbol: drake::math::SparseMatrixToRowColumnValueVectors
      struct /* SparseMatrixToRowColumnValueVectors */ {
        // Source: drake/math/eigen_sparse_triplet.h
        const char* doc =
R"""(For a sparse matrix, return the row indices, the column indices, and
value of the non-zero entries. For example, the matrix

.. math:: mat = \begin{bmatrix} 1 & 0 & 2\
                      0 & 3 & 4\end{bmatrix}

has

.. math:: row = \begin{bmatrix} 0 & 1 & 0 & 1\end{bmatrix}\
col = \begin{bmatrix} 0 & 1 & 2 & 2\end{bmatrix}\
val = \begin{bmatrix} 1 & 3 & 2 & 4\end{bmatrix}

Parameter ``matrix``:
    the input sparse matrix

Parameter ``row_indices``:
    a vector containing the row indices of the non-zero entries

Parameter ``col_indices``:
    a vector containing the column indices of the non-zero entries

Parameter ``val``:
    a vector containing the values of the non-zero entries.)""";
      } SparseMatrixToRowColumnValueVectors;
      // Symbol: drake::math::SparseMatrixToTriplets
      struct /* SparseMatrixToTriplets */ {
        // Source: drake/math/eigen_sparse_triplet.h
        const char* doc =
R"""(For a sparse matrix, return a vector of triplets, such that we can
reconstruct the matrix using setFromTriplet function

Parameter ``matrix``:
    A sparse matrix

Returns:
    A triplet with the row, column and value of the non-zero entries.
    See https://eigen.tuxfamily.org/dox/group__TutorialSparse.html for
    more information on the triplet)""";
      } SparseMatrixToTriplets;
      // Symbol: drake::math::StdVectorToEigen
      struct /* StdVectorToEigen */ {
        // Source: drake/math/matrix_util.h
        const char* doc =
R"""(Converts a std∷vector<MatrixX<T>> into a MatrixX<T>, composing each
element of ``vec`` into a column of the returned matrix.

Precondition:
    all elements of ``vec`` must have one column and the same number
    of rows.)""";
      } StdVectorToEigen;
      // Symbol: drake::math::ToLowerTriangularColumnsFromMatrix
      struct /* ToLowerTriangularColumnsFromMatrix */ {
        // Source: drake/math/matrix_util.h
        const char* doc =
R"""(Given a square matrix, extract the lower triangular part as a stacked
column vector. This is a particularly useful operation when
vectorizing symmetric matrices.)""";
      } ToLowerTriangularColumnsFromMatrix;
      // Symbol: drake::math::ToSymmetricMatrixFromLowerTriangularColumns
      struct /* ToSymmetricMatrixFromLowerTriangularColumns */ {
        // Source: drake/math/matrix_util.h
        const char* doc_dynamic_size =
R"""(Given a column vector containing the stacked columns of the lower
triangular part of a square matrix, returning a symmetric matrix whose
lower triangular part is the same as the original matrix.)""";
        // Source: drake/math/matrix_util.h
        const char* doc =
R"""(Given a column vector containing the stacked columns of the lower
triangular part of a square matrix, returning a symmetric matrix whose
lower triangular part is the same as the original matrix.

Template parameter ``rows``:
    The number of rows in the symmetric matrix.)""";
      } ToSymmetricMatrixFromLowerTriangularColumns;
      // Symbol: drake::math::UniformPtsOnSphereFibonacci
      struct /* UniformPtsOnSphereFibonacci */ {
        // Source: drake/math/evenly_distributed_pts_on_sphere.h
        const char* doc =
R"""(Deterministically generates approximate evenly distributed points on a
unit sphere. This method uses Fibonacci number. For the detailed math,
please refer to
http://stackoverflow.com/questions/9600801/evenly-distributing-n-points-on-a-sphere
This algorithm generates the points in O(n) time, where ``n`` is the
number of points.

Parameter ``num_points``:
    The number of points we want on the unit sphere.

Returns:
    The generated points.

Precondition:
    num_samples >= 1. Throw RuntimeError if num_points < 1)""";
      } UniformPtsOnSphereFibonacci;
      // Symbol: drake::math::UniformlyRandomAngleAxis
      struct /* UniformlyRandomAngleAxis */ {
        // Source: drake/math/random_rotation.h
        const char* doc =
R"""(Generates a rotation (in the axis-angle representation) that rotates a
point on the unit sphere to another point on the unit sphere with a
uniform distribution over the sphere.)""";
      } UniformlyRandomAngleAxis;
      // Symbol: drake::math::UniformlyRandomQuaternion
      struct /* UniformlyRandomQuaternion */ {
        // Source: drake/math/random_rotation.h
        const char* doc =
R"""(Generates a rotation (in the quaternion representation) that rotates a
point on the unit sphere to another point on the unit sphere with a
uniform distribution over the sphere. This method is briefly explained
in http://planning.cs.uiuc.edu/node198.html, a full explanation can be
found in K. Shoemake. Uniform Random Rotations in D. Kirk, editor,
Graphics Gems III, pages 124-132. Academic, New York, 1992.)""";
      } UniformlyRandomQuaternion;
      // Symbol: drake::math::UniformlyRandomRPY
      struct /* UniformlyRandomRPY */ {
        // Source: drake/math/random_rotation.h
        const char* doc =
R"""(Generates a rotation (in the roll-pitch-yaw representation) that
rotates a point on the unit sphere to another point on the unit sphere
with a uniform distribution over the sphere.)""";
      } UniformlyRandomRPY;
      // Symbol: drake::math::UniformlyRandomRotationMatrix
      struct /* UniformlyRandomRotationMatrix */ {
        // Source: drake/math/random_rotation.h
        const char* doc =
R"""(Generates a rotation (in the rotation matrix representation) that
rotates a point on the unit sphere to another point on the unit sphere
with a uniform distribution over the sphere.)""";
      } UniformlyRandomRotationMatrix;
      // Symbol: drake::math::VectorToSkewSymmetric
      struct /* VectorToSkewSymmetric */ {
        // Source: drake/math/cross_product.h
        const char* doc = R"""()""";
      } VectorToSkewSymmetric;
      // Symbol: drake::math::dquat2rotmat
      struct /* dquat2rotmat */ {
        // Source: drake/math/rotation_conversion_gradient.h
        const char* doc =
R"""(Computes the gradient of the function that converts a unit length
quaternion to a rotation matrix.

Parameter ``quaternion``:
    A unit length quaternion [w;x;y;z]

Returns:
    The gradient)""";
      } dquat2rotmat;
      // Symbol: drake::math::drotmat2quat
      struct /* drotmat2quat */ {
        // Source: drake/math/rotation_conversion_gradient.h
        const char* doc =
R"""(Computes the gradient of the function that converts rotation matrix to
quaternion.

Parameter ``R``:
    A 3 x 3 rotation matrix

Parameter ``dR``:
    A 9 x N matrix, dR(i,j) is the gradient of R(i) w.r.t x_var(j)

Returns:
    The gradient G. G is a 4 x N matrix G(0,j) is the gradient of w
    w.r.t x_var(j) G(1,j) is the gradient of x w.r.t x_var(j) G(2,j)
    is the gradient of y w.r.t x_var(j) G(3,j) is the gradient of z
    w.r.t x_var(j))""";
      } drotmat2quat;
      // Symbol: drake::math::drotmat2rpy
      struct /* drotmat2rpy */ {
        // Source: drake/math/rotation_conversion_gradient.h
        const char* doc =
R"""(Computes the gradient of the function that converts a rotation matrix
to body-fixed z-y'-x'' Euler angles.

Parameter ``R``:
    A 3 x 3 rotation matrix

Parameter ``dR``:
    A 9 x N matrix, dR(i,j) is the gradient of R(i) w.r.t x(j)

Returns:
    The gradient G. G is a 3 x N matrix. G(0,j) is the gradient of
    roll w.r.t x(j) G(1,j) is the gradient of pitch w.r.t x(j) G(2,j)
    is the gradient of yaw w.r.t x(j))""";
      } drotmat2rpy;
      // Symbol: drake::math::getSubMatrixGradient
      struct /* getSubMatrixGradient */ {
        // Source: drake/math/gradient_util.h
        const char* doc = R"""()""";
      } getSubMatrixGradient;
      // Symbol: drake::math::hessian
      struct /* hessian */ {
        // Source: drake/math/jacobian.h
        const char* doc_deprecated =
R"""(Computes a matrix of AutoDiffScalars from which the value, Jacobian,
and Hessian of a function

.. math:: f:\mathbb{R}^{n\times m}\rightarrow\mathbb{R}^{p\times q}

(f: R^n*m -> R^p*q) can be extracted.

The output is a matrix of nested AutoDiffScalars, being the result of
calling ∷jacobian on a function that returns the output of ∷jacobian,
called on ``f``.

``MaxChunkSizeOuter`` and ``MaxChunkSizeInner`` can be used to control
chunk sizes (see ∷jacobian).

See ∷jacobian for requirements on the function ``f`` and the argument
``x``.

Parameter ``f``:
    function

Parameter ``x``:
    function argument value at which Hessian will be evaluated

Returns:
    AutoDiffScalar matrix corresponding to the Hessian of f evaluated
    at x / (Deprecated.)

Deprecated:
    Removed with no replacement; copy the code into your project if
    you still need it This will be removed from Drake on or after
    2026-04-01.)""";
      } hessian;
      // Symbol: drake::math::intRange
      struct /* intRange */ {
        // Source: drake/math/gradient_util.h
        const char* doc = R"""()""";
      } intRange;
      // Symbol: drake::math::is_quaternion_in_canonical_form
      struct /* is_quaternion_in_canonical_form */ {
        // Source: drake/math/quaternion.h
        const char* doc =
R"""(This function tests whether a quaternion is in "canonical form"
meaning that it tests whether the quaternion [w, x, y, z] has a
non-negative w value. Example: [-0.3, +0.4, +0.5, +0.707] is not in
canonical form. Example: [+0.3, -0.4, -0.5, -0.707] is in canonical
form.

Parameter ``quat``:
    Quaternion [w, x, y, z] that relates two right-handed orthogonal
    unitary bases e.g., Ax, Ay, Az (A) to Bx, By, Bz (B). Note: quat
    is analogous to the rotation matrix R_AB.

Returns:
    ``True`` if quat.w() is nonnegative (in canonical form), else
    ``False``.)""";
      } is_quaternion_in_canonical_form;
      // Symbol: drake::math::jacobian
      struct /* jacobian */ {
        // Source: drake/math/jacobian.h
        const char* doc_deprecated =
R"""(Computes a matrix of AutoDiffScalars from which both the value and the
Jacobian of a function

.. math:: f:\mathbb{R}^{n\times m}\rightarrow\mathbb{R}^{p\times q}

(f: R^n*m -> R^p*q) can be extracted.

The derivative vector for each AutoDiffScalar in the output contains
the derivatives with respect to all components of the argument
:math:`x`.

The return type of this function is a matrix with the 'best' possible
AutoDiffScalar scalar type, in the following sense: - If the number of
derivatives can be determined at compile time, the AutoDiffScalar
derivative vector will have that fixed size. - If the maximum number
of derivatives can be determined at compile time, the AutoDiffScalar
derivative vector will have that maximum fixed size. - If neither the
number, nor the maximum number of derivatives can be determined at
compile time, the output AutoDiffScalar derivative vector will be
dynamically sized.

``f`` should have a templated call operator that maps an Eigen matrix
argument to another Eigen matrix. The scalar type of the output of
:math:`f` need not match the scalar type of the input (useful in
recursive calls to the function to determine higher order
derivatives). The easiest way to create an ``f`` is using a C++14
generic lambda.

The algorithm computes the Jacobian in chunks of up to
``MaxChunkSize`` derivatives at a time. This has three purposes: - It
makes it so that derivative vectors can be allocated on the stack,
eliminating dynamic allocations and improving performance if the
maximum number of derivatives cannot be determined at compile time. -
It gives control over, and limits the number of required
instantiations of the call operator of f and all the functions it
calls. - Excessively large derivative vectors can result in CPU
capacity cache misses; even if the number of derivatives is fixed at
compile time, it may be better to break up into chunks if that means
that capacity cache misses can be prevented.

Parameter ``f``:
    function

Parameter ``x``:
    function argument value at which Jacobian will be evaluated

Returns:
    AutoDiffScalar matrix corresponding to the Jacobian of f evaluated
    at x. / (Deprecated.)

Deprecated:
    Removed with no replacement; copy the code into your project if
    you still need it This will be removed from Drake on or after
    2026-04-01.)""";
      } jacobian;
      // Symbol: drake::math::matGradMult
      struct /* matGradMult */ {
        // Source: drake/math/gradient_util.h
        const char* doc = R"""()""";
      } matGradMult;
      // Symbol: drake::math::matGradMultMat
      struct /* matGradMultMat */ {
        // Source: drake/math/gradient_util.h
        const char* doc = R"""()""";
      } matGradMultMat;
      // Symbol: drake::math::quatConjugate
      struct /* quatConjugate */ {
        // Source: drake/math/quaternion.h
        const char* doc = R"""()""";
      } quatConjugate;
      // Symbol: drake::math::quatDiff
      struct /* quatDiff */ {
        // Source: drake/math/quaternion.h
        const char* doc = R"""()""";
      } quatDiff;
      // Symbol: drake::math::quatDiffAxisInvar
      struct /* quatDiffAxisInvar */ {
        // Source: drake/math/quaternion.h
        const char* doc = R"""()""";
      } quatDiffAxisInvar;
      // Symbol: drake::math::quatProduct
      struct /* quatProduct */ {
        // Source: drake/math/quaternion.h
        const char* doc = R"""()""";
      } quatProduct;
      // Symbol: drake::math::quatRotateVec
      struct /* quatRotateVec */ {
        // Source: drake/math/quaternion.h
        const char* doc = R"""()""";
      } quatRotateVec;
      // Symbol: drake::math::setSubMatrixGradient
      struct /* setSubMatrixGradient */ {
        // Source: drake/math/gradient_util.h
        const char* doc = R"""()""";
      } setSubMatrixGradient;
      // Symbol: drake::math::to_string
      struct /* to_string */ {
        // Source: drake/math/roll_pitch_yaw.h
        const char* doc =
R"""(Represents a RollPitchYaw object as a string. Especially useful for
debugging.)""";
      } to_string;
      // Symbol: drake::math::transposeGrad
      struct /* transposeGrad */ {
        // Source: drake/math/gradient_util.h
        const char* doc = R"""()""";
      } transposeGrad;
      // Symbol: drake::math::wrap_to
      struct /* wrap_to */ {
        // Source: drake/math/wrap_to.h
        const char* doc =
R"""(For variables that are meant to be periodic, (e.g. over a 2π
interval), wraps ``value`` into the interval ``[low, high)``.
Precisely, ``wrap_to`` returns: value + k*(high-low) for the unique
integer value ``k`` that lands the output in the desired interval.
``low`` and ``high`` must be finite, and low < high.)""";
      } wrap_to;
    } math;
  } drake;
} pydrake_doc_math;

#if defined(__GNUG__)
#pragma GCC diagnostic pop
#endif
