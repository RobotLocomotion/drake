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

// #include "drake/multibody/optimization/centroidal_momentum_constraint.h"
// #include "drake/multibody/optimization/contact_wrench.h"
// #include "drake/multibody/optimization/contact_wrench_evaluator.h"
// #include "drake/multibody/optimization/manipulator_equation_constraint.h"
// #include "drake/multibody/optimization/quaternion_integration_constraint.h"
// #include "drake/multibody/optimization/sliding_friction_complementarity_constraint.h"
// #include "drake/multibody/optimization/spatial_velocity_constraint.h"
// #include "drake/multibody/optimization/static_equilibrium_constraint.h"
// #include "drake/multibody/optimization/static_equilibrium_problem.h"
// #include "drake/multibody/optimization/static_friction_cone_complementarity_constraint.h"
// #include "drake/multibody/optimization/static_friction_cone_constraint.h"
// #include "drake/multibody/optimization/toppra.h"

// Symbol: pydrake_doc_multibody_optimization
constexpr struct /* pydrake_doc_multibody_optimization */ {
  // Symbol: drake
  struct /* drake */ {
    // Symbol: drake::multibody
    struct /* multibody */ {
      // Symbol: drake::multibody::AddSlidingFrictionComplementarityExplicitContactConstraint
      struct /* AddSlidingFrictionComplementarityExplicitContactConstraint */ {
        // Source: drake/multibody/optimization/sliding_friction_complementarity_constraint.h
        const char* doc =
R"""(For a pair of geometries in explicit contact, adds the sliding
friction complementarity constraint explained in
sliding_friction_complementarity_constraint to an optimization
program. This function adds the slack variables (f_static, f_sliding,
c), and impose all the constraints in
sliding_friction_complementarity_constraint.

Parameter ``contact_wrench_evaluator``:
    Evaluates the contact wrench between a pair of geometries.

Parameter ``complementarity_tolerance``:
    The tolerance on the complementarity constraint.

Parameter ``q_vars``:
    The variable for the generalized position q in ``prog``.

Parameter ``v_vars``:
    The variable for the generalized velocity v in ``prog``.

Parameter ``lambda_vars``:
    The variables to parameterize the contact wrench between this pair
    of geometry.

Parameter ``prog``:
    The optimization program to which the sliding friction
    complementarity constraint is imposed.

Returns:
    (sliding_friction_complementarity_constraint,
    static_friction_cone_constraint), the pair of constraint that
    imposes (1)-(4) and (6) in
    sliding_friction_complementarity_constraint.)""";
      } AddSlidingFrictionComplementarityExplicitContactConstraint;
      // Symbol: drake::multibody::AddSlidingFrictionComplementarityImplicitContactConstraint
      struct /* AddSlidingFrictionComplementarityImplicitContactConstraint */ {
        // Source: drake/multibody/optimization/sliding_friction_complementarity_constraint.h
        const char* doc =
R"""(For a pair of geometries in implicit contact (they may or may not be
in contact, adds the sliding friction complementarity constraint
explained in sliding_friction_complementarity_constraint. The input
arguments are the same as those in
AddSlidingFrictionComplementarityExplicitContactConstraint(). The
difference is that the returned argument includes the nonlinear
complementarity binding 0 ≤ φ(q) ⊥ fₙ≥ 0, which imposes the constraint
for implicit contact.)""";
      } AddSlidingFrictionComplementarityImplicitContactConstraint;
      // Symbol: drake::multibody::AddStaticFrictionConeComplementarityConstraint
      struct /* AddStaticFrictionConeComplementarityConstraint */ {
        // Source: drake/multibody/optimization/static_friction_cone_complementarity_constraint.h
        const char* doc =
R"""(Adds the complementarity constraint on the static friction force
between a pair of contacts |ft_W| <= μ * n_Wᵀ * f_W (static friction
force in the friction cone). fn_W * sdf = 0 (complementarity
condition) sdf >= 0 (no penetration) where sdf stands for signed
distance function, ft_W stands for the tangential friction force
expressed in the world frame.

Mathematically, we add the following constraints to the optimization
program

f_Wᵀ * ((μ² + 1)* n_W * n_Wᵀ - I) * f_W ≥ 0 (1) n_Wᵀ * f_W = α (2)
sdf(q) = β (3) 0 ≤ α * β ≤ ε (4) α ≥ 0 (5) β ≥ 0 (6) the slack
variables α and β are added to the optimization program as well.

Parameter ``contact_wrench_evaluator``:
    The evaluator to compute the contact wrench expressed in the world
    frame.

Parameter ``complementarity_tolerance``:
    ε in the documentation above.

Parameter ``q_vars``:
    The decision variable for the generalized configuration q.

Parameter ``lambda_vars``:
    The decision variable to parameterize the contact wrench.

Parameter ``prog``:
    The optimization program to which the constraint is added.

Returns:
    binding The binding containing the nonlinear constraints (1)-(4).

Precondition:
    Both ``q_vars`` and ``lambda_vars`` have been added to ``prog``
    before calling this function.)""";
      } AddStaticFrictionConeComplementarityConstraint;
      // Symbol: drake::multibody::CalcGridPointsOptions
      struct /* CalcGridPointsOptions */ {
        // Source: drake/multibody/optimization/toppra.h
        const char* doc = R"""()""";
        // Symbol: drake::multibody::CalcGridPointsOptions::Serialize
        struct /* Serialize */ {
          // Source: drake/multibody/optimization/toppra.h
          const char* doc =
R"""(Passes this object to an Archive. Refer to yaml_serialization "YAML
Serialization" for background.)""";
        } Serialize;
        // Symbol: drake::multibody::CalcGridPointsOptions::max_err
        struct /* max_err */ {
          // Source: drake/multibody/optimization/toppra.h
          const char* doc = R"""()""";
        } max_err;
        // Symbol: drake::multibody::CalcGridPointsOptions::max_iter
        struct /* max_iter */ {
          // Source: drake/multibody/optimization/toppra.h
          const char* doc = R"""()""";
        } max_iter;
        // Symbol: drake::multibody::CalcGridPointsOptions::max_seg_length
        struct /* max_seg_length */ {
          // Source: drake/multibody/optimization/toppra.h
          const char* doc = R"""()""";
        } max_seg_length;
        // Symbol: drake::multibody::CalcGridPointsOptions::min_points
        struct /* min_points */ {
          // Source: drake/multibody/optimization/toppra.h
          const char* doc = R"""()""";
        } min_points;
        auto Serialize__fields() const {
          return std::array{
            std::make_pair("max_err", max_err.doc),
            std::make_pair("max_iter", max_iter.doc),
            std::make_pair("max_seg_length", max_seg_length.doc),
            std::make_pair("min_points", min_points.doc),
          };
        }
      } CalcGridPointsOptions;
      // Symbol: drake::multibody::CentroidalMomentumConstraint
      struct /* CentroidalMomentumConstraint */ {
        // Source: drake/multibody/optimization/centroidal_momentum_constraint.h
        const char* doc =
R"""(Impose the constraint CentroidalMomentum(q, v) - h_WC = 0 with
decision variables [q;v;h_WC] or CentroidalAngularMomentum(q, v) -
k_WC = 0 with decision variables [q; v; k_WC] h_WC is the 6D spatial
momentum (linear and angular momentum about the center of mass C)
expressed in the world frame (W). k_WC is the 3D vector representing
the angular momentum about the center of mass C expressed in the world
frame W.)""";
        // Symbol: drake::multibody::CentroidalMomentumConstraint::CentroidalMomentumConstraint
        struct /* ctor */ {
          // Source: drake/multibody/optimization/centroidal_momentum_constraint.h
          const char* doc =
R"""(Construct centroidal momentum constraint If ``angular_only`` = false,
we impose the constraint CentroidalMomentum(q, v) - h_WC = 0 where
CentroidalMomentum computes the spatial momentum of the robot about
its center-of-mass, expressed in the world frame. The decision
variables are [q;v;h_WC]. If ``angular_only`` = true, we impose the
constraint CentroidalAngularMomentum(q, v) - k_WC = 0 where
CentroidalAngularMomentum(q, v) computes the angular momentum of the
robot about its center-of-mass, expressed in the world frame. The
decision variables are [q; v; k_WC]

Note:
    Currently, we can only construct this constraint using
    MultibodyPlant<AutoDiffXd> instead of MultibodyPlant<double>,
    since we can't compute the Jacobian of the momentum using
    MultibodyPlant<double> yet.

Parameter ``plant``:
    The plant for which the constraint is imposed.

Parameter ``model_instances``:
    We compute the model with these model instances in ``plant``. If
    model_instances=std∷nullopt, then we compute the momentum of all
    model instances except the world.)""";
        } ctor;
        // Symbol: drake::multibody::CentroidalMomentumConstraint::ComposeVariable
        struct /* ComposeVariable */ {
          // Source: drake/multibody/optimization/centroidal_momentum_constraint.h
          const char* doc = R"""()""";
        } ComposeVariable;
      } CentroidalMomentumConstraint;
      // Symbol: drake::multibody::ContactWrench
      struct /* ContactWrench */ {
        // Source: drake/multibody/optimization/contact_wrench.h
        const char* doc =
R"""(Stores the contact wrench (spatial force) from Body A to Body B
applied at point Cb.)""";
        // Symbol: drake::multibody::ContactWrench::ContactWrench
        struct /* ctor */ {
          // Source: drake/multibody/optimization/contact_wrench.h
          const char* doc =
R"""(Refer to the documentation for each attribute.)""";
        } ctor;
        // Symbol: drake::multibody::ContactWrench::F_Cb_W
        struct /* F_Cb_W */ {
          // Source: drake/multibody/optimization/contact_wrench.h
          const char* doc =
R"""(F_Cb_W_in The wrench (spatial force) applied at point Cb from Body A
to Body B, measured in the world frame.)""";
        } F_Cb_W;
        // Symbol: drake::multibody::ContactWrench::bodyA_index
        struct /* bodyA_index */ {
          // Source: drake/multibody/optimization/contact_wrench.h
          const char* doc = R"""(The index of Body A.)""";
        } bodyA_index;
        // Symbol: drake::multibody::ContactWrench::bodyB_index
        struct /* bodyB_index */ {
          // Source: drake/multibody/optimization/contact_wrench.h
          const char* doc = R"""(The index of Body B.)""";
        } bodyB_index;
        // Symbol: drake::multibody::ContactWrench::p_WCb_W
        struct /* p_WCb_W */ {
          // Source: drake/multibody/optimization/contact_wrench.h
          const char* doc =
R"""(The position of the point Cb (where the wrench is applied) expressed
in the world frame W.)""";
        } p_WCb_W;
      } ContactWrench;
      // Symbol: drake::multibody::ContactWrenchEvaluator
      struct /* ContactWrenchEvaluator */ {
        // Source: drake/multibody/optimization/contact_wrench_evaluator.h
        const char* doc = R"""()""";
        // Symbol: drake::multibody::ContactWrenchEvaluator::ComposeVariableValues
        struct /* ComposeVariableValues */ {
          // Source: drake/multibody/optimization/contact_wrench_evaluator.h
          const char* doc_2args_constsystemsContext_constDerived = R"""(Overloads ComposeVariableValues)""";
          // Source: drake/multibody/optimization/contact_wrench_evaluator.h
          const char* doc_2args_constEigenMatrixBase_constEigenMatrixBase =
R"""(Overloads ComposeVariableValues with q, λ as the input instead of
context, λ.)""";
        } ComposeVariableValues;
        // Symbol: drake::multibody::ContactWrenchEvaluator::ContactWrenchEvaluator
        struct /* ctor */ {
          // Source: drake/multibody/optimization/contact_wrench_evaluator.h
          const char* doc =
R"""(Each derived class should call this constructor.

Parameter ``plant``:
    The MultibodyPlant on which the contact wrench is computed. The
    lifetime of plant should outlive this object.

Parameter ``context``:
    The context of ``plant``. The lifetime of context should outlive
    this object.

Parameter ``num_lambda``:
    The size of lambda.

Parameter ``geometry_id_pair``:
    The pair of geometries for which the contact wrench is computed.
    Notice that the order of the geometries in the pair should match
    with that in SceneGraphInspector∷GetCollisionCandidates().)""";
        } ctor;
        // Symbol: drake::multibody::ContactWrenchEvaluator::context
        struct /* context */ {
          // Source: drake/multibody/optimization/contact_wrench_evaluator.h
          const char* doc = R"""(Getter for const context)""";
        } context;
        // Symbol: drake::multibody::ContactWrenchEvaluator::geometry_id_pair
        struct /* geometry_id_pair */ {
          // Source: drake/multibody/optimization/contact_wrench_evaluator.h
          const char* doc = R"""(Returns the pair of geometry IDs.)""";
        } geometry_id_pair;
        // Symbol: drake::multibody::ContactWrenchEvaluator::get_mutable_context
        struct /* get_mutable_context */ {
          // Source: drake/multibody/optimization/contact_wrench_evaluator.h
          const char* doc = R"""(Getter for the mutable context)""";
        } get_mutable_context;
        // Symbol: drake::multibody::ContactWrenchEvaluator::lambda
        struct /* lambda */ {
          // Source: drake/multibody/optimization/contact_wrench_evaluator.h
          const char* doc =
R"""(Extract lambda from x (x is used in Eval(x, &y)).)""";
        } lambda;
        // Symbol: drake::multibody::ContactWrenchEvaluator::num_lambda
        struct /* num_lambda */ {
          // Source: drake/multibody/optimization/contact_wrench_evaluator.h
          const char* doc = R"""(Returns the size of lambda.)""";
        } num_lambda;
        // Symbol: drake::multibody::ContactWrenchEvaluator::plant
        struct /* plant */ {
          // Source: drake/multibody/optimization/contact_wrench_evaluator.h
          const char* doc = R"""()""";
        } plant;
        // Symbol: drake::multibody::ContactWrenchEvaluator::q
        struct /* q */ {
          // Source: drake/multibody/optimization/contact_wrench_evaluator.h
          const char* doc =
R"""(Extract the generalized configuration q from x (x is used in Eval(x,
&y)).)""";
        } q;
      } ContactWrenchEvaluator;
      // Symbol: drake::multibody::ContactWrenchFromForceInWorldFrameEvaluator
      struct /* ContactWrenchFromForceInWorldFrameEvaluator */ {
        // Source: drake/multibody/optimization/contact_wrench_evaluator.h
        const char* doc =
R"""(The contact wrench is τ_AB_W = 0, f_AB_W = λ Namely we assume that λ
is the contact force from A to B, applied directly at B's witness
point.)""";
        // Symbol: drake::multibody::ContactWrenchFromForceInWorldFrameEvaluator::ContactWrenchFromForceInWorldFrameEvaluator
        struct /* ctor */ {
          // Source: drake/multibody/optimization/contact_wrench_evaluator.h
          const char* doc =
R"""(Parameter ``plant``:
    The MultibodyPlant on which the contact wrench is computed. The
    lifetime of ``plant`` should outlive this object.

Parameter ``context``:
    The context of the MultibodyPlant. The lifetime of ``context``
    should outlive this object.

Parameter ``geometry_id_pair``:
    The pair of geometries for which the contact wrench is computed.
    Notice that the order of the geometries in the pair should match
    with that in SceneGraphInspector∷GetCollisionCandidates().)""";
        } ctor;
      } ContactWrenchFromForceInWorldFrameEvaluator;
      // Symbol: drake::multibody::GeometryPairContactWrenchEvaluatorBinding
      struct /* GeometryPairContactWrenchEvaluatorBinding */ {
        // Source: drake/multibody/optimization/contact_wrench_evaluator.h
        const char* doc = R"""()""";
        // Symbol: drake::multibody::GeometryPairContactWrenchEvaluatorBinding::GeometryPairContactWrenchEvaluatorBinding
        struct /* ctor */ {
          // Source: drake/multibody/optimization/contact_wrench_evaluator.h
          const char* doc = R"""()""";
        } ctor;
        // Symbol: drake::multibody::GeometryPairContactWrenchEvaluatorBinding::contact_wrench_evaluator
        struct /* contact_wrench_evaluator */ {
          // Source: drake/multibody/optimization/contact_wrench_evaluator.h
          const char* doc = R"""()""";
        } contact_wrench_evaluator;
        // Symbol: drake::multibody::GeometryPairContactWrenchEvaluatorBinding::lambda_indices_in_all_lambda
        struct /* lambda_indices_in_all_lambda */ {
          // Source: drake/multibody/optimization/contact_wrench_evaluator.h
          const char* doc = R"""()""";
        } lambda_indices_in_all_lambda;
      } GeometryPairContactWrenchEvaluatorBinding;
      // Symbol: drake::multibody::ManipulatorEquationConstraint
      struct /* ManipulatorEquationConstraint */ {
        // Source: drake/multibody/optimization/manipulator_equation_constraint.h
        const char* doc =
R"""(A Constraint to impose the manipulator equation: 0 = (Buₙ₊₁ + ∑ᵢ
(Jᵢ_WBᵀ(qₙ₊₁)ᵀ * Fᵢ_AB_W(λᵢ,ₙ₊₁)) + tau_g(qₙ₊₁) - C(qₙ₊₁, Vₙ₊₁)) * dt
- M(qₙ₊₁) * (Vₙ₊₁ - Vₙ))""";
        // Symbol: drake::multibody::ManipulatorEquationConstraint::MakeBinding
        struct /* MakeBinding */ {
          // Source: drake/multibody/optimization/manipulator_equation_constraint.h
          const char* doc =
R"""(This constraint depends on the decision variable vector: {vₙ, qₙ₊₁,
vₙ₊₁, uₙ₊₁, λₙ₊₁, dt}.

Parameter ``plant``:
    The plant on which the constraint is imposed.

Parameter ``context``:
    The context for the subsystem ``plant``. This context stores the
    next state {qₙ₊₁, vₙ₊₁}.

Parameter ``contact_wrench_evaluators_and_lambda``:
    For each contact pair, we need to compute the contact wrench
    applied at the point of contact, expressed in the world frame,
    namely Fᵢ_AB_W(λᵢ,ₙ₊₁) at time n+1.
    ``contact_wrench_evaluators_and_lambda.first`` is the evaluator
    for computing this contact wrench from the variables λᵢ[.].
    ``contact_wrench_evaluators_and_lambda.second`` are the decision
    variable λᵢ[n+1] used in computing the contact wrench at time step
    n+1. Notice the generalized position ``q`` is not included in
    variables contact_wrench_evaluators_and_lambda.second.

Parameter ``v_vars``:
    The decision variables for vₙ.

Parameter ``q_next_vars``:
    The decision variables for qₙ₊₁.

Parameter ``v_next_vars``:
    The decision variables for vₙ₊₁.

Parameter ``u_next_vars``:
    The decision variables for uₙ₊₁.

Parameter ``dt_var``:
    The decision variable for dt.

Returns:
    binding The binding between the manipulator equation constraint
    and the variables vₙ, qₙ₊₁, vₙ₊₁, uₙ₊₁, λₙ₊₁, and dt.

Precondition:
    ``plant`` must have been connected to a SceneGraph properly. Refer
    to AddMultibodyPlantSceneGraph for documentation on connecting a
    MultibodyPlant to a SceneGraph.)""";
        } MakeBinding;
        // Symbol: drake::multibody::ManipulatorEquationConstraint::ManipulatorEquationConstraint
        struct /* ctor */ {
          // Source: drake/multibody/optimization/manipulator_equation_constraint.h
          const char* doc = R"""()""";
        } ctor;
        // Symbol: drake::multibody::ManipulatorEquationConstraint::contact_pair_to_wrench_evaluator
        struct /* contact_pair_to_wrench_evaluator */ {
          // Source: drake/multibody/optimization/manipulator_equation_constraint.h
          const char* doc =
R"""(Getter for contact_pair_to_wrench_evaluator, passed in the
constructor.)""";
        } contact_pair_to_wrench_evaluator;
      } ManipulatorEquationConstraint;
      // Symbol: drake::multibody::QuaternionEulerIntegrationConstraint
      struct /* QuaternionEulerIntegrationConstraint */ {
        // Source: drake/multibody/optimization/quaternion_integration_constraint.h
        const char* doc =
R"""(If we have a body with orientation quaternion z₁ at time t₁, and a
quaternion z₂ at time t₂ = t₁ + h, with the angular velocity ω
(expressed in the world frame), we impose the constraint that the body
rotates at a constant velocity ω from quaternion z₁ to quaternion z₂
within time interval h. Namely we want to enforce the relationship
that z₂ and Δz⊗z₁ represent the same orientation, where Δz is the
quaternion [cos(|ω|h/2), ω/|ω|*sin(|ω|h/2)], and ⊗ is the Hamiltonian
product between quaternions.

It is well-known that for any quaternion z, its element-wise negation
-z correspond to the same rotation matrix as z does. One way to
understand this is that -z represents the rotation that first rotate
the frame by a quaternion z, and then continue to rotate about that
axis for 360 degrees. We provide the option
``allow_quaternion_negation`` flag, that if set to true, then we
require that the quaternion z₂ = ±Δz⊗z₁. Otherwise we require z₂ =
Δz⊗z₁. Mathematically, the constraint we impose is

If allow_quaternion_negation = true:

(z₂ • (Δz⊗z₁))² = 1

else

z₂ • (Δz⊗z₁) = 1

If your robot link orientation only changes slightly, and you are free
to search for both z₁ and z₂, then we would recommend to set
allow_quaternion_negation to false, as the left hand side of
constraint z₂ • (Δz⊗z₁) = 1 is less nonlinear than the left hand side
of (z₂ • (Δz⊗z₁))² = 1.

The operation • is the dot product between two quaternions, which
computes the cosine of the half angle between these two orientations.
Dot product equals to ±1 means that angle between the two quaternions
are 2kπ, hence they represent the same orientation.

Note:
    The constraint is not differentiable at ω=0 (due to the
    non-differentiability of |ω| at ω = 0). So it is better to
    initialize the angular velocity to a non-zero value in the
    optimization.

The decision variables of this constraint are [z₁, z₂, ω, h]

Note:
    We need to evaluate sin(|ω|h/2)/|ω|, when h is huge (larger than
    1/machine_epsilon), and |ω| is tiny (less than machine epsilon),
    this evaluation is inaccurate. So don't use this constraint if you
    have a huge h (which would be bad practice in trajectory
    optimization anyway).)""";
        // Symbol: drake::multibody::QuaternionEulerIntegrationConstraint::ComposeVariable
        struct /* ComposeVariable */ {
          // Source: drake/multibody/optimization/quaternion_integration_constraint.h
          const char* doc = R"""()""";
        } ComposeVariable;
        // Symbol: drake::multibody::QuaternionEulerIntegrationConstraint::QuaternionEulerIntegrationConstraint
        struct /* ctor */ {
          // Source: drake/multibody/optimization/quaternion_integration_constraint.h
          const char* doc =
R"""(Parameter ``allow_quaternion_negation``:
    Refer to the class documentation.)""";
        } ctor;
        // Symbol: drake::multibody::QuaternionEulerIntegrationConstraint::allow_quaternion_negation
        struct /* allow_quaternion_negation */ {
          // Source: drake/multibody/optimization/quaternion_integration_constraint.h
          const char* doc = R"""()""";
        } allow_quaternion_negation;
      } QuaternionEulerIntegrationConstraint;
      // Symbol: drake::multibody::SpatialVelocityConstraint
      struct /* SpatialVelocityConstraint */ {
        // Source: drake/multibody/optimization/spatial_velocity_constraint.h
        const char* doc =
R"""(Constrains the spatial velocity of a frame C, rigidly attached to a
frame B, measured and expressed in frame A. It should be bound with
decision variables corresponding to the multibody state x=[q,v] of the
``plant`` passed to the constructor.)""";
        // Symbol: drake::multibody::SpatialVelocityConstraint::AngularVelocityBounds
        struct /* AngularVelocityBounds */ {
          // Source: drake/multibody/optimization/spatial_velocity_constraint.h
          const char* doc =
R"""(Parametrizes bounds on the magnitude and direction of the angular
velocity vector.)""";
          // Symbol: drake::multibody::SpatialVelocityConstraint::AngularVelocityBounds::magnitude_lower
          struct /* magnitude_lower */ {
            // Source: drake/multibody/optimization/spatial_velocity_constraint.h
            const char* doc =
R"""(Lower bound on the magnitude of the angular velocity vector. Must be
non-negative. The actual constraint will be implemented as a
constraint on the squared magnitude.)""";
          } magnitude_lower;
          // Symbol: drake::multibody::SpatialVelocityConstraint::AngularVelocityBounds::magnitude_upper
          struct /* magnitude_upper */ {
            // Source: drake/multibody/optimization/spatial_velocity_constraint.h
            const char* doc =
R"""(Upper bound on the magnitude of the angular velocity vector. Must be ≥
magnitude_lower. The actual constraint will be implemented as a
constraint on the squared magnitude.)""";
          } magnitude_upper;
          // Symbol: drake::multibody::SpatialVelocityConstraint::AngularVelocityBounds::reference_direction
          struct /* reference_direction */ {
            // Source: drake/multibody/optimization/spatial_velocity_constraint.h
            const char* doc =
R"""(Reference direction of the angular velocity vector. (Only the
direction matters, the magnitude does not).)""";
          } reference_direction;
          // Symbol: drake::multibody::SpatialVelocityConstraint::AngularVelocityBounds::theta_bound
          struct /* theta_bound */ {
            // Source: drake/multibody/optimization/spatial_velocity_constraint.h
            const char* doc =
R"""(The angle difference between w_AC and reference_direction will be
constrained to θ ∈ [0,π] ≤ θ_bound. Must be nonnegative. The actual
constraint will be implemented as cos(θ_bound) ≤ cos(θ) ≤ 1. When the
norm of w_AC is very close to zero, we add a small number to the norm
to avoid numerical difficulties.)""";
          } theta_bound;
        } AngularVelocityBounds;
        // Symbol: drake::multibody::SpatialVelocityConstraint::SpatialVelocityConstraint
        struct /* ctor */ {
          // Source: drake/multibody/optimization/spatial_velocity_constraint.h
          const char* doc =
R"""(Constructs SpatialVelocityConstraint object.

Parameter ``plant``:
    The MultibodyPlant on which the constraint is imposed. ``plant``
    should be alive during the lifetime of this constraint.

Parameter ``frameA``:
    The frame in which frame C's spatial velocity is measured and
    expressed.

Parameter ``v_AC_lower``:
    The lower bound on the translational velocity of C, expressed in
    frame A.

Parameter ``v_AC_upper``:
    The upper bound on the translational velocity of C, expressed in
    frame A.

Parameter ``frameB``:
    The frame to which frame C is rigidly attached.

Parameter ``p_BCo``:
    The position of the origin of frame C, rigidly attached to frame
    B, expressed in frame B. We take R_BC to be the identity rotation.

Parameter ``plant_context``:
    The Context that has been allocated for this ``plant``. We will
    update the context when evaluating the constraint.
    ``plant_context`` should be alive during the lifetime of this
    constraint.

Parameter ``w_AC_bounds``:
    If provided, then the number of constraints will be 5: 3 for
    translational velocities and then two more for the angular
    velocity magnitude and angle.

Precondition:
    ``frameA`` and ``frameB`` must belong to ``plant``.

Precondition:
    v_AC_lower(i) <= v_AC_upper(i) for i = 1, 2, 3.

Raises:
    RuntimeError if ``plant`` is nullptr.

Raises:
    RuntimeError if ``plant_context`` is nullptr.

Raises:
    RuntimeError if invalid w_AC_bounds are provided.)""";
        } ctor;
      } SpatialVelocityConstraint;
      // Symbol: drake::multibody::StaticEquilibriumConstraint
      struct /* StaticEquilibriumConstraint */ {
        // Source: drake/multibody/optimization/static_equilibrium_constraint.h
        const char* doc =
R"""(Impose the static equilibrium constraint 0 = τ_g + Bu + ∑J_WBᵀ(q) *
Fapp_B_W)""";
        // Symbol: drake::multibody::StaticEquilibriumConstraint::MakeBinding
        struct /* MakeBinding */ {
          // Source: drake/multibody/optimization/static_equilibrium_constraint.h
          const char* doc =
R"""(Create a static equilibrium constraint 0 = g(q) + Bu + ∑ᵢ
JᵢᵀFᵢ_AB_W(λᵢ) This constraint depends on the variables q, u and λ.

Parameter ``plant``:
    The plant on which the constraint is imposed.

Parameter ``context``:
    The context for the subsystem ``plant``.

Parameter ``contact_wrench_evaluators_and_lambda``:
    For each contact pair, we need to compute the contact wrench
    applied at the point of contact, expressed in the world frame,
    namely Fᵢ_AB_W(λᵢ). ``contact_wrench_evaluators_and_lambda.first``
    is the evaluator for computing this contact wrench from the
    variables λᵢ. ``contact_wrench_evaluators_and_lambda.second`` are
    the decision variable λᵢ used in computing the contact wrench.
    Notice the generalized position ``q`` is not included in variables
    contact_wrench_evaluators_and_lambda.second.

Parameter ``q_vars``:
    The decision variables for q (the generalized position).

Parameter ``u_vars``:
    The decision variables for u (the input).

Returns:
    binding The binding between the static equilibrium constraint and
    the variables q, u and λ.

Precondition:
    ``plant`` must have been connected to a SceneGraph properly. You
    could refer to AddMultibodyPlantSceneGraph on how to connect a
    MultibodyPlant to a SceneGraph.)""";
        } MakeBinding;
        // Symbol: drake::multibody::StaticEquilibriumConstraint::StaticEquilibriumConstraint
        struct /* ctor */ {
          // Source: drake/multibody/optimization/static_equilibrium_constraint.h
          const char* doc = R"""()""";
        } ctor;
        // Symbol: drake::multibody::StaticEquilibriumConstraint::contact_pair_to_wrench_evaluator
        struct /* contact_pair_to_wrench_evaluator */ {
          // Source: drake/multibody/optimization/static_equilibrium_constraint.h
          const char* doc =
R"""(Getter for contact_pair_to_wrench_evaluator, passed in the
constructor.)""";
        } contact_pair_to_wrench_evaluator;
      } StaticEquilibriumConstraint;
      // Symbol: drake::multibody::StaticEquilibriumProblem
      struct /* StaticEquilibriumProblem */ {
        // Source: drake/multibody/optimization/static_equilibrium_problem.h
        const char* doc =
R"""(Finds the static equilibrium pose of a multibody system through
optimization. The constraints are 1. 0 = g(q) + Bu + ∑ᵢ JᵢᵀFᵢ_AB_W(λᵢ)
(generalized force equals to 0). 2. Fᵢ_AB_W(λᵢ) is within the
admissible contact wrench (for example, contact force is in the
friction cone). 3. sdf(q) >= 0 (signed distance function is no smaller
than 0, hence no penetration). 4. complementarity condition between
the contact force and the signed distance. 5. q within the joint
limit.)""";
        // Symbol: drake::multibody::StaticEquilibriumProblem::GetContactWrenchSolution
        struct /* GetContactWrenchSolution */ {
          // Source: drake/multibody/optimization/static_equilibrium_problem.h
          const char* doc =
R"""(Retrieve the solution to all contact wrenches.

Parameter ``result``:
    The result of solving prog().)""";
        } GetContactWrenchSolution;
        // Symbol: drake::multibody::StaticEquilibriumProblem::StaticEquilibriumProblem
        struct /* ctor */ {
          // Source: drake/multibody/optimization/static_equilibrium_problem.h
          const char* doc =
R"""(Parameter ``plant``:
    The plant for which the static equilibrium posture is computed.
    ``plant`` should remain alive as long as this
    StaticEquilibriumProblem exists.

Parameter ``context``:
    The context for ``plant``. ``context`` should remain alive as long
    as this StaticEquilibriumProblem exists.

Parameter ``ignored_collision_pairs``:
    The contact between the pair of geometry in
    ``ignored_collision_pairs`` will be ignored. We will not impose
    non-penetration constraint between these pairs, and no contact
    wrench will be applied between these pairs.)""";
        } ctor;
        // Symbol: drake::multibody::StaticEquilibriumProblem::UpdateComplementarityTolerance
        struct /* UpdateComplementarityTolerance */ {
          // Source: drake/multibody/optimization/static_equilibrium_problem.h
          const char* doc =
R"""(Updates the tolerance on all the complementarity constraints α * β =
0. The complementarity constraint is relaxed as 0 ≤ α * β ≤ tol. See
AddStaticFrictionConeComplementarityConstraint() for more details.)""";
        } UpdateComplementarityTolerance;
        // Symbol: drake::multibody::StaticEquilibriumProblem::get_mutable_prog
        struct /* get_mutable_prog */ {
          // Source: drake/multibody/optimization/static_equilibrium_problem.h
          const char* doc = R"""()""";
        } get_mutable_prog;
        // Symbol: drake::multibody::StaticEquilibriumProblem::prog
        struct /* prog */ {
          // Source: drake/multibody/optimization/static_equilibrium_problem.h
          const char* doc =
R"""(Getter for the immutable optimization program.)""";
        } prog;
        // Symbol: drake::multibody::StaticEquilibriumProblem::q_vars
        struct /* q_vars */ {
          // Source: drake/multibody/optimization/static_equilibrium_problem.h
          const char* doc =
R"""(Getter for q, the decision variable for the generalized position.)""";
        } q_vars;
        // Symbol: drake::multibody::StaticEquilibriumProblem::u_vars
        struct /* u_vars */ {
          // Source: drake/multibody/optimization/static_equilibrium_problem.h
          const char* doc =
R"""(Getter for u, the decision variable for the input.)""";
        } u_vars;
      } StaticEquilibriumProblem;
      // Symbol: drake::multibody::StaticFrictionConeConstraint
      struct /* StaticFrictionConeConstraint */ {
        // Source: drake/multibody/optimization/static_friction_cone_constraint.h
        const char* doc =
R"""(Formulates the nonlinear friction cone constraint |fₜ| ≤ μ*fₙ, where
fₜ is the tangential contact force, fₙ is the normal contact force,
and μ is the friction coefficient.

The mathematical formulation of this constraint is

0 ≤ μ*fᵀn fᵀ((1+μ²)nnᵀ - I)f ≥ 0 where n is the unit length normal
vector. This formulation is equivalent to |fₜ| ≤ μ*fₙ, but the
constraint is differentiable everywhere (while |fₜ| ≤ μ*fₙ is not
differentiable at fₜ = 0.)

The bound variables for this constraint is x = [q;λ], where q is the
generalized position, and λ is the parameterization of the contact
wrench.)""";
        // Symbol: drake::multibody::StaticFrictionConeConstraint::DecomposeX
        struct /* DecomposeX */ {
          // Source: drake/multibody/optimization/static_friction_cone_constraint.h
          const char* doc =
R"""(Given the bound variable ``x``, decompose it into the generalized
position q, and λ as a parameterization of the contact wrench. x = [q;
λ].)""";
        } DecomposeX;
        // Symbol: drake::multibody::StaticFrictionConeConstraint::StaticFrictionConeConstraint
        struct /* ctor */ {
          // Source: drake/multibody/optimization/static_friction_cone_constraint.h
          const char* doc =
R"""(Parameter ``contact_wrench_evaluator``:
    The evaluator takes in the generalized position q, and a
    parameterization of the contact wrench λ, and evaluates the
    contact wrench from body A to body B applied at the witness point
    of geometry B expressed in the world frame, i.e., computes the
    contact wrench F_Cb_W at the witness point p_WCb_W (see
    SignedDistancePair for the definition of witness point), and
    F_Ca_W = -F_Cb_W (assuming Ca and Cb are at the same spatial
    location).

Note:
    although contact_wrench_evaluator computes both the contact force
    and torque in the wrench, we only constrain the contact force to
    be within the friction cone, and leave the torque unconstrained.)""";
        } ctor;
      } StaticFrictionConeConstraint;
      // Symbol: drake::multibody::Toppra
      struct /* Toppra */ {
        // Source: drake/multibody/optimization/toppra.h
        const char* doc =
R"""(Solves a Time Optimal Path Parameterization based on Reachability
Analysis (TOPPRA) to find the fastest traversal of a given path,
satisfying the given constraints. The approach is described in "A new
approach to Time-Optimal Path Parameterization based on Reachability
Analysis" by Hung Pham and Quang Cuong Pham, IEEE Transactions on
Robotics, 2018.)""";
        // Symbol: drake::multibody::Toppra::AddFrameAccelerationLimit
        struct /* AddFrameAccelerationLimit */ {
          // Source: drake/multibody/optimization/toppra.h
          const char* doc_const =
R"""(Adds a limit on the elements of the spatial acceleration of the given
frame, measured and expressed in the world frame. The limits should be
given as [α_WF, a_WF], where α_WF is the frame's angular acceleration
and a_WF is the frame's translational acceleration.

Parameter ``constraint_frame``:
    The frame to limit the acceleration of.

Parameter ``lower_limit``:
    The lower acceleration limit for constraint_frame.

Parameter ``upper_limit``:
    The upper acceleration limit for constraint_frame.

Parameter ``discretization``:
    The discretization scheme to use for this linear constraint. See
    ToppraDiscretization for details.

Returns:
    A pair containing the linear constraints that will enforce the
    frame acceleration limit on the backward pass and forward pass
    respectively.)""";
          // Source: drake/multibody/optimization/toppra.h
          const char* doc_trajectory =
R"""(A version of acceleration limit that uses a trajectory for the upper
and lower limits.

Parameter ``constraint_frame``:
    The frame to limit the acceleration of.

Parameter ``lower_limit``:
    The lower acceleration limit trajectory for constraint_frame.

Parameter ``upper_limit``:
    The upper acceleration limit trajectory for constraint_frame.

Parameter ``discretization``:
    The discretization scheme to use for this linear constraint. See
    ToppraDiscretization for details.

Returns:
    A pair containing the linear constraints that will enforce the
    frame acceleration limit on the backward pass and forward pass
    respectively.

Precondition:
    Both lower_limit and upper_limit trajectories must have values in
    ℜ⁶. The six-dimensional column vector is interpreted as [α_WF,
    a_WF], where α_WF is the frame's angular acceleration and a_WF is
    the frame's translational acceleration.

Raises:
    If the intervals [upper_limit.start_time(),
    upper_limit.end_time()] and [lower_limit.start_time(),
    lower_limit.end_time()] don't overlap with [path.start_time(),
    path.end_time()].

Note:
    The constraints are only added in the constraint trajectories
    domains (where they overlap the path). The rest of the path is
    *not* constrained.)""";
        } AddFrameAccelerationLimit;
        // Symbol: drake::multibody::Toppra::AddFrameTranslationalSpeedLimit
        struct /* AddFrameTranslationalSpeedLimit */ {
          // Source: drake/multibody/optimization/toppra.h
          const char* doc_const =
R"""(Adds a limit on the magnitude of the translational velocity of the
given frame, measured and expressed in the world frame.

Parameter ``constraint_frame``:
    The frame to limit the translational speed of.

Parameter ``upper_limit``:
    The upper translational speed limit for constraint_frame.

Returns:
    The bounding box constraint that will enforce the frame
    translational speed limit during the backward pass.)""";
          // Source: drake/multibody/optimization/toppra.h
          const char* doc_trajectory =
R"""(A version of the frame translational speed limit that uses a
trajectory for the limit.

Parameter ``constraint_frame``:
    The frame to limit the translational speed of.

Parameter ``upper_limit``:
    The upper translational speed limit trajectory for
    constraint_frame.

Precondition:
    upper_limit must have scalar values (a 1x1 matrix).

Returns:
    The bounding box constraint that will enforce the frame
    translational speed limit during the backward pass.

Raises:
    If the interval [upper_limit.start_time(), upper_limit.end_time()]
    doesn't overlap with [path.start_time(), path.end_time()].

Note:
    The constraints are only added between upper_limit.start_time()
    and upper_limit.end_time(). The rest of the path is *not*
    constrained.)""";
        } AddFrameTranslationalSpeedLimit;
        // Symbol: drake::multibody::Toppra::AddFrameVelocityLimit
        struct /* AddFrameVelocityLimit */ {
          // Source: drake/multibody/optimization/toppra.h
          const char* doc =
R"""(Adds a limit on the elements of the spatial velocity of the given
frame, measured and and expressed in the world frame. The limits
should be given as [ω_WF, v_WF], where ω_WF is the frame's angular
velocity and v_WF is the frame's translational velocity.

Parameter ``constraint_frame``:
    The frame to limit the velocity of.

Parameter ``lower_limit``:
    The lower velocity limit for constraint_frame.

Parameter ``upper_limit``:
    The upper velocity limit for constraint_frame.

Returns:
    The bounding box constraint that will enforce the frame velocity
    limit during the backward pass.)""";
        } AddFrameVelocityLimit;
        // Symbol: drake::multibody::Toppra::AddJointAccelerationLimit
        struct /* AddJointAccelerationLimit */ {
          // Source: drake/multibody/optimization/toppra.h
          const char* doc =
R"""(Adds an acceleration limit to all the degrees of freedom in the plant.
The limits must be arranged in the same order as the entries in the
path.

Parameter ``lower_limit``:
    The lower acceleration limit for each degree of freedom.

Parameter ``upper_limit``:
    The upper acceleration limit for each degree of freedom.

Parameter ``discretization``:
    The discretization scheme to use for this linear constraint. See
    ToppraDiscretization for details.

Returns:
    A pair containing the linear constraints that will enforce the
    acceleration limit on the backward pass and forward pass
    respectively.)""";
        } AddJointAccelerationLimit;
        // Symbol: drake::multibody::Toppra::AddJointTorqueLimit
        struct /* AddJointTorqueLimit */ {
          // Source: drake/multibody/optimization/toppra.h
          const char* doc =
R"""(Adds a torque limit to all the degrees of freedom in the plant. The
limits must be arranged in the same order as the entries in the path.
This constrains the generalized torques applied to the plant and does
not reason about contact forces.

Parameter ``lower_limit``:
    The lower torque limit for each degree of freedom.

Parameter ``upper_limit``:
    The upper torque limit for each degree of freedom.

Parameter ``discretization``:
    The discretization scheme to use for this linear constraint. See
    ToppraDiscretization for details.

Returns:
    A pair containing the linear constraints that will enforce the
    torque limit on the backward pass and forward pass respectively.)""";
        } AddJointTorqueLimit;
        // Symbol: drake::multibody::Toppra::AddJointVelocityLimit
        struct /* AddJointVelocityLimit */ {
          // Source: drake/multibody/optimization/toppra.h
          const char* doc =
R"""(Adds a velocity limit to all the degrees of freedom in the plant. The
limits must be arranged in the same order as the entries in the path.

Parameter ``lower_limit``:
    The lower velocity limit for each degree of freedom.

Parameter ``upper_limit``:
    The upper velocity limit for each degree of freedom.)""";
        } AddJointVelocityLimit;
        // Symbol: drake::multibody::Toppra::CalcGridPoints
        struct /* CalcGridPoints */ {
          // Source: drake/multibody/optimization/toppra.h
          const char* doc =
R"""(Takes a path and generates a sequence of gridpoints selected to
control the interpolation error of the optimization. The gridpoints
are selected such that the distance between them is below
``max_seg_length``, there are at least ``min_points`` number of
gridpoints and the interpolation error, estimated with the equation


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    errₑₛₜ = max(|q̈ Δₛ²|) / 2

.. raw:: html

    </details>

where Δₛ is the distance between sequential gridpoints, is less than
``max_err``. Gridpoints are selected by adding the midpoint between
two gridpoints whenever the distance between them is too large or the
estimated error is too high. This results in more points in parts of
the path with higher curvature. All grid points will lie between
path.start_time() and path.end_time().)""";
        } CalcGridPoints;
        // Symbol: drake::multibody::Toppra::SolvePathParameterization
        struct /* SolvePathParameterization */ {
          // Source: drake/multibody/optimization/toppra.h
          const char* doc =
R"""(Solves the TOPPRA optimization and returns the time optimized path
parameterization s(t). This can be used with the original path q(s) to
generate a time parameterized trajectory. The path parameterization
has the same start time as the original path's starting break.

Parameter ``s_dot_start``:
    ṡ(0). The default value is zero (trajectory starts at zero
    velocity).

Parameter ``s_dot_end``:
    ṡ(T), where T is the end break of the path. The default value is
    zero (trajectory ends at zero velocity).)""";
        } SolvePathParameterization;
        // Symbol: drake::multibody::Toppra::Toppra
        struct /* ctor */ {
          // Source: drake/multibody/optimization/toppra.h
          const char* doc =
R"""(Constructs an inverse kinematics problem for a MultibodyPlant. This
constructor will create and own a context for ``plant``.

Parameter ``path``:
    The trajectory on which the TOPPRA problem will be solved.

Parameter ``plant``:
    The robot that will follow the solved trajectory. Used for
    enforcing torque and frame specific constraints.

Parameter ``gridpoints``:
    The points along the path to discretize the problem and enforce
    constraints at. The first and last gridpoint must equal the path
    start and end time respectively. Gridpoints must also be
    monotonically increasing.

Note:
    Toppra does not currently support plants that contain bodies with
    quaternion degrees of freedom. In addition, any plant where q̇ ≠ v
    will have undefined behavior.

Note:
    The path velocity, ṡ(t), is limited to be between 0 and 1e8 to
    ensure the reachable set calculated in the backward pass is always
    bounded.)""";
        } ctor;
      } Toppra;
      // Symbol: drake::multibody::ToppraDiscretization
      struct /* ToppraDiscretization */ {
        // Source: drake/multibody/optimization/toppra.h
        const char* doc =
R"""(Selects how linear constraints are enforced for TOPPRA's optimization.
kCollocation - enforces constraints only at each gridpoint.
kInterpolation - enforces constraints at each gridpoint and at the
following gridpoint using forward integration. Yields higher accuracy
at minor computational cost.)""";
        // Symbol: drake::multibody::ToppraDiscretization::kCollocation
        struct /* kCollocation */ {
          // Source: drake/multibody/optimization/toppra.h
          const char* doc = R"""()""";
        } kCollocation;
        // Symbol: drake::multibody::ToppraDiscretization::kInterpolation
        struct /* kInterpolation */ {
          // Source: drake/multibody/optimization/toppra.h
          const char* doc = R"""()""";
        } kInterpolation;
      } ToppraDiscretization;
    } multibody;
  } drake;
} pydrake_doc_multibody_optimization;

#if defined(__GNUG__)
#pragma GCC diagnostic pop
#endif
