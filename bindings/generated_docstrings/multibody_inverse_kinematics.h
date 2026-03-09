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

// #include "drake/multibody/inverse_kinematics/add_multibody_plant_constraints.h"
// #include "drake/multibody/inverse_kinematics/angle_between_vectors_constraint.h"
// #include "drake/multibody/inverse_kinematics/angle_between_vectors_cost.h"
// #include "drake/multibody/inverse_kinematics/com_in_polyhedron_constraint.h"
// #include "drake/multibody/inverse_kinematics/com_position_constraint.h"
// #include "drake/multibody/inverse_kinematics/constraint_relaxing_ik.h"
// #include "drake/multibody/inverse_kinematics/differential_inverse_kinematics.h"
// #include "drake/multibody/inverse_kinematics/differential_inverse_kinematics_controller.h"
// #include "drake/multibody/inverse_kinematics/differential_inverse_kinematics_integrator.h"
// #include "drake/multibody/inverse_kinematics/differential_inverse_kinematics_system.h"
// #include "drake/multibody/inverse_kinematics/distance_constraint.h"
// #include "drake/multibody/inverse_kinematics/distance_constraint_utilities.h"
// #include "drake/multibody/inverse_kinematics/gaze_target_constraint.h"
// #include "drake/multibody/inverse_kinematics/global_inverse_kinematics.h"
// #include "drake/multibody/inverse_kinematics/inverse_kinematics.h"
// #include "drake/multibody/inverse_kinematics/kinematic_evaluator_utilities.h"
// #include "drake/multibody/inverse_kinematics/minimum_distance_lower_bound_constraint.h"
// #include "drake/multibody/inverse_kinematics/minimum_distance_upper_bound_constraint.h"
// #include "drake/multibody/inverse_kinematics/orientation_constraint.h"
// #include "drake/multibody/inverse_kinematics/orientation_cost.h"
// #include "drake/multibody/inverse_kinematics/point_to_line_distance_constraint.h"
// #include "drake/multibody/inverse_kinematics/point_to_point_distance_constraint.h"
// #include "drake/multibody/inverse_kinematics/polyhedron_constraint.h"
// #include "drake/multibody/inverse_kinematics/position_constraint.h"
// #include "drake/multibody/inverse_kinematics/position_cost.h"
// #include "drake/multibody/inverse_kinematics/unit_quaternion_constraint.h"

// Symbol: pydrake_doc_multibody_inverse_kinematics
constexpr struct /* pydrake_doc_multibody_inverse_kinematics */ {
  // Symbol: drake
  struct /* drake */ {
    // Symbol: drake::multibody
    struct /* multibody */ {
      // Symbol: drake::multibody::AddMultibodyPlantConstraints
      struct /* AddMultibodyPlantConstraints */ {
        // Source: drake/multibody/inverse_kinematics/add_multibody_plant_constraints.h
        const char* doc =
R"""(For all kinematic constraints associated with ``plant`` adds a
corresponding solver∷Constraint to ``prog``, using decision variables
``q`` to represent the generalized positions of the plant.

Adds joint limits constraints, unit quaternion constraints, and
constraints for any locked joints (via Joint∷Lock()). Note that you
must pass a valid ``plant_context`` to use joint locking.

Adds constraints for coupler, distance, ball, and weld constraints.
The distance constraint is implemented here as a hard kinematic
constraint (i.e., d(q) == d₀), instead of a soft "spring" force.

See also:
    AddUnitQuaternionConstraintOnPlant() for the unit quaternion
    constraints.

Precondition:
    plant.is_finalized() == true.

Raises:
    RuntimeError if ``plant`` has constraints registered that are not
    yet supported by this method.

Raises:
    RuntimeError if ``prog`` is nullptr.

Raises:
    RuntimeError if ``plant_context`` is nullptr and one of the
    MultibodyPlant constraints requires it. (unit quaternion
    constraints and coupler constraints do not).)""";
      } AddMultibodyPlantConstraints;
      // Symbol: drake::multibody::AddUnitQuaternionConstraintOnPlant
      struct /* AddUnitQuaternionConstraintOnPlant */ {
        // Source: drake/multibody/inverse_kinematics/unit_quaternion_constraint.h
        const char* doc =
R"""(Add unit length constraints to all the variables representing
quaternion in ``q_vars``. Namely the quaternions for floating base
joints in ``plant`` will be enforced to have a unit length, and all
quaternion variables will be bounded to be within [-1, 1].

Additionally, if the initial guess for the quaternion variables has
not been set (it is nan), then this method calls
MathematicalProgram∷SetInitialGuess() with [1, 0, 0, 0], to help the
solver avoid singularities.

Parameter ``plant``:
    The plant on which we impose the unit quaternion constraints.

Parameter ``q_vars``:
    The decision variables for the generalized position of the plant.

Parameter ``prog``:
    The unit quaternion constraints are added to this prog.)""";
      } AddUnitQuaternionConstraintOnPlant;
      // Symbol: drake::multibody::AngleBetweenVectorsConstraint
      struct /* AngleBetweenVectorsConstraint */ {
        // Source: drake/multibody/inverse_kinematics/angle_between_vectors_constraint.h
        const char* doc =
R"""(Constrains that the angle between a vector ``a`` and another vector
``b`` is between [θ_lower, θ_upper]. ``a`` is fixed to a frame A,
while ``b`` is fixed to a frame B. Mathematically, if we denote
a_unit_A as ``a`` expressed in frame A after normalization (a_unit_A
has unit length), and b_unit_B as ``b`` expressed in frame B after
normalization, the constraint is cos(θ_upper) ≤ a_unit_Aᵀ * R_AB *
b_unit_B ≤ cos(θ_lower))""";
        // Symbol: drake::multibody::AngleBetweenVectorsConstraint::AngleBetweenVectorsConstraint
        struct /* ctor */ {
          // Source: drake/multibody/inverse_kinematics/angle_between_vectors_constraint.h
          const char* doc_double =
R"""(Constructs an AngleBetweenVectorsConstraint.

Parameter ``plant``:
    The MultibodyPlant on which the constraint is imposed. ``plant``
    should be alive during the lifetime of this constraint.

Parameter ``frameA``:
    The Frame object for frame A.

Parameter ``a_A``:
    The vector ``a`` fixed to frame A, expressed in frame A.

Parameter ``frameB``:
    The Frame object for frame B.

Parameter ``b_B``:
    The vector ``b`` fixed to frame B, expressed in frameB.

Parameter ``angle_lower``:
    The lower bound on the angle between ``a`` and ``b``. It is
    denoted as θ_lower in the class documentation.

Parameter ``angle_upper``:
    The upper bound on the angle between ``a`` and ``b``. it is
    denoted as θ_upper in the class documentation.

Parameter ``plant_context``:
    The Context that has been allocated for this ``plant``. We will
    update the context when evaluating the constraint.
    ``plant_context`` should be alive during the lifetime of this
    constraint.

Precondition:
    ``frameA`` and ``frameB`` must belong to ``plant``.

Raises:
    RuntimeError if ``plant`` is nullptr.

Raises:
    RuntimeError if ``a_A`` is close to zero.

Raises:
    RuntimeError if ``b_B`` is close to zero.

Raises:
    RuntimeError if ``angle_lower`` is negative.

Raises:
    RuntimeError if ``angle_upper`` ∉ [`angle_lower`, π].

Raises:
    RuntimeError if ``plant_context`` is nullptr.)""";
          // Source: drake/multibody/inverse_kinematics/angle_between_vectors_constraint.h
          const char* doc_autodiff =
R"""(Overloaded constructor. Use MultibodyPlant<AutoDiffXd> instead of
MultibodyPlant<double>.)""";
        } ctor;
      } AngleBetweenVectorsConstraint;
      // Symbol: drake::multibody::AngleBetweenVectorsCost
      struct /* AngleBetweenVectorsCost */ {
        // Source: drake/multibody/inverse_kinematics/angle_between_vectors_cost.h
        const char* doc =
R"""(Implements a cost of the form c*(1-cosθ), where θ is the angle between
two vectors ``a`` and ``b``. `c` is a constant scalar.)""";
        // Symbol: drake::multibody::AngleBetweenVectorsCost::AngleBetweenVectorsCost
        struct /* ctor */ {
          // Source: drake/multibody/inverse_kinematics/angle_between_vectors_cost.h
          const char* doc_double =
R"""(Constructs an AngleBetweenVectorsCost.

Parameter ``plant``:
    The MultibodyPlant on which the cost is imposed. ``plant`` should
    be alive during the lifetime of this cost.

Parameter ``frameA``:
    The Frame object for frame A.

Parameter ``a_A``:
    The vector ``a`` fixed to frame A, expressed in frame A.

Parameter ``frameB``:
    The Frame object for frame B.

Parameter ``b_B``:
    The vector ``b`` fixed to frame B, expressed in frameB.

Parameter ``c``:
    The cost is c*(1-cosθ).

Parameter ``plant_context``:
    The Context that has been allocated for this ``plant``. We will
    update the context when evaluating the constraint.
    ``plant_context`` should be alive during the lifetime of this
    cost.

Precondition:
    ``frameA`` and ``frameB`` must belong to ``plant``.

Raises:
    RuntimeError if ``plant`` is nullptr.

Raises:
    RuntimeError if ``a_A`` is close to zero.

Raises:
    RuntimeError if ``b_B`` is close to zero.

Raises:
    RuntimeError if ``plant_context`` is nullptr.)""";
          // Source: drake/multibody/inverse_kinematics/angle_between_vectors_cost.h
          const char* doc_autodiff =
R"""(Overloaded constructor. Use MultibodyPlant<AutoDiffXd> instead of
MultibodyPlant<double>.)""";
        } ctor;
      } AngleBetweenVectorsCost;
      // Symbol: drake::multibody::ComInPolyhedronConstraint
      struct /* ComInPolyhedronConstraint */ {
        // Source: drake/multibody/inverse_kinematics/com_in_polyhedron_constraint.h
        const char* doc =
R"""(Constrains the center of mass to lie within a polyhedron lb <= A *
p_EC <= ub where p_EC is the position of the center-of-mass (C)
expressed in a frame E.

For example, if you set A as identity matrix, then this constraint
enforces a box-region on the CoM position p_EC. If you set the
expressed frame E as the robot foot frame, and choose A to describe
the foot support polygon, this constraint could enforce the projection
of CoM to be within the foot support polygon, which is commonly used
to ensure static equilibrium.)""";
        // Symbol: drake::multibody::ComInPolyhedronConstraint::ComInPolyhedronConstraint
        struct /* ctor */ {
          // Source: drake/multibody/inverse_kinematics/com_in_polyhedron_constraint.h
          const char* doc_double =
R"""(Constructs a ComInPolyhedronConstraint object.

Parameter ``plant``:
    The MultibodyPlant on which the constraint is imposed. ``plant``
    must be alive during the lifetime of this constraint.

Parameter ``model_instances``:
    The CoM of these model instances are computed. If model_instances
    = std∷nullopt, then we compute the CoM of all model instances
    (except the world).

Parameter ``expressed_frame``:
    The frame in which the CoM is expressed.

Parameter ``A``:
    The CoM position p_EC satisfies lb <= A * p_EC <= ub

Parameter ``lb``:
    The CoM position p_EC satisfies lb <= A * p_EC <= ub

Parameter ``ub``:
    The CoM position p_EC satisfies lb <= A * p_EC <= ub

Parameter ``plant_context``:
    The Context that has been allocated for this ``plant``. We will
    update the context when evaluating the constraint.
    ``plant_context`` must be alive during the lifetime of this
    constraint.)""";
          // Source: drake/multibody/inverse_kinematics/com_in_polyhedron_constraint.h
          const char* doc_autodiff =
R"""(Overloaded constructor. Same as the constructor with the double
version (using MultibodyPlant<double> and Context<double>. Except the
gradient of the constraint is computed from autodiff.

Precondition:
    if model_instances is not std∷nullopt, then all indices in
    ``model_instances`` refer to valid model instances in ``plant``.)""";
        } ctor;
      } ComInPolyhedronConstraint;
      // Symbol: drake::multibody::ComPositionConstraint
      struct /* ComPositionConstraint */ {
        // Source: drake/multibody/inverse_kinematics/com_position_constraint.h
        const char* doc =
R"""(Impose the constraint p_EScm(q) - p_EC = 0, where p_EScm(q) is a
function that computes the center-of-mass (COM) position from robot
generalized position q, expressed in a frame E. p_EC ∈ ℝ³ is the
variable representing robot CoM (C) position expressed in frame E. The
evaluated variables are [q;r], where q is the generalized position
vector of the entire plant.)""";
        // Symbol: drake::multibody::ComPositionConstraint::ComPositionConstraint
        struct /* ctor */ {
          // Source: drake/multibody/inverse_kinematics/com_position_constraint.h
          const char* doc_double =
R"""(Constructor, constrain f(q) = p_EC, where f(q) evaluates the CoM
position expressed in frame E using the generalized position q.

Parameter ``plant``:
    The MultibodyPlant on which the constraint is imposed. ``plant``
    should be alive during the lifetime of this constraint.

Parameter ``model_instances``:
    We compute the model with these model instances in ``plant``. If
    model_instances=std∷nullopt, then we compute the CoM position of
    all model instances except the world.

Parameter ``expressed_frame``:
    The frame in which the CoM position is expressed.

Parameter ``plant_context``:
    The Context that has been allocated for this ``plant``. We will
    update the context when evaluating the constraint.
    ``plant_context`` should be alive during the lifetime of this
    constraint.

Raises:
    RuntimeError if ``plant`` or ``plant_context`` is nullptr.)""";
          // Source: drake/multibody/inverse_kinematics/com_position_constraint.h
          const char* doc_autodiff =
R"""(Overloaded constructor with MultibodyPlant<AutoDiffXd> and
Context<AutoDiffXd>. It is preferable to use the constructor with
MBP<double> and Context<double>. But if you only have MBP<AutoDiffXd>
and Context<AutoDiffXd>, then use this constructor.)""";
        } ctor;
        // Symbol: drake::multibody::ComPositionConstraint::ComposeVariable
        struct /* ComposeVariable */ {
          // Source: drake/multibody/inverse_kinematics/com_position_constraint.h
          const char* doc =
R"""(Compose the variables for Eval function from generalized position q
and the CoM position p_EC.)""";
        } ComposeVariable;
      } ComPositionConstraint;
      // Symbol: drake::multibody::ComputePoseDiffInCommonFrame
      struct /* ComputePoseDiffInCommonFrame */ {
        // Source: drake/multibody/inverse_kinematics/differential_inverse_kinematics.h
        const char* doc =
R"""(Computes the pose "difference" between ``X_C0`` and ``X_C1`` such that
the linear part equals p_C1 - p_C0, and the angular part equals R_C1 *
R_C0.inv(), where p and R stand for the position and rotation parts,
and C is the common frame.)""";
      } ComputePoseDiffInCommonFrame;
      // Symbol: drake::multibody::ConstraintRelaxingIk
      struct /* ConstraintRelaxingIk */ {
        // Source: drake/multibody/inverse_kinematics/constraint_relaxing_ik.h
        const char* doc =
R"""(A wrapper class around the IK planner. This class improves IK's
usability by handling constraint relaxing and multiple initial guesses
internally.)""";
        // Symbol: drake::multibody::ConstraintRelaxingIk::ConstraintRelaxingIk
        struct /* ctor */ {
          // Source: drake/multibody/inverse_kinematics/constraint_relaxing_ik.h
          const char* doc =
R"""(Constructor. Instantiates an internal MultibodyPlant from
``model_path``.

Parameter ``model_path``:
    Path to the model file.

Parameter ``end_effector_link_name``:
    Link name of the end effector.)""";
        } ctor;
        // Symbol: drake::multibody::ConstraintRelaxingIk::IkCartesianWaypoint
        struct /* IkCartesianWaypoint */ {
          // Source: drake/multibody/inverse_kinematics/constraint_relaxing_ik.h
          const char* doc =
R"""(Cartesian waypoint. Input to the IK solver.)""";
          // Symbol: drake::multibody::ConstraintRelaxingIk::IkCartesianWaypoint::constrain_orientation
          struct /* constrain_orientation */ {
            // Source: drake/multibody/inverse_kinematics/constraint_relaxing_ik.h
            const char* doc =
R"""(Signals if orientation constraint is enabled.)""";
          } constrain_orientation;
          // Symbol: drake::multibody::ConstraintRelaxingIk::IkCartesianWaypoint::pos_tol
          struct /* pos_tol */ {
            // Source: drake/multibody/inverse_kinematics/constraint_relaxing_ik.h
            const char* doc =
R"""(Bounding box for the end effector in the world frame.)""";
          } pos_tol;
          // Symbol: drake::multibody::ConstraintRelaxingIk::IkCartesianWaypoint::pose
          struct /* pose */ {
            // Source: drake/multibody/inverse_kinematics/constraint_relaxing_ik.h
            const char* doc =
R"""(Desired end effector pose in the world frame.)""";
          } pose;
          // Symbol: drake::multibody::ConstraintRelaxingIk::IkCartesianWaypoint::rot_tol
          struct /* rot_tol */ {
            // Source: drake/multibody/inverse_kinematics/constraint_relaxing_ik.h
            const char* doc =
R"""(Max angle difference (in radians) between solved end effector's
orientation and the desired.)""";
          } rot_tol;
        } IkCartesianWaypoint;
        // Symbol: drake::multibody::ConstraintRelaxingIk::PlanSequentialTrajectory
        struct /* PlanSequentialTrajectory */ {
          // Source: drake/multibody/inverse_kinematics/constraint_relaxing_ik.h
          const char* doc =
R"""(Generates IK solutions for each waypoint sequentially. For waypoint
wp_i, the IK tries to solve q_i that satisfies the end effector
constraints in wp_i and minimizes the squared difference to q_{i-1},
where q_{i-1} is the solution to the previous wp_{i-1}. q_{i-1} =
``q_current`` when i = 0. This function internally does constraint
relaxing and initial condition guessing if necessary.

Note that ``q_current`` is inserted at the beginning of ``q_sol``.

Parameter ``waypoints``:
    A sequence of desired waypoints.

Parameter ``q_current``:
    The initial generalized position.

Parameter ``q_sol``:
    Results.

Parameter ``keep_going``:
    Optional callback to allow for cancellation. When given, this
    function will be called prior to every IK solve; if it returns
    false, the PlanSequentialTrajectory will stop and return false.
    The function is passed the index of the waypoint currently being
    solved. This can be used to enable timeouts for difficult plans.

Returns:
    True if solved successfully.)""";
        } PlanSequentialTrajectory;
        // Symbol: drake::multibody::ConstraintRelaxingIk::SetEndEffector
        struct /* SetEndEffector */ {
          // Source: drake/multibody/inverse_kinematics/constraint_relaxing_ik.h
          const char* doc = R"""(Sets end effector to ``link_name``.)""";
        } SetEndEffector;
      } ConstraintRelaxingIk;
      // Symbol: drake::multibody::DifferentialInverseKinematicsController
      struct /* DifferentialInverseKinematicsController */ {
        // Source: drake/multibody/inverse_kinematics/differential_inverse_kinematics_controller.h
        const char* doc =
R"""(Differential Inverse Kinematics controller that tracks desired poses /
velocities for multiple operational points. The controller tracks a
nominal posture to resolve nullspace.

The diagram implements an open loop controller where the previous
commanded position is fed back as input estimated position to the the
diagram.

TODO(Aditya.Bhat): Add port switch to switch between open and closed
loop control.

Precondition:
    The initial position must be set using the
    ``set_initial_position(systems∷Context<double>* context, const
    Eigen∷Ref<const Eigen∷VectorXd>& value)`` so that the open loop
    controller can start from the correct robot state.

.. pydrake_system::

    name: DifferentialInverseKinematicsController
    input_ports:
    - estimated_state
    - desired_poses
    - nominal_posture
    output_ports:
    - commanded_position
    - commanded_velocity)""";
        // Symbol: drake::multibody::DifferentialInverseKinematicsController::DifferentialInverseKinematicsController
        struct /* ctor */ {
          // Source: drake/multibody/inverse_kinematics/differential_inverse_kinematics_controller.h
          const char* doc =
R"""(Constructs a DifferentialInverseKinematicsController.

Parameter ``differential_inverse_kinematics``:
    a DifferentialInverseKinematicsSystem leaf system.

Parameter ``planar_rotation_dof_indices``:
    the indices of commanded position output that will be wrapped to
    ±π.

Warning:
    a DifferentialInverseKinematicsSystem leaf system may only be
    added to at most one controller. Multiple controller instances
    cannot share the same leaf system.)""";
        } ctor;
        // Symbol: drake::multibody::DifferentialInverseKinematicsController::SetDefaultState
        struct /* SetDefaultState */ {
          // Source: drake/multibody/inverse_kinematics/differential_inverse_kinematics_controller.h
          const char* doc =
R"""(Sets the default state of the controller.)""";
        } SetDefaultState;
        // Symbol: drake::multibody::DifferentialInverseKinematicsController::SetRandomState
        struct /* SetRandomState */ {
          // Source: drake/multibody/inverse_kinematics/differential_inverse_kinematics_controller.h
          const char* doc =
R"""(Sets the random state of the controller.)""";
        } SetRandomState;
        // Symbol: drake::multibody::DifferentialInverseKinematicsController::differential_inverse_kinematics
        struct /* differential_inverse_kinematics */ {
          // Source: drake/multibody/inverse_kinematics/differential_inverse_kinematics_controller.h
          const char* doc = R"""()""";
        } differential_inverse_kinematics;
        // Symbol: drake::multibody::DifferentialInverseKinematicsController::get_mutable_differential_inverse_kinematics
        struct /* get_mutable_differential_inverse_kinematics */ {
          // Source: drake/multibody/inverse_kinematics/differential_inverse_kinematics_controller.h
          const char* doc = R"""()""";
        } get_mutable_differential_inverse_kinematics;
        // Symbol: drake::multibody::DifferentialInverseKinematicsController::set_initial_position
        struct /* set_initial_position */ {
          // Source: drake/multibody/inverse_kinematics/differential_inverse_kinematics_controller.h
          const char* doc =
R"""(Sets the integral part of the DiscreteTimeIntegrator to ``value``.
`value` has the dimension of the full ``plant.num_positions()``;
non-active dofs will be ignored. This in effect sets the initial
position that is fed to the DifferentialInverseKinematicsSystem leaf
system.)""";
        } set_initial_position;
      } DifferentialInverseKinematicsController;
      // Symbol: drake::multibody::DifferentialInverseKinematicsIntegrator
      struct /* DifferentialInverseKinematicsIntegrator */ {
        // Source: drake/multibody/inverse_kinematics/differential_inverse_kinematics_integrator.h
        const char* doc =
R"""(A LeafSystem that integrates successive calls to
DoDifferentialInverseKinematics (which produces joint velocity
commands) to produce joint position commands.

In each evaluation, DoDifferentialInverseKinematics uses a
linearization of the robot kinematics around a nominal position. The
nominal position is obtained by either: 1. If the optional boolean
(abstract-)valued input port ``use_robot_state`` is connected and set
to ``True``, then differential IK is computed using the
``robot_state`` input port (which must also be connected). Note: Using
measured joint positions in a feedback loop can lead to undamped
oscillations in the redundant joints; we hope to resolve this and are
tracking it in #9773. 2. Otherwise, differential IK is computed using
this System's internal state, representing the current joint position
command. This is equivalent to integrating (open loop) the velocity
commands obtained from the differential IK solutions.

It is also important to set the initial state of the integrator: 1. If
the ``robot_state`` port is connected, then the initial state of the
integrator is set to match the positions from this port (the port
accepts the state vector with positions and velocities for easy of use
with MultibodyPlant, but only the positions are used). 2. Otherwise,
it is highly recommended that the user call SetPositions() to
initialize the integrator state.

.. pydrake_system::

    name: DifferentialInverseKinematicsIntegrator
    input_ports:
    - X_AE_desired
    - robot_state (optional)
    - use_robot_state (optional)
    output_ports:
    - joint_positions)""";
        // Symbol: drake::multibody::DifferentialInverseKinematicsIntegrator::DifferentialInverseKinematicsIntegrator
        struct /* ctor */ {
          // Source: drake/multibody/inverse_kinematics/differential_inverse_kinematics_integrator.h
          const char* doc_7args =
R"""(Constructs the system.

Parameter ``robot``:
    A MultibodyPlant describing the robot.

Parameter ``frame_A``:
    Reference frame (inertial or non-inertial).

Parameter ``frame_E``:
    End-effector frame.

Parameter ``time_step``:
    the discrete time step of the (Euler) integration.

Parameter ``parameters``:
    Collection of various problem specific constraints and constants.
    The ``time_step`` parameter will be set to ``time_step``.

Parameter ``robot_context``:
    Optional Context of the MultibodyPlant. The position values of
    this context will be overwritten during integration; you only need
    to pass this in if the robot has any non-default parameters.
    $*Default:* ``robot.CreateDefaultContext()``.

Parameter ``log_only_when_result_state_changes``:
    is a boolean that determines whether the system will log on every
    differential IK failure, or only when the failure state changes.
    When the value is ``True``, it will cause the system to have an
    additional discrete state variable to store the most recent
    DifferentialInverseKinematicsStatus. Set this to ``False`` if you
    want IsDifferenceEquationSystem() to return ``True``.

Note: All references must remain valid for the lifetime of this
system.

Precondition:
    frame_E != frame_A.)""";
          // Source: drake/multibody/inverse_kinematics/differential_inverse_kinematics_integrator.h
          const char* doc_6args =
R"""(Constructs the system.

Parameter ``robot``:
    A MultibodyPlant describing the robot.

Parameter ``frame_E``:
    End-effector frame.

Parameter ``time_step``:
    the discrete time step of the (Euler) integration.

Parameter ``parameters``:
    Collection of various problem specific constraints and constants.
    The ``time_step`` parameter will be set to ``time_step``.

Parameter ``robot_context``:
    Optional Context of the MultibodyPlant. The position values of
    this context will be overwritten during integration; you only need
    to pass this in if the robot has any non-default parameters.
    $*Default:* ``robot.CreateDefaultContext()``.

Parameter ``log_only_when_result_state_changes``:
    is a boolean that determines whether the system will log on every
    differential IK failure, or only when the failure state changes.
    When the value is ``True``, it will cause the system to have an
    additional discrete state variable to store the most recent
    DifferentialInverseKinematicsStatus. Set this to ``False`` if you
    want IsDifferenceEquationSystem() to return ``True``.

In this overload, the reference frame, A, is taken to be the world
frame.

Note: All references must remain valid for the lifetime of this
system.

Precondition:
    frame_E != robot.world_frame().)""";
        } ctor;
        // Symbol: drake::multibody::DifferentialInverseKinematicsIntegrator::ForwardKinematics
        struct /* ForwardKinematics */ {
          // Source: drake/multibody/inverse_kinematics/differential_inverse_kinematics_integrator.h
          const char* doc =
R"""(Provides X_AE as a function of the joint position set in ``context``.)""";
        } ForwardKinematics;
        // Symbol: drake::multibody::DifferentialInverseKinematicsIntegrator::SetPositions
        struct /* SetPositions */ {
          // Source: drake/multibody/inverse_kinematics/differential_inverse_kinematics_integrator.h
          const char* doc =
R"""(Sets the joint positions, which are stored as state in the context. It
is recommended that the user calls this method to initialize the
position commands to match the initial positions of the robot.)""";
        } SetPositions;
        // Symbol: drake::multibody::DifferentialInverseKinematicsIntegrator::get_mutable_parameters
        struct /* get_mutable_parameters */ {
          // Source: drake/multibody/inverse_kinematics/differential_inverse_kinematics_integrator.h
          const char* doc =
R"""(Returns a mutable reference to the differential IK parameters owned by
this system.)""";
        } get_mutable_parameters;
        // Symbol: drake::multibody::DifferentialInverseKinematicsIntegrator::get_parameters
        struct /* get_parameters */ {
          // Source: drake/multibody/inverse_kinematics/differential_inverse_kinematics_integrator.h
          const char* doc =
R"""(Returns a const reference to the differential IK parameters owned by
this system.)""";
        } get_parameters;
      } DifferentialInverseKinematicsIntegrator;
      // Symbol: drake::multibody::DifferentialInverseKinematicsParameters
      struct /* DifferentialInverseKinematicsParameters */ {
        // Source: drake/multibody/inverse_kinematics/differential_inverse_kinematics.h
        const char* doc =
R"""(Contains parameters for the family of differential inverse kinematics
function overloads below, each named
DoDifferentialInverseKinematics().)""";
        // Symbol: drake::multibody::DifferentialInverseKinematicsParameters::AddLinearVelocityConstraint
        struct /* AddLinearVelocityConstraint */ {
          // Source: drake/multibody/inverse_kinematics/differential_inverse_kinematics.h
          const char* doc =
R"""(Adds a linear velocity constraint.

Parameter ``constraint``:
    A linear constraint on joint velocities.

Raises:
    RuntimeError if ``constraint->num_vars !=
    this->get_num_velocities()``.)""";
        } AddLinearVelocityConstraint;
        // Symbol: drake::multibody::DifferentialInverseKinematicsParameters::ClearLinearVelocityConstraints
        struct /* ClearLinearVelocityConstraints */ {
          // Source: drake/multibody/inverse_kinematics/differential_inverse_kinematics.h
          const char* doc = R"""(Clears all linear velocity constraints.)""";
        } ClearLinearVelocityConstraints;
        // Symbol: drake::multibody::DifferentialInverseKinematicsParameters::DifferentialInverseKinematicsParameters
        struct /* ctor */ {
          // Source: drake/multibody/inverse_kinematics/differential_inverse_kinematics.h
          const char* doc =
R"""(Constructor. Initializes the nominal joint position to zeros of size
``num_positions``. The time step is initialized to 1. The end effector
flags are initialized to True. The joint centering gains are
initialized to zero. All constraints are initialized to nullopt.

Parameter ``num_positions``:
    Number of generalized positions.

Parameter ``num_velocities``:
    Number of generalized velocities (by default it will be set to
    num_positions).)""";
        } ctor;
        // Symbol: drake::multibody::DifferentialInverseKinematicsParameters::get_end_effector_angular_speed_limit
        struct /* get_end_effector_angular_speed_limit */ {
          // Source: drake/multibody/inverse_kinematics/differential_inverse_kinematics.h
          const char* doc = R"""()""";
        } get_end_effector_angular_speed_limit;
        // Symbol: drake::multibody::DifferentialInverseKinematicsParameters::get_end_effector_translational_velocity_limits
        struct /* get_end_effector_translational_velocity_limits */ {
          // Source: drake/multibody/inverse_kinematics/differential_inverse_kinematics.h
          const char* doc = R"""()""";
        } get_end_effector_translational_velocity_limits;
        // Symbol: drake::multibody::DifferentialInverseKinematicsParameters::get_end_effector_velocity_flag
        struct /* get_end_effector_velocity_flag */ {
          // Source: drake/multibody/inverse_kinematics/differential_inverse_kinematics.h
          const char* doc = R"""()""";
        } get_end_effector_velocity_flag;
        // Symbol: drake::multibody::DifferentialInverseKinematicsParameters::get_joint_acceleration_limits
        struct /* get_joint_acceleration_limits */ {
          // Source: drake/multibody/inverse_kinematics/differential_inverse_kinematics.h
          const char* doc = R"""()""";
        } get_joint_acceleration_limits;
        // Symbol: drake::multibody::DifferentialInverseKinematicsParameters::get_joint_centering_gain
        struct /* get_joint_centering_gain */ {
          // Source: drake/multibody/inverse_kinematics/differential_inverse_kinematics.h
          const char* doc = R"""()""";
        } get_joint_centering_gain;
        // Symbol: drake::multibody::DifferentialInverseKinematicsParameters::get_joint_position_limits
        struct /* get_joint_position_limits */ {
          // Source: drake/multibody/inverse_kinematics/differential_inverse_kinematics.h
          const char* doc = R"""()""";
        } get_joint_position_limits;
        // Symbol: drake::multibody::DifferentialInverseKinematicsParameters::get_joint_velocity_limits
        struct /* get_joint_velocity_limits */ {
          // Source: drake/multibody/inverse_kinematics/differential_inverse_kinematics.h
          const char* doc = R"""()""";
        } get_joint_velocity_limits;
        // Symbol: drake::multibody::DifferentialInverseKinematicsParameters::get_linear_velocity_constraints
        struct /* get_linear_velocity_constraints */ {
          // Source: drake/multibody/inverse_kinematics/differential_inverse_kinematics.h
          const char* doc = R"""()""";
        } get_linear_velocity_constraints;
        // Symbol: drake::multibody::DifferentialInverseKinematicsParameters::get_maximum_scaling_to_report_stuck
        struct /* get_maximum_scaling_to_report_stuck */ {
          // Source: drake/multibody/inverse_kinematics/differential_inverse_kinematics.h
          const char* doc = R"""()""";
        } get_maximum_scaling_to_report_stuck;
        // Symbol: drake::multibody::DifferentialInverseKinematicsParameters::get_mutable_solver_options
        struct /* get_mutable_solver_options */ {
          // Source: drake/multibody/inverse_kinematics/differential_inverse_kinematics.h
          const char* doc =
R"""(Provides mutable access to change the solver options, e.g., to tune
for speed vs accuracy.)""";
        } get_mutable_solver_options;
        // Symbol: drake::multibody::DifferentialInverseKinematicsParameters::get_nominal_joint_position
        struct /* get_nominal_joint_position */ {
          // Source: drake/multibody/inverse_kinematics/differential_inverse_kinematics.h
          const char* doc = R"""()""";
        } get_nominal_joint_position;
        // Symbol: drake::multibody::DifferentialInverseKinematicsParameters::get_num_positions
        struct /* get_num_positions */ {
          // Source: drake/multibody/inverse_kinematics/differential_inverse_kinematics.h
          const char* doc = R"""()""";
        } get_num_positions;
        // Symbol: drake::multibody::DifferentialInverseKinematicsParameters::get_num_velocities
        struct /* get_num_velocities */ {
          // Source: drake/multibody/inverse_kinematics/differential_inverse_kinematics.h
          const char* doc = R"""()""";
        } get_num_velocities;
        // Symbol: drake::multibody::DifferentialInverseKinematicsParameters::get_solver_options
        struct /* get_solver_options */ {
          // Source: drake/multibody/inverse_kinematics/differential_inverse_kinematics.h
          const char* doc =
R"""(Provides const access to read the solver options.)""";
        } get_solver_options;
        // Symbol: drake::multibody::DifferentialInverseKinematicsParameters::get_time_step
        struct /* get_time_step */ {
          // Source: drake/multibody/inverse_kinematics/differential_inverse_kinematics.h
          const char* doc = R"""(@name Getters.)""";
        } get_time_step;
        // Symbol: drake::multibody::DifferentialInverseKinematicsParameters::set_end_effector_angular_speed_limit
        struct /* set_end_effector_angular_speed_limit */ {
          // Source: drake/multibody/inverse_kinematics/differential_inverse_kinematics.h
          const char* doc =
R"""(When calling DoDifferentialInverseKinematics with a desired
end-effector pose, this limits the magnitude of the angular velocity
vector.)""";
        } set_end_effector_angular_speed_limit;
        // Symbol: drake::multibody::DifferentialInverseKinematicsParameters::set_end_effector_translational_velocity_limits
        struct /* set_end_effector_translational_velocity_limits */ {
          // Source: drake/multibody/inverse_kinematics/differential_inverse_kinematics.h
          const char* doc =
R"""(When calling DoDifferentialInverseKinematics with a desired
end-effector pose, this sets limits on the translational velocity.)""";
        } set_end_effector_translational_velocity_limits;
        // Symbol: drake::multibody::DifferentialInverseKinematicsParameters::set_end_effector_velocity_flag
        struct /* set_end_effector_velocity_flag */ {
          // Source: drake/multibody/inverse_kinematics/differential_inverse_kinematics.h
          const char* doc =
R"""(Sets the end effector flags in the body frame. If a spatial velocity
flag is set to false, it will not be included in the differential IK
formulation.)""";
        } set_end_effector_velocity_flag;
        // Symbol: drake::multibody::DifferentialInverseKinematicsParameters::set_joint_acceleration_limits
        struct /* set_joint_acceleration_limits */ {
          // Source: drake/multibody/inverse_kinematics/differential_inverse_kinematics.h
          const char* doc =
R"""(Sets the joint acceleration limits.

Parameter ``vd_bounds``:
    The first element is the lower bound, and the second is the upper
    bound.

Raises:
    RuntimeError if the first or second element of ``q_bounds`` has
    the wrong dimension or any element of the second element is
    smaller than its corresponding part in the first element.)""";
        } set_joint_acceleration_limits;
        // Symbol: drake::multibody::DifferentialInverseKinematicsParameters::set_joint_centering_gain
        struct /* set_joint_centering_gain */ {
          // Source: drake/multibody/inverse_kinematics/differential_inverse_kinematics.h
          const char* doc =
R"""(Sets the joint centering gain, K, so that the joint centering command
is attempting to achieve v_next = N⁺(q) * K * (q_nominal - q_current).

Precondition:
    K must be num_positions x num_positions.)""";
        } set_joint_centering_gain;
        // Symbol: drake::multibody::DifferentialInverseKinematicsParameters::set_joint_position_limits
        struct /* set_joint_position_limits */ {
          // Source: drake/multibody/inverse_kinematics/differential_inverse_kinematics.h
          const char* doc =
R"""(Sets the joint position limits.

Parameter ``q_bounds``:
    The first element is the lower bound, and the second is the upper
    bound.

Raises:
    RuntimeError if the first or second element of ``q_bounds`` has
    the wrong dimension or any element of the second element is
    smaller than its corresponding part in the first element.)""";
        } set_joint_position_limits;
        // Symbol: drake::multibody::DifferentialInverseKinematicsParameters::set_joint_velocity_limits
        struct /* set_joint_velocity_limits */ {
          // Source: drake/multibody/inverse_kinematics/differential_inverse_kinematics.h
          const char* doc =
R"""(Sets the joint velocity limits.

Parameter ``v_bounds``:
    The first element is the lower bound, and the second is the upper
    bound.

Raises:
    RuntimeError if the first or second element of ``q_bounds`` has
    the wrong dimension or any element of the second element is
    smaller than its corresponding part in the first element.)""";
        } set_joint_velocity_limits;
        // Symbol: drake::multibody::DifferentialInverseKinematicsParameters::set_maximum_scaling_to_report_stuck
        struct /* set_maximum_scaling_to_report_stuck */ {
          // Source: drake/multibody/inverse_kinematics/differential_inverse_kinematics.h
          const char* doc =
R"""(Sets the threshold for α below which the status returned is
DifferentialInverseKinematicsStatus∷kStuck. α is the scaling of the
commanded spatial velocity, so when α is small, it means that the
actual spatial velocity magnitude will be small proportional to the
commanded.

*Default:* 0.01.)""";
        } set_maximum_scaling_to_report_stuck;
        // Symbol: drake::multibody::DifferentialInverseKinematicsParameters::set_nominal_joint_position
        struct /* set_nominal_joint_position */ {
          // Source: drake/multibody/inverse_kinematics/differential_inverse_kinematics.h
          const char* doc =
R"""(Sets the nominal joint position.

Raises:
    RuntimeError if ``nominal_joint_position``'s dimension differs.)""";
        } set_nominal_joint_position;
        // Symbol: drake::multibody::DifferentialInverseKinematicsParameters::set_time_step
        struct /* set_time_step */ {
          // Source: drake/multibody/inverse_kinematics/differential_inverse_kinematics.h
          const char* doc =
R"""(@name Setters. Sets time step to ``dt``.

Raises:
    RuntimeError if dt <= 0.)""";
        } set_time_step;
      } DifferentialInverseKinematicsParameters;
      // Symbol: drake::multibody::DifferentialInverseKinematicsResult
      struct /* DifferentialInverseKinematicsResult */ {
        // Source: drake/multibody/inverse_kinematics/differential_inverse_kinematics.h
        const char* doc = R"""()""";
        // Symbol: drake::multibody::DifferentialInverseKinematicsResult::joint_velocities
        struct /* joint_velocities */ {
          // Source: drake/multibody/inverse_kinematics/differential_inverse_kinematics.h
          const char* doc = R"""()""";
        } joint_velocities;
        // Symbol: drake::multibody::DifferentialInverseKinematicsResult::status
        struct /* status */ {
          // Source: drake/multibody/inverse_kinematics/differential_inverse_kinematics.h
          const char* doc = R"""()""";
        } status;
      } DifferentialInverseKinematicsResult;
      // Symbol: drake::multibody::DifferentialInverseKinematicsStatus
      struct /* DifferentialInverseKinematicsStatus */ {
        // Source: drake/multibody/inverse_kinematics/differential_inverse_kinematics.h
        const char* doc = R"""()""";
        // Symbol: drake::multibody::DifferentialInverseKinematicsStatus::kNoSolutionFound
        struct /* kNoSolutionFound */ {
          // Source: drake/multibody/inverse_kinematics/differential_inverse_kinematics.h
          const char* doc = R"""(Solver unable to find a solution.)""";
        } kNoSolutionFound;
        // Symbol: drake::multibody::DifferentialInverseKinematicsStatus::kSolutionFound
        struct /* kSolutionFound */ {
          // Source: drake/multibody/inverse_kinematics/differential_inverse_kinematics.h
          const char* doc = R"""(Found the optimal solution.)""";
        } kSolutionFound;
        // Symbol: drake::multibody::DifferentialInverseKinematicsStatus::kStuck
        struct /* kStuck */ {
          // Source: drake/multibody/inverse_kinematics/differential_inverse_kinematics.h
          const char* doc =
R"""(Unable to follow the desired velocity direction)""";
        } kStuck;
      } DifferentialInverseKinematicsStatus;
      // Symbol: drake::multibody::DifferentialInverseKinematicsSystem
      struct /* DifferentialInverseKinematicsSystem */ {
        // Source: drake/multibody/inverse_kinematics/differential_inverse_kinematics_system.h
        const char* doc =
R"""(The DifferentialInverseKinematicsSystem takes as input desired
cartesian poses (or cartesian velocities) for an arbitrary number of
"goal" frames on the robot, and produces a generalized velocity
command as output to move the goal frames toward the desired state.
This system is stateless, but is intended to be clocked at a known,
fixed time step Δt by evaluating its output port at integer multiples
of Δt.

The velocity command is computed by solving a MathematicalProgram
optimization problem, consisting of: - one primary objective
(LeastSquaresCost), - typically a secondary objective
(JointCenteringCost) to resolve the nullspace within the primary
objective, and - various optional constraints: -
CartesianPositionLimitConstraint - CartesianVelocityLimitConstraint -
CollisionConstraint - JointVelocityLimitConstraint

In brief, we solve for ``v_next`` such that ``Jv_TGs * v_next`` is
close to ``Vd_TGs`` subject to the constraints, where: - v_next is the
generalized velocity command on the output port, which has the
dimension of the number of active degrees of freedom, - Vd_TGs is the
desired spatial velocities of the goal frames (when desired positions
are input, the desired velocity is inferred using the difference from
the current position vs the time step), - Jv_TGs is the jacobian
relating spatial velocities to generalized velocities, i.e., V_TGs
(rows) with respect to v_active (cols) -- v_active is the subset of
generalized velocities for the active degrees of freedom.

For an introduction to differential inverse kinematics via
optimization, see section 10.6 of:
https://manipulation.csail.mit.edu/pick.html#diff_ik_w_constraints

.. pydrake_system::

    name: DifferentialInverseKinematicsSystem
    input_ports:
    - position
    - nominal_posture
    - desired_cartesian_velocities (optional)
    - desired_cartesian_poses (optional)
    output_ports:
    - commanded_velocity

Port ``position`` accepts the current generalized position (for the
full ``plant``, not just the active dofs).

Port ``desired_cartesian_velocities`` accepts desired cartesian
velocities, typed as systems∷BusValue where key is the name of the
frame to track and the value is the SpatialVelocity<double> w.r.t the
task frame. Frame names should be provided as fully-scoped names
(``model_instance∷frame``).

Port ``desired_cartesian_poses`` accepts desired cartesian poses,
typed as systems∷BusValue where key is the name of the frame to track
and the value is the math∷RigidTransformd spatial pose w.r.t the task
frame. Frame names should be provided as fully-scoped names
(``model_instance∷frame``).

Port ``nominal_posture`` accepts a generalized position to be used to
handle nullspace resolution; this has the dimension of the full
degrees of freedom (not just active dofs). Typical choices for setting
this input would be to use a constant "home" reference posture, or to
use a dynamically changing recent posture.

Port ``commanded_velocity`` emits generalized velocity that realizes
the desired cartesian velocities or poses within the constraints. This
has the dimension of the number of active degrees of freedom.

Either ``desired_cartesian_velocities`` or ``desired_cartesian_poses``
must be connected. Connecting both ports will result in an exception.

Note:
    There is no consistency check to ensure that the frames being
    tracked are the same across multiple time steps.

Warning:
    This class only works correctly when the plant has v = q̇.

= Notation =

The implementation uses "monogram notation" abbreviations throughout.
See
https://drake.mit.edu/doxygen_cxx/group__multibody__quantities.html
for details. The relevant frame names are: - B: base frame - Gi: the
i'th goal frame (per the desired_cartesian_... input port) - T: task
frame - W: world frame

To denote desired spatial velocities, we use a "d" suffix (i.e.,
"Vd"). Quantities like Vd_TGi (and therefore also Vd_TGlist and
Vd_TGs) refer to the desired velocity, not the current velocity. Other
quantities without the "d" subscript (e.g., X_TGi, Jv_TGi, etc.) refer
to the current kinematics.

When 's' is used as a suffix (e.g., 'Vd_TGs'), it refers to the stack
of all goals one after another, e.g., Vd_TGs refers to the
concatenation of all Vd_TGi in Vd_TGlist. You can think of the 's'
either as an abbreviation for "stack" or as a plural.

We also use the letter 'S' to refer a "multibody system", in our case
the robot portion of the controller plant (not including the
environment), as defined by the collision_checker. For example, use
the notation 'Scm' to denote the robot's center of mass.)""";
        // Symbol: drake::multibody::DifferentialInverseKinematicsSystem::CallbackDetails
        struct /* CallbackDetails */ {
          // Source: drake/multibody/inverse_kinematics/differential_inverse_kinematics_system.h
          const char* doc =
R"""((Internal use only) A group of common arguments relevant to multiple
different costs and constraints within the
DifferentialInverseKinematicsSystem program formulation. Think of this
struct as a short-lived pack of function arguments that is set once
and then processed by multiple helper functions; it is not intended to
be a long-lived abstract data type.)""";
          // Symbol: drake::multibody::DifferentialInverseKinematicsSystem::CallbackDetails::Jv_TGs
          struct /* Jv_TGs */ {
            // Source: drake/multibody/inverse_kinematics/differential_inverse_kinematics_system.h
            const char* doc =
R"""(The jacobian relating spatial velocities to generalized velocities,
i.e., V_TGs (rows) with respect to v_active (cols).)""";
          } Jv_TGs;
          // Symbol: drake::multibody::DifferentialInverseKinematicsSystem::CallbackDetails::Vd_TGlist
          struct /* Vd_TGlist */ {
            // Source: drake/multibody/inverse_kinematics/differential_inverse_kinematics_system.h
            const char* doc =
R"""(The desired velocities of the goal frames.)""";
          } Vd_TGlist;
          // Symbol: drake::multibody::DifferentialInverseKinematicsSystem::CallbackDetails::X_TGlist
          struct /* X_TGlist */ {
            // Source: drake/multibody/inverse_kinematics/differential_inverse_kinematics_system.h
            const char* doc = R"""(The current poses of the goal frames.)""";
          } X_TGlist;
          // Symbol: drake::multibody::DifferentialInverseKinematicsSystem::CallbackDetails::active_dof
          struct /* active_dof */ {
            // Source: drake/multibody/inverse_kinematics/differential_inverse_kinematics_system.h
            const char* doc =
R"""(The active degrees of freedom in ``collision_checker.plant()``.)""";
          } active_dof;
          // Symbol: drake::multibody::DifferentialInverseKinematicsSystem::CallbackDetails::collision_checker
          struct /* collision_checker */ {
            // Source: drake/multibody/inverse_kinematics/differential_inverse_kinematics_system.h
            const char* doc =
R"""(The collision checker for the robot being controlled. Note that its
robot_model_instances() accessor also partitions which parts of the
``plant`` are the robot model vs its environment.)""";
          } collision_checker;
          // Symbol: drake::multibody::DifferentialInverseKinematicsSystem::CallbackDetails::collision_checker_context
          struct /* collision_checker_context */ {
            // Source: drake/multibody/inverse_kinematics/differential_inverse_kinematics_system.h
            const char* doc =
R"""(A mutable context for the collision checker.)""";
          } collision_checker_context;
          // Symbol: drake::multibody::DifferentialInverseKinematicsSystem::CallbackDetails::frame_list
          struct /* frame_list */ {
            // Source: drake/multibody/inverse_kinematics/differential_inverse_kinematics_system.h
            const char* doc = R"""(The list of frames being controlled.)""";
          } frame_list;
          // Symbol: drake::multibody::DifferentialInverseKinematicsSystem::CallbackDetails::mathematical_program
          struct /* mathematical_program */ {
            // Source: drake/multibody/inverse_kinematics/differential_inverse_kinematics_system.h
            const char* doc =
R"""(The mutable, work-in-progress optimization program.)""";
          } mathematical_program;
          // Symbol: drake::multibody::DifferentialInverseKinematicsSystem::CallbackDetails::nominal_posture
          struct /* nominal_posture */ {
            // Source: drake/multibody/inverse_kinematics/differential_inverse_kinematics_system.h
            const char* doc =
R"""(The current value of the ``nominal_posture`` input port. This has the
dimension of the full degrees of freedom (not just active dofs).)""";
          } nominal_posture;
          // Symbol: drake::multibody::DifferentialInverseKinematicsSystem::CallbackDetails::plant_context
          struct /* plant_context */ {
            // Source: drake/multibody/inverse_kinematics/differential_inverse_kinematics_system.h
            const char* doc =
R"""(A context for the control plant, set to current positions. (At the
moment, velocities are zero but that might change down the road.))""";
          } plant_context;
          // Symbol: drake::multibody::DifferentialInverseKinematicsSystem::CallbackDetails::time_step
          struct /* time_step */ {
            // Source: drake/multibody/inverse_kinematics/differential_inverse_kinematics_system.h
            const char* doc =
R"""(The control rate for DifferentialInverseKinematicsSystem (the pace at
which velocity commands are expected to be applied).)""";
          } time_step;
          // Symbol: drake::multibody::DifferentialInverseKinematicsSystem::CallbackDetails::v_next
          struct /* v_next */ {
            // Source: drake/multibody/inverse_kinematics/differential_inverse_kinematics_system.h
            const char* doc =
R"""(The decision variables being optimized. This has the dimension of the
number of active degrees of freedom (see ``active_dof``).)""";
          } v_next;
        } CallbackDetails;
        // Symbol: drake::multibody::DifferentialInverseKinematicsSystem::CartesianPositionLimitConstraint
        struct /* CartesianPositionLimitConstraint */ {
          // Source: drake/multibody/inverse_kinematics/differential_inverse_kinematics_system.h
          const char* doc =
R"""(Constrains the goal frames to a cartesian bounding box: ``∀i
p_TG_next_lower ≤ p_TGi + Jv_TGi[3:6] * v_next * Δt ≤
p_TG_next_upper`` where: - p_TGi is the translation component of the
i'th goal point w.r.t the task frame.)""";
          // Symbol: drake::multibody::DifferentialInverseKinematicsSystem::CartesianPositionLimitConstraint::AddToProgram
          struct /* AddToProgram */ {
            // Source: drake/multibody/inverse_kinematics/differential_inverse_kinematics_system.h
            const char* doc = R"""()""";
          } AddToProgram;
          // Symbol: drake::multibody::DifferentialInverseKinematicsSystem::CartesianPositionLimitConstraint::CartesianPositionLimitConstraint
          struct /* ctor */ {
            // Source: drake/multibody/inverse_kinematics/differential_inverse_kinematics_system.h
            const char* doc = R"""()""";
          } ctor;
          // Symbol: drake::multibody::DifferentialInverseKinematicsSystem::CartesianPositionLimitConstraint::Config
          struct /* Config */ {
            // Source: drake/multibody/inverse_kinematics/differential_inverse_kinematics_system.h
            const char* doc = R"""()""";
            // Symbol: drake::multibody::DifferentialInverseKinematicsSystem::CartesianPositionLimitConstraint::Config::Serialize
            struct /* Serialize */ {
              // Source: drake/multibody/inverse_kinematics/differential_inverse_kinematics_system.h
              const char* doc =
R"""(Passes this object to an Archive. Refer to yaml_serialization "YAML
Serialization" for background.)""";
            } Serialize;
            // Symbol: drake::multibody::DifferentialInverseKinematicsSystem::CartesianPositionLimitConstraint::Config::p_TG_next_lower
            struct /* p_TG_next_lower */ {
              // Source: drake/multibody/inverse_kinematics/differential_inverse_kinematics_system.h
              const char* doc = R"""(Lower bound on p_TGi for all i.)""";
            } p_TG_next_lower;
            // Symbol: drake::multibody::DifferentialInverseKinematicsSystem::CartesianPositionLimitConstraint::Config::p_TG_next_upper
            struct /* p_TG_next_upper */ {
              // Source: drake/multibody/inverse_kinematics/differential_inverse_kinematics_system.h
              const char* doc = R"""(Upper bound on p_TGi for all i.)""";
            } p_TG_next_upper;
            auto Serialize__fields() const {
              return std::array{
                std::make_pair("p_TG_next_lower", p_TG_next_lower.doc),
                std::make_pair("p_TG_next_upper", p_TG_next_upper.doc),
              };
            }
          } Config;
          // Symbol: drake::multibody::DifferentialInverseKinematicsSystem::CartesianPositionLimitConstraint::GetConfig
          struct /* GetConfig */ {
            // Source: drake/multibody/inverse_kinematics/differential_inverse_kinematics_system.h
            const char* doc = R"""(Returns the current config.)""";
          } GetConfig;
          // Symbol: drake::multibody::DifferentialInverseKinematicsSystem::CartesianPositionLimitConstraint::SetConfig
          struct /* SetConfig */ {
            // Source: drake/multibody/inverse_kinematics/differential_inverse_kinematics_system.h
            const char* doc =
R"""(Replaces the config set in the constructor.)""";
          } SetConfig;
        } CartesianPositionLimitConstraint;
        // Symbol: drake::multibody::DifferentialInverseKinematicsSystem::CartesianVelocityLimitConstraint
        struct /* CartesianVelocityLimitConstraint */ {
          // Source: drake/multibody/inverse_kinematics/differential_inverse_kinematics_system.h
          const char* doc =
R"""(Constrains the spatial velocities of the goal frames: ``∀i, ∀j ∈ [0,
5]: abs(Jv_TGi * v_next)[j] ≤ V_next_TG_limit[j]``.)""";
          // Symbol: drake::multibody::DifferentialInverseKinematicsSystem::CartesianVelocityLimitConstraint::AddToProgram
          struct /* AddToProgram */ {
            // Source: drake/multibody/inverse_kinematics/differential_inverse_kinematics_system.h
            const char* doc = R"""()""";
          } AddToProgram;
          // Symbol: drake::multibody::DifferentialInverseKinematicsSystem::CartesianVelocityLimitConstraint::CartesianVelocityLimitConstraint
          struct /* ctor */ {
            // Source: drake/multibody/inverse_kinematics/differential_inverse_kinematics_system.h
            const char* doc = R"""()""";
          } ctor;
          // Symbol: drake::multibody::DifferentialInverseKinematicsSystem::CartesianVelocityLimitConstraint::Config
          struct /* Config */ {
            // Source: drake/multibody/inverse_kinematics/differential_inverse_kinematics_system.h
            const char* doc = R"""()""";
            // Symbol: drake::multibody::DifferentialInverseKinematicsSystem::CartesianVelocityLimitConstraint::Config::Serialize
            struct /* Serialize */ {
              // Source: drake/multibody/inverse_kinematics/differential_inverse_kinematics_system.h
              const char* doc =
R"""(Passes this object to an Archive. Refer to yaml_serialization "YAML
Serialization" for background.)""";
            } Serialize;
            // Symbol: drake::multibody::DifferentialInverseKinematicsSystem::CartesianVelocityLimitConstraint::Config::V_next_TG_limit
            struct /* V_next_TG_limit */ {
              // Source: drake/multibody/inverse_kinematics/differential_inverse_kinematics_system.h
              const char* doc =
R"""(This limits the absolute value of the measures of the spatial
velocities V_next[i] for all goal frames, so must be entry-wise
non-negative. If the "desired_cartesian_poses" input port to
DifferentialInverseKinematicsSystem is in use, then typically this
should be set to the same value as the Vd_TG_limit passed to that
system's constructor. The element order is [ωx, ωy, ωz, vx, vy, vz],
which matches the SpatialVelocity order.)""";
            } V_next_TG_limit;
            auto Serialize__fields() const {
              return std::array{
                std::make_pair("V_next_TG_limit", V_next_TG_limit.doc),
              };
            }
          } Config;
          // Symbol: drake::multibody::DifferentialInverseKinematicsSystem::CartesianVelocityLimitConstraint::GetConfig
          struct /* GetConfig */ {
            // Source: drake/multibody/inverse_kinematics/differential_inverse_kinematics_system.h
            const char* doc = R"""(Returns the current config.)""";
          } GetConfig;
          // Symbol: drake::multibody::DifferentialInverseKinematicsSystem::CartesianVelocityLimitConstraint::SetConfig
          struct /* SetConfig */ {
            // Source: drake/multibody/inverse_kinematics/differential_inverse_kinematics_system.h
            const char* doc =
R"""(Replaces the config set in the constructor.)""";
          } SetConfig;
        } CartesianVelocityLimitConstraint;
        // Symbol: drake::multibody::DifferentialInverseKinematicsSystem::CollisionConstraint
        struct /* CollisionConstraint */ {
          // Source: drake/multibody/inverse_kinematics/differential_inverse_kinematics_system.h
          const char* doc =
R"""(Constrains the collision clearance around the robot to remain above
the safety distance: ``∀j ϕₛ ≤ ϕⱼ + ∂ϕⱼ/∂q_active * v_next * Δt``
where: - ϕₛ is the safety_distance; - ϕⱼ is the current signed
distance between the robot and some j'th obstacle; - ∂ϕⱼ/∂q_active is
the gradient of ϕⱼ with respect to the active dof positions. Obstacles
beyond the influence distance are ignored. The optional
select_data_for_collision_constraint may be used to preprocess the
clearance data prior to adding the constraint (e.g., to ignore some
parts).

TODO(jeremy-nimmer) This constraint should account for passive dof
velocities as well (i.e., v_passive), using the full ∂q (not just
∂q_active).)""";
          // Symbol: drake::multibody::DifferentialInverseKinematicsSystem::CollisionConstraint::AddToProgram
          struct /* AddToProgram */ {
            // Source: drake/multibody/inverse_kinematics/differential_inverse_kinematics_system.h
            const char* doc = R"""()""";
          } AddToProgram;
          // Symbol: drake::multibody::DifferentialInverseKinematicsSystem::CollisionConstraint::CollisionConstraint
          struct /* ctor */ {
            // Source: drake/multibody/inverse_kinematics/differential_inverse_kinematics_system.h
            const char* doc = R"""()""";
          } ctor;
          // Symbol: drake::multibody::DifferentialInverseKinematicsSystem::CollisionConstraint::Config
          struct /* Config */ {
            // Source: drake/multibody/inverse_kinematics/differential_inverse_kinematics_system.h
            const char* doc = R"""()""";
            // Symbol: drake::multibody::DifferentialInverseKinematicsSystem::CollisionConstraint::Config::Serialize
            struct /* Serialize */ {
              // Source: drake/multibody/inverse_kinematics/differential_inverse_kinematics_system.h
              const char* doc =
R"""(Passes this object to an Archive. Refer to yaml_serialization "YAML
Serialization" for background.)""";
            } Serialize;
            // Symbol: drake::multibody::DifferentialInverseKinematicsSystem::CollisionConstraint::Config::influence_distance
            struct /* influence_distance */ {
              // Source: drake/multibody/inverse_kinematics/differential_inverse_kinematics_system.h
              const char* doc =
R"""(Obstacles beyond the influence distance (in meters) are ignored. Must
be non-negative.)""";
            } influence_distance;
            // Symbol: drake::multibody::DifferentialInverseKinematicsSystem::CollisionConstraint::Config::safety_distance
            struct /* safety_distance */ {
              // Source: drake/multibody/inverse_kinematics/differential_inverse_kinematics_system.h
              const char* doc =
R"""("ϕₛ" in the class overview (in meters). Must be finite.)""";
            } safety_distance;
            auto Serialize__fields() const {
              return std::array{
                std::make_pair("influence_distance", influence_distance.doc),
                std::make_pair("safety_distance", safety_distance.doc),
              };
            }
          } Config;
          // Symbol: drake::multibody::DifferentialInverseKinematicsSystem::CollisionConstraint::GetConfig
          struct /* GetConfig */ {
            // Source: drake/multibody/inverse_kinematics/differential_inverse_kinematics_system.h
            const char* doc = R"""(Returns the current config.)""";
          } GetConfig;
          // Symbol: drake::multibody::DifferentialInverseKinematicsSystem::CollisionConstraint::SetConfig
          struct /* SetConfig */ {
            // Source: drake/multibody/inverse_kinematics/differential_inverse_kinematics_system.h
            const char* doc =
R"""(Replaces the config set in the constructor.)""";
          } SetConfig;
          // Symbol: drake::multibody::DifferentialInverseKinematicsSystem::CollisionConstraint::SetSelectDataForCollisionConstraintFunction
          struct /* SetSelectDataForCollisionConstraintFunction */ {
            // Source: drake/multibody/inverse_kinematics/differential_inverse_kinematics_system.h
            const char* doc =
R"""((Advanced) Provides a mechanism for ignoring certain clearance rows.)""";
          } SetSelectDataForCollisionConstraintFunction;
        } CollisionConstraint;
        // Symbol: drake::multibody::DifferentialInverseKinematicsSystem::DifferentialInverseKinematicsSystem
        struct /* ctor */ {
          // Source: drake/multibody/inverse_kinematics/differential_inverse_kinematics_system.h
          const char* doc =
R"""((Advanced) Constructs the DifferentialInverseKinematicsSystem with a
user-provided recipe for the mathematical program formulation.

Parameter ``recipe``:
    Specifies the mathematical formation and its parameters.

Parameter ``task_frame``:
    Specifies the task frame name (i.e., "T") that cartesian goal
    poses are expressed in. Frame names should be provided as
    fully-scoped names (``model_instance∷frame``). This must be a
    frame in ``collision_checker.plant()``.

Parameter ``collision_checker``:
    Specifies the plant and collision model to use.

Parameter ``active_dof``:
    Specifies which generalized velocities of the plant are commanded
    by this system and appear on the "commanded_velocity" output port.

Parameter ``time_step``:
    Specifies Δt, the time horizon assumed when relating position
    values to velocity values. The output commands from this system
    should be sent to the robot at this period.

Parameter ``K_VX``:
    Specifies a scale factor used to convert
    ``desired_cartesian_poses`` input to velocities. The i'th desired
    cartesian velocity is computed as: ``Vd_TGi = K_VX *
    (X_TGi_requested - X_TGi_current) / Δt`` A typical value is 1.0 so
    that the desired velocity exactly matches what's necessary to move
    to the request pose in the next step, but may be tuned smaller
    (e.g., 0.5) to mitigate overshoot. This value is not used when the
    ``desired_cartesian_velocities`` input port is being used for
    commands.

Parameter ``Vd_TG_limit``:
    A clamping limit applied to desired cartesian velocities. All
    desired cartesian velocities (whether specified explicitly on the
    ``desired_cartesian_velocities`` input port, or inferred from the
    ``desired_cartesian_poses`` input port) are "clamped"; the
    absolute value of each measure in the clamped spatial velocity
    will be no greater than the corresponding measure in Vd_TG_limit.
    This *clamped* velocity becomes the input to the optimization
    problem. (NOTE: Clamping for ``desired_cartesian_velocities`` is
    not yet implemented, but will be soon.) If the
    CartesianVelocityLimitConstraint is in use, typically this value
    should be the same as that ingredient's V_next_TG_limit.)""";
        } ctor;
        // Symbol: drake::multibody::DifferentialInverseKinematicsSystem::Ingredient
        struct /* Ingredient */ {
          // Source: drake/multibody/inverse_kinematics/differential_inverse_kinematics_system.h
          const char* doc =
R"""((Internal use only) A user-provided set of constraint(s) and/or
cost(s) for a DifferentialInverseKinematicsSystem recipe, to allow for
user customization of the mathematical program formulation.

TODO(jeremy-nimmer) In the future, we should remove "internal use
only" so that users can officially bake their own recipes for
DifferentialInverseKinematicsSystem.)""";
          // Symbol: drake::multibody::DifferentialInverseKinematicsSystem::Ingredient::AddToProgram
          struct /* AddToProgram */ {
            // Source: drake/multibody/inverse_kinematics/differential_inverse_kinematics_system.h
            const char* doc =
R"""(Adds this ingredient into DifferentialInverseKinematicsSystem's
mathematical program, returning the listing of bindings for all
newly-added costs and/or constraints.

Parameter ``details``:
    A group of arguments commonly used by most costs and constraints.)""";
          } AddToProgram;
          // Symbol: drake::multibody::DifferentialInverseKinematicsSystem::Ingredient::BuildBlockDiagonalAxisSelector
          struct /* BuildBlockDiagonalAxisSelector */ {
            // Source: drake/multibody/inverse_kinematics/differential_inverse_kinematics_system.h
            const char* doc =
R"""(Constructs a block-diagonal matrix selecting Cartesian velocity axes
for each goal frame, based on per-frame axis masks. This operation is
common to several ingredients so is offered as a utility function this
base class.

Each controlled frame Gi has a 6×6 diagonal mask matrix selecting
which spatial velocity components (angular, linear) are active. These
individual mask matrices are stacked into a single block-diagonal
matrix for use in constraints and cost terms.

For example, given N goal frames, the output matrix has size 6N × 6N
and has the form:

[ diag(mask_G₁) 0 ... 0 ] [ 0 diag(mask_G₂) ... 0 ] [ ... ... ... ...
] [ 0 0 ... diag(mask_Gₙ)]

Parameter ``frame_list``:
    The list of controlled frames {Gi}, borrowed from the plant.

Parameter ``cartesian_axis_masks``:
    Map from fully-scoped frame names to their 6D axis mask.

Returns:
    6N × 6N block-diagonal matrix applying the per-frame axis mask.)""";
          } BuildBlockDiagonalAxisSelector;
          // Symbol: drake::multibody::DifferentialInverseKinematicsSystem::Ingredient::Ingredient
          struct /* ctor */ {
            // Source: drake/multibody/inverse_kinematics/differential_inverse_kinematics_system.h
            const char* doc = R"""()""";
          } ctor;
        } Ingredient;
        // Symbol: drake::multibody::DifferentialInverseKinematicsSystem::JointCenteringCost
        struct /* JointCenteringCost */ {
          // Source: drake/multibody/inverse_kinematics/differential_inverse_kinematics_system.h
          const char* doc =
R"""(Provides a secondary minimization objective. There will almost
inevitably be times when the jacobian ``Jv_TGs`` is not full rank.
Sometimes because of current position values and sometimes because
``Jv_TGs`` is rectangular by construction. At those times, there's no
longer a single solution and relying on the mathematical program to
pick a reasonable solution from the large space is unreasonable.

This cost is intended to work in conjunction with the primary cost
(e.g., LeastSquaresCost). When ``Jv_TGs`` is not full rank, this
provides guidance for selecting a *unique* velocity from the space of
possible solutions. If the primary cost produces a space of optimal
velocities, this secondary cost will select the velocity from that
space that brings the arm closer to the "nominal posture":

|P⋅(v_next - K⋅(nominal_posture_active - q_active))|²

where, - ``K`` is the proportional gain of a joint-space controller
that pulls toward the nominal posture. - ``P`` is the linear map from
generalized velocities to the *null space* of masked Jacobian ``S ⋅
Jv_TGs``. Mapping to the null space, allows refinement of the velocity
without changing the primary cost. - S is a block-diagonal matrix of
axis masks, enabling per-axis tracking.

Notes: - When combined with a primary cost, the primary cost should be
weighed far more heavily to keep this secondary cost from interfering.
A factor of 100X or so is advisable. (See
LeastSqauresCost∷cartesian_qp_weight for tuning the gain on the
primary objective.) - For this cost to behave as expected, it is
critical that the null space of ``S ⋅ Jv_TGs`` be a subspace of the
null space used by the primary cost. The simplest way is to make sure
both costs receive the same axis masks and jacobian. Failure to do so
would lead this cost to *fight* the primary cost instead of
complementing it.

For more details on this cost, see:
https://manipulation.csail.mit.edu/pick.html#diff_ik_w_constraints#joint_centering

TODO(sean.curtis) As with the other costs, this should also have a
scaling weight (e.g. what ``G`` is for LeastSquaresCost).)""";
          // Symbol: drake::multibody::DifferentialInverseKinematicsSystem::JointCenteringCost::AddToProgram
          struct /* AddToProgram */ {
            // Source: drake/multibody/inverse_kinematics/differential_inverse_kinematics_system.h
            const char* doc = R"""()""";
          } AddToProgram;
          // Symbol: drake::multibody::DifferentialInverseKinematicsSystem::JointCenteringCost::Config
          struct /* Config */ {
            // Source: drake/multibody/inverse_kinematics/differential_inverse_kinematics_system.h
            const char* doc = R"""()""";
            // Symbol: drake::multibody::DifferentialInverseKinematicsSystem::JointCenteringCost::Config::Serialize
            struct /* Serialize */ {
              // Source: drake/multibody/inverse_kinematics/differential_inverse_kinematics_system.h
              const char* doc =
R"""(Passes this object to an Archive. Refer to yaml_serialization "YAML
Serialization" for background.)""";
            } Serialize;
            // Symbol: drake::multibody::DifferentialInverseKinematicsSystem::JointCenteringCost::Config::cartesian_axis_masks
            struct /* cartesian_axis_masks */ {
              // Source: drake/multibody/inverse_kinematics/differential_inverse_kinematics_system.h
              const char* doc =
R"""(Map from fully-scoped frame names to their 6D spatial velocity axis
mask. Each Vector6d is a binary mask [ωx, ωy, ωz, vx, vy, vz]
indicating which Cartesian velocity components (angular and
translational) are being tracked. All elements must be set to either 0
or 1, and at least one element must be 1.)""";
            } cartesian_axis_masks;
            // Symbol: drake::multibody::DifferentialInverseKinematicsSystem::JointCenteringCost::Config::posture_gain
            struct /* posture_gain */ {
              // Source: drake/multibody/inverse_kinematics/differential_inverse_kinematics_system.h
              const char* doc =
R"""(The proportional gain matrix ``K`` is a diagonal matrix with the
diagonal set to this value (i.e., all joints use the same
``posture_gain``). Must be non-negative.)""";
            } posture_gain;
            auto Serialize__fields() const {
              return std::array{
                std::make_pair("cartesian_axis_masks", cartesian_axis_masks.doc),
                std::make_pair("posture_gain", posture_gain.doc),
              };
            }
          } Config;
          // Symbol: drake::multibody::DifferentialInverseKinematicsSystem::JointCenteringCost::GetConfig
          struct /* GetConfig */ {
            // Source: drake/multibody/inverse_kinematics/differential_inverse_kinematics_system.h
            const char* doc = R"""(Returns the current config.)""";
          } GetConfig;
          // Symbol: drake::multibody::DifferentialInverseKinematicsSystem::JointCenteringCost::JointCenteringCost
          struct /* ctor */ {
            // Source: drake/multibody/inverse_kinematics/differential_inverse_kinematics_system.h
            const char* doc = R"""()""";
          } ctor;
          // Symbol: drake::multibody::DifferentialInverseKinematicsSystem::JointCenteringCost::SetConfig
          struct /* SetConfig */ {
            // Source: drake/multibody/inverse_kinematics/differential_inverse_kinematics_system.h
            const char* doc =
R"""(Replaces the config set in the constructor.)""";
          } SetConfig;
        } JointCenteringCost;
        // Symbol: drake::multibody::DifferentialInverseKinematicsSystem::JointVelocityLimitConstraint
        struct /* JointVelocityLimitConstraint */ {
          // Source: drake/multibody/inverse_kinematics/differential_inverse_kinematics_system.h
          const char* doc =
R"""(Constrains the generalized velocity to prevent a commanded velocity
that would push the generalized position outside its limits.

The configured joint velocity limits are not used directly. When the
joint position is near a limit, the joint velocity limit may be
insufficient to constrain ``v_next`` from pushing the position beyond
its limits.

Instead, we use a *scaled* velocity limit. In simple terms, the closer
``q`` is to its position limit, the more velocity towards that limit
is constrained. When ``q`` lies at the limit boundary, the velocity in
that direction would be zero. However, the configured velocity limit
should be applied when q is "sufficiently" far away from the position
limit boundary. So, we compute and apply a scale factor to attenuate
the "near" velocity limit.

The constraint is parameterized to define the domain of the
attenuation; how far away from the near limit boundary should the
scaled limit be zero and how far should it be restored to its
configured value? We parameterize those two distances as:

- ``min_margin``: the distance at which the velocity limit is scaled to be zero.
Must be non-negative and is typically positive.
- ``influence_margin``: the distance at which the velocity limit is
restored to its configured value. This value must be
strictly greater than ``min_margin``.

The velocity limit is only ever attenuated on one boundary.

If we define ``distᵢ`` as the distance to the near position limit
boundary, and ``near_limit`` as the value of that limit, we can define
the attenuation as:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    distᵢ = min(qᵢ_current - qᵢ_min, qᵢ_max - qᵢ_current)
    scale = clamp((distᵢ - min_margin) / (influence_margin - min_margin), 0, 1)
    scaled_near_limit = near_limit * scale.

.. raw:: html

    </details>

Implications: - Because we're *attenuating* the configured velocity
limits, an infinite limit value won't be attenuated. It *will* be zero
when ``distᵢ ≤ min_margin``. - Position limits define an interval for
q, which we'll call P. If ``P / 2 - min_margin < influence_margin -
min_margin`` then one limit will *always* be attenuated to be less
than its configured value.)""";
          // Symbol: drake::multibody::DifferentialInverseKinematicsSystem::JointVelocityLimitConstraint::AddToProgram
          struct /* AddToProgram */ {
            // Source: drake/multibody/inverse_kinematics/differential_inverse_kinematics_system.h
            const char* doc = R"""()""";
          } AddToProgram;
          // Symbol: drake::multibody::DifferentialInverseKinematicsSystem::JointVelocityLimitConstraint::Config
          struct /* Config */ {
            // Source: drake/multibody/inverse_kinematics/differential_inverse_kinematics_system.h
            const char* doc = R"""()""";
            // Symbol: drake::multibody::DifferentialInverseKinematicsSystem::JointVelocityLimitConstraint::Config::Serialize
            struct /* Serialize */ {
              // Source: drake/multibody/inverse_kinematics/differential_inverse_kinematics_system.h
              const char* doc =
R"""(Passes this object to an Archive. Refer to yaml_serialization "YAML
Serialization" for background.)""";
            } Serialize;
            // Symbol: drake::multibody::DifferentialInverseKinematicsSystem::JointVelocityLimitConstraint::Config::influence_margin
            struct /* influence_margin */ {
              // Source: drake/multibody/inverse_kinematics/differential_inverse_kinematics_system.h
              const char* doc =
R"""(The distance (in each joint's configuration space) at which the
velocity limit is restored to its configured value. The units will
depend on what kind of joint is being limited. This value must be
strictly greater than ``min_margin``.)""";
            } influence_margin;
            // Symbol: drake::multibody::DifferentialInverseKinematicsSystem::JointVelocityLimitConstraint::Config::min_margin
            struct /* min_margin */ {
              // Source: drake/multibody/inverse_kinematics/differential_inverse_kinematics_system.h
              const char* doc =
R"""(The distance (in each joint's configuration space) at which the
velocity limit is scaled to be zero. Must be non-negative and is
typically positive. The units will depend on what kind of joint is
being limited.)""";
            } min_margin;
            auto Serialize__fields() const {
              return std::array{
                std::make_pair("influence_margin", influence_margin.doc),
                std::make_pair("min_margin", min_margin.doc),
              };
            }
          } Config;
          // Symbol: drake::multibody::DifferentialInverseKinematicsSystem::JointVelocityLimitConstraint::GetConfig
          struct /* GetConfig */ {
            // Source: drake/multibody/inverse_kinematics/differential_inverse_kinematics_system.h
            const char* doc = R"""(Returns the current config.)""";
          } GetConfig;
          // Symbol: drake::multibody::DifferentialInverseKinematicsSystem::JointVelocityLimitConstraint::GetJointLimits
          struct /* GetJointLimits */ {
            // Source: drake/multibody/inverse_kinematics/differential_inverse_kinematics_system.h
            const char* doc = R"""(Returns the current joint limits.)""";
          } GetJointLimits;
          // Symbol: drake::multibody::DifferentialInverseKinematicsSystem::JointVelocityLimitConstraint::JointVelocityLimitConstraint
          struct /* ctor */ {
            // Source: drake/multibody/inverse_kinematics/differential_inverse_kinematics_system.h
            const char* doc = R"""()""";
          } ctor;
          // Symbol: drake::multibody::DifferentialInverseKinematicsSystem::JointVelocityLimitConstraint::SetConfig
          struct /* SetConfig */ {
            // Source: drake/multibody/inverse_kinematics/differential_inverse_kinematics_system.h
            const char* doc =
R"""(Replaces the config set in the constructor.)""";
          } SetConfig;
          // Symbol: drake::multibody::DifferentialInverseKinematicsSystem::JointVelocityLimitConstraint::SetJointLimits
          struct /* SetJointLimits */ {
            // Source: drake/multibody/inverse_kinematics/differential_inverse_kinematics_system.h
            const char* doc =
R"""(Replaces the limits set in the constructor.)""";
          } SetJointLimits;
        } JointVelocityLimitConstraint;
        // Symbol: drake::multibody::DifferentialInverseKinematicsSystem::K_VX
        struct /* K_VX */ {
          // Source: drake/multibody/inverse_kinematics/differential_inverse_kinematics_system.h
          const char* doc =
R"""(Gets the gain factor used to convert desired cartesian poses to
velocities.)""";
        } K_VX;
        // Symbol: drake::multibody::DifferentialInverseKinematicsSystem::LeastSquaresCost
        struct /* LeastSquaresCost */ {
          // Source: drake/multibody/inverse_kinematics/differential_inverse_kinematics_system.h
          const char* doc =
R"""(Provides a primary DifferentialInverseKinematicsSystem objective to
minimize ``G*| S * (Vd_TGs - Jv_TGs * v_next)|²``, also known as the
"least squares" formulation.

Where: - G is ``cartesian_qp_weight``; this coefficient can be used to
balance the relative weight of multiple costs in the mathematical
program. - S is a block-diagonal selector matrix applying per-frame
axis masks to filter out components that are not tracked. Mask
behavior: - Each 6×6 block of ``S`` corresponds to a goal frame and is
constructed from the frame's entry in ``cartesian_axis_masks``. If a
frame is not explicitly listed, all axes are enabled. - Each axis mask
is a binary Vector6d of the form [ωx, ωy, ωz, vx, vy, vz] ∈ {0, 1}⁶.)""";
          // Symbol: drake::multibody::DifferentialInverseKinematicsSystem::LeastSquaresCost::AddToProgram
          struct /* AddToProgram */ {
            // Source: drake/multibody/inverse_kinematics/differential_inverse_kinematics_system.h
            const char* doc = R"""()""";
          } AddToProgram;
          // Symbol: drake::multibody::DifferentialInverseKinematicsSystem::LeastSquaresCost::Config
          struct /* Config */ {
            // Source: drake/multibody/inverse_kinematics/differential_inverse_kinematics_system.h
            const char* doc = R"""()""";
            // Symbol: drake::multibody::DifferentialInverseKinematicsSystem::LeastSquaresCost::Config::Serialize
            struct /* Serialize */ {
              // Source: drake/multibody/inverse_kinematics/differential_inverse_kinematics_system.h
              const char* doc =
R"""(Passes this object to an Archive. Refer to yaml_serialization "YAML
Serialization" for background.)""";
            } Serialize;
            // Symbol: drake::multibody::DifferentialInverseKinematicsSystem::LeastSquaresCost::Config::cartesian_axis_masks
            struct /* cartesian_axis_masks */ {
              // Source: drake/multibody/inverse_kinematics/differential_inverse_kinematics_system.h
              const char* doc =
R"""(Map from fully-scoped frame names to their 6D spatial velocity axis
mask. Each Vector6d is a binary mask [ωx, ωy, ωz, vx, vy, vz]
indicating which Cartesian velocity components (angular and
translational) are being tracked. All elements must be set to either 0
or 1, and at least one element must be 1.)""";
            } cartesian_axis_masks;
            // Symbol: drake::multibody::DifferentialInverseKinematicsSystem::LeastSquaresCost::Config::cartesian_qp_weight
            struct /* cartesian_qp_weight */ {
              // Source: drake/multibody/inverse_kinematics/differential_inverse_kinematics_system.h
              const char* doc =
R"""('G' in the class overview. Must be non-negative and finite.)""";
            } cartesian_qp_weight;
            // Symbol: drake::multibody::DifferentialInverseKinematicsSystem::LeastSquaresCost::Config::use_legacy_implementation
            struct /* use_legacy_implementation */ {
              // Source: drake/multibody/inverse_kinematics/differential_inverse_kinematics_system.h
              const char* doc =
R"""(This is a temporary parameter intended for backwards compatibility. Do
not change this value (unless you really, really know what you're
doing)!)""";
            } use_legacy_implementation;
            auto Serialize__fields() const {
              return std::array{
                std::make_pair("cartesian_axis_masks", cartesian_axis_masks.doc),
                std::make_pair("cartesian_qp_weight", cartesian_qp_weight.doc),
                std::make_pair("use_legacy_implementation", use_legacy_implementation.doc),
              };
            }
          } Config;
          // Symbol: drake::multibody::DifferentialInverseKinematicsSystem::LeastSquaresCost::GetConfig
          struct /* GetConfig */ {
            // Source: drake/multibody/inverse_kinematics/differential_inverse_kinematics_system.h
            const char* doc = R"""(Returns the current config.)""";
          } GetConfig;
          // Symbol: drake::multibody::DifferentialInverseKinematicsSystem::LeastSquaresCost::LeastSquaresCost
          struct /* ctor */ {
            // Source: drake/multibody/inverse_kinematics/differential_inverse_kinematics_system.h
            const char* doc = R"""()""";
          } ctor;
          // Symbol: drake::multibody::DifferentialInverseKinematicsSystem::LeastSquaresCost::SetConfig
          struct /* SetConfig */ {
            // Source: drake/multibody/inverse_kinematics/differential_inverse_kinematics_system.h
            const char* doc =
R"""(Replaces the config set in the constructor.)""";
          } SetConfig;
        } LeastSquaresCost;
        // Symbol: drake::multibody::DifferentialInverseKinematicsSystem::Recipe
        struct /* Recipe */ {
          // Source: drake/multibody/inverse_kinematics/differential_inverse_kinematics_system.h
          const char* doc =
R"""(A recipe collects a list of ingredients for
DifferentialInverseKinematicsSystem, allowing the user to customize
the program being solved.)""";
          // Symbol: drake::multibody::DifferentialInverseKinematicsSystem::Recipe::AddIngredient
          struct /* AddIngredient */ {
            // Source: drake/multibody/inverse_kinematics/differential_inverse_kinematics_system.h
            const char* doc = R"""()""";
          } AddIngredient;
          // Symbol: drake::multibody::DifferentialInverseKinematicsSystem::Recipe::AddToProgram
          struct /* AddToProgram */ {
            // Source: drake/multibody/inverse_kinematics/differential_inverse_kinematics_system.h
            const char* doc =
R"""(Calls DifferentialInverseKinematicsSystem∷Ingredient∷AddToProgram on
all of the ingredients in this recipe.)""";
          } AddToProgram;
          // Symbol: drake::multibody::DifferentialInverseKinematicsSystem::Recipe::Recipe
          struct /* ctor */ {
            // Source: drake/multibody/inverse_kinematics/differential_inverse_kinematics_system.h
            const char* doc = R"""()""";
          } ctor;
          // Symbol: drake::multibody::DifferentialInverseKinematicsSystem::Recipe::ingredient
          struct /* ingredient */ {
            // Source: drake/multibody/inverse_kinematics/differential_inverse_kinematics_system.h
            const char* doc = R"""()""";
          } ingredient;
          // Symbol: drake::multibody::DifferentialInverseKinematicsSystem::Recipe::num_ingredients
          struct /* num_ingredients */ {
            // Source: drake/multibody/inverse_kinematics/differential_inverse_kinematics_system.h
            const char* doc = R"""()""";
          } num_ingredients;
        } Recipe;
        // Symbol: drake::multibody::DifferentialInverseKinematicsSystem::Vd_TG_limit
        struct /* Vd_TG_limit */ {
          // Source: drake/multibody/inverse_kinematics/differential_inverse_kinematics_system.h
          const char* doc =
R"""(Gets the clamping limit applied to inferred desired cartesian
velocities.)""";
        } Vd_TG_limit;
        // Symbol: drake::multibody::DifferentialInverseKinematicsSystem::active_dof
        struct /* active_dof */ {
          // Source: drake/multibody/inverse_kinematics/differential_inverse_kinematics_system.h
          const char* doc =
R"""(Gets the mask of active DOFs in plant() that are being controlled.)""";
        } active_dof;
        // Symbol: drake::multibody::DifferentialInverseKinematicsSystem::collision_checker
        struct /* collision_checker */ {
          // Source: drake/multibody/inverse_kinematics/differential_inverse_kinematics_system.h
          const char* doc =
R"""(Gets the collision checker used by the controller.)""";
        } collision_checker;
        // Symbol: drake::multibody::DifferentialInverseKinematicsSystem::get_input_port_desired_cartesian_poses
        struct /* get_input_port_desired_cartesian_poses */ {
          // Source: drake/multibody/inverse_kinematics/differential_inverse_kinematics_system.h
          const char* doc =
R"""(Returns the input port for the desired cartesian poses (of type
systems∷BusValue containing math∷RigidTransformd).)""";
        } get_input_port_desired_cartesian_poses;
        // Symbol: drake::multibody::DifferentialInverseKinematicsSystem::get_input_port_desired_cartesian_velocities
        struct /* get_input_port_desired_cartesian_velocities */ {
          // Source: drake/multibody/inverse_kinematics/differential_inverse_kinematics_system.h
          const char* doc =
R"""(Returns the input port for the desired cartesian velocities (of type
systems∷BusValue containing SpatialVelocity).)""";
        } get_input_port_desired_cartesian_velocities;
        // Symbol: drake::multibody::DifferentialInverseKinematicsSystem::get_input_port_nominal_posture
        struct /* get_input_port_nominal_posture */ {
          // Source: drake/multibody/inverse_kinematics/differential_inverse_kinematics_system.h
          const char* doc =
R"""(Returns the input port for the nominal joint positions to be used to
handle nullspace resolution. This has the dimension of the full
``plant.num_positions()``; non-active dofs will be ignored.)""";
        } get_input_port_nominal_posture;
        // Symbol: drake::multibody::DifferentialInverseKinematicsSystem::get_input_port_position
        struct /* get_input_port_position */ {
          // Source: drake/multibody/inverse_kinematics/differential_inverse_kinematics_system.h
          const char* doc =
R"""(Returns the input port for the joint positions.)""";
        } get_input_port_position;
        // Symbol: drake::multibody::DifferentialInverseKinematicsSystem::get_output_port_commanded_velocity
        struct /* get_output_port_commanded_velocity */ {
          // Source: drake/multibody/inverse_kinematics/differential_inverse_kinematics_system.h
          const char* doc =
R"""(Returns the output port for the generalized velocity command that
realizes the desired poses within the constraints. The size is equal
to ``get_active_dof().count()``.)""";
        } get_output_port_commanded_velocity;
        // Symbol: drake::multibody::DifferentialInverseKinematicsSystem::plant
        struct /* plant */ {
          // Source: drake/multibody/inverse_kinematics/differential_inverse_kinematics_system.h
          const char* doc = R"""(Gets the plant used by the controller.)""";
        } plant;
        // Symbol: drake::multibody::DifferentialInverseKinematicsSystem::recipe
        struct /* recipe */ {
          // Source: drake/multibody/inverse_kinematics/differential_inverse_kinematics_system.h
          const char* doc =
R"""(Gets the mathematical formulation recipe.)""";
        } recipe;
        // Symbol: drake::multibody::DifferentialInverseKinematicsSystem::task_frame
        struct /* task_frame */ {
          // Source: drake/multibody/inverse_kinematics/differential_inverse_kinematics_system.h
          const char* doc =
R"""(Gets the frame assumed on the desired_cartesian_poses input port.)""";
        } task_frame;
        // Symbol: drake::multibody::DifferentialInverseKinematicsSystem::time_step
        struct /* time_step */ {
          // Source: drake/multibody/inverse_kinematics/differential_inverse_kinematics_system.h
          const char* doc =
R"""(Gets the time step used by the controller.)""";
        } time_step;
      } DifferentialInverseKinematicsSystem;
      // Symbol: drake::multibody::DistanceConstraint
      struct /* DistanceConstraint */ {
        // Source: drake/multibody/inverse_kinematics/distance_constraint.h
        const char* doc =
R"""(Constrains the distance between a pair of geometries to be within a
range [distance_lower, distance_upper].)""";
        // Symbol: drake::multibody::DistanceConstraint::DistanceConstraint
        struct /* ctor */ {
          // Source: drake/multibody/inverse_kinematics/distance_constraint.h
          const char* doc_double =
R"""(Parameter ``plant``:
    The plant to which the pair of geometries belong. ``plant`` should
    outlive this DistanceConstraint object.

Parameter ``geometry_pair``:
    The pair of geometries between which the distance is constrained.
    Notice that we only consider the distance between a static
    geometry and a dynamic geometry, or a pair of dynamic geometries.
    We don't allow constraining the distance between two static
    geometries.

Parameter ``plant_context``:
    The context for the plant. ``plant_context`` should outlive this
    DistanceConstraint object.

Parameter ``distance_lower``:
    The lower bound on the distance.

Parameter ``distance_upper``:
    The upper bound on the distance.)""";
          // Source: drake/multibody/inverse_kinematics/distance_constraint.h
          const char* doc_autodiff =
R"""(Overloaded constructor. Constructs the constraint with
MultibodyPlant<AutoDiffXd>.)""";
        } ctor;
      } DistanceConstraint;
      // Symbol: drake::multibody::DoDifferentialInverseKinematics
      struct /* DoDifferentialInverseKinematics */ {
        // Source: drake/multibody/inverse_kinematics/differential_inverse_kinematics.h
        const char* doc_7args_q_current_v_current_V_J_parameters_N_Nplus =
R"""(Computes a generalized velocity v_next, via the following
MathematicalProgram:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    min_{v_next,alpha}
        -100 * alpha + |P⋅(v_next - N⁺(q)⋅K⋅(q_nominal - q_current))|²
    
      s.t.
        J⋅v_next = alpha⋅V, // J⋅v_next has the same direction as V
        0 <= alpha <= 1,        // Never go faster than V
        joint_lim_min <= q_current + N⋅v_next⋅dt <= joint_lim_max,
        joint_vel_lim_min <= v_next <= joint_vel_lim_max,
        joint_accel_lim_min <= (v_next - v_current)/dt <= joint_accel_lim_max,
        and additional linear velocity constraints,

.. raw:: html

    </details>

where: - The rows of P form an orthonormal basis for the nullspace of
J, - J.rows() == V.size(), - J.cols() == v_current.size() ==
v_next.size(), - V can have any size, with each element representing a
constraint on the solution (6 constraints specifying an end-effector
spatial velocity is typical, but not required), - K is the
joint_centering_gain, - the "additional linear velocity constraints"
are added via
DifferentialInverseKinematicsParameters∷AddLinearVelocityConstraint().

Intuitively, this finds a v_next such that J*v_next is in the same
direction as V, and the difference between |V| and |J * v_next| is
minimized while all constraints in ``parameters`` are satisfied as
well. In the nullspace of this objective, we have a secondary
objective to minimize |v_next - N⁺(q)⋅K⋅(q_nominal - q_current)|².

For more details, see
https://manipulation.csail.mit.edu/pick.html#diff_ik_w_constraints .

If q_current is a feasible point, then v_next = 0 should always be a
feasible solution. If the problem data is bad (q_current is
infeasible, and no feasible velocities can restore feasibility in one
step), then it is possible that the solver cannot find a solution, in
which case, status will be set to kNoSolution in the returned
DifferentialInverseKinematicsResult. If the velocity scaling, alpha,
is very small, then the status will be set to kStuck.

Parameter ``q_current``:
    The current generalized position.

Parameter ``v_current``:
    The current generalized position.

Parameter ``V``:
    Desired spatial velocity. It must have the same number of rows as
    ``J``.

Parameter ``J``:
    Jacobian with respect to generalized velocities v. It must have
    the same number of rows as ``V``. J * v needs to represent the
    same spatial velocity as ``V``.

Parameter ``parameters``:
    Collection of various problem specific constraints and constants.

Parameter ``N``:
    (optional) matrix which maps q̇ = N(q)⋅v. See
    MultibodyPlant∷MakeVelocityToQDotMap(). By default, it is taken to
    be the identity matrix. If dim(q) != dim(v) and any joint position
    limits are set in ``parameters``, then you *must* provide N.

Parameter ``Nplus``:
    (optional) matrix which maps q̇ = N⁺(q)⋅v. See
    MultibodyPlant∷MakeQDotToVelocityMap(). By default, it is taken to
    be the identity matrix. If dim(q) != dim(v) and J is not full
    column rank, then you *must* provide Nplus.

Returns:
    If the solver successfully finds a solution, joint_velocities will
    be set to v, otherwise it will be nullopt.

Note:
    There is a newer framework-based formulation for differential
    inverse kinematics: DifferentialInverseKinematicsSystem. This
    implementation has been shown to be more effective for real-world
    robots. Furthermore, its architecture is more flexible, allowing
    for more customization of the cost and constraint functions.)""";
        // Source: drake/multibody/inverse_kinematics/differential_inverse_kinematics.h
        const char* doc_5args_robot_context_V_WE_desired_frame_E_parameters =
R"""(A wrapper over DoDifferentialInverseKinematics(q_current, v_current,
V, J, params) that tracks frame E's spatial velocity. q_current and
v_current are taken from ``context``. V and J are expressed in E, and
only the elements with non-zero gains in ``parameters``
get_end_effector_velocity_gains() are used in the program.

Parameter ``robot``:
    A MultibodyPlant model.

Parameter ``context``:
    Must be the Context of the MultibodyPlant. Contains the current
    generalized position and velocity.

Parameter ``V_WE_desired``:
    Desired world frame spatial velocity of ``frame_E``.

Parameter ``frame_E``:
    End effector frame.

Parameter ``parameters``:
    Collection of various problem specific constraints and constants.

Returns:
    If the solver successfully finds a solution, joint_velocities will
    be set to v, otherwise it will be nullopt.

Note:
    There is a newer framework-based formulation for differential
    inverse kinematics: DifferentialInverseKinematicsSystem. This
    implementation has been shown to be more effective for real-world
    robots. Furthermore, its architecture is more flexible, allowing
    for more customization of the cost and constraint functions.)""";
        // Source: drake/multibody/inverse_kinematics/differential_inverse_kinematics.h
        const char* doc_6args_robot_context_V_AE_desired_frame_A_frame_E_parameters =
R"""(A wrapper over DoDifferentialInverseKinematics(q_current, v_current,
V, J, params) that tracks frame E's spatial velocity in frame A.
q_current and v_current are taken from ``context``. V and J are
expressed in E, and only the elements with non-zero gains in
``parameters`` get_end_effector_velocity_gains() are used in the
program.

Parameter ``robot``:
    A MultibodyPlant model.

Parameter ``context``:
    Must be the Context of the MultibodyPlant. Contains the current
    generalized position and velocity.

Parameter ``V_AE_desired``:
    Desired spatial velocity of ``frame_E`` in ``frame`` A.

Parameter ``frame_A``:
    Reference frame (inertial or non-inertial).

Parameter ``frame_E``:
    End effector frame.

Parameter ``parameters``:
    Collection of various problem specific constraints and constants.

Returns:
    If the solver successfully finds a solution, joint_velocities will
    be set to v, otherwise it will be nullopt.

Note:
    There is a newer framework-based formulation for differential
    inverse kinematics: DifferentialInverseKinematicsSystem. This
    implementation has been shown to be more effective for real-world
    robots. Furthermore, its architecture is more flexible, allowing
    for more customization of the cost and constraint functions.)""";
        // Source: drake/multibody/inverse_kinematics/differential_inverse_kinematics.h
        const char* doc_5args_robot_context_X_WE_desired_frame_E_parameters =
R"""(A wrapper over DoDifferentialInverseKinematics(robot, context,
V_WE_desired, frame_E, params) that tracks frame E's pose in the world
frame. q_current and v_current are taken from ``context``.
V_WE_desired is computed by ComputePoseDiffInCommonFrame(X_WE,
X_WE_desired) / dt, where X_WE is computed from ``context``, and dt is
taken from ``parameters``.

Parameter ``robot``:
    A MultibodyPlant model.

Parameter ``context``:
    Must be the Context of the MultibodyPlant. Contains the current
    generalized position and velocity.

Parameter ``X_WE_desired``:
    Desired pose of ``frame_E`` in the world frame.

Parameter ``frame_E``:
    End effector frame.

Parameter ``parameters``:
    Collection of various problem specific constraints and constants.

Returns:
    If the solver successfully finds a solution, joint_velocities will
    be set to v, otherwise it will be nullopt.

Note:
    There is a newer framework-based formulation for differential
    inverse kinematics: DifferentialInverseKinematicsSystem. This
    implementation has been shown to be more effective for real-world
    robots. Furthermore, its architecture is more flexible, allowing
    for more customization of the cost and constraint functions.)""";
        // Source: drake/multibody/inverse_kinematics/differential_inverse_kinematics.h
        const char* doc_6args_robot_context_X_AE_desired_frame_A_frame_E_parameters =
R"""(A wrapper over DoDifferentialInverseKinematics(robot, context,
V_AE_desired, frame_A, frame_E, params) that tracks frame E's pose in
frame A. q_current and v_current are taken from ``context``.
V_AE_desired is computed by ComputePoseDiffInCommonFrame(X_AE,
X_AE_desired) / dt, where X_WE is computed from ``context``, and dt is
taken from ``parameters``.

Parameter ``robot``:
    A MultibodyPlant model.

Parameter ``context``:
    Must be the Context of the MultibodyPlant. Contains the current
    generalized position and velocity.

Parameter ``X_AE_desired``:
    Desired pose of ``frame_E`` in ``frame_A``.

Parameter ``frame_A``:
    Reference frame (inertial or non-inertial).

Parameter ``frame_E``:
    End effector frame.

Parameter ``parameters``:
    Collection of various problem specific constraints and constants.

Returns:
    If the solver successfully finds a solution, joint_velocities will
    be set to v, otherwise it will be nullopt.

Note:
    There is a newer framework-based formulation for differential
    inverse kinematics: DifferentialInverseKinematicsSystem. This
    implementation has been shown to be more effective for real-world
    robots. Furthermore, its architecture is more flexible, allowing
    for more customization of the cost and constraint functions.)""";
      } DoDifferentialInverseKinematics;
      // Symbol: drake::multibody::GazeTargetConstraint
      struct /* GazeTargetConstraint */ {
        // Source: drake/multibody/inverse_kinematics/gaze_target_constraint.h
        const char* doc =
R"""(Constrains a target point T to be within a cone K. The point T ("T"
stands for "target") is fixed in a frame B, with position p_BT. The
cone originates from a point S ("S" stands for "source"), fixed in
frame A with position p_AS, with the axis of the cone being n, also
fixed in frame A. The half angle of the cone is θ. A common usage of
this constraint is that a camera should gaze at some target; namely
the target falls within a gaze cone, originating from the camera eye.

Mathematically the constraint is p_ST_Aᵀ * n_unit_A ≥ 0 (p_ST_Aᵀ *
n_unit_A)² ≥ (cosθ)²p_ST_Aᵀ * p_ST_A where p_ST_A is the vector from S
to T, expressed in frame A. n_unit_A is the unit length directional
vector representing the center ray of the cone.)""";
        // Symbol: drake::multibody::GazeTargetConstraint::GazeTargetConstraint
        struct /* ctor */ {
          // Source: drake/multibody/inverse_kinematics/gaze_target_constraint.h
          const char* doc_double =
R"""(Parameter ``plant``:
    The MultibodyPlant on which the constraint is imposed. ``plant``
    should be alive during the lifetime of this constraint.

Parameter ``frameA``:
    The frame to which the gaze cone is fixed.

Parameter ``p_AS``:
    The position of the cone source point S, measured and expressed in
    frame A.

Parameter ``n_A``:
    The directional vector representing the center ray of the cone,
    expressed in frame A.

Parameter ``frameB``:
    The frame to which the target point T is fixed.

Parameter ``p_BT``:
    The position of the target point T, measured and expressed in
    frame B.

Parameter ``cone_half_angle``:
    The half angle of the cone. We denote it as θ in the class
    documentation. ``cone_half_angle`` is in radians.

Parameter ``plant_context``:
    The Context that has been allocated for this ``plant``. We will
    update the context when evaluating the constraint.
    ``plant_context`` should be alive during the lifetime of this
    constraint.

Precondition:
    ``frameA`` and ``frameB`` must belong to ``plant``.

Raises:
    RuntimeError if ``plant`` is nullptr.

Raises:
    RuntimeError if ``n_A`` is close to zero.

Raises:
    RuntimeError if ``cone_half_angle`` ∉ [0, π/2].

Raises:
    RuntimeError if ``plant_context`` is nullptr.)""";
          // Source: drake/multibody/inverse_kinematics/gaze_target_constraint.h
          const char* doc_autodiff =
R"""(Overloaded constructor. Construct from MultibodyPlant<AutoDiffXd>
instead of MultibodyPlant<double>.)""";
        } ctor;
      } GazeTargetConstraint;
      // Symbol: drake::multibody::GlobalInverseKinematics
      struct /* GlobalInverseKinematics */ {
        // Source: drake/multibody/inverse_kinematics/global_inverse_kinematics.h
        const char* doc =
R"""(Solves the inverse kinematics problem as a mixed integer convex
optimization problem. We use a mixed-integer convex relaxation of the
rotation matrix. So if this global inverse kinematics problem says the
solution is infeasible, then it is guaranteed that the kinematics
constraints are not satisfiable. If the global inverse kinematics
returns a solution, the posture should approximately satisfy the
kinematics constraints, with some error. The approach is described in
Global Inverse Kinematics via Mixed-integer Convex Optimization by
Hongkai Dai, Gregory Izatt and Russ Tedrake, International Journal of
Robotics Research, 2019.)""";
        // Symbol: drake::multibody::GlobalInverseKinematics::AddJointLimitConstraint
        struct /* AddJointLimitConstraint */ {
          // Source: drake/multibody/inverse_kinematics/global_inverse_kinematics.h
          const char* doc =
R"""(Adds joint limits on a specified joint.

Note:
    This method is called from the constructor.

Parameter ``body_index``:
    The joint connecting the parent link to this body will be
    constrained.

Parameter ``joint_lower_bound``:
    The lower bound for the joint.

Parameter ``joint_upper_bound``:
    The upper bound for the joint.

Parameter ``linear_constraint_approximation``:
    If true, joint limits are approximated as linear constraints on
    parent and child link orientations, otherwise they are imposed as
    Lorentz cone constraints. With the Lorentz cone formulation, the
    joint limit constraint would be tight if our mixed-integer
    constraint on SO(3) were tight. By enforcing the joint limits as
    linear constraint, the original inverse kinematics problem is
    further relaxed, on top of SO(3) relaxation, but potentially with
    faster computation. $*Default:* is false.)""";
        } AddJointLimitConstraint;
        // Symbol: drake::multibody::GlobalInverseKinematics::AddPostureCost
        struct /* AddPostureCost */ {
          // Source: drake/multibody/inverse_kinematics/global_inverse_kinematics.h
          const char* doc =
R"""(Penalizes the deviation to the desired posture.

For each body (except the world) in the kinematic tree, we add the
cost

∑ᵢ body_position_cost(i) * body_position_error(i) +
body_orientation_cost(i) * body_orientation_error(i) where
``body_position_error(i)`` is computed as the Euclidean distance error
|p_WBo(i) - p_WBo_desired(i)| where - p_WBo(i) : position of body i'th
origin ``Bo`` in the world frame ``W``. - p_WBo_desired(i): position
of body i'th origin ``Bo`` in the world frame ``W``, computed from the
desired posture ``q_desired``.

body_orientation_error(i) is computed as (1 - cos(θ)), where θ is the
angle between the orientation of body i'th frame and body i'th frame
using the desired posture. Notice that 1 - cos(θ) = θ²/2 + O(θ⁴), so
this cost is on the square of θ, when θ is small. Notice that since
body 0 is the world, the cost on that body is always 0, no matter what
value ``body_position_cost(0)`` and ``body_orientation_cost(0)`` take.

Parameter ``q_desired``:
    The desired posture.

Parameter ``body_position_cost``:
    The cost for each body's position error. Unit is [1/m] (one over
    meters).

Precondition:
1. body_position_cost.rows() == plant.num_bodies(), where ``plant`` is the
   input argument in the constructor of the class.
2. body_position_cost(i) is non-negative.

    $Raises:

RuntimeError if the precondition is not satisfied.

Parameter ``body_orientation_cost``:
    The cost for each body's orientation error.

Precondition:
1. body_orientation_cost.rows() == plant.num_bodies() , where ``plant`` is
   the input argument in the constructor of the class.
2. body_position_cost(i) is non-negative.

    $Raises:

RuntimeError if the precondition is not satisfied.)""";
        } AddPostureCost;
        // Symbol: drake::multibody::GlobalInverseKinematics::AddWorldOrientationConstraint
        struct /* AddWorldOrientationConstraint */ {
          // Source: drake/multibody/inverse_kinematics/global_inverse_kinematics.h
          const char* doc =
R"""(Adds a constraint that the angle between the body orientation and the
desired orientation should not be larger than ``angle_tol``. If we
denote the angle between two rotation matrices ``R1`` and ``R2`` as
``θ``, i.e. θ is the angle of the angle-axis representation of the
rotation matrix ``R1ᵀ * R2``, we then know

trace(R1ᵀ * R2) = 2 * cos(θ) + 1 as in
http://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToAngle/
To constraint ``θ < angle_tol``, we can impose the following
constraint

2 * cos(angle_tol) + 1 <= trace(R1ᵀ * R2) <= 3

Parameter ``body_index``:
    The index of the body whose orientation will be constrained.

Parameter ``desired_orientation``:
    The desired orientation of the body.

Parameter ``angle_tol``:
    The tolerance on the angle between the body orientation and the
    desired orientation. Unit is radians.

Returns ``binding``:
    The newly added constraint, together with the bound variables.)""";
        } AddWorldOrientationConstraint;
        // Symbol: drake::multibody::GlobalInverseKinematics::AddWorldPositionConstraint
        struct /* AddWorldPositionConstraint */ {
          // Source: drake/multibody/inverse_kinematics/global_inverse_kinematics.h
          const char* doc =
R"""(Adds the constraint that the position of a point ``Q`` on a body ``B``
(whose index is ``body_index``), is within a box in a specified frame
``F``. The constraint is that the point ``Q`'s position should lie
within a bounding box in the frame `F``. Namely

box_lb_F <= p_FQ <= box_ub_F

where p_FQ is the position of the point Q measured and expressed in
the ``F``, computed as

p_FQ = X_FW * (p_WBo + R_WB * p_BQ)

hence this is a linear constraint on the decision variables p_WBo and
R_WB. The inequality is imposed elementwise.

Note:
    since the rotation matrix ``R_WB`` does not lie exactly on the
    SO(3), due to the McCormick envelope relaxation, this constraint
    is subject to the accumulated error from the root of the
    kinematics tree.

Parameter ``body_index``:
    The index of the body on which the position of a point is
    constrained.

Parameter ``p_BQ``:
    The position of the point Q measured and expressed in the body
    frame B.

Parameter ``box_lb_F``:
    The lower bound of the box in frame ``F``.

Parameter ``box_ub_F``:
    The upper bound of the box in frame ``F``.

Parameter ``X_WF``:
    The frame in which the box is specified. This frame is represented
    by a RigidTransform X_WF, the transform from the constraint frame
    F to the world frame W. Namely if the position of the point ``Q``
    in the world frame is ``p_WQ``, then the constraint is

box_lb_F <= R_FW * (p_WQ-p_WFo) <= box_ub_F where - R_FW is the
rotation matrix of frame ``W`` expressed and measured in frame ``F``.
`R_FW = X_WF.linear().transpose()``. - p_WFo is the position of frame
`F`'s origin, expressed and measured in frame `W``. `p_WFo =
X_WF.translation()`.

*Default:* is the identity transform.
    $Returns ``binding``:

The newly added constraint, together with the bound variables.)""";
        } AddWorldPositionConstraint;
        // Symbol: drake::multibody::GlobalInverseKinematics::AddWorldRelativePositionConstraint
        struct /* AddWorldRelativePositionConstraint */ {
          // Source: drake/multibody/inverse_kinematics/global_inverse_kinematics.h
          const char* doc =
R"""(Adds the constraint that the position of a point ``Q`` on a body ``B``
relative to a point ``P`` on body ``A``, is within a box in a
specified frame ``F``. Using monogram notation we have:

box_lb_F <= p_FQ - p_FP <= box_ub_F

where p_FQ and p_FP are the position of the points measured and
expressed in ``F``. The inequality is imposed elementwise. See
AddWorldPositionConstraint for more details.

Parameter ``body_index_B``:
    The index of the body B.

Parameter ``p_BQ``:
    The position of the point Q measured and expressed in the body
    frame B.

Parameter ``body_index_A``:
    The index of the body A.

Parameter ``p_AP``:
    The position of the point P measured and expressed in the body
    frame A.

Parameter ``box_lb_F``:
    The lower bound of the box in frame ``F``.

Parameter ``box_ub_F``:
    The upper bound of the box in frame ``F``.

Parameter ``X_WF``:
    Defines the frame in which the box is specified. $*Default:* is
    the identity transform.

Returns ``binding``:
    The newly added constraint, together with the bound variables.)""";
        } AddWorldRelativePositionConstraint;
        // Symbol: drake::multibody::GlobalInverseKinematics::BodyPointInOneOfRegions
        struct /* BodyPointInOneOfRegions */ {
          // Source: drake/multibody/inverse_kinematics/global_inverse_kinematics.h
          const char* doc =
R"""(Constrain the point ``Q`` lying within one of the convex polytopes.
Each convex polytope Pᵢ is represented by its vertices as Pᵢ =
ConvexHull(v_i1, v_i2, ... v_in). Mathematically we want to impose the
constraint that the p_WQ, i.e., the position of point ``Q`` in world
frame ``W``, satisfies

p_WQ ∈ Pᵢ for one i. To impose this constraint, we consider to
introduce binary variable zᵢ, and continuous variables w_i1, w_i2,
..., w_in for each vertex of Pᵢ, with the following constraints

p_WQ = sum_i (w_i1 * v_i1 + w_i2 * v_i2 + ... + w_in * v_in) w_ij >=
0, ∀i,j w_i1 + w_i2 + ... + w_in = zᵢ sum_i zᵢ = 1 zᵢ ∈ {0, 1} Notice
that if zᵢ = 0, then w_i1 * v_i1 + w_i2 * v_i2 + ... + w_in * v_in is
just 0. This function can be used for collision avoidance, where each
region Pᵢ is a free space region. It can also be used for grasping,
where each region Pᵢ is a surface patch on the grasped object. Note
this approach also works if the region Pᵢ overlaps with each other.

Parameter ``body_index``:
    The index of the body to on which point ``Q`` is attached.

Parameter ``p_BQ``:
    The position of point ``Q`` in the body frame ``B``.

Parameter ``region_vertices``:
    region_vertices[i] is the vertices for the i'th region.

Returns ``z``:
    The newly added binary variables. If point ``Q`` is in the i'th
    region, z(i) = 1.

Precondition:
    region_vertices[i] has at least 3 columns. Throw a RuntimeError if
    the precondition is not satisfied.)""";
        } BodyPointInOneOfRegions;
        // Symbol: drake::multibody::GlobalInverseKinematics::BodySphereInOneOfPolytopes
        struct /* BodySphereInOneOfPolytopes */ {
          // Source: drake/multibody/inverse_kinematics/global_inverse_kinematics.h
          const char* doc =
R"""(Adds the constraint that a sphere rigidly attached to a body has to be
within at least one of the given bounded polytopes. If the polytopes
don't intersect, then the sphere is in one and only one polytope.
Otherwise the sphere is in at least one of the polytopes (could be in
the intersection of multiple polytopes.) If the i'th polytope is
described as

Aᵢ * x ≤ bᵢ where Aᵢ ∈ ℝⁿ ˣ ³, bᵢ ∈ ℝⁿ. Then a sphere with center
position p_WQ and radius r is within the i'th polytope, if

Aᵢ * p_WQ ≤ bᵢ - aᵢr where aᵢ(j) = Aᵢ.row(j).norm() To constrain that
the sphere is in one of the n polytopes, we introduce the binary
variable z ∈{0, 1}ⁿ, together with continuous variables yᵢ ∈ ℝ³, i =
1, ..., n, such that p_WQ = y₁ + ... + yₙ Aᵢ * yᵢ ≤ (bᵢ - aᵢr)zᵢ z₁ +
... +zₙ = 1 Notice that when zᵢ = 0, Aᵢ * yᵢ ≤ 0 implies that yᵢ = 0.
This is due to the boundedness of the polytope. If Aᵢ * yᵢ ≤ 0 has a
non-zero solution y̅, that y̅ ≠ 0 and Aᵢ * y̅ ≤ 0. Then for any point
x̂ in the polytope satisfying Aᵢ * x̂ ≤ bᵢ, we know the ray x̂ + ty̅,
∀ t ≥ 0 also satisfies Aᵢ * (x̂ + ty̅) ≤ bᵢ, thus the ray is within
the polytope, violating the boundedness assumption.

Parameter ``body_index``:
    The index of the body to which the sphere is attached.

Parameter ``p_BQ``:
    The position of the sphere center in the body frame B.

Parameter ``radius``:
    The radius of the sphere.

Parameter ``polytopes``:
    polytopes[i] = (Aᵢ, bᵢ). We assume that Aᵢx≤ bᵢ is a bounded
    polytope. It is the user's responsibility to guarantee the
    boundedness.

Returns ``z``:
    The newly added binary variables. If z(i) = 1, then the sphere is
    in the i'th polytope. If two or more polytopes are intersecting,
    and the sphere is in the intersection region, then it is up to the
    solver to choose one of z(i) to be 1.)""";
        } BodySphereInOneOfPolytopes;
        // Symbol: drake::multibody::GlobalInverseKinematics::GlobalInverseKinematics
        struct /* ctor */ {
          // Source: drake/multibody/inverse_kinematics/global_inverse_kinematics.h
          const char* doc =
R"""(Parses the robot kinematics tree. The decision variables include the
pose for each body (position/orientation). This constructor loops
through each body inside the robot kinematics tree, adds the
constraint on each body pose, so that the adjacent bodies are
connected correctly by the joint in between the bodies.

Parameter ``plant``:
    The robot on which the inverse kinematics problem is solved. plant
    must be alive for as long as this object is around.

Parameter ``options``:
    The options to relax SO(3) constraint as mixed-integer convex
    constraints. Refer to MixedIntegerRotationConstraintGenerator for
    more details on the parameters in options.)""";
        } ctor;
        // Symbol: drake::multibody::GlobalInverseKinematics::Options
        struct /* Options */ {
          // Source: drake/multibody/inverse_kinematics/global_inverse_kinematics.h
          const char* doc = R"""()""";
          // Symbol: drake::multibody::GlobalInverseKinematics::Options::Options
          struct /* ctor */ {
            // Source: drake/multibody/inverse_kinematics/global_inverse_kinematics.h
            const char* doc = R"""()""";
          } ctor;
          // Symbol: drake::multibody::GlobalInverseKinematics::Options::approach
          struct /* approach */ {
            // Source: drake/multibody/inverse_kinematics/global_inverse_kinematics.h
            const char* doc = R"""()""";
          } approach;
          // Symbol: drake::multibody::GlobalInverseKinematics::Options::interval_binning
          struct /* interval_binning */ {
            // Source: drake/multibody/inverse_kinematics/global_inverse_kinematics.h
            const char* doc = R"""()""";
          } interval_binning;
          // Symbol: drake::multibody::GlobalInverseKinematics::Options::linear_constraint_only
          struct /* linear_constraint_only */ {
            // Source: drake/multibody/inverse_kinematics/global_inverse_kinematics.h
            const char* doc =
R"""(If true, add only mixed-integer linear constraints in the constructor
of GlobalInverseKinematics. The mixed-integer relaxation is tighter
with nonlinear constraints (such as Lorentz cone constraint) than with
linear constraints, but the optimization takes more time with
nonlinear constraints.)""";
          } linear_constraint_only;
          // Symbol: drake::multibody::GlobalInverseKinematics::Options::num_intervals_per_half_axis
          struct /* num_intervals_per_half_axis */ {
            // Source: drake/multibody/inverse_kinematics/global_inverse_kinematics.h
            const char* doc = R"""()""";
          } num_intervals_per_half_axis;
        } Options;
        // Symbol: drake::multibody::GlobalInverseKinematics::Polytope3D
        struct /* Polytope3D */ {
          // Source: drake/multibody/inverse_kinematics/global_inverse_kinematics.h
          const char* doc =
R"""(Describes a polytope in 3D as 𝐀 * 𝐱 ≤ 𝐛 (a set of half-spaces), where
𝐀 ∈ ℝⁿˣ³, 𝐱 ∈ ℝ³, 𝐛 ∈ ℝⁿ.)""";
          // Symbol: drake::multibody::GlobalInverseKinematics::Polytope3D::A
          struct /* A */ {
            // Source: drake/multibody/inverse_kinematics/global_inverse_kinematics.h
            const char* doc = R"""()""";
          } A;
          // Symbol: drake::multibody::GlobalInverseKinematics::Polytope3D::Polytope3D
          struct /* ctor */ {
            // Source: drake/multibody/inverse_kinematics/global_inverse_kinematics.h
            const char* doc = R"""()""";
          } ctor;
          // Symbol: drake::multibody::GlobalInverseKinematics::Polytope3D::b
          struct /* b */ {
            // Source: drake/multibody/inverse_kinematics/global_inverse_kinematics.h
            const char* doc = R"""()""";
          } b;
        } Polytope3D;
        // Symbol: drake::multibody::GlobalInverseKinematics::ReconstructGeneralizedPositionSolution
        struct /* ReconstructGeneralizedPositionSolution */ {
          // Source: drake/multibody/inverse_kinematics/global_inverse_kinematics.h
          const char* doc =
R"""(After solving the inverse kinematics problem and finding out the pose
of each body, reconstruct the robot generalized position (joint
angles, etc) that matches with the body poses. Notice that since the
rotation matrix is approximated, that the solution of body_rotmat()
might not be on SO(3) exactly, the reconstructed body posture might
not match with the body poses exactly, and the kinematics constraint
might not be satisfied exactly with this reconstructed posture.

Warning:
    Do not call this method if the problem is not solved successfully!
    The returned value can be NaN or meaningless number if the problem
    is not solved.

Returns ``q``:
    The reconstructed posture of the robot of the generalized
    coordinates, corresponding to the RigidBodyTree on which the
    inverse kinematics problem is solved.)""";
        } ReconstructGeneralizedPositionSolution;
        // Symbol: drake::multibody::GlobalInverseKinematics::SetInitialGuess
        struct /* SetInitialGuess */ {
          // Source: drake/multibody/inverse_kinematics/global_inverse_kinematics.h
          const char* doc =
R"""(Sets an initial guess for all variables (including the binary
variables) by evaluating the kinematics of the plant at ``q``.
Currently, this is accomplished by solving the global IK problem
subject to constraints that the positions and rotation matrices match
the kinematics, which is dramatically faster than solving the original
problem.

Raises:
    RuntimeError if solving results in an infeasible program.)""";
        } SetInitialGuess;
        // Symbol: drake::multibody::GlobalInverseKinematics::body_position
        struct /* body_position */ {
          // Source: drake/multibody/inverse_kinematics/global_inverse_kinematics.h
          const char* doc =
R"""(Getter for the decision variables on the position p_WBo of the body
B's origin measured and expressed in the world frame.

Parameter ``body_index``:
    The index of the queried body. Notice that body 0 is the world,
    and thus not a decision variable.

Raises:
    RuntimeError if the index is smaller than 1, or greater than or
    equal to the total number of bodies in the robot.)""";
        } body_position;
        // Symbol: drake::multibody::GlobalInverseKinematics::body_rotation_matrix
        struct /* body_rotation_matrix */ {
          // Source: drake/multibody/inverse_kinematics/global_inverse_kinematics.h
          const char* doc =
R"""(Getter for the decision variables on the rotation matrix ``R_WB`` for
a body with the specified index. This is the orientation of body i's
frame measured and expressed in the world frame.

Parameter ``body_index``:
    The index of the queried body. Notice that body 0 is the world,
    and thus not a decision variable.

Raises:
    RuntimeError if the index is smaller than 1, or greater than or
    equal to the total number of bodies in the robot.)""";
        } body_rotation_matrix;
        // Symbol: drake::multibody::GlobalInverseKinematics::get_mutable_prog
        struct /* get_mutable_prog */ {
          // Source: drake/multibody/inverse_kinematics/global_inverse_kinematics.h
          const char* doc = R"""()""";
        } get_mutable_prog;
        // Symbol: drake::multibody::GlobalInverseKinematics::prog
        struct /* prog */ {
          // Source: drake/multibody/inverse_kinematics/global_inverse_kinematics.h
          const char* doc = R"""()""";
        } prog;
      } GlobalInverseKinematics;
      // Symbol: drake::multibody::InverseKinematics
      struct /* InverseKinematics */ {
        // Source: drake/multibody/inverse_kinematics/inverse_kinematics.h
        const char* doc =
R"""(Solves an inverse kinematics (IK) problem on a MultibodyPlant, to find
the postures of the robot satisfying certain constraints. The decision
variables include the generalized position of the robot.

To perform IK on a subset of the plant, use the constructor overload
that takes a ``plant_context`` and use ``Joint∷Lock`` on the joints in
that Context that should be fixed during IK.)""";
        // Symbol: drake::multibody::InverseKinematics::AddAngleBetweenVectorsConstraint
        struct /* AddAngleBetweenVectorsConstraint */ {
          // Source: drake/multibody/inverse_kinematics/inverse_kinematics.h
          const char* doc =
R"""(Constrains that the angle between a vector na and another vector nb is
between [θ_lower, θ_upper]. na is fixed to a frame A, while nb is
fixed to a frame B. Mathematically, if we denote na_unit_A as na
expressed in frame A after normalization (na_unit_A has unit length),
and nb_unit_B as nb expressed in frame B after normalization, the
constraint is cos(θ_upper) ≤ na_unit_Aᵀ * R_AB * nb_unit_B ≤
cos(θ_lower), where R_AB is the rotation matrix, representing the
orientation of frame B expressed in frame A.

Parameter ``frameA``:
    The frame to which na is fixed.

Parameter ``na_A``:
    The vector na fixed to frame A, expressed in frame A.

Precondition:
    na_A should be a non-zero vector.

Raises:
    RuntimeError if na_A is close to zero.

Parameter ``frameB``:
    The frame to which nb is fixed.

Parameter ``nb_B``:
    The vector nb fixed to frame B, expressed in frame B.

Precondition:
    nb_B should be a non-zero vector.

Raises:
    RuntimeError if nb_B is close to zero.

Parameter ``angle_lower``:
    The lower bound on the angle between na and nb. It is denoted as
    θ_lower in the documentation. ``angle_lower`` is in radians.

Precondition:
    angle_lower >= 0.

Raises:
    RuntimeError if angle_lower is negative.

Parameter ``angle_upper``:
    The upper bound on the angle between na and nb. it is denoted as
    θ_upper in the class documentation. ``angle_upper`` is in radians.

Precondition:
    angle_lower <= angle_upper <= pi.

Raises:
    RuntimeError if angle_upper is outside the bounds.)""";
        } AddAngleBetweenVectorsConstraint;
        // Symbol: drake::multibody::InverseKinematics::AddAngleBetweenVectorsCost
        struct /* AddAngleBetweenVectorsCost */ {
          // Source: drake/multibody/inverse_kinematics/inverse_kinematics.h
          const char* doc =
R"""(Add a cost c * (1-cosθ) where θ is the angle between the vector ``na``
and ``nb``. na is fixed to a frame A, while nb is fixed to a frame B.

Parameter ``frameA``:
    The frame to which na is fixed.

Parameter ``na_A``:
    The vector na fixed to frame A, expressed in frame A.

Precondition:
    na_A should be a non-zero vector.

Raises:
    RuntimeError if na_A is close to zero.

Parameter ``frameB``:
    The frame to which nb is fixed.

Parameter ``nb_B``:
    The vector nb fixed to frame B, expressed in frame B.

Precondition:
    nb_B should be a non-zero vector.

Raises:
    RuntimeError if nb_B is close to zero.

Parameter ``c``:
    The cost is c * (1-cosθ).)""";
        } AddAngleBetweenVectorsCost;
        // Symbol: drake::multibody::InverseKinematics::AddDistanceConstraint
        struct /* AddDistanceConstraint */ {
          // Source: drake/multibody/inverse_kinematics/inverse_kinematics.h
          const char* doc =
R"""(Adds the constraint that the distance between a pair of geometries is
within some bounds.

Parameter ``geometry_pair``:
    The pair of geometries between which the distance is constrained.
    Notice that we only consider the distance between a static
    geometry and a dynamic geometry, or a pair of dynamic geometries.
    We don't allow constraining the distance between two static
    geometries.

Parameter ``distance_lower``:
    The lower bound on the distance.

Parameter ``distance_upper``:
    The upper bound on the distance.)""";
        } AddDistanceConstraint;
        // Symbol: drake::multibody::InverseKinematics::AddGazeTargetConstraint
        struct /* AddGazeTargetConstraint */ {
          // Source: drake/multibody/inverse_kinematics/inverse_kinematics.h
          const char* doc =
R"""(Constrains a target point T to be within a cone K. The point T ("T"
stands for "target") is fixed in a frame B, with position p_BT. The
cone originates from a point S ("S" stands for "source"), fixed in
frame A with position p_AS, with the axis of the cone being n, also
fixed in frame A. The half angle of the cone is θ. A common usage of
this constraint is that a camera should gaze at some target; namely
the target falls within a gaze cone, originating from the camera eye.

Parameter ``frameA``:
    The frame where the gaze cone is fixed to.

Parameter ``p_AS``:
    The position of the cone source point S, measured and expressed in
    frame A.

Parameter ``n_A``:
    The directional vector representing the center ray of the cone,
    expressed in frame A.

Precondition:
    ``n_A`` cannot be a zero vector.

Raises:
    RuntimeError is n_A is close to a zero vector.

Parameter ``frameB``:
    The frame where the target point T is fixed to.

Parameter ``p_BT``:
    The position of the target point T, measured and expressed in
    frame B.

Parameter ``cone_half_angle``:
    The half angle of the cone. We denote it as θ in the
    documentation. ``cone_half_angle`` is in radians.

Precondition:
    ``0`` <= cone_half_angle <= pi.

Raises:
    RuntimeError if cone_half_angle is outside of the bound.)""";
        } AddGazeTargetConstraint;
        // Symbol: drake::multibody::InverseKinematics::AddMinimumDistanceLowerBoundConstraint
        struct /* AddMinimumDistanceLowerBoundConstraint */ {
          // Source: drake/multibody/inverse_kinematics/inverse_kinematics.h
          const char* doc =
R"""(Adds the constraint that the pairwise distance between objects should
be no smaller than ``bound``. We consider the distance between pairs
of 1. Anchored (static) object and a dynamic object. 2. A dynamic
object and another dynamic object, if one is not the parent link of
the other.

Parameter ``bound``:
    The minimum allowed value, dₘᵢₙ, of the signed distance between
    any candidate pair of geometries.

Parameter ``influence_distance_offset``:
    See MinimumDistanceLowerBoundConstraint for explanation.

Precondition:
    The MultibodyPlant passed to the constructor of ``this`` has
    registered its geometry with a SceneGraph.

Precondition:
    0 < ``influence_distance_offset`` < ∞)""";
        } AddMinimumDistanceLowerBoundConstraint;
        // Symbol: drake::multibody::InverseKinematics::AddMinimumDistanceUpperBoundConstraint
        struct /* AddMinimumDistanceUpperBoundConstraint */ {
          // Source: drake/multibody/inverse_kinematics/inverse_kinematics.h
          const char* doc =
R"""(Adds the constraint that at least one pair of geometries has distance
no larger than ``bound``. We consider the distance between pairs of 1.
Anchored (static) object and a dynamic object. 2. A dynamic object and
another dynamic object, if one is not the parent link of the other.

Parameter ``bound``:
    The upper bound of the minimum signed distance between any
    candidate pair of geometries. Notice this is NOT the upper bound
    of every distance, but the upper bound of the smallest distance.

Parameter ``influence_distance_offset``:
    See MinimumDistanceUpperBoundConstraint for more details on
    influence_distance_offset.

Precondition:
    The MultibodyPlant passed to the constructor of ``this`` has
    registered its geometry with a SceneGraph.

Precondition:
    0 < ``influence_distance_offset`` < ∞)""";
        } AddMinimumDistanceUpperBoundConstraint;
        // Symbol: drake::multibody::InverseKinematics::AddOrientationConstraint
        struct /* AddOrientationConstraint */ {
          // Source: drake/multibody/inverse_kinematics/inverse_kinematics.h
          const char* doc =
R"""(Constrains that the angle difference θ between the orientation of
frame A and the orientation of frame B to satisfy θ ≤ θ_bound. Frame A
is fixed to frame A_bar, with orientation R_AbarA measured in frame
A_bar. Frame B is fixed to frame B_bar, with orientation R_BbarB
measured in frame B_bar. The angle difference between frame A's
orientation R_WA and B's orientation R_WB is θ, (θ ∈ [0, π]), if there
exists a rotation axis a, such that rotating frame A by angle θ about
axis a aligns it with frame B. Namely R_AB = I + sinθ â + (1-cosθ)â²
(1) where R_AB is the orientation of frame B expressed in frame A. â
is the skew symmetric matrix of the rotation axis a. Equation (1) is
the Rodrigues formula that computes the rotation matrix from a
rotation axis a and an angle θ,
https://en.wikipedia.org/wiki/Rodrigues%27_rotation_formula If the
users want frame A and frame B to align perfectly, they can set
θ_bound = 0. Mathematically, this constraint is imposed as trace(R_AB)
≥ 2cos(θ_bound) + 1 (1) To derive (1), using Rodrigues formula R_AB =
I + sinθ â + (1-cosθ)â² where trace(R_AB) = 2cos(θ) + 1 ≥
2cos(θ_bound) + 1

Parameter ``frameAbar``:
    frame A_bar, the frame A is fixed to frame A_bar.

Parameter ``R_AbarA``:
    The orientation of frame A measured in frame A_bar.

Parameter ``frameBbar``:
    frame B_bar, the frame B is fixed to frame B_bar.

Parameter ``R_BbarB``:
    The orientation of frame B measured in frame B_bar.

Parameter ``theta_bound``:
    The bound on the angle difference between frame A's orientation
    and frame B's orientation. It is denoted as θ_bound in the
    documentation. ``theta_bound`` is in radians.)""";
        } AddOrientationConstraint;
        // Symbol: drake::multibody::InverseKinematics::AddOrientationCost
        struct /* AddOrientationCost */ {
          // Source: drake/multibody/inverse_kinematics/inverse_kinematics.h
          const char* doc =
R"""(Adds a cost of the form ``c * (1 - cos(θ))``, where θ is the angle
between the orientation of frame A and the orientation of frame B, and
``c`` is a cost scaling.

Parameter ``frameAbar``:
    A frame on the MultibodyPlant.

Parameter ``R_AbarA``:
    The rotation matrix describing the orientation of frame A relative
    to Abar.

Parameter ``frameBbar``:
    A frame on the MultibodyPlant.

Parameter ``R_BbarB``:
    The rotation matrix describing the orientation of frame B relative
    to Bbar.

Parameter ``c``:
    A scalar cost weight.)""";
        } AddOrientationCost;
        // Symbol: drake::multibody::InverseKinematics::AddPointToLineDistanceConstraint
        struct /* AddPointToLineDistanceConstraint */ {
          // Source: drake/multibody/inverse_kinematics/inverse_kinematics.h
          const char* doc =
R"""(Add a constraint that the distance between point P attached to
frame_point (denoted as B1) and a line attached to frame_line (denoted
as B2) is within the range [distance_lower, distance_upper]. The line
passes through a point Q with a directional vector n.

Parameter ``frame_point``:
    The frame to which P is attached.

Parameter ``p_B1P``:
    The position of P measured and expressed in frame_point.

Parameter ``frame_line``:
    The frame to which the line is attached.

Parameter ``p_B2Q``:
    The position of Q measured and expressed in frame_line, the line
    passes through Q.

Parameter ``n_B2``:
    The direction vector of the line measured and expressed in
    frame_line.

Parameter ``distance_lower``:
    The lower bound on the distance.

Parameter ``distance_upper``:
    The upper bound on the distance.)""";
        } AddPointToLineDistanceConstraint;
        // Symbol: drake::multibody::InverseKinematics::AddPointToPointDistanceConstraint
        struct /* AddPointToPointDistanceConstraint */ {
          // Source: drake/multibody/inverse_kinematics/inverse_kinematics.h
          const char* doc =
R"""(Add a constraint that the distance between point P1 attached to frame
1 and point P2 attached to frame 2 is within the range
[distance_lower, distance_upper].

Parameter ``frame1``:
    The frame to which P1 is attached.

Parameter ``p_B1P1``:
    The position of P1 measured and expressed in frame 1.

Parameter ``frame2``:
    The frame to which P2 is attached.

Parameter ``p_B2P2``:
    The position of P2 measured and expressed in frame 2.

Parameter ``distance_lower``:
    The lower bound on the distance.

Parameter ``distance_upper``:
    The upper bound on the distance.)""";
        } AddPointToPointDistanceConstraint;
        // Symbol: drake::multibody::InverseKinematics::AddPolyhedronConstraint
        struct /* AddPolyhedronConstraint */ {
          // Source: drake/multibody/inverse_kinematics/inverse_kinematics.h
          const char* doc =
R"""(Adds the constraint that the position of P1, ..., Pn satisfy A *
[p_FP1; p_FP2; ...; p_FPn] <= b.

Parameter ``frameF``:
    The frame in which the position P is measured and expressed

Parameter ``frameG``:
    The frame in which the point P is rigidly attached.

Parameter ``p_GP``:
    p_GP.col(i) is the position of the i'th point Pi measured and
    expressed in frame G.

Parameter ``A``:
    We impose the constraint A * [p_FP1; p_FP2; ...; p_FPn] <= b.

Precondition:
    A.cols() = 3 * p_GP.cols().

Parameter ``b``:
    We impose the constraint A * [p_FP1; p_FP2; ...; p_FPn] <= b.)""";
        } AddPolyhedronConstraint;
        // Symbol: drake::multibody::InverseKinematics::AddPositionConstraint
        struct /* AddPositionConstraint */ {
          // Source: drake/multibody/inverse_kinematics/inverse_kinematics.h
          const char* doc_5args =
R"""(Adds the kinematic constraint that a point Q, fixed in frame B, should
lie within a bounding box expressed in another frame A as p_AQ_lower
<= p_AQ <= p_AQ_upper, where p_AQ is the position of point Q measured
and expressed in frame A.

Parameter ``frameB``:
    The frame in which point Q is fixed.

Parameter ``p_BQ``:
    The position of the point Q, rigidly attached to frame B, measured
    and expressed in frame B.

Parameter ``frameA``:
    The frame in which the bounding box p_AQ_lower <= p_AQ <=
    p_AQ_upper is expressed.

Parameter ``p_AQ_lower``:
    The lower bound on the position of point Q, measured and expressed
    in frame A.

Parameter ``p_AQ_upper``:
    The upper bound on the position of point Q, measured and expressed
    in frame A.)""";
          // Source: drake/multibody/inverse_kinematics/inverse_kinematics.h
          const char* doc_6args =
R"""(Adds the kinematic constraint that a point Q, fixed in frame B, should
lie within a bounding box expressed in another frame A as p_AQ_lower
<= p_AQ <= p_AQ_upper, where p_AQ is the position of point Q measured
and expressed in frame A.

Parameter ``frameB``:
    The frame in which point Q is fixed.

Parameter ``p_BQ``:
    The position of the point Q, rigidly attached to frame B, measured
    and expressed in frame B.

Parameter ``frameAbar``:
    We will compute frame A from frame Abar. The bounding box
    p_AQ_lower <= p_AQ <= p_AQ_upper is expressed in frame A.

Parameter ``X_AbarA``:
    The relative transform between frame Abar and A. If empty, then we
    use the identity transform.

Parameter ``p_AQ_lower``:
    The lower bound on the position of point Q, measured and expressed
    in frame A.

Parameter ``p_AQ_upper``:
    The upper bound on the position of point Q, measured and expressed
    in frame A.)""";
        } AddPositionConstraint;
        // Symbol: drake::multibody::InverseKinematics::AddPositionCost
        struct /* AddPositionCost */ {
          // Source: drake/multibody/inverse_kinematics/inverse_kinematics.h
          const char* doc =
R"""(Adds a cost of the form (p_AP - p_AQ)ᵀ C (p_AP - p_AQ), where point P
is specified relative to frame A and point Q is specified relative to
frame B, and the cost is evaluated in frame A.

Parameter ``frameA``:
    The frame in which point P's position is measured.

Parameter ``p_AP``:
    The point P.

Parameter ``frameB``:
    The frame in which point Q's position is measured.

Parameter ``p_BQ``:
    The point Q.

Parameter ``C``:
    A 3x3 matrix representing the cost in quadratic form.)""";
        } AddPositionCost;
        // Symbol: drake::multibody::InverseKinematics::InverseKinematics
        struct /* ctor */ {
          // Source: drake/multibody/inverse_kinematics/inverse_kinematics.h
          const char* doc_2args =
R"""(Constructs an inverse kinematics problem for a MultibodyPlant. This
constructor will create and own a context for ``plant``.

Parameter ``plant``:
    The robot on which the inverse kinematics problem will be solved.

Parameter ``with_joint_limits``:
    If set to true, then the constructor imposes the joint limits
    (obtained from plant.GetPositionLowerLimits() and
    plant.GetPositionUpperLimits()). If set to false, then the
    constructor does not impose the joint limit constraints in the
    constructor.

Note:
    The inverse kinematics problem constructed in this way doesn't
    permit collision related constraint (such as calling
    AddMinimumDistanceConstraint). To enable collision related
    constraint, call InverseKinematics(const MultibodyPlant<double>&
    plant, systems∷Context<double>* plant_context);)""";
          // Source: drake/multibody/inverse_kinematics/inverse_kinematics.h
          const char* doc_3args =
R"""(Constructs an inverse kinematics problem for a MultibodyPlant. If the
user wants to solve the problem with collision related constraint
(like calling AddMinimumDistanceConstraint), please use this
constructor.

Parameter ``plant``:
    The robot on which the inverse kinematics problem will be solved.
    This plant should have been connected to a SceneGraph within a
    Diagram

Parameter ``plant_context``:
    The context for the plant. This context should be a part of the
    Diagram context. Any locked joints in the ``plant_context`` will
    remain fixed at their locked value. (This provides a convenient
    way to perform IK on a subset of the plant.) To construct a plant
    connected to a SceneGraph, with the corresponding plant_context,
    the steps are:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    // 1. Add a diagram containing the MultibodyPlant and SceneGraph
    systems∷DiagramBuilder<double> builder;
    auto items = AddMultibodyPlantSceneGraph(&builder, 0.0);
    // 2. Add collision geometries to the plant
    Parser(&builder).AddModels("model.sdf");
    // 3. Construct the diagram
    auto diagram = builder.Build();
    // 4. Create diagram context.
    auto diagram_context= diagram->CreateDefaultContext();
    // 5. Get the context for the plant.
    auto plant_context = &(diagram->GetMutableSubsystemContext(items.plant,
    diagram_context.get()));

.. raw:: html

    </details>

This context will be modified during calling ik.prog.Solve(...). When
Solve() returns ``result``, context will store the optimized posture,
namely plant.GetPositions(*context) will be the same as in
result.GetSolution(ik.q()). The user could then use this context to
perform kinematic computation (like computing the position of the
end-effector etc.).

Parameter ``with_joint_limits``:
    If set to true, then the constructor imposes the joint limits
    (obtained from plant.GetPositionLowerLimits() and
    plant.GetPositionUpperLimits(), and from any body/joint locks set
    in ``plant_context``). If set to false, then the constructor does
    not impose the joint limit constraints in the constructor.)""";
        } ctor;
        // Symbol: drake::multibody::InverseKinematics::context
        struct /* context */ {
          // Source: drake/multibody/inverse_kinematics/inverse_kinematics.h
          const char* doc = R"""(Getter for the plant context.)""";
        } context;
        // Symbol: drake::multibody::InverseKinematics::get_mutable_context
        struct /* get_mutable_context */ {
          // Source: drake/multibody/inverse_kinematics/inverse_kinematics.h
          const char* doc = R"""(Getter for the mutable plant context.)""";
        } get_mutable_context;
        // Symbol: drake::multibody::InverseKinematics::get_mutable_prog
        struct /* get_mutable_prog */ {
          // Source: drake/multibody/inverse_kinematics/inverse_kinematics.h
          const char* doc =
R"""(Getter for the optimization program constructed by InverseKinematics.)""";
        } get_mutable_prog;
        // Symbol: drake::multibody::InverseKinematics::prog
        struct /* prog */ {
          // Source: drake/multibody/inverse_kinematics/inverse_kinematics.h
          const char* doc =
R"""(Getter for the optimization program constructed by InverseKinematics.)""";
        } prog;
        // Symbol: drake::multibody::InverseKinematics::q
        struct /* q */ {
          // Source: drake/multibody/inverse_kinematics/inverse_kinematics.h
          const char* doc =
R"""(Getter for q. q is the decision variable for the generalized positions
of the robot.)""";
        } q;
      } InverseKinematics;
      // Symbol: drake::multibody::MinimumDistanceLowerBoundConstraint
      struct /* MinimumDistanceLowerBoundConstraint */ {
        // Source: drake/multibody/inverse_kinematics/minimum_distance_lower_bound_constraint.h
        const char* doc =
R"""(Constrain min(d) >= lb, namely the signed distance between all
candidate pairs of geometries (according to the logic of
SceneGraphInspector∷GetCollisionCandidates()) to be no smaller than a
specified minimum distance lb. This constraint should be bound to
decision variables corresponding to the configuration vector, q, of
the associated MultibodyPlant.

The formulation of the constraint is

SmoothOverMax( φ((dᵢ(q) - d_influence)/(d_influence - lb)) / φ(-1) ) ≤
1

where dᵢ(q) is the signed distance of the i-th pair, lb is the minimum
allowable distance, d_influence is the "influence distance" (the
distance below which a pair of geometries influences the constraint),
φ is a solvers∷MinimumValuePenaltyFunction. SmoothOverMax(d) is smooth
over approximation of max(d). We require that lb < d_influence. The
input scaling (dᵢ(q) - d_influence)/(d_influence - lb) ensures that at
the boundary of the feasible set (when dᵢ(q) == lb), we evaluate the
penalty function at -1, where it is required to have a non-zero
gradient.)""";
        // Symbol: drake::multibody::MinimumDistanceLowerBoundConstraint::MinimumDistanceLowerBoundConstraint
        struct /* ctor */ {
          // Source: drake/multibody/inverse_kinematics/minimum_distance_lower_bound_constraint.h
          const char* doc_double_mbp =
R"""(Constructs a MinimumDistanceLowerBoundConstraint.

Parameter ``plant``:
    The multibody system on which the constraint will be evaluated.
    ``plant`` cannot be a nullptr. ``plant`` must outlive this
    constraint.

Parameter ``bound``:
    The minimum allowed value, lb, of the signed distance between any
    candidate pair of geometries.

Parameter ``penalty_function``:
    The penalty function formulation.

*Default:* QuadraticallySmoothedHinge
    $Parameter ``plant_context``:

The context of ``plant``. The context should be obtained as a
subsystem context from the diagram context, where the diagram (that
contains both the MultibodyPlant and SceneGraph) creates the diagram
context. ``plant_context`` cannot be a nullptr. ``plant_context`` must
outlive this constraint. An example code of getting the plant context
is


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    {cc}
    auto diagram_context = diagram.CreateDefaultContext();
    auto plant_context = plant.GetMyMutableContextFromRoot(diagram_context.get());

.. raw:: html

    </details>

Parameter ``influence_distance_offset``:
    The difference (in meters) between the influence distance,
    d_influence, and the minimum distance, lb (see class
    documentation), namely influence_distance = bound +
    influence_distance_offset. This value must be finite and strictly
    positive, as it is used to scale the signed distances between
    pairs of geometries. Smaller values may improve performance, as
    fewer pairs of geometries need to be considered in each constraint
    evaluation. $*Default:* 0.01 meter. The chosen
    influence_distance_offset can significantly affect the runtime and
    optimization performance of using this constraint. Larger values
    result in more expensive collision checks (since more potential
    collision candidates must be considered) and may result in worse
    optimization performance (the optimizer may not be able to find a
    configuration that satisfies the constraint). In work at TRI, we
    have used much lower values (e.g. 1e-6) for
    influence_distance_offset with good results.

Raises:
    RuntimeError if ``plant`` has not registered its geometry with a
    SceneGraph object.

Raises:
    RuntimeError if influence_distance_offset = ∞.

Raises:
    RuntimeError if influence_distance_offset ≤ 0.)""";
          // Source: drake/multibody/inverse_kinematics/minimum_distance_lower_bound_constraint.h
          const char* doc_autodiff_mbp =
R"""(Overloaded constructor. Constructs the constraint using
MultibodyPlant<AutoDiffXd>.)""";
          // Source: drake/multibody/inverse_kinematics/minimum_distance_lower_bound_constraint.h
          const char* doc_collision_checker =
R"""(Overloaded constructor. Constructs the constraint with
CollisionChecker instead of MultibodyPlant.

Parameter ``collision_checker``:
    collision_checker must outlive this constraint.

Parameter ``collision_checker_context``:
    The context for the collision checker. See CollisionChecker class
    for more details.)""";
        } ctor;
        // Symbol: drake::multibody::MinimumDistanceLowerBoundConstraint::distance_bound
        struct /* distance_bound */ {
          // Source: drake/multibody/inverse_kinematics/minimum_distance_lower_bound_constraint.h
          const char* doc =
R"""(Getter for the lower bound of the minimum distance.)""";
        } distance_bound;
        // Symbol: drake::multibody::MinimumDistanceLowerBoundConstraint::influence_distance
        struct /* influence_distance */ {
          // Source: drake/multibody/inverse_kinematics/minimum_distance_lower_bound_constraint.h
          const char* doc = R"""(Getter for the influence distance.)""";
        } influence_distance;
      } MinimumDistanceLowerBoundConstraint;
      // Symbol: drake::multibody::MinimumDistanceUpperBoundConstraint
      struct /* MinimumDistanceUpperBoundConstraint */ {
        // Source: drake/multibody/inverse_kinematics/minimum_distance_upper_bound_constraint.h
        const char* doc =
R"""(Constrain min(d) <= ub, namely at least one signed distance between a
candidate pairs of geometries (according to the logic of
SceneGraphInspector∷GetCollisionCandidates()) to be no larger than a
specified ub. This constraint should be bound to decision variables
corresponding to the configuration vector, q, of the associated
MultibodyPlant.

The formulation of the constraint is

SmoothUnderMax( φ((dᵢ(q) - d_influence)/(d_influence - ub)) / φ(-1) )
≥ 1

where dᵢ(q) is the signed distance of the i-th pair, ub is the upper
bound of the minimum distance, d_influence is the "influence distance"
(the distance below which a pair of geometries influences the
constraint), φ is a solvers∷MinimumValuePenaltyFunction.
SmoothUnderMax(d) is smooth under approximation of max(d). We require
that ub < d_influence. The input scaling (dᵢ(q) -
d_influence)/(d_influence - ub) ensures that at the boundary of the
feasible set (when dᵢ(q) == ub), we evaluate the penalty function at
-1, where it is required to have a non-zero gradient.)""";
        // Symbol: drake::multibody::MinimumDistanceUpperBoundConstraint::MinimumDistanceUpperBoundConstraint
        struct /* ctor */ {
          // Source: drake/multibody/inverse_kinematics/minimum_distance_upper_bound_constraint.h
          const char* doc_double_mbp =
R"""(Constructs a MinimumDistanceUpperBoundConstraint.

Parameter ``plant``:
    The multibody system on which the constraint will be evaluated.

Parameter ``bound``:
    ``ub`` in the class documentation. The upper bound minimum signed
    distance between any candidate pair of geometries.

Parameter ``plant_context``:
    The context of ``plant``. The context should be obtained as a
    subsystem context from the diagram context, where the diagram
    (that contains both the MultibodyPlant and SceneGraph) creates the
    diagram context. ``plant_context`` cannot be a nullptr.
    ``plant_context`` must outlive this constraint. An example code of
    getting the plant context is


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    {cc}
    auto diagram_context = diagram.CreateDefaultContext();
    auto plant_context = plant.GetMyMutableContextFromRoot(diagram_context.get());

.. raw:: html

    </details>

Parameter ``penalty_function``:
    The penalty function formulation.

*Default:* QuadraticallySmoothedHinge
    $Parameter ``influence_distance_offset``:

The difference (in meters) between the influence distance,
d_influence, and the minimum distance_upper, ub (see class
documentation), namely influence_distance = bound +
influence_distance_offset. This value must be finite and strictly
positive, as it is used to scale the signed distances between pairs of
geometries. Larger value might increase the possibility of finding a
solution through gradient based nonlinear optimization. This is
because a geometry pair with distance larger than
``influence_distance`` is ignored, so is its gradient; hence the
gradient-based optimizer doesn't know to actively reduce the distance
between that pair. We strongly suggest to use a different (and larger)
``influence_distance_offset`` as the one used in
MinimumValueLowerBoundConstraint.

Raises:
    RuntimeError if ``plant`` has not registered its geometry with a
    SceneGraph object.

Raises:
    RuntimeError if influence_distance_offset = ∞.

Raises:
    RuntimeError if influence_distance_offset ≤ 0.)""";
          // Source: drake/multibody/inverse_kinematics/minimum_distance_upper_bound_constraint.h
          const char* doc_autodiff_mbp =
R"""(Overloaded constructor. Constructs the constraint using
MultibodyPlant<AutoDiffXd>.)""";
          // Source: drake/multibody/inverse_kinematics/minimum_distance_upper_bound_constraint.h
          const char* doc_collision_checker =
R"""(Overloaded constructor. Constructs the constraint with
CollisionChecker instead of MultibodyPlant.

Parameter ``collision_checker``:
    collision_checker must outlive this constraint.

Parameter ``collision_checker_context``:
    The context for the collision checker. See CollisionChecker class
    for more details.)""";
        } ctor;
        // Symbol: drake::multibody::MinimumDistanceUpperBoundConstraint::distance_bound
        struct /* distance_bound */ {
          // Source: drake/multibody/inverse_kinematics/minimum_distance_upper_bound_constraint.h
          const char* doc =
R"""(Getter for the upper bound of the minimum distance.)""";
        } distance_bound;
        // Symbol: drake::multibody::MinimumDistanceUpperBoundConstraint::influence_distance
        struct /* influence_distance */ {
          // Source: drake/multibody/inverse_kinematics/minimum_distance_upper_bound_constraint.h
          const char* doc = R"""(Getter for the influence distance.)""";
        } influence_distance;
      } MinimumDistanceUpperBoundConstraint;
      // Symbol: drake::multibody::OrientationConstraint
      struct /* OrientationConstraint */ {
        // Source: drake/multibody/inverse_kinematics/orientation_constraint.h
        const char* doc =
R"""(Constrains that the angle difference θ between the orientation of
frame A and the orientation of frame B to satisfy θ ≤ θ_bound. The
angle difference between frame A's orientation R_WA and B's
orientation R_WB is θ (θ ∈ [0, π]), if there exists a rotation axis a,
such that rotating frame A by angle θ about axis a aligns it with
frame B. Namely R_AB = I + sinθ â + (1-cosθ)â² (1) where R_AB is the
orientation of frame B expressed in frame A. â is the skew symmetric
matrix of the rotation axis a. Equation (1) is the Rodrigues formula
that computes the rotation matrix froma rotation axis a and an angle
θ, https://en.wikipedia.org/wiki/Rodrigues%27_rotation_formula If the
users want frame A and frame B to align perfectly, they can set
θ_bound = 0. Mathematically, this constraint is imposed as trace(R_AB)
≥ 2cos(θ_bound) + 1 (1) To derive (1), using Rodrigues formula R_AB =
I + sinθ â + (1-cosθ)â² where trace(R_AB) = 2cos(θ) + 1 ≥
2cos(θ_bound) + 1)""";
        // Symbol: drake::multibody::OrientationConstraint::OrientationConstraint
        struct /* ctor */ {
          // Source: drake/multibody/inverse_kinematics/orientation_constraint.h
          const char* doc_double =
R"""(Constructs an OrientationConstraint object. The frame A is fixed to a
frame A̅, with orientatation ``R_AbarA`` measured in frame A̅. The
frame B is fixed to a frame B̅, with orientation ``R_BbarB`` measured
in frame B. We constrain the angle between frame A and B to be less
than ``theta_bound``.

Parameter ``plant``:
    The MultibodyPlant on which the constraint is imposed. ``plant``
    should be alive during the lifetime of this constraint.

Parameter ``frameAbar``:
    The frame A̅ in the model to which frame A is fixed.

Parameter ``R_AbarA``:
    The orientation of frame A measured in frame A̅.

Parameter ``frameBbar``:
    The frame B̅ in the model to which frame B is fixed.

Parameter ``R_BbarB``:
    The orientation of frame B measured in frame B̅.

Parameter ``theta_bound``:
    The bound on the angle difference between frame A's orientation
    and frame B's orientation. It is denoted as θ_bound in the class
    documentation. ``theta_bound`` is in radians.

Parameter ``plant_context``:
    The Context that has been allocated for this ``plant``. We will
    update the context when evaluating the constraint.
    ``plant_context`` should be alive during the lifetime of this
    constraint.

Raises:
    RuntimeError if ``plant`` is nullptr.

Raises:
    RuntimeError if ``frameAbar`` or ``frameBbar`` does not belong to
    ``plant``.

Raises:
    RuntimeError if angle_bound < 0.

Raises:
    RuntimeError if ``plant_context`` is nullptr.)""";
          // Source: drake/multibody/inverse_kinematics/orientation_constraint.h
          const char* doc_autodiff =
R"""(Overloaded constructor. Constructs the constraint using
MultibodyPlant<AutoDiffXd>)""";
        } ctor;
      } OrientationConstraint;
      // Symbol: drake::multibody::OrientationCost
      struct /* OrientationCost */ {
        // Source: drake/multibody/inverse_kinematics/orientation_cost.h
        const char* doc =
R"""(Implements a cost of the form ``c * (1 - cos(θ))``, where θ is the
angle between the orientation of frame A and the orientation of frame
B, and ``c`` is a cost scaling.)""";
        // Symbol: drake::multibody::OrientationCost::OrientationCost
        struct /* ctor */ {
          // Source: drake/multibody/inverse_kinematics/orientation_cost.h
          const char* doc_double =
R"""(Constructs OrientationCost object.

Parameter ``plant``:
    The MultibodyPlant on which the cost is implemented. ``plant``
    should be alive during the lifetime of this cost.

Parameter ``frameAbar``:
    A frame on the MultibodyPlant.

Parameter ``R_AbarA``:
    The rotation matrix describing the orientation of frame A relative
    to Abar.

Parameter ``frameBbar``:
    A frame on the MultibodyPlant.

Parameter ``R_BbarB``:
    The rotation matrix describing the orientation of frame B relative
    to Bbar.

Parameter ``c``:
    A scalar cost weight.

Parameter ``plant_context``:
    A context for the ``plant``.

Raises:
    RuntimeError if ``plant`` is nullptr.

Raises:
    RuntimeError if ``plant_context`` is nullptr.)""";
          // Source: drake/multibody/inverse_kinematics/orientation_cost.h
          const char* doc_autodiff =
R"""(Overloaded constructor. Same as the constructor with the double
version (using MultibodyPlant<double> and Context<double>). Except the
gradient of the cost is computed from autodiff.)""";
        } ctor;
      } OrientationCost;
      // Symbol: drake::multibody::PointToLineDistanceConstraint
      struct /* PointToLineDistanceConstraint */ {
        // Source: drake/multibody/inverse_kinematics/point_to_line_distance_constraint.h
        const char* doc =
R"""(Constrain that the distance between a point P on frame B1 and another
line L on frame B2 is within a range [distance_lower, distance_upper].)""";
        // Symbol: drake::multibody::PointToLineDistanceConstraint::PointToLineDistanceConstraint
        struct /* ctor */ {
          // Source: drake/multibody/inverse_kinematics/point_to_line_distance_constraint.h
          const char* doc_double =
R"""(Constrain the distance between a point P attached to frame_point
(denoted as B1) and the line L attached to frame_line (denoted as B2)
is within the range [distance_lower, distance_upper].

Mathematically, we impose the constraint distance_lower² <=
distance(P, L)² <= distance_upper². We impose the constraint on the
distance square instead of distance directly, because the gradient of
distance is not well defined at distance=0; on the other hand, the
gradient of the distance square is well defined everywhere.

We parameterize the line using a point Q on the line, and a
directional vector n along the line.

Parameter ``plant``:
    The MultibodyPlant on which the constraint is imposed. ``plant``
    must be alive during the lifetime of this constraint.

Parameter ``frame_point``:
    The frame B1 to which the point P is attached.

Parameter ``p_B1P``:
    The position of point P measured and expressed in B1.

Parameter ``frame_line``:
    The frame B2 to which the line is attached.

Parameter ``p_B2Q``:
    Q is a point on the line, p_B2Q is the position of this point Q
    measured and expressed in B2.

Parameter ``n_B2``:
    n is the directional vector of the line, n_B2 is this vector
    measured and expressed in B2.

Parameter ``distance_lower``:
    The lower bound on the distance, must be non-negative.

Parameter ``distance_upper``:
    The upper bound on the distance, must be non-negative.

Parameter ``plant_context``:
    The Context that has been allocated for this ``plant``. We will
    update the context when evaluating the constraint.
    ``plant_context`` must be alive during the lifetime of this
    constraint.)""";
          // Source: drake/multibody/inverse_kinematics/point_to_line_distance_constraint.h
          const char* doc_autodiff =
R"""(Overloaded constructor. Same as the constructor with the double
version (using MultibodyPlant<double> and Context<double>), except the
gradient of the constraint is computed from autodiff.)""";
        } ctor;
      } PointToLineDistanceConstraint;
      // Symbol: drake::multibody::PointToPointDistanceConstraint
      struct /* PointToPointDistanceConstraint */ {
        // Source: drake/multibody/inverse_kinematics/point_to_point_distance_constraint.h
        const char* doc =
R"""(Constrain that the distance between a point P1 on frame B1 and another
point P2 on frame B2 is within a range [distance_lower,
distance_upper].)""";
        // Symbol: drake::multibody::PointToPointDistanceConstraint::PointToPointDistanceConstraint
        struct /* ctor */ {
          // Source: drake/multibody/inverse_kinematics/point_to_point_distance_constraint.h
          const char* doc_double =
R"""(Constrain that the distance between a point P1 attached to frame B1
and another point P2 attached to frame B2 is within the range
[distance_lower, distance_upper]. Mathematically, we impose the
constraint distance_lower² <= distance(P1, P2)² <= distance_upper². We
impose the constraint on the distance square instead of distance
directly, because the gradient of distance is not well defined at
distance=0, the gradient of the distance square is well defined
everywhere.

Parameter ``plant``:
    The MultibodyPlant on which the constraint is imposed. ``plant``
    should be alive during the lifetime of this constraint.

Parameter ``frame1``:
    The frame in which P1 is attached to.

Parameter ``p_B1P1``:
    The position of P1 measured and expressed in B1.

Parameter ``frame2``:
    The frame in which P2 is attached to.

Parameter ``p_B2P2``:
    The position of P2 measured and expressed in B2.

Parameter ``distance_lower``:
    The lower bound on the distance, must be non-negative.

Parameter ``distance_upper``:
    The upper bound on the distance, must be non-negative.

Parameter ``plant_context``:
    The Context that has been allocated for this ``plant``. We will
    update the context when evaluating the constraint.
    ``plant_context`` should be alive during the lifetime of this
    constraint.)""";
          // Source: drake/multibody/inverse_kinematics/point_to_point_distance_constraint.h
          const char* doc_autodiff =
R"""(Overloaded constructor. Same as the constructor with the double
version (using MultibodyPlant<double> and Context<double>), except the
gradient of the constraint is computed from autodiff.)""";
        } ctor;
      } PointToPointDistanceConstraint;
      // Symbol: drake::multibody::PolyhedronConstraint
      struct /* PolyhedronConstraint */ {
        // Source: drake/multibody/inverse_kinematics/polyhedron_constraint.h
        const char* doc =
R"""(Constrain the position of points P1, P2, ..., Pn to satisfy the
constraint A * [p_FP1; p_FP2; ...; p_FPn] <= b, where p_FPi is the
position of point Pi measured and expressed in frame F. Notice the
constraint is imposed on the stacked column vector [p_FP1; p_FP2; ...;
p_FPn], not on each individual point.)""";
        // Symbol: drake::multibody::PolyhedronConstraint::PolyhedronConstraint
        struct /* ctor */ {
          // Source: drake/multibody/inverse_kinematics/polyhedron_constraint.h
          const char* doc_double =
R"""(Construct the constraint that the position of P1, ..., Pn satisfy A *
[p_FP1; p_FP2; ...; p_FPn] <= b.

Parameter ``plant``:
    The MultibodyPlant on which the constraint is imposed. ``plant``
    should be alive during the lifetime of this constraint.

Parameter ``frameF``:
    The frame in which the position P is measured and expressed

Parameter ``frameG``:
    The frame in which the point P is rigidly attached.

Parameter ``p_GP``:
    p_GP.col(i) is the position of the i'th point Pi measured and
    expressed in frame G.

Parameter ``A``:
    We impose the constraint A * [p_FP1; p_FP2; ...; p_FPn] <= b.

Precondition:
    A.cols() = 3 * p_GP.cols();

Parameter ``b``:
    We impose the constraint A * [p_FP1; p_FP2; ...; p_FPn] <= b

Parameter ``plant_context``:
    The Context that has been allocated for this ``plant``. We will
    update the context when evaluating the constraint.
    ``plant_context`` should be alive during the lifetime of this
    constraint.)""";
          // Source: drake/multibody/inverse_kinematics/polyhedron_constraint.h
          const char* doc_autodiff =
R"""(Overloaded constructor. Same as the constructor with the double
version (using MultibodyPlant<double> and Context<double>). Except the
gradient of the constraint is computed from autodiff.)""";
        } ctor;
      } PolyhedronConstraint;
      // Symbol: drake::multibody::PositionConstraint
      struct /* PositionConstraint */ {
        // Source: drake/multibody/inverse_kinematics/position_constraint.h
        const char* doc =
R"""(Constrains the position of a point Q, rigidly attached to a frame B,
to be within a bounding box measured and expressed in frame A. Namely
p_AQ_lower <= p_AQ <= p_AQ_upper.

Note that p_BQ may or may not be a decision variable. Common use cases
include: 1. We want a specified point Q on the frame B to be within a
bounding box. In this case, p_BQ is specified and not a decision
variable. 2. We want some point Q on the frame B to be within a
bounding box, but we don't know the exact position of Q on the frame
B. For example, we want some point on the robot palm to touch a table,
but we don't care which point on the robot palm. In this case, p_BQ is
a decision variable, and we need an additional constraint to say "Q is
on the surface of the robot palm".

When p_BQ is a decision variable (i.e., it is *not* specified in the
ctor), the constraint is evaluated on the vector x = [q, p_BQ]. When
p_BQ is not a decision variable (i.e. it *is* specified
non-`nullopt`in the ctor), the constraint is evaluated on the vector x
= q.)""";
        // Symbol: drake::multibody::PositionConstraint::PositionConstraint
        struct /* ctor */ {
          // Source: drake/multibody/inverse_kinematics/position_constraint.h
          const char* doc_double =
R"""(Constructs PositionConstraint object.

Parameter ``plant``:
    The MultibodyPlant on which the constraint is imposed. ``plant``
    should be alive during the lifetime of this constraint.

Parameter ``frameA``:
    The frame in which point Q's position is measured.

Parameter ``p_AQ_lower``:
    The lower bound on the position of point Q, measured and expressed
    in frame A.

Parameter ``p_AQ_upper``:
    The upper bound on the position of point Q, measured and expressed
    in frame A.

Parameter ``frameB``:
    The frame to which point Q is rigidly attached.

Parameter ``p_BQ``:
    The position of the point Q, rigidly attached to frame B, measured
    and expressed in frame B. If set to nullopt, then p_BQ is also a
    decision variable.

Parameter ``plant_context``:
    The Context that has been allocated for this ``plant``. We will
    update the context when evaluating the constraint.
    ``plant_context`` should be alive during the lifetime of this
    constraint.

Precondition:
    ``frameA`` and ``frameB`` must belong to ``plant``.

Precondition:
    p_AQ_lower(i) <= p_AQ_upper(i) for i = 1, 2, 3.

Raises:
    RuntimeError if ``plant`` is nullptr.

Raises:
    RuntimeError if ``plant_context`` is nullptr.)""";
          // Source: drake/multibody/inverse_kinematics/position_constraint.h
          const char* doc_autodiff =
R"""(Overloaded constructor. Same as the constructor with the double
version (using MultibodyPlant<double> and Context<double>). Except the
gradient of the constraint is computed from autodiff.)""";
          // Source: drake/multibody/inverse_kinematics/position_constraint.h
          const char* doc_double_Abar =
R"""(Overloaded constructor. Except that the constructor takes in a frame
A̅ and a pose X_AAbar between the frame A and A̅. We will constrain
the position of point Q expressed in the frame A to lie within a
bounding box of A.

Parameter ``plant``:
    The MultibodyPlant on which the constraint is imposed. ``plant``
    should be alive during the lifetime of this constraint.

Parameter ``frameAbar``:
    The frame A̅ in which point Q's position is measured.

Parameter ``X_AbarA``:
    relative transform between the frame A̅ and A. If empty, then we
    use identity transform.

Parameter ``p_AQ_lower``:
    The lower bound on the position of point Q, measured and expressed
    in frame A.

Parameter ``p_AQ_upper``:
    The upper bound on the position of point Q, measured and expressed
    in frame A.

Parameter ``frameB``:
    The frame to which point Q is rigidly attached.

Parameter ``p_BQ``:
    The position of the point Q, rigidly attached to frame B, measured
    and expressed in frame B. If set to nullopt, then p_BQ is also a
    decision variable.

Parameter ``plant_context``:
    The Context that has been allocated for this ``plant``. We will
    update the context when evaluating the constraint.
    ``plant_context`` should be alive during the lifetime of this
    constraint.

Precondition:
    ``frameA`` and ``frameB`` must belong to ``plant``.

Precondition:
    p_AQ_lower(i) <= p_AQ_upper(i) for i = 1, 2, 3.

Raises:
    RuntimeError if ``plant`` is nullptr.

Raises:
    RuntimeError if ``plant_context`` is nullptr.)""";
          // Source: drake/multibody/inverse_kinematics/position_constraint.h
          const char* doc_autodiff_Abar =
R"""(Overloaded constructor. Same as the constructor with the double
version (using MultibodyPlant<double> and Context<double>). Except the
gradient of the constraint is computed from autodiff.)""";
        } ctor;
      } PositionConstraint;
      // Symbol: drake::multibody::PositionCost
      struct /* PositionCost */ {
        // Source: drake/multibody/inverse_kinematics/position_cost.h
        const char* doc =
R"""(Implements a cost of the form (p_AP - p_AQ)ᵀ C (p_AP - p_AQ), where
point P is specified relative to frame A and point Q is specified
relative to frame B, and the cost is evaluated in frame A.)""";
        // Symbol: drake::multibody::PositionCost::PositionCost
        struct /* ctor */ {
          // Source: drake/multibody/inverse_kinematics/position_cost.h
          const char* doc_double =
R"""(Constructs PositionCost object.

Parameter ``plant``:
    The MultibodyPlant on which the cost is implemented. ``plant``
    should be alive during the lifetime of this cost.

Parameter ``frameA``:
    The frame in which point P's position is measured.

Parameter ``p_AP``:
    The point P.

Parameter ``frameB``:
    The frame in which point Q's position is measured.

Parameter ``p_BQ``:
    The point Q.

Parameter ``C``:
    A 3x3 matrix representing the cost in quadratic form.

Parameter ``plant_context``:
    A context for the ``plant``.

Raises:
    RuntimeError if ``plant`` is nullptr.

Raises:
    RuntimeError if ``plant_context`` is nullptr.)""";
          // Source: drake/multibody/inverse_kinematics/position_cost.h
          const char* doc_autodiff =
R"""(Overloaded constructor. Same as the constructor with the double
version (using MultibodyPlant<double> and Context<double>). Except the
gradient of the cost is computed from autodiff.)""";
        } ctor;
      } PositionCost;
      // Symbol: drake::multibody::SelectDataForCollisionConstraintFunction
      struct /* SelectDataForCollisionConstraintFunction */ {
        // Source: drake/multibody/inverse_kinematics/differential_inverse_kinematics_system.h
        const char* doc =
R"""((Advanced) Filters clearance data for defining a collision constraint,
as used by DifferentialInverseKinematicsSystem∷CollisionConstraint.
This provides a mechanism for ignoring certain clearance rows (i.e.,
collision hazards) in case they are known to be not relevant to the
active dofs under control here. (For example, this might be the case
when this system is controlling only certain limbs of a robot, but the
collision checker contains the whole robot.)

Parameter ``active_dof``:
    indicates active degrees of freedom.

Parameter ``robot_clearance``:
    the clearance summary for the current robot state.

Parameter ``dist_out``:
    [out] On return, contains the distances. If vector zero size on
    return, no collision constraint will be added. Guaranteed to be
    non-null on entry.

Parameter ``ddist_dq_out``:
    [out] On return, contains motion derivatives. On return, the
    number of columns must match the number of active degrees of
    freedom (or may also be zero size when dist_out is zero size).
    Guaranteed to be non-null on entry.)""";
      } SelectDataForCollisionConstraintFunction;
      // Symbol: drake::multibody::UnitQuaternionConstraint
      struct /* UnitQuaternionConstraint */ {
        // Source: drake/multibody/inverse_kinematics/unit_quaternion_constraint.h
        const char* doc =
R"""(Constrains the quaternion to have a unit length.

Note:
    : It is highly recommended that in addition to adding this
    constraint, you also call MathematicalProgram∷SetInitialGuess(),
    e.g.


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    // Set a non-zero initial guess to help avoid singularities.
    prog_->SetInitialGuess(q_.segment<4>(quaternion_start),
    Eigen∷Vector4d{1, 0, 0, 0});

.. raw:: html

    </details>)""";
        // Symbol: drake::multibody::UnitQuaternionConstraint::UnitQuaternionConstraint
        struct /* ctor */ {
          // Source: drake/multibody/inverse_kinematics/unit_quaternion_constraint.h
          const char* doc = R"""()""";
        } ctor;
      } UnitQuaternionConstraint;
    } multibody;
  } drake;
} pydrake_doc_multibody_inverse_kinematics;

#if defined(__GNUG__)
#pragma GCC diagnostic pop
#endif
