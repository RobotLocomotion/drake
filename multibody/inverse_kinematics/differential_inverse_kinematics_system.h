#pragma once

#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/name_value.h"
#include "drake/common/string_unordered_map.h"
#include "drake/planning/collision_checker.h"
#include "drake/planning/dof_mask.h"
#include "drake/planning/joint_limits.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace multibody {

/** The %DifferentialInverseKinematicsSystem takes as input desired cartesian
poses (or cartesian velocities) for an arbitrary number of "goal" frames on the
robot, and produces a generalized velocity command as output to move the goal
frames toward the desired state. This system is stateless, but is intended to be
clocked at a known, fixed time step Δt by evaluating its output port at integer
multiples of Δt.

The velocity command is computed by solving a MathematicalProgram optimization
problem, consisting of:
- one primary objective (LeastSquaresCost),
- typically a secondary objective (JointCenteringCost) to resolve the nullspace
  within the primary objective, and
- various optional constraints:
  - CartesianPositionLimitConstraint
  - CartesianVelocityLimitConstraint
  - CollisionConstraint
  - JointVelocityLimitConstraint

In brief, we solve for `v_next` such that `Jv_TGs * v_next` is close to `Vd_TGs`
subject to the constraints, where:
- v_next is the generalized velocity command on the output port, which has the
  dimension of the number of active degrees of freedom,
- Vd_TGs is the desired spatial velocities of the goal frames (when desired
  positions are input, the desired velocity is inferred using the difference
  from the current position vs the time step),
- Jv_TGs is the jacobian relating spatial velocities to generalized velocities,
  i.e., V_TGs (rows) with respect to v_active (cols) -- v_active is the subset
  of generalized velocities for the active degrees of freedom.

For an introduction to differential inverse kinematics via optimization,
see section 10.6 of:
https://manipulation.csail.mit.edu/pick.html#diff_ik_w_constraints

@system
name: DifferentialInverseKinematicsSystem
input_ports:
- position
- nominal_posture
- desired_cartesian_velocities (optional)
- desired_cartesian_poses (optional)
output_ports:
- commanded_velocity
@endsystem

Port `position` accepts the current generalized position (for the full `plant`,
not just the active dofs).

Port `desired_cartesian_velocities` accepts desired cartesian velocities, typed
as systems::BusValue where key is the name of the frame to track and the value
is the SpatialVelocity<double> w.r.t the task frame. Frame names should be
provided as fully-scoped names (`model_instance::frame`).

Port `desired_cartesian_poses` accepts desired cartesian poses, typed as
systems::BusValue where key is the name of the frame to track and the
value is the math::RigidTransformd spatial pose w.r.t the task frame.
Frame names should be provided as fully-scoped names (`model_instance::frame`).

Port `nominal_posture` accepts a generalized position to be used to handle
nullspace resolution; this has the dimension of the full degrees of freedom
(not just active dofs). Typical choices for setting this input would be to use a
constant "home" reference posture, or to use a dynamically changing recent
posture.

Port `commanded_velocity` emits generalized velocity that realizes the desired
cartesian velocities or poses within the constraints. This has the dimension of
the number of active degrees of freedom.

Either `desired_cartesian_velocities` or `desired_cartesian_poses` must be
connected. Connecting both ports will result in an exception.

@note There is no consistency check to ensure that the frames being tracked are
the same across multiple time steps.

@warning This class only works correctly when the plant has v = q̇.

= Notation =

The implementation uses "monogram notation" abbreviations throughout. See
https://drake.mit.edu/doxygen_cxx/group__multibody__quantities.html for
details. The relevant frame names are:
- B: base frame
- Gi: the i'th goal frame (per the desired_cartesian_... input port)
- T: task frame
- W: world frame

To denote desired spatial velocities, we use a "d" suffix (i.e., "Vd").
Quantities like Vd_TGi (and therefore also Vd_TGlist and Vd_TGs) refer to the
desired velocity, not the current velocity. Other quantities without the "d"
subscript (e.g., X_TGi, Jv_TGi, etc.) refer to the current kinematics.

When 's' is used as a suffix (e.g., 'Vd_TGs'), it refers to the stack of all
goals one after another, e.g., Vd_TGs refers to the concatenation of all Vd_TGi
in Vd_TGlist. You can think of the 's' either as an abbreviation for "stack" or
as a plural.

We also use the letter 'S' to refer a "multibody system", in our case the robot
portion of the controller plant (not including the environment), as defined by
the collision_checker. For example, use the notation 'Scm' to denote the robot's
center of mass.

@ingroup control_systems */
class DifferentialInverseKinematicsSystem final
    : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DifferentialInverseKinematicsSystem);

  // These nested classes are defined later in this file.
  struct CallbackDetails;
  class Recipe;
  class Ingredient;

  // These nested classes are defined later in this file.
  // Keep this list in alphabetical order.
  class CartesianPositionLimitConstraint;
  class CartesianVelocityLimitConstraint;
  class CollisionConstraint;
  class JointCenteringCost;
  class JointVelocityLimitConstraint;
  class LeastSquaresCost;

  /** (Advanced) Constructs the DifferentialInverseKinematicsSystem with a
  user-provided recipe for the mathematical program formulation.
  @param recipe Specifies the mathematical formation and its parameters.
  @param task_frame Specifies the task frame name (i.e., "T") that cartesian
    goal poses are expressed in. Frame names should be provided as fully-scoped
    names (`model_instance::frame`). This must be a frame in
    `collision_checker.plant()`.
  @param collision_checker Specifies the plant and collision model to use.
  @param active_dof Specifies which generalized velocities of the plant are
    commanded by this system and appear on the "commanded_velocity" output port.
  @param time_step Specifies Δt, the time horizon assumed when relating position
    values to velocity values. The output commands from this system should be
    sent to the robot at this period.
  @param K_VX Specifies a scale factor used to convert `desired_cartesian_poses`
    input to velocities. The i'th desired cartesian velocity is computed as:
      `Vd_TGi = K_VX * (X_TGi_requested - X_TGi_current) / Δt`
    A typical value is 1.0 so that the desired velocity exactly matches what's
    necessary to move to the request pose in the next step, but may be tuned
    smaller (e.g., 0.5) to mitigate overshoot. This value is not used when the
    `desired_cartesian_velocities` input port is being used for commands.
  @param Vd_TG_limit A clamping limit applied to desired cartesian velocities.
    All desired cartesian velocities (whether specified explicitly on the
    `desired_cartesian_velocities` input port, or inferred from the
    `desired_cartesian_poses` input port) are "clamped"; the absolute value of
    each measure in the clamped spatial velocity will be no greater than the
    corresponding measure in Vd_TG_limit. This _clamped_ velocity becomes the
    input to the optimization problem. (NOTE: Clamping for
    `desired_cartesian_velocities` is not yet implemented, but will be soon.)
    If the CartesianVelocityLimitConstraint is in use, typically this value
    should be the same as that ingredient's V_next_TG_limit. */
  DifferentialInverseKinematicsSystem(
      std::shared_ptr<const Recipe> recipe, std::string_view task_frame,
      std::shared_ptr<const planning::CollisionChecker> collision_checker,
      const planning::DofMask& active_dof, double time_step, double K_VX,
      const SpatialVelocity<double>& Vd_TG_limit);

  ~DifferentialInverseKinematicsSystem() final;

  /** Gets the mathematical formulation recipe. */
  const Recipe& recipe() const { return *recipe_; }

  /** Gets the frame assumed on the desired_cartesian_poses input port. */
  const Frame<double>& task_frame() const { return *task_frame_; }

  /** Gets the plant used by the controller. */
  const MultibodyPlant<double>& plant() const {
    return collision_checker_->plant();
  }

  /** Gets the collision checker used by the controller. */
  const planning::CollisionChecker& collision_checker() const {
    return *collision_checker_;
  }

  /** Gets the mask of active DOFs in plant() that are being controlled. */
  const planning::DofMask& active_dof() const { return active_dof_; }

  /** Gets the time step used by the controller. */
  double time_step() const { return time_step_; }

  /** Gets the gain factor used to convert desired cartesian poses to
  velocities. */
  double K_VX() const { return K_VX_; }

  /** Gets the clamping limit applied to inferred desired cartesian velocities.
   */
  const SpatialVelocity<double>& Vd_TG_limit() const { return Vd_TG_limit_; }

  /** Returns the input port for the joint positions. */
  const systems::InputPort<double>& get_input_port_position() const {
    return this->get_input_port(input_port_index_position_);
  }

  /** Returns the input port for the nominal joint positions to be used to
  handle nullspace resolution. This has the dimension of the full
  `plant.num_positions()`; non-active dofs will be ignored. */
  const systems::InputPort<double>& get_input_port_nominal_posture() const {
    return this->get_input_port(input_port_index_nominal_posture_);
  }

  /** Returns the input port for the desired cartesian poses (of type
  systems::BusValue containing math::RigidTransformd). */
  const systems::InputPort<double>& get_input_port_desired_cartesian_poses()
      const {
    return this->get_input_port(input_port_index_desired_cartesian_poses_);
  }

  /** Returns the input port for the desired cartesian velocities (of type
  systems::BusValue containing SpatialVelocity). */
  const systems::InputPort<double>&
  get_input_port_desired_cartesian_velocities() const {
    return this->get_input_port(input_port_index_desired_cartesian_velocities_);
  }

  /** Returns the output port for the generalized velocity command that realizes
  the desired poses within the constraints. The size is equal to
  `get_active_dof().count()`. */
  const systems::OutputPort<double>& get_output_port_commanded_velocity()
      const {
    return this->get_output_port(output_port_index_commanded_velocity_);
  }

 private:
  struct CartesianDesires;
  void PrepareMultibodyContext(const systems::Context<double>&,
                               systems::Context<double>*) const;
  void PrepareCartesianDesires(const systems::Context<double>&,
                               CartesianDesires*) const;

  void CalcCommandedVelocity(const systems::Context<double>& context,
                             systems::BasicVector<double>* output) const;

  // Constructor arguments.
  const std::shared_ptr<const Recipe> recipe_;
  const std::shared_ptr<const planning::CollisionChecker> collision_checker_;
  const planning::DofMask active_dof_;
  const double time_step_;
  const double K_VX_;
  const SpatialVelocity<double> Vd_TG_limit_;

  // Derived from constructor arguments.
  const Frame<double>* const task_frame_;

  // LeafSystem plumbing, set once during the constructor and then never changed
  // again.
  systems::InputPortIndex input_port_index_position_;
  systems::InputPortIndex input_port_index_nominal_posture_;
  systems::InputPortIndex input_port_index_desired_cartesian_poses_;
  systems::InputPortIndex input_port_index_desired_cartesian_velocities_;
  systems::OutputPortIndex output_port_index_commanded_velocity_;
  systems::CacheIndex plant_context_cache_index_;
  systems::CacheIndex cartesian_desires_cache_index_;
  systems::CacheIndex collision_checker_context_scratch_index_;
};

/** (Internal use only) A group of common arguments relevant to multiple
different costs and constraints within the DifferentialInverseKinematicsSystem
program formulation. Think of this struct as a short-lived pack of function
arguments that is set once and then processed by multiple helper functions; it
is not intended to be a long-lived abstract data type. */
struct DifferentialInverseKinematicsSystem::CallbackDetails {
  /** The mutable, work-in-progress optimization program. */
  solvers::MathematicalProgram& mathematical_program;

  /** The decision variables being optimized. This has the dimension of the
  number of active degrees of freedom (see `active_dof`). */
  const solvers::VectorXDecisionVariable& v_next;

  /** A context for the control plant, set to current positions.
  (At the moment, velocities are zero but that might change down the road.) */
  const systems::Context<double>& plant_context;

  /** The collision checker for the robot being controlled.
  Note that its robot_model_instances() accessor also partitions which parts of
  the `plant` are the robot model vs its environment. */
  const planning::CollisionChecker& collision_checker;

  /** A mutable context for the collision checker. */
  planning::CollisionCheckerContext& collision_checker_context;

  /** The active degrees of freedom in `collision_checker.plant()`. */
  const planning::DofMask& active_dof;

  /** The control rate for DifferentialInverseKinematicsSystem (the pace at
  which velocity commands are expected to be applied). */
  const double time_step;

  /** The current value of the `nominal_posture` input port. This has the
  dimension of the full degrees of freedom (not just active dofs). */
  const Eigen::VectorXd& nominal_posture;

  /** The list of frames being controlled. */
  std::vector<const Frame<double>*> frame_list;

  /** The current poses of the goal frames. */
  const std::vector<math::RigidTransformd>& X_TGlist;

  /** The desired velocities of the goal frames. */
  const std::vector<SpatialVelocity<double>> Vd_TGlist;

  /** The jacobian relating spatial velocities to generalized velocities, i.e.,
  V_TGs (rows) with respect to v_active (cols). */
  const Eigen::MatrixXd& Jv_TGs;
};

/** (Internal use only) A user-provided set of constraint(s) and/or cost(s) for
a DifferentialInverseKinematicsSystem recipe, to allow for user customization of
the mathematical program formulation.

TODO(jeremy-nimmer) In the future, we should remove "internal use only" so that
users can officially bake their own recipes for
DifferentialInverseKinematicsSystem.
*/
class DifferentialInverseKinematicsSystem::Ingredient {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Ingredient);
  virtual ~Ingredient();

  /** Adds this ingredient into DifferentialInverseKinematicsSystem's
  mathematical program, returning the listing of bindings for all newly-added
  costs and/or constraints.

  @param[in,out] details A group of arguments commonly used by most costs and
    constraints. */
  virtual std::vector<solvers::Binding<solvers::EvaluatorBase>> AddToProgram(
      CallbackDetails* details) const = 0;

  // TODO(jeremy-nimmer) Add an 'update program' virtual method (which accepts
  // the vector-of-bindings returned by `AddToProgram` plus the new `details`),
  // that can modify the costs/constraints in place, instead of creating new
  // ones. When not overridden in a subclass, the base implementation can simply
  // remove the old bindings from the program and then delegate to 'add to
  // program' to re-add them.

 protected:
  Ingredient() = default;

  /** Constructs a block-diagonal matrix selecting Cartesian velocity axes
  for each goal frame, based on per-frame axis masks. This operation is common
  to several ingredients so is offered as a utility function this base class.

  Each controlled frame Gi has a 6×6 diagonal mask matrix selecting which
  spatial velocity components (angular, linear) are active. These individual
  mask matrices are stacked into a single block-diagonal matrix for use
  in constraints and cost terms.

  For example, given N goal frames, the output matrix has size 6N × 6N and
  has the form:

    [ diag(mask_G₁)     0            ...     0         ]
    [     0         diag(mask_G₂)    ...     0         ]
    [    ...           ...           ...    ...        ]
    [     0             0            ...  diag(mask_Gₙ)]

  @param frame_list The list of controlled frames {Gi}, borrowed from the plant.
  @param cartesian_axis_masks Map from fully-scoped frame names to their 6D axis
  mask.

  @return 6N × 6N block-diagonal matrix applying the per-frame axis mask. */
  static Eigen::DiagonalMatrix<double, Eigen::Dynamic>
  BuildBlockDiagonalAxisSelector(
      const std::vector<const Frame<double>*>& frame_list,
      const string_unordered_map<Vector6d>& cartesian_axis_masks);
};

// TODO(jeremy-nimmer) Should this simply be a composite (inherit from the
// Ingredient base class)?
/** A recipe collects a list of ingredients for
DifferentialInverseKinematicsSystem, allowing the user to customize the program
being solved. */
class DifferentialInverseKinematicsSystem::Recipe final {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Recipe);
  Recipe();
  ~Recipe();

  void AddIngredient(std::shared_ptr<const Ingredient> ingredient) {
    ingredients_.push_back(std::move(ingredient));
  }

  /* Reports the number of ingredients in this recipe. */
  int num_ingredients() const { return ssize(ingredients_); }

  /* Returns the ith ingredient.
  @pre `0 <= i < num_ingredients()`. */
  const Ingredient& ingredient(int i) const {
    DRAKE_THROW_UNLESS(i >= 0 && i < num_ingredients());
    return *ingredients_[i];
  }

  /** Calls DifferentialInverseKinematicsSystem::Ingredient::AddToProgram on all
  of the ingredients in this recipe. */
  std::vector<solvers::Binding<solvers::EvaluatorBase>> AddToProgram(
      CallbackDetails* details) const;

 private:
  std::vector<std::shared_ptr<const Ingredient>> ingredients_;
};

// ============================================================================
// The available objectives (costs).
// ============================================================================

/** Provides a primary DifferentialInverseKinematicsSystem objective to minimize
`G*| S * (Vd_TGs - Jv_TGs * v_next)|²`, also known as the "least squares"
formulation.

Where:
- G is `cartesian_qp_weight`; this coefficient can be used to balance the
  relative weight of multiple costs in the mathematical program.
- S is a block-diagonal selector matrix applying per-frame axis masks
  to filter out components that are not tracked.
Mask behavior:
- Each 6×6 block of `S` corresponds to a goal frame and is constructed from the
  frame's entry in `cartesian_axis_masks`. If a frame is not explicitly listed,
  all axes are enabled.
- Each axis mask is a binary Vector6d of the form
  [ωx, ωy, ωz, vx, vy, vz] ∈ {0, 1}⁶.*/
class DifferentialInverseKinematicsSystem::LeastSquaresCost final
    : public Ingredient {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(LeastSquaresCost);

  struct Config {
    /** Passes this object to an Archive.
    Refer to @ref yaml_serialization "YAML Serialization" for background. */
    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(DRAKE_NVP(cartesian_qp_weight));
      a->Visit(DRAKE_NVP(cartesian_axis_masks));
      a->Visit(DRAKE_NVP(use_legacy_implementation));
    }

    /** 'G' in the class overview. Must be non-negative and finite. */
    double cartesian_qp_weight{1.0};

    /** Map from fully-scoped frame names to their 6D spatial velocity axis
    mask. Each Vector6d is a binary mask [ωx, ωy, ωz, vx, vy, vz] indicating
    which Cartesian velocity components (angular and translational) are being
    tracked. All elements must be set to either 0 or 1, and at least one element
    must be 1. */
    string_unordered_map<Vector6d> cartesian_axis_masks;

    // TODO(SeanCurtis-TRI) Kill this parameter. See anzu#17024.
    /** This is a temporary parameter intended for backwards compatibility. Do
    not change this value (unless you really, really know what you're doing)! */
    bool use_legacy_implementation{false};
  };

  explicit LeastSquaresCost(const Config& config);
  ~LeastSquaresCost() final;

  /** Returns the current config. */
  const Config& GetConfig() const { return config_; }

  /** Replaces the config set in the constructor. */
  void SetConfig(const Config& config);

  std::vector<solvers::Binding<solvers::EvaluatorBase>> AddToProgram(
      CallbackDetails* details) const final;

 private:
  Config config_;
};

/** Provides a secondary minimization objective. There will almost inevitably be
times when the jacobian `Jv_TGs` is not full rank. Sometimes because of current
position values and sometimes because `Jv_TGs` is rectangular by construction.
At those times, there's no longer a single solution and relying on the
mathematical program to pick a reasonable solution from the large space is
unreasonable.

This cost is intended to work in conjunction with the primary cost (e.g.,
LeastSquaresCost). When `Jv_TGs` is not full rank, this provides guidance for
selecting a _unique_ velocity from the space of possible solutions. If the
primary cost produces a space of optimal velocities, this secondary cost will
select the velocity from that space that brings the arm closer to the "nominal
posture":

   |P⋅(v_next - K⋅(nominal_posture_active - q_active))|²

where,
- `K` is the proportional gain of a joint-space controller that pulls toward the
  nominal posture.
- `P` is the linear map from generalized velocities to the _null space_ of
  masked Jacobian `S ⋅ Jv_TGs`. Mapping to the null space, allows refinement of
  the velocity without changing the primary cost.
- S is a block-diagonal matrix of axis masks, enabling per-axis tracking.

Notes:
- When combined with a primary cost, the primary cost should be weighed far more
  heavily to keep this secondary cost from interfering. A factor of 100X or so
  is advisable. (See LeastSqauresCost::cartesian_qp_weight for tuning the gain
  on the primary objective.)
- For this cost to behave as expected, it is critical that the null space of
  `S ⋅ Jv_TGs` be a subspace of the null space used by the primary cost. The
  simplest way is to make sure both costs receive the same axis masks and
  jacobian. Failure to do so would lead this cost to _fight_ the primary cost
  instead of complementing it.

For more details on this cost, see:
https://manipulation.csail.mit.edu/pick.html#diff_ik_w_constraints#joint_centering

TODO(sean.curtis) As with the other costs, this should also have a scaling
weight (e.g. what `G` is for LeastSquaresCost). */
class DifferentialInverseKinematicsSystem::JointCenteringCost final
    : public Ingredient {
 public:
  struct Config {
    /** Passes this object to an Archive.
    Refer to @ref yaml_serialization "YAML Serialization" for background. */
    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(DRAKE_NVP(posture_gain));
      a->Visit(DRAKE_NVP(cartesian_axis_masks));
    }

    /** The proportional gain matrix `K` is a diagonal matrix with the diagonal
    set to this value (i.e., all joints use the same `posture_gain`). Must be
    non-negative. */
    double posture_gain{1.0};

    /** Map from fully-scoped frame names to their 6D spatial velocity axis
    mask. Each Vector6d is a binary mask [ωx, ωy, ωz, vx, vy, vz] indicating
    which Cartesian velocity components (angular and translational) are being
    tracked. All elements must be set to either 0 or 1, and at least one element
    must be 1. */
    string_unordered_map<Vector6d> cartesian_axis_masks;
  };

  explicit JointCenteringCost(const Config& config);
  ~JointCenteringCost() final;

  /** Returns the current config. */
  const Config& GetConfig() const { return config_; }

  /** Replaces the config set in the constructor. */
  void SetConfig(const Config& config);

  std::vector<solvers::Binding<solvers::EvaluatorBase>> AddToProgram(
      CallbackDetails* details) const final;

 private:
  Config config_;
};

// ============================================================================
// The available constraints, in alphabetical order.
// ============================================================================

/** Constrains the goal frames to a cartesian bounding box:
`∀i p_TG_next_lower ≤ p_TGi + Jv_TGi[3:6] * v_next * Δt ≤ p_TG_next_upper`
where:
- p_TGi is the translation component of the i'th goal point w.r.t the task
  frame. */
class DifferentialInverseKinematicsSystem::CartesianPositionLimitConstraint
    final : public Ingredient {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(CartesianPositionLimitConstraint);

  struct Config {
    /** Passes this object to an Archive.
    Refer to @ref yaml_serialization "YAML Serialization" for background. */
    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(DRAKE_NVP(p_TG_next_lower));
      a->Visit(DRAKE_NVP(p_TG_next_upper));
    }

    /** Lower bound on p_TGi for all i. */
    Eigen::Vector3d p_TG_next_lower;

    /** Upper bound on p_TGi for all i. */
    Eigen::Vector3d p_TG_next_upper;
  };

  explicit CartesianPositionLimitConstraint(const Config& config);
  ~CartesianPositionLimitConstraint() final;

  /** Returns the current config. */
  const Config& GetConfig() const { return config_; }

  /** Replaces the config set in the constructor. */
  void SetConfig(const Config& config);

  std::vector<solvers::Binding<solvers::EvaluatorBase>> AddToProgram(
      CallbackDetails* details) const final;

 private:
  Config config_;
};

/** Constrains the spatial velocities of the goal frames:
`∀i, ∀j ∈ [0, 5]: abs(Jv_TGi * v_next)[j] ≤ V_next_TG_limit[j]`. */
class DifferentialInverseKinematicsSystem::CartesianVelocityLimitConstraint
    final : public Ingredient {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(CartesianVelocityLimitConstraint);

  struct Config {
    /** Passes this object to an Archive.
    Refer to @ref yaml_serialization "YAML Serialization" for background. */
    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(DRAKE_NVP(V_next_TG_limit));
    }

    /** This limits the absolute value of the measures of the spatial velocities
    V_next[i] for all goal frames, so must be entry-wise non-negative. If the
    "desired_cartesian_poses" input port to DifferentialInverseKinematicsSystem
    is in use, then typically this should be set to the same value as the
    Vd_TG_limit passed to that system's constructor. The element order is [ωx,
    ωy, ωz, vx, vy, vz], which matches the SpatialVelocity order. */
    Vector6d V_next_TG_limit;
  };

  explicit CartesianVelocityLimitConstraint(const Config& config);
  ~CartesianVelocityLimitConstraint() final;

  /** Returns the current config. */
  const Config& GetConfig() const { return config_; }

  /** Replaces the config set in the constructor. */
  void SetConfig(const Config& config);

  std::vector<solvers::Binding<solvers::EvaluatorBase>> AddToProgram(
      CallbackDetails* details) const final;

 private:
  Config config_;
};

/** (Advanced) Filters clearance data for defining a collision constraint, as
used by DifferentialInverseKinematicsSystem::CollisionConstraint. This provides
a mechanism for ignoring certain clearance rows (i.e., collision hazards) in
case they are known to be not relevant to the active dofs under control here.
(For example, this might be the case when this system is controlling only
certain limbs of a robot, but the collision checker contains the whole robot.)

@param active_dof indicates active degrees of freedom.
@param robot_clearance the clearance summary for the current robot state.
@param dist_out[out] On return, contains the distances. If vector zero size on
  return, no collision constraint will be added. Guaranteed to be non-null on
  entry.
@param ddist_dq_out[out] On return, contains motion derivatives. On return, the
  number of columns must match the number of active degrees of freedom (or may
  also be zero size when dist_out is zero size).
  Guaranteed to be non-null on entry. */
using SelectDataForCollisionConstraintFunction = std::function<void(
    const planning::DofMask& active_dof,
    const planning::RobotClearance& robot_clearance, Eigen::VectorXd* dist_out,
    Eigen::MatrixXd* ddist_dq_out)>;

/** Constrains the collision clearance around the robot to remain above the
safety distance:
`∀j ϕₛ ≤ ϕⱼ + ∂ϕⱼ/∂q_active * v_next * Δt`
where:
- ϕₛ is the safety_distance;
- ϕⱼ is the current signed distance between the robot and some j'th obstacle;
- ∂ϕⱼ/∂q_active is the gradient of ϕⱼ with respect to the active dof positions.
Obstacles beyond the influence distance are ignored.
The optional select_data_for_collision_constraint may be used to preprocess the
clearance data prior to adding the constraint (e.g., to ignore some parts).

TODO(jeremy-nimmer) This constraint should account for passive dof velocities as
well (i.e., v_passive), using the full ∂q (not just ∂q_active). */
class DifferentialInverseKinematicsSystem::CollisionConstraint final
    : public Ingredient {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(CollisionConstraint);

  struct Config {
    /** Passes this object to an Archive.
    Refer to @ref yaml_serialization "YAML Serialization" for background. */
    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(DRAKE_NVP(safety_distance));
      a->Visit(DRAKE_NVP(influence_distance));
    }

    /** "ϕₛ" in the class overview (in meters). Must be finite. */
    double safety_distance{0.0};

    /** Obstacles beyond the influence distance (in meters) are ignored. Must be
    non-negative. */
    double influence_distance{std::numeric_limits<double>::infinity()};
  };

  explicit CollisionConstraint(const Config& config);
  ~CollisionConstraint() final;

  /** Returns the current config. */
  const Config& GetConfig() const { return config_; }

  /** Replaces the config set in the constructor. */
  void SetConfig(const Config& config);

  /** (Advanced) Provides a mechanism for ignoring certain clearance rows. */
  void SetSelectDataForCollisionConstraintFunction(
      const SelectDataForCollisionConstraintFunction&
          select_data_for_collision_constraint);

  std::vector<solvers::Binding<solvers::EvaluatorBase>> AddToProgram(
      CallbackDetails* details) const final;

 private:
  Config config_;
  SelectDataForCollisionConstraintFunction
      select_data_for_collision_constraint_;
};

/** Constrains the generalized velocity to prevent a commanded velocity that
would push the generalized position outside its limits.

The configured joint velocity limits are not used directly. When the joint
position is near a limit, the joint velocity limit may be insufficient to
constrain `v_next` from pushing the position beyond its limits.

Instead, we use a _scaled_ velocity limit. In simple terms, the closer `q` is to
its position limit, the more velocity towards that limit is constrained. When
`q` lies at the limit boundary, the velocity in that direction would be zero.
However, the configured velocity limit should be applied when q is
"sufficiently" far away from the position limit boundary. So, we compute and
apply a scale factor to attenuate the "near" velocity limit.

The constraint is parameterized to define the domain of the attenuation; how far
away from the near limit boundary should the scaled limit be zero and how far
should it be restored to its configured value? We parameterize those two
distances as:

  - `min_margin`: the distance at which the velocity limit is scaled to be zero.
                  Must be non-negative and is typically positive.
  - `influence_margin`: the distance at which the velocity limit is
                        restored to its configured value. This value must be
                        strictly greater than `min_margin`.

The velocity limit is only ever attenuated on one boundary.

If we define `distᵢ` as the distance to the near position limit boundary, and
`near_limit` as the value of that limit, we can define the attenuation as:

```
  distᵢ = min(qᵢ_current - qᵢ_min, qᵢ_max - qᵢ_current)
  scale = clamp((distᵢ - min_margin) / (influence_margin - min_margin), 0, 1)
  scaled_near_limit = near_limit * scale.
```

Implications:
 - Because we're _attenuating_ the configured velocity limits, an infinite limit
   value won't be attenuated. It _will_ be zero when `distᵢ ≤ min_margin`.
 - Position limits define an interval for q, which we'll call P. If
   `P / 2 - min_margin < influence_margin - min_margin` then one limit will
   _always_ be attenuated to be less than its configured value. */
class DifferentialInverseKinematicsSystem::JointVelocityLimitConstraint final
    : public Ingredient {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(JointVelocityLimitConstraint);

  struct Config {
    /** Passes this object to an Archive.
    Refer to @ref yaml_serialization "YAML Serialization" for background. */
    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(DRAKE_NVP(min_margin));
      a->Visit(DRAKE_NVP(influence_margin));
    }

    /** The distance (in each joint's configuration space) at which the velocity
    limit is scaled to be zero. Must be non-negative and is typically
    positive. The units will depend on what kind of joint is being limited. */
    double min_margin{0.0025};

    /** The distance (in each joint's configuration space) at which the velocity
    limit is restored to its configured value. The units will depend on what
    kind of joint is being limited. This value must be strictly greater than
    `min_margin`. */
    double influence_margin{0.1};
  };

  JointVelocityLimitConstraint(const Config& config,
                               const planning::JointLimits& joint_limits);
  ~JointVelocityLimitConstraint() final;

  /** Returns the current config. */
  const Config& GetConfig() const { return config_; }

  /** Replaces the config set in the constructor. */
  void SetConfig(const Config& config);

  /** Returns the current joint limits. */
  const planning::JointLimits& GetJointLimits() const { return joint_limits_; }

  /** Replaces the limits set in the constructor. */
  void SetJointLimits(const planning::JointLimits& joint_limits);

  std::vector<solvers::Binding<solvers::EvaluatorBase>> AddToProgram(
      CallbackDetails* details) const final;

 private:
  Config config_;
  planning::JointLimits joint_limits_;
};

}  // namespace multibody
}  // namespace drake
