#pragma once

#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <string_view>
#include <tuple>
#include <type_traits>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_deprecated.h"
#include "drake/common/drake_export.h"
#include "drake/common/random.h"
#include "drake/geometry/scene_graph.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/contact_solvers/contact_solver_results.h"
#include "drake/multibody/plant/constraint_specs.h"
#include "drake/multibody/plant/contact_results.h"
#include "drake/multibody/plant/coulomb_friction.h"
#include "drake/multibody/plant/desired_state_input.h"
#include "drake/multibody/plant/distance_constraint_params.h"
#include "drake/multibody/plant/dummy_physical_model.h"
#include "drake/multibody/plant/multibody_plant_config.h"
#include "drake/multibody/plant/physical_model_collection.h"
#include "drake/multibody/topology/graph.h"
#include "drake/multibody/tree/force_element.h"
#include "drake/multibody/tree/frame.h"
#include "drake/multibody/tree/joint.h"
#include "drake/multibody/tree/joint_actuator.h"
#include "drake/multibody/tree/multibody_forces.h"
#include "drake/multibody/tree/multibody_tree-inl.h"
#include "drake/multibody/tree/multibody_tree_system.h"
#include "drake/multibody/tree/rigid_body.h"
#include "drake/multibody/tree/uniform_gravity_field_element.h"
#include "drake/multibody/tree/weld_joint.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace multibody {
namespace internal {

// Data stored in the cache entry for joint locking.
template <typename T>
struct JointLockingCacheData {
  // @name Dense joint locking indices.
  // Values of each will be a sorted subset of [0, num_velocities()].
  // `unlocked_velocity_indices` and `locked_velocity_indices` are disjoint and
  // their union is [0, num_velocities()].
  // @{
  // Stores indices of unlocked DoFs in the context.
  std::vector<int> unlocked_velocity_indices;
  // Stores indices of locked DoFs in the context.
  std::vector<int> locked_velocity_indices;
  // @}

  // @name Per-tree joint locking indices.
  // Each has the same size as the number of trees in the plant's topology and
  // is resized accordingly on output. For both unlocked and locked, element i
  // is a sorted subset of [0, tree_i.num_velocities()] where tree_i is the i-th
  // tree in this plant's topology. They are likewise disjoint and their union
  // is [0, tree_i.num_velocities()].
  // @{
  // Stores indices of unlocked DoFs per tree in the plant's topology.
  std::vector<std::vector<int>> unlocked_velocity_indices_per_tree;
  // Stores indices of locked DoFs per tree in the plant's topology.
  std::vector<std::vector<int>> locked_velocity_indices_per_tree;
  // @}
};

// Wrapper struct for using std::map<MultibodyConstraintId, bool> as a Value
// type for an abstract parameter.
struct ConstraintActiveStatusMap {
  std::map<MultibodyConstraintId, bool> map;
};

// Wrapper struct so that hashing works for a
// std::map<MultibodyConstraintId, DistanceConstraintParams> packed as a
// Value parameter in the context.
struct DistanceConstraintParamsMap {
  std::map<MultibodyConstraintId, DistanceConstraintParams> map;
};

// This struct contains the parameters to compute forces to enforce
// no-interpenetration between bodies by a penalty method.
// TODO(amcastro-tri): remove this struct along with penetration allowance.
struct ContactByPenaltyMethodParameters {
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(ContactByPenaltyMethodParameters);

  ContactByPenaltyMethodParameters() = default;

  // Penalty method coefficients used to compute contact forces.
  double geometry_stiffness{0};
  double dissipation{0};
  // TODO(xuchenhan-tri): Consider using std::optional instead of an illegal
  //  value as a flag.
  // An estimated time scale in which objects come to a relative stop during
  // contact.
  double time_scale{-1.0};
  // Acceleration of gravity in the model. Used to estimate penalty method
  // constants from a static equilibrium analysis.
  std::optional<double> gravity;
};

// Forward declarations for discrete_update_manager.h.
template <typename>
class DiscreteUpdateManager;
// Forward declarations for geometry_contact_data.h.
template <typename>
class GeometryContactData;
// Forward declarations for hydroelastic_contact_forces_continuous_cache_data.h.
template <typename>
struct HydroelasticContactForcesContinuousCacheData;
// Forward declarations for multibody_plant_discrete_update_manager_attorney.h.
template <typename>
class MultibodyPlantDiscreteUpdateManagerAttorney;
// Forward declarations for multibody_plant_icf_attorney.h.
template <typename>
class MultibodyPlantIcfAttorney;
// Forward declarations for multibody_plant_model_attorney.h.
template <typename>
class MultibodyPlantModelAttorney;

}  // namespace internal

// TODO(amcastro-tri): Add a section on contact models in
// contact_model_doxygen.h.
/// Enumeration for contact model options.
enum class ContactModel {
  /// Contact forces are computed using the Hydroelastic model. Contact between
  /// unsupported geometries will cause a runtime exception.
  kHydroelastic,

  /// Contact forces are computed using a point contact model,
  /// see @ref compliant_point_contact.
  kPoint,

  /// Contact forces are computed using the hydroelastic model, where possible.
  /// For most other unsupported colliding pairs, the point model from
  /// kPoint is used. See
  /// geometry::QueryObject::ComputeContactSurfacesWithFallback for more
  /// details.
  kHydroelasticWithFallback,

  /// Legacy alias. TODO(jwnimmer-tri) Deprecate this constant.
  kHydroelasticsOnly = kHydroelastic,
  /// Legacy alias. TODO(jwnimmer-tri) Deprecate this constant.
  kPointContactOnly = kPoint,
};

/// The type of the contact solver used for a discrete MultibodyPlant model.
///
/// Note: the SAP solver only fully supports scalar type `double`. For
/// scalar type `AutoDiffXd`, the SAP solver throws if any constraint (including
/// contact) is detected. As a consequence, one can only run dynamic simulations
/// without any constraints under the combination of SAP and `AutoDiffXd`. The
/// SAP solver does not support symbolic calculations.
///
/// <h2>References</h2>
///
/// - [Castro et al., 2019] Castro, A.M, Qu, A., Kuppuswamy, N., Alspach, A.,
///   Sherman, M.A., 2019. A Transition-Aware Method for the Simulation of
///   Compliant Contact with Regularized Friction. Available online at
///   https://arxiv.org/abs/1909.05700.
/// - [Castro et al., 2022] Castro A., Permenter F. and Han X., 2022. An
///   Unconstrained Convex Formulation of Compliant Contact. Available online at
///   https://arxiv.org/abs/2110.10107.
enum class DiscreteContactSolver {
  /// TAMSI solver, see [Castro et al., 2019].
  kTamsi DRAKE_DEPRECATED("2026-09-01",
                          "The TAMSI solver is deprecated for removal."),
  /// SAP solver, see [Castro et al., 2022].
  kSap,
};

/// The type of the contact approximation used for a discrete MultibodyPlant
/// model.
///
/// kTamsi, kSimilar and kLagged are all approximations to the same contact
/// model --  Compliant contact with regularized friction, refer to
/// @ref mbp_contact_modeling "Contact Modeling" for further details.
/// The key difference however, is that the kSimilar and kLagged approximations
/// are convex and therefore our contact solver has both theoretical and
/// practical convergence guarantees ---  the solver will always succeed.
/// Conversely, being non-convex, kTamsi can fail to find a solution.
///
/// kSap is also a convex model of compliant contact with regularized friction.
/// There are a couple of key differences however:
/// - Dissipation is modeled using a linear Kelvin–Voigt model, parameterized by
///   a relaxation time constant.
///   See @ref accessing_contact_properties "contact parameters".
/// - Unlike kTamsi, kSimilar and kLagged where regularization of friction is
///   parameterized by a stiction tolerance (see set_stiction_tolerance()), SAP
///   determines regularization automatically solely based on numerics. Users
///   have no control on the amount of regularization.
///
/// ## How to choose an approximation
///
/// The Hunt & Crossley model is based on physics, it is continuous and has been
/// experimentally validated. Therefore it is the preferred model to capture the
/// physics of contact.
///
/// Being approximations, kSap and kSimilar introduce a spurious
/// effect of "gliding" in sliding contact, see [Castro et al., 2023]. This
/// artifact is 𝒪(δt) but can be significant at large time steps and/or large
/// slip velocities. kLagged does not suffer from this, but introduces a "weak"
/// coupling of friction that can introduce non-negligible effects in the
/// dynamics during impacts or strong transients.
///
/// Summarizing, kLagged is the preferred approximation when strong transients
/// are not expected or don't need to be accurately resolved.
/// If strong transients do need to be accurately resolved (unusual for robotics
/// applications), kSimilar is the preferred approximation.
///
/// <h2>References</h2>
/// - [Castro et al., 2019] Castro A., Qu A., Kuppuswamy N., Alspach A.,
///   Sherman M, 2019. A Transition-Aware Method for the Simulation of
///   Compliant Contact with Regularized Friction. Available online at
///   https://arxiv.org/abs/1909.05700.
/// - [Castro et al., 2022] Castro A., Permenter F. and Han X., 2022. An
///   Unconstrained Convex Formulation of Compliant Contact. Available online at
///   https://arxiv.org/abs/2110.10107.
/// - [Castro et al., 2023] Castro A., Han X., and Masterjohn J., 2023. A Theory
///   of Irrotational Contact Fields. Available online at
///   https://arxiv.org/abs/2312.03908
enum class DiscreteContactApproximation {
  /// TAMSI solver approximation, see [Castro et al., 2019].
  kTamsi DRAKE_DEPRECATED("2026-09-01",
                          "The TAMSI solver is deprecated for removal."),
  /// SAP solver model approximation, see [Castro et al., 2022].
  kSap,
  /// Similarity approximation found in [Castro et al., 2023].
  kSimilar,
  /// Approximation in which the normal force is lagged in Coulomb's law, such
  /// that ‖γₜ‖ ≤ μ γₙ₀, [Castro et al., 2023].
  kLagged,
};

/// The kind of joint to be used to connect base bodies to world at Finalize().
/// See @ref mbp_working_with_free_bodies "Working with free bodies"
/// for definitions and discussion.
/// @see SetBaseBodyJointType() for details.
enum class BaseBodyJointType {
  kQuaternionFloatingJoint,  ///< 6 dofs, unrestricted orientation.
  kRpyFloatingJoint,         ///< 6 dofs using 3 angles; has singularity.
  kWeldJoint,                ///< 0 dofs, fixed to World.
};

/// @cond
// Helper macro to throw an exception within methods that should not be called
// post-finalize.
#define DRAKE_MBP_THROW_IF_FINALIZED() ThrowIfFinalized(__func__)

// Helper macro to throw an exception within methods that should not be called
// pre-finalize.
#define DRAKE_MBP_THROW_IF_NOT_FINALIZED() ThrowIfNotFinalized(__func__)
/// @endcond

/**
%MultibodyPlant is a Drake system framework representation (see
systems::System) for the model of a physical system consisting of a
collection of interconnected bodies.  See @ref multibody for an overview of
concepts/notation.

@system
name: MultibodyPlant
input_ports:
- actuation
- applied_generalized_force
- applied_spatial_force
- <em style="color:gray">model_instance_name[i]</em>_actuation
- <em style="color:gray">model_instance_name[i]</em>_desired_state
- <span style="color:green">geometry_query</span>
output_ports:
- state
- body_poses
- body_spatial_velocities
- body_spatial_accelerations
- generalized_acceleration
- net_actuation
- reaction_forces
- contact_results
- <em style="color:gray">model_instance_name[i]</em>_state
- <em style="color:gray">model_instance_name[i]</em>_generalized_acceleration
- <em style="color:gray">model_instance_name[i]</em>_generalized_contact_forces
- <em style="color:gray">model_instance_name[i]</em>_net_actuation
- <span style="color:green">geometry_pose</span>
- <span style="color:green">deformable_body_configuration</span>
@endsystem

The ports whose names begin with <em style="color:gray">
model_instance_name[i]</em> represent groups of ports, one for each of the
@ref model_instances "model instances", with i ∈ {0, ..., N-1} for the N
model instances. If a model instance does not contain any data of the
indicated type the port will still be present but its value will be a
zero-length vector. (Model instances `world_model_instance()` and
`default_model_instance()` always exist.)

The ports shown in <span style="color:green">green</span> are for communication
with Drake's @ref geometry::SceneGraph "SceneGraph" system for dealing with
geometry.

%MultibodyPlant provides a user-facing API for:

- @ref mbp_input_and_output_ports "Ports":
  Access input and output ports.
- @ref mbp_construction "Construction":
  Add bodies, joints, frames, force elements, and actuators.
- @ref mbp_geometry "Geometry":
  Register geometries to a provided SceneGraph instance.
- @ref mbp_contact_modeling "Contact modeling":
  Select and parameterize contact models.
- @ref mbp_state_accessors_and_mutators "State access and modification":
  Obtain and manipulate position and velocity state variables.
- @ref mbp_parameters "Parameters"
  Working with system parameters for various multibody elements.
- @ref mbp_working_with_free_bodies "Free and floating base bodies":
  Work conveniently with free (floating) bodies.
- @ref mbp_kinematic_and_dynamic_computations "Kinematics and dynamics":
  Perform @ref systems::Context "Context"-dependent kinematic and dynamic
  queries.
- @ref mbp_system_matrix_computations "System matrices":
  Explicitly form matrices that appear in the equations of motion.
- @ref mbp_introspection "Introspection":
  Perform introspection to find out what's in the %MultibodyPlant.

@anchor model_instances
                        ### Model Instances

A MultibodyPlant may contain multiple model instances. Each model instance
corresponds to a
set of bodies and their connections (joints). Model instances provide
methods to get or set the state of the set of bodies (e.g., through
GetPositionsAndVelocities() and SetPositionsAndVelocities()), connecting
controllers (through get_state_output_port()
and get_actuation_input_port()), and organizing duplicate models (read
through a parser). In fact, many %MultibodyPlant methods are overloaded
to allow operating on the entire plant or just the subset corresponding to
the model instance; for example, one GetPositions() method obtains the
generalized positions for the entire plant while another GetPositions()
method obtains the generalized positions for model instance.

Model instances are frequently defined through SDFormat files
(using the `model` tag) and are automatically created when SDFormat
files are parsed (by Parser). There are two special
multibody::ModelInstanceIndex values. The world body is always
multibody::ModelInstanceIndex 0. multibody::ModelInstanceIndex 1 is
reserved for all elements with no explicit model instance and
is generally only relevant for elements
created programmatically (and only when a model instance is not explicitly
specified). Note that Parser creates model instances (resulting in a
multibody::ModelInstanceIndex ≥ 2) as needed.

See num_model_instances(),
num_positions(),
num_velocities(), num_actuated_dofs(),
AddModelInstance() GetPositionsAndVelocities(),
GetPositions(), GetVelocities(),
SetPositionsAndVelocities(),
SetPositions(), SetVelocities(),
GetPositionsFromArray(), GetVelocitiesFromArray(),
SetPositionsInArray(), SetVelocitiesInArray(), SetActuationInArray(),
HasModelInstanceNamed(), GetModelInstanceName(),
get_state_output_port(),
get_actuation_input_port().

@anchor mbp_equations_of_motion
                        ### System dynamics

<!-- TODO(amcastro-tri): Update this documentation to include:
     - Bilateral constraints.
     - Unilateral constraints and contact. -->

The state of a multibody system `x = [q; v]` is given by its generalized
positions vector q, of size `nq` (see num_positions()), and by its
generalized velocities vector v, of size `nv` (see num_velocities()).

A %MultibodyPlant can be constructed to be either continuous or discrete. The
choice is indicated by the time_step passed to the constructor -- a non-zero
time_step indicates a discrete plant, while a zero time_step indicates
continuous. A @ref systems::Simulator "Simulator" will step a discrete plant
using the indicated time_step, but will allow a numerical integrator to choose
how to advance time for a continuous %MultibodyPlant.

We'll discuss continuous plant dynamics in this section. Discrete dynamics is
more complicated and gets its own section below.

As a Drake @ref systems::System "System", %MultibodyPlant implements the
governing equations for a multibody dynamical system in the form
`ẋ = f(t, x, u)` with t being time and u external inputs such as actuation
forces. The governing equations for the dynamics of a multibody system modeled
with %MultibodyPlant are [Featherstone 2008, Jain 2010]: <pre>
         q̇ = N(q)v
  (1)    M(q)v̇ + C(q, v)v = τ
</pre>
where `M(q)` is the mass matrix of the multibody system (including rigid body
mass properties and @ref reflected_inertia "reflected inertias"), `C(q, v)v`
contains Coriolis, centripetal, and gyroscopic terms and
`N(q)` is the kinematic coupling matrix describing the relationship between
q̇ (the time derivatives of the generalized positions) and the generalized
velocities v, [Seth 2010]. `N(q)` is an `nq x nv` matrix.
The vector `τ ∈ ℝⁿᵛ` on the right hand side of Eq. (1) is
the system's generalized forces. These incorporate gravity, springs,
externally applied body forces, constraint forces, and contact forces.

@anchor mbp_discrete_dynamics
                         ### Discrete system dynamics

We'll start with the basic difference equation interpretation of a discrete
plant and then explain some Drake-specific subtleties.

@note We use "kinematics" here to refer to quantities that involve only position
or velocity, and "dynamics" to refer to quantities that also involve forces.

By default, a discrete %MultibodyPlant has these update dynamics:

       x[0] = initial kinematics      state variables x (={q, v}), s
       s[0] = empty (no sample yet)

     s[n+1] = g(t[n], x[n], u[n])     record sample
     x[n+1] = f(t[n], x[n], u[n])     update kinematics
    yd[n+1] = gd(s)                   dynamic outputs use sampled values
    yk[n+1] = gk(x)                   kinematic outputs use current x

Optionally, output port sampling can be disabled. In that case we have:

     x[n+1] = f(t[n], x[n], u[n])       update kinematics
    yd[n+1] = gd(g(t, x, u))            dynamic outputs use current values
    yk[n+1] = gk(x)                     kinematic outputs use current x

We're using `yd` and `yk` above to represent the calculated values of dynamic
and kinematic output ports, resp. Kinematic output ports are those that depend
only on position and velocity: `state`, `body_poses`, `body_spatial_velocities`.
Everything else depends on forces so is a dynamic output port:
`body_spatial_accelerations`, `generalized_acceleration`, `net_actuation`,
`reaction_forces`, and `contact_results`.

Use the function SetUseSampledOutputPorts() to choose which dynamics you prefer.
The default behavior (output port sampling) is more efficient for simulation,
but use slightly-different kinematics for the dynamic output port computations
versus the kinematic output ports. Disabling output port sampling provides
"live" output port results that are recalculated from the current state and
inputs whenever changes occur. It also eliminates the sampling state variable
(s above). Note that kinematic output ports (that is, those depending only on
position and velocity) are always "live" -- they are calculated as needed from
the current (updated) state.

The reason that the default mode is more efficient for simulation is that the
sample variable s records expensive-to-compute results (such as hydroelastic
contact forces) that are needed to advance the state x. Those results are thus
available for free at the start of step n. If instead we wait until after the
state is updated to n+1, we would have to recalculate those expensive results
at the new state in order to report them. Thus sampling means the output ports
show the results that were calculated using kinematics values x[n], although
the Context has been updated to kinematics values x[n+1]. If that isn't
tolerable you should disable output port sampling. You can also force an update
to occur using ExecuteForcedEvents().

See @ref output_port_sampling "Output port sampling" below for more practical
considerations.

Minor details most users won't care about:

  - The sample variable s is a Drake Abstract state variable. When it is
    present, the plant update is performed using an Unrestricted update; when it
    is absent we are able to use a Discrete update. Some Drake features (e.g.
    linearization of discrete systems) may be restricted to systems that use
    only Discrete (numeric) state variables and Discrete update.
  - The sample variable s is used only by output ports. It does not affect the
    behavior of any MultibodyPlant "Calc" or "Eval" functions -- those are
    always calculated using the current values of time, kinematic state, and
    input port values.

@anchor output_port_sampling
  ### Output port sampling

As described in @ref mbp_discrete_dynamics "Discrete system dynamics" above,
the semantics of certain %MultibodyPlant output ports depends on whether the
plant is configured to advance using continuous time integration or discrete
time steps (see is_discrete()). This section explains the details, focusing
on the practical aspects moreso than the equations.

Output ports that only depend on the [q, v] kinematic state (such as
get_body_poses_output_port() or get_body_spatial_velocities_output_port())
do _not_ change semantics for continuous vs discrete time. In all cases,
the output value is a function of the kinematic state in the context.

Output ports that incorporate dynamics (i.e., forces) _do_ change semantics
based on the plant mode. Imagine that the get_applied_spatial_force_input_port()
provides a continuously time-varying input force. The
get_body_spatial_accelerations_output_port() output is dependent on that force.
We could return a snapshot of the acceleration that was used in the last
time step, or we could recalculate the acceleration to immediately reflect
the changing forces. We call the former a "sampled" port and the latter a
"live" port.

For a continuous-time plant, there is no distinction -- the output port is
always live -- it immediately reflects the instantaneous input value. It is a
"direct feedthrough" output port (see SystemBase::GetDirectFeedthroughs()).

For a discrete-time plant, the user can choose whether the output should be
sampled or live: Use the function SetUseSampledOutputPorts() to change whether
output ports are sampled or not, and has_sampled_output_ports() to check the
current setting. When sampling is disabled, the only state in the context
is the kinematic [q, v], so dynamics output ports will always reflect the
instantaneous answer (i.e., direct feedthrough).  When sampling is enabled
(the default), the plant state incorporates a snapshot of the most recent step's
kinematics and dynamics, and the output ports will reflect that sampled state
(i.e., not direct feedthrough). For a detailed discussion, see
@ref mbp_discrete_dynamics "Discrete system dynamics".

For a discrete-time plant, the sampled outputs are generally _much_
faster to calculate than the feedthrough outputs when any inputs ports are
changing values faster than the discrete time step, e.g., during a simulation.
When input ports are fixed, or change at the time step rate (e.g., during motion
planning), sampled vs feedthrough will have similar computational performance.

Direct plant API function calls (e.g., EvalBodySpatialAccelerationInWorld())
that depend on forces always use the instantaneous (not sampled) accelerations.

Here are some practical tips that might help inform your particular situation:

(1) If you need a minimal-state representation for motion planning, mathematical
optimization, or similar, then you can either use a continuous-time plant or set
the config option `use_sampled_output_ports=false` on a discrete-time plant.

(2) By default, setting the positions of a discrete-time plant in the Context
will not have any effect on the dynamics-related output ports, e.g., the contact
results will not change. If you need to see changes to outputs without running
the plant in a Simulator, then you can either use a continuous-time plant, set
the config option `use_sampled_output_ports=false`, or use ExecuteForcedEvents()
to force a dynamics step and then the outputs (and positions) will change.

@anchor mbp_actuation
                ### Actuation

In a %MultibodyPlant model an actuator can be added as a JointActuator, see
AddJointActuator(). The plant declares actuation input ports to provide
feedforward actuation, both for the %MultibodyPlant as a whole (see
get_actuation_input_port()) and for each individual @ref model_instances
"model instance" in the %MultibodyPlant (see
@ref get_actuation_input_port(ModelInstanceIndex)const
"get_actuation_input_port(ModelInstanceIndex)").
- Actuation inputs and actuation effort limits are taken to be in joint
  coordinates (they are not affected by the actuator gear ratio).
- Any actuation input ports not connected are assumed to be zero.
- Actuation values from the full %MultibodyPlant model port
  (get_actuation_input_port()) and from the per model-instance ports (
@ref get_actuation_input_port(ModelInstanceIndex)const
"get_actuation_input_port(ModelInstanceIndex)") are summed up.

@note A JointActuator's index into the vector data supplied to %MultibodyPlant's
actuation input port for all actuators (get_actuation_input_port()) is given by
JointActuator::input_start(), NOT by its JointActuatorIndex. That is, the vector
element data for a JointActuator at index JointActuatorIndex(i) in the full
input port vector is found at index:
  MultibodyPlant::get_joint_actuator(JointActuatorIndex(i)).input_start().
For the @ref get_actuation_input_port(ModelInstanceIndex)const
"get_actuation_input_port(ModelInstanceIndex)" specific to a model index, the
vector data is ordered by monotonically increasing @ref JointActuatorIndex for
the actuators within that model instance: the 0ᵗʰ vector element
corresponds to the lowest-numbered %JointActuatorIndex of that instance, the 1ˢᵗ
vector element corresponds to the second-lowest-numbered %JointActuatorIndex of
that instance, etc.

@note The following snippet shows how per model instance actuation can be set:
```
ModelInstanceIndex model_instance_index = ...;
VectorX<T> u_instance(plant.num_actuated_dofs(model_instance_index));
int offset = 0;
for (JointActuatorIndex joint_actuator_index :
         plant.GetJointActuatorIndices(model_instance_index)) {
  const JointActuator<T>& actuator = plant.get_joint_actuator(
      joint_actuator_index);
  const Joint<T>& joint = actuator.joint();
  VectorX<T> u_joint = ... my_actuation_logic_for(joint) ...;
  ASSERT(u_joint.size() == joint_actuator.num_inputs());
  u_instance.segment(offset, u_joint.size()) = u_joint;
  offset += u_joint.size();
}
plant.get_actuation_input_port(model_instance_index).FixValue(
    plant_context, u_instance);
```

@note To inter-operate between the whole plant actuation vector and sets of
per-model instance actuation vectors, see SetActuationInArray() to gather the
model instance vectors into a whole plant vector and GetActuationFromArray() to
scatter the whole plant vector into per-model instance vectors.

@anchor pd_controllers
  #### Using PD controlled actuators

While PD controllers can be modeled externally and be connected to the
%MultibodyPlant model via the get_actuation_input_port(), simulation stability
at discrete-time steps can be compromised for high controller gains. For such
cases, simulation stability and robustness can be improved significantly by
moving your PD controller into the plant where the discrete solver can strongly
couple controller and model dynamics.

@note PD controllers are ignored when a joint is locked (see Joint::Lock()).

@warning Currently, this feature is only supported for discrete models
(is_discrete() is true) using the SAP solver (get_discrete_contact_solver()
returns DiscreteContactSolver::kSap.)

PD controlled joint actuators can be defined by setting PD gains for each joint
actuator, see JointActuator::set_controller_gains(). Unless these gains are
specified, joint actuators will not be PD controlled and
JointActuator::has_controller() will return `false`.

For models with PD controllers, the actuation torque per actuator is computed
according to: <pre>
  ũ = -Kp⋅(q − qd) - Kd⋅(v − vd) + u_ff
  u = max(−e, min(e, ũ))
</pre>
where qd and vd are desired configuration and velocity (see
get_desired_state_input_port()) for the actuated joint (see
JointActuator::joint()), Kp and Kd are the proportional and derivative gains of
the actuator (see JointActuator::get_controller_gains()), `u_ff` is the
feed-forward actuation specified with get_actuation_input_port(), and `e`
corresponds to effort limit (see JointActuator::effort_limit()).

Notice that actuation through get_actuation_input_port() and PD control are not
mutually exclusive, and they can be used together. This is better explained
through examples:
  1. **PD controlled gripper**. In this case, only PD control is used to drive
     the opening and closing of the fingers. The feed-forward term is assumed to
     be zero and the actuation input port is not required to be connected.
  2. **Robot arm**. A typical configuration consists on applying gravity
     compensation in the feed-forward term plus PD control to drive the robot to
     a given desired state.

@anchor pd_controllers_and_ports
  #### Actuation input ports requirements

Actuation input ports and desired state input ports need not be connected:
  - Unconnected actuation inputs default to zero, simplifying diagram wiring for
    models relying solely on PD controllers.
  - PD controllers are disarmed when their model instance's desired state input
    port is disconnected. In this state, they have no effect on dynamics,
    behaving as if no PD controller exists. This allows a %MultibodyPlant model
    to be used outside simulation (e.g., for visualization).

Note that both ports are always created but will be zero-sized for model
instances without actuation.

  #### Net actuation

The total joint actuation applied via the actuation input port
(get_actuation_input_port()) and applied by the PD controllers is reported by
the net actuation port (get_net_actuation_output_port()). That is, the net
actuation port reports the total actuation applied by a given actuator.

@note PD controllers are ignored when a joint is locked (see Joint::Lock()), and
thus they have no effect on the actuation output nor reaction forces.

@anchor sdf_loading
                 ### Loading models from SDFormat files

Drake has the capability to load multibody models from SDFormat and URDF
files.  Consider the example below which loads an acrobot model:
@code
  MultibodyPlant<T> acrobot;
  SceneGraph<T> scene_graph;
  Parser parser(&acrobot, &scene_graph);
  const std::string url =
      "package://drake/multibody/benchmarks/acrobot/acrobot.sdf";
  parser.AddModelsFromUrl(url);
@endcode
As in the example above, for models including visual geometry, collision
geometry or both, the user must specify a SceneGraph for geometry handling.
You can find a full example of the LQR controlled acrobot in
examples/multibody/acrobot/run_lqr.cc.

AddModelFromFile() can be invoked multiple times on the same plant in order
to load multiple model instances.  Other methods are available on Parser
such as AddModels() which allows creating model instances per
each `<model>` tag found in the file. Please refer to each of these
methods' documentation for further details.

@anchor working_with_scenegraph
                  ### Working with %SceneGraph

@anchor add_multibody_plant_scene_graph
  #### Adding a %MultibodyPlant connected to a %SceneGraph to your %Diagram

Probably the simplest way to add and wire up a MultibodyPlant with
a SceneGraph in your Diagram is using AddMultibodyPlantSceneGraph().

Recommended usages:

Assign to a MultibodyPlant reference (ignoring the SceneGraph):
@code
  MultibodyPlant<double>& plant =
      AddMultibodyPlantSceneGraph(&builder, 0.0 /+ time_step +/);
  plant.DoFoo(...);
@endcode
This flavor is the simplest, when the SceneGraph is not explicitly needed.
(It can always be retrieved later via GetSubsystemByName("scene_graph").)

Assign to auto, and use the named public fields:
@code
  auto items = AddMultibodyPlantSceneGraph(&builder, 0.0 /+ time_step +/);
  items.plant.DoFoo(...);
  items.scene_graph.DoBar(...);
@endcode
or taking advantage of C++'s structured binding:
@code
  auto [plant, scene_graph] = AddMultibodyPlantSceneGraph(&builder, 0.0);
  ...
  plant.DoFoo(...);
  scene_graph.DoBar(...);
@endcode
This is the easiest way to use both the plant and scene_graph.

Assign to already-declared pointer variables:
@code
  MultibodyPlant<double>* plant{};
  SceneGraph<double>* scene_graph{};
  std::tie(plant, scene_graph) =
      AddMultibodyPlantSceneGraph(&builder, 0.0 /+ time_step +/);
  plant->DoFoo(...);
  scene_graph->DoBar(...);
@endcode
This flavor is most useful when the pointers are class member fields
(and so perhaps cannot be references).

@anchor mbp_geometry_registration
              #### Registering geometry with a SceneGraph

%MultibodyPlant users can register geometry with a SceneGraph for
essentially two purposes; a) visualization and, b) contact modeling.

<!--TODO(SeanCurtis-TRI): update this comment as the number of SceneGraph
    roles changes. -->

Before any geometry registration takes place, a user **must** first make a
call to RegisterAsSourceForSceneGraph() in order to register the
%MultibodyPlant as a client of a SceneGraph instance, point at which the
plant will have assigned a valid geometry::SourceId.
At Finalize(), %MultibodyPlant will declare input/output ports as
appropriate to communicate with the SceneGraph instance on which
registrations took place. All geometry registration **must** be performed
pre-finalize.

%Multibodyplant declares an input port for geometric queries, see
get_geometry_query_input_port(). If %MultibodyPlant registers geometry with
a SceneGraph via calls to RegisterCollisionGeometry(), users may use this
port for geometric queries. The port must be connected to the same SceneGraph
used for registration. The preferred mechanism is to use
AddMultibodyPlantSceneGraph() as documented above.

In extraordinary circumstances, this can be done by hand and the setup process
will include:

1. Call to RegisterAsSourceForSceneGraph().
2. Calls to RegisterCollisionGeometry(), as many as needed.
3. Call to Finalize(), user is done specifying the model.
4. Connect geometry::SceneGraph::get_query_output_port() to
   get_geometry_query_input_port().
5. Connect get_geometry_pose_output_port() to
   geometry::SceneGraph::get_source_pose_port()

Refer to the documentation provided in each of the methods above for further
details.

@anchor accessing_contact_properties
              #### Accessing point contact parameters
%MultibodyPlant's point contact model looks for model parameters stored as
geometry::ProximityProperties by geometry::SceneGraph. These properties can
be obtained before or after context creation through
geometry::SceneGraphInspector APIs as outlined below. %MultibodyPlant expects
the following properties for point contact modeling:

|Group name|Property Name|Required|Property Type|Property Description|
|:--------:|:-----------:|:------:|:----------------:|:-------------------|
|material|coulomb_friction|yes¹|CoulombFriction<T>|Static and Dynamic friction.|
|material|point_contact_stiffness|no²|T| Compliant point contact stiffness.|
|material|hunt_crossley_dissipation |no²⁴|T| Compliant contact dissipation.|
|material|relaxation_time|yes³⁴|T|Linear Kelvin–Voigt model parameter.|

¹ Collision geometry is required to be registered with a
  geometry::ProximityProperties object that contains the
  ("material", "coulomb_friction") property. If the property
  is missing, %MultibodyPlant will throw an exception.

² If the property is missing, %MultibodyPlant will use
  a heuristic value as the default. Refer to the
  section @ref point_contact_defaults "Point Contact Default Parameters" for
  further details.

³ When using a linear Kelvin–Voigt model of dissipation (for instance when
  selecting the SAP solver), collision geometry is required to be registered
  with a geometry::ProximityProperties object that contains the ("material",
  "relaxation_time") property. If the property is missing, an exception will be
  thrown.

⁴ We allow to specify both hunt_crossley_dissipation and relaxation_time for a
  given geometry. However only one of these will get used, depending on the
  configuration of the %MultibodyPlant. As an example, if the SAP contact
  approximation is specified (see set_discrete_contact_approximation()) only the
  relaxation_time is used while hunt_crossley_dissipation is ignored.
  Conversely, if the TAMSI, Similar or Lagged approximation is used (see
  set_discrete_contact_approximation()) only hunt_crossley_dissipation is used
  while relaxation_time is ignored. Currently, a continuous %MultibodyPlant
  model will always use the Hunt & Crossley model and relaxation_time will be
  ignored.

Accessing and modifying contact properties requires interfacing with
geometry::SceneGraph's model inspector. Interfacing with a model inspector
obtained from geometry::SceneGraph will provide the default registered
values for a given parameter. These are the values that will
initially appear in a systems::Context created by CreateDefaultContext().
Subsequently, true system parameters can be accessed and changed through a
systems::Context once available. For both of the above cases, proximity
properties are accessed through geometry::SceneGraphInspector APIs.

Before context creation an inspector can be retrieved directly from SceneGraph
as:
@code
// For a SceneGraph<T> instance called scene_graph.
const geometry::SceneGraphInspector<T>& inspector =
    scene_graph.model_inspector();
@endcode
After context creation, an inspector can be retrieved from the state
stored in the context:
@code
// For a MultibodyPlant<T> instance called mbp and a Context<T> called
// context.
const geometry::SceneGraphInspector<T>& inspector =
    mbp.EvalSceneGraphInspector(context);
@endcode
Once an inspector is available, proximity properties can be retrieved as:
@code
// For a body with GeometryId called geometry_id
const geometry::ProximityProperties* props =
    inspector.GetProximityProperties(geometry_id);
const CoulombFriction<T>& geometry_friction =
    props->GetProperty<CoulombFriction<T>>("material",
                                           "coulomb_friction");
@endcode

@anchor mbp_parameters
              ### Working with %MultibodyElement parameters
Several %MultibodyElements expose parameters, allowing the user flexible
modification of some aspects of the plant's model, post systems::Context
creation. For details, refer to the documentation for the MultibodyElement
whose parameters you are trying to modify/access (e.g. RigidBody,
FixedOffsetFrame, etc.)

As an example, here is how to access and modify rigid body mass parameters:
@code
  MultibodyPlant<double> plant;
  // ... Code to add bodies, finalize plant, and to obtain a context.
  const RigidBody<double>& body =
      plant.GetRigidBodyByName("BodyName");
  const SpatialInertia<double> M_BBo_B =
      body.GetSpatialInertiaInBodyFrame(context);
  // .. logic to determine a new SpatialInertia parameter for body.
  const SpatialInertia<double>& M_BBo_B_new = ....

  // Modify the body parameter for spatial inertia.
  body.SetSpatialInertiaInBodyFrame(&context, M_BBo_B_new);
@endcode

Another example, working with automatic differentiation in order to take
derivatives with respect to one of the bodies' masses:
@code
  MultibodyPlant<double> plant;
  // ... Code to add bodies, finalize plant, and to obtain a
  // context and a body's spatial inertia M_BBo_B.

  // Scalar convert the plant.
  unique_ptr<MultibodyPlant<AutoDiffXd>> plant_autodiff =
      systems::System<double>::ToAutoDiffXd(plant);
  unique_ptr<Context<AutoDiffXd>> context_autodiff =
      plant_autodiff->CreateDefaultContext();
  context_autodiff->SetTimeStateAndParametersFrom(context);

  const RigidBody<AutoDiffXd>& body =
      plant_autodiff->GetRigidBodyByName("BodyName");

  // Modify the body parameter for mass.
  const AutoDiffXd mass_autodiff(mass, Vector1d(1));
  body.SetMass(context_autodiff.get(), mass_autodiff);

  // M_autodiff(i, j).derivatives()(0), contains the derivatives of
  // M(i, j) with respect to the body's mass.
  MatrixX<AutoDiffXd> M_autodiff(plant_autodiff->num_velocities(),
      plant_autodiff->num_velocities());
  plant_autodiff->CalcMassMatrix(*context_autodiff, &M_autodiff);
@endcode

@anchor mbp_adding_elements
                   ### Adding modeling elements

<!-- TODO(amcastro-tri): Update this section to add force elements and
     constraints. -->

Add multibody elements to a %MultibodyPlant with methods like:

- Bodies: AddRigidBody()
- Joints: AddJoint()
- see @ref mbp_construction "Construction" for more.

All modeling elements **must** be added before Finalize() is called.
See @ref mbp_finalize_stage "Finalize stage" for a discussion.

@anchor mbp_modeling_contact
                          ### Modeling contact

Please refer to @ref drake_contacts "Contact Modeling in Drake" for details
on the available approximations, setup, and considerations for a multibody
simulation with frictional contact.

@anchor mbp_energy_and_power
                        ### Energy and Power
<!-- TODO(sherm1) Update this as issue #12942 gets resolved. -->
%MultibodyPlant implements the System energy and power methods, with
some limitations.
- Kinetic energy: fully implemented.
- Potential energy and conservative power: currently include only gravity
  and contributions from ForceElement objects; potential energy from
  compliant contact and joint limits are not included.
- Nonconservative power: currently includes only contributions from
  ForceElement objects; actuation and input port forces, joint damping,
  and dissipation from joint limits, friction, and contact dissipation
  are not included.

See Drake issue #12942 for more discussion.

@anchor mbp_finalize_stage
                           ### %Finalize() stage

Once the user is done adding modeling elements and registering geometry, a
call to Finalize() must be performed. This call will:
- Build the underlying tree structure of the multibody model,
- declare the plant's state,
- declare the plant's input and output ports,
- declare collision filters to ignore collisions among rigid bodies:
  - between rigid bodies connected by a joint,
  - within subgraphs of welded rigid bodies.

Note that MultibodyPlant will *not* introduce any automatic collision filters on
deformable bodies. Collision filters for deformable bodies can be explicitly
applied via ExcludeCollisionGeometriesWithCollisionFilterGroupPair() or during
parsing.

<!-- TODO(amcastro-tri): Consider making the actual geometry registration
     with GS AFTER Finalize() so that we can tell if there are any bodies
     welded to the world to which we could just assign anchored geometry
     instead of dynamic geometry. This is an optimization and the API, and
     pre/post-finalize conditions should not change. -->

@anchor mbp_table_of_contents

@anchor mbp_references
                           ### References

- [Featherstone 2008] Featherstone, R., 2008.
    Rigid body dynamics algorithms. Springer.
- [Jain 2010] Jain, A., 2010.
    Robot and multibody dynamics: analysis and algorithms.
    Springer Science & Business Media.
- [Seth 2010] Seth, A., Sherman, M., Eastman, P. and Delp, S., 2010.
    Minimal formulation of joint motion for biomechanisms.
    Nonlinear dynamics, 62(1), pp.291-303.

@tparam_default_scalar
@ingroup systems */
template <typename T>
class MultibodyPlant final : public internal::MultibodyTreeSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MultibodyPlant);

  /// @anchor mbp_input_and_output_ports
  /// @name                 Input and output ports
  /// These methods provide access to the Drake
  /// @ref systems::System "System" input and output ports
  /// as depicted in the MultibodyPlant class documentation.
  ///
  /// Actuation values can be provided through a single input port which
  /// describes the entire plant, or through multiple input ports which each
  /// provide the actuation values for a specific model instance. See
  /// AddJointActuator() and num_actuated_dofs().
  ///
  /// Output ports provide information about the entire %MultibodyPlant
  /// or its individual model instances.
  /// @{

  /// Returns the output port of all body poses in the world frame.
  /// You can obtain the pose `X_WB` of a body B in the world frame W with:
  /// @code
  ///   const auto& X_WB_all = plant.get_body_poses_output_port().
  ///       .Eval<std::vector<math::RigidTransform<double>>>(plant_context);
  ///   const BodyIndex arm_body_index = plant.GetBodyByName("arm").index();
  ///   const math::RigidTransform<double>& X_WArm = X_WB_all[arm_body_index];
  /// @endcode
  /// As shown in the example above, the resulting `std::vector` of body poses
  /// is indexed by BodyIndex, and it has size num_bodies().
  /// BodyIndex "zero" (0) always corresponds to the world body, with pose
  /// equal to the identity at all times.
  /// @throws std::exception if called pre-finalize.
  const systems::OutputPort<T>& get_body_poses_output_port() const;

  /// Returns the output port of all body spatial velocities in the world frame.
  /// You can obtain the spatial velocity `V_WB` of a body B in the world frame
  /// W with:
  /// @code
  ///   const auto& V_WB_all = plant.get_body_spatial_velocities_output_port().
  ///       .Eval<std::vector<SpatialVelocity<double>>>(plant_context);
  ///   const BodyIndex arm_body_index = plant.GetBodyByName("arm").index();
  ///   const SpatialVelocity<double>& V_WArm = V_WB_all[arm_body_index];
  /// @endcode
  /// As shown in the example above, the resulting `std::vector` of body spatial
  /// velocities is indexed by BodyIndex, and it has size num_bodies().
  /// BodyIndex "zero" (0) always corresponds to the world body, with zero
  /// spatial velocity at all times.
  /// @throws std::exception if called pre-finalize.
  const systems::OutputPort<T>& get_body_spatial_velocities_output_port() const;

  /// Returns the output port of all body spatial accelerations in the world
  /// frame. You can obtain the spatial acceleration `A_WB` of a body B (for
  /// point Bo, the body's origin) in the world frame W with:
  /// @code
  ///   const auto& A_WB_all =
  ///   plant.get_body_spatial_accelerations_output_port().
  ///       .Eval<std::vector<SpatialAcceleration<double>>>(plant_context);
  ///   const BodyIndex arm_body_index = plant.GetBodyByName("arm").index();
  ///   const SpatialVelocity<double>& A_WArm = A_WB_all[arm_body_index];
  /// @endcode
  /// As shown in the example above, the resulting `std::vector` of body spatial
  /// accelerations is indexed by BodyIndex, and it has size num_bodies().
  /// BodyIndex "zero" (0) always corresponds to the world body, with zero
  /// spatial acceleration at all times.
  ///
  /// In a discrete-time plant, the use_sampled_output_ports setting affects the
  /// output of this port.  See @ref output_port_sampling "Output port sampling"
  /// for details. When sampling is enabled and the plant has not yet taken a
  /// step, the output value will be all zeros.
  ///
  /// @throws std::exception if called pre-finalize.
  const systems::OutputPort<T>& get_body_spatial_accelerations_output_port()
      const;

  /// Returns a constant reference to the input port for external actuation for
  /// all actuated dofs. This input port is a vector valued port and can be set
  /// with JointActuator::set_actuation_vector(). The actuation value for a
  /// particular actuator can be found at offset JointActuator::input_start() in
  /// this vector. Refer to @ref mbp_actuation "Actuation" for further details.
  /// @pre Finalize() was already called on `this` plant.
  /// @throws std::exception if called before Finalize().
  const systems::InputPort<T>& get_actuation_input_port() const;

  /// Returns a constant reference to the input port for external actuation for
  /// a specific model instance. This is a vector valued port with entries
  /// ordered by monotonically increasing @ref JointActuatorIndex within
  /// `model_instance`. Refer to @ref mbp_actuation "Actuation" for further
  /// details.
  ///
  /// Every model instance in `this` plant model has an actuation input port,
  /// even if zero sized (for model instance with no actuators).
  ///
  /// @see GetJointActuatorIndices(), GetActuatedJointIndices().
  ///
  /// @pre Finalize() was already called on `this` plant.
  /// @throws std::exception if called before Finalize().
  /// @throws std::exception if the model instance does not exist.
  const systems::InputPort<T>& get_actuation_input_port(
      ModelInstanceIndex model_instance) const;

  /// Returns a constant reference to the output port that reports actuation
  /// values applied through joint actuators. This output port is a vector
  /// valued port. The actuation value for a particular actuator can be found at
  /// offset JointActuator::input_start() in this vector. Models that include PD
  /// controllers will include their contribution in this port, refer to @ref
  /// mbp_actuation "Actuation" for further details.
  ///
  /// In a discrete-time plant, the use_sampled_output_ports setting affects the
  /// output of this port.  See @ref output_port_sampling "Output port sampling"
  /// for details. When sampling is enabled and the plant has not yet taken a
  /// step, the output value will be all zeros.
  ///
  /// @note PD controllers are not considered for actuators on locked joints,
  /// see Joint::Lock(). Therefore they do not contribute to this port if the
  /// joint is locked.
  /// @pre Finalize() was already called on `this` plant.
  /// @throws std::exception if called before Finalize().
  const systems::OutputPort<T>& get_net_actuation_output_port() const;

  /// Returns a constant reference to the output port that reports actuation
  /// values applied through joint actuators, for a specific model instance.
  /// Models that include PD controllers will include their contribution in this
  /// port, refer to @ref mbp_actuation "Actuation" for further details. This is
  /// a vector valued port with entries ordered by monotonically increasing @ref
  /// JointActuatorIndex within `model_instance`.
  ///
  /// Every model instance in `this` plant model has a net actuation output
  /// port, even if zero sized (for model instance with no actuators).
  ///
  /// In a discrete-time plant, the use_sampled_output_ports setting affects the
  /// output of this port.  See @ref output_port_sampling "Output port sampling"
  /// for details. When sampling is enabled and the plant has not yet taken a
  /// step, the output value will be all zeros.
  ///
  /// @note PD controllers are not considered for actuators on locked joints,
  /// see Joint::Lock(). Therefore they do not contribute to this port if the
  /// joint is locked.
  /// @pre Finalize() was already called on `this` plant.
  /// @throws std::exception if called before Finalize().
  const systems::OutputPort<T>& get_net_actuation_output_port(
      ModelInstanceIndex model_instance) const;

  /// For models with PD controlled joint actuators, returns the port to provide
  /// the desired state for the given `model_instance`.
  /// Refer to @ref mbp_actuation "Actuation" for further details.
  ///
  /// For consistency with get_actuation_input_port(), each model instance in
  /// `this` plant model has a desired states input port, even if zero sized
  /// (for model instance with no actuators.)
  ///
  /// @note This port always has size 2 * num_actuators(model_instance), where
  /// we assume 1-DOF actuated joints. This port must provide one desired
  /// position and one desired velocity per joint actuator, packed as xd = [qd,
  /// vd], with positions and velocities in order of increasing
  /// JointActuatorIndex. Only desired states corresponding to PD-controlled
  /// actuators on non-locked joints (Joint::is_locked()) are used, the rest are
  /// ignored. That is PD control on just a subset of actuators is allowed.
  ///
  /// @note The desired state input port for a given model instance is not
  /// required to be connected. If disconnected, the controllers for such model
  /// instance will be _disarmed_. Refer to @ref pd_controllers_and_ports for
  /// further details.
  ///
  /// As an example of this structure, consider the following code to fix
  /// desired states input values:
  /// ```
  /// MultibodyPlant<double> plant;
  /// // ... Load/parse plant model ...
  /// plant.Finalize();
  /// auto context = plant.CreateDefaultContext();
  /// const int num_u = plant.num_actuators(model_instance);
  /// const VectorXd model_xd(2 * num_u);
  /// auto model_qd = model_xd.head(num_u);
  /// auto model_vd = model_xd.tail(num_u);
  ///
  /// int a = 0;
  /// // Specify qd and vd in increasing order of @ref JointActuatorIndex, as
  /// // returned by GetJointActuatorIndices().
  /// for (const JointActuatorIndex actuator_index :
  ///     plant.GetJointActuatorIndices(model_instance)) {
  ///   qd[a] = .... desired q value for actuator_index
  ///   vd[a] = .... desired v value for actuator_index
  ///   ++a;
  /// }
  /// // As an example, fix values in the context.
  /// plant.get_desired_state_input_port(model_instance).FixValue(
  ///     &plant_context, model_xd);
  /// ```
  const systems::InputPort<T>& get_desired_state_input_port(
      ModelInstanceIndex model_instance) const;

  /// Returns a constant reference to the vector-valued input port for applied
  /// generalized forces, and the vector will be added directly into `tau` (see
  /// @ref mbp_equations_of_motion "System dynamics"). This vector is ordered
  /// using the same convention as the plant velocities: you can set the
  /// generalized forces that will be applied to model instance i using, e.g.,
  /// `SetVelocitiesInArray(i, model_forces, &force_array)`.
  /// @throws std::exception if called before Finalize().
  const systems::InputPort<T>& get_applied_generalized_force_input_port() const;

  // TODO(jwnimmer-tri) This input port should use BusValue instead of vector<>,
  // so that the ExternallyAppliedSpatialForceMultiplexer hassle is unnecessary.
  // Add the new port with a different name (maybe "applied_spatial_force_bus")
  // and deprecate this port and that force mux for removal.
  /// Returns a constant reference to the input port for applying spatial
  /// forces to bodies in the plant. The data type for the port is an
  /// std::vector of ExternallyAppliedSpatialForce; any number of spatial forces
  /// can be applied to any number of bodies in the plant.
  const systems::InputPort<T>& get_applied_spatial_force_input_port() const;

  /// Returns a constant reference to the input port used to perform geometric
  /// queries on a SceneGraph. See SceneGraph::get_query_output_port().
  /// Refer to section @ref mbp_geometry "Geometry" of this class's
  /// documentation for further details on collision geometry registration and
  /// connection with a SceneGraph.
  const systems::InputPort<T>& get_geometry_query_input_port() const;

  /// Returns a constant reference to the output port for the multibody state
  /// x = [q, v] of the model.
  /// @pre Finalize() was already called on `this` plant.
  /// @throws std::exception if called before Finalize().
  const systems::OutputPort<T>& get_state_output_port() const;

  /// Returns a constant reference to the output port for the state
  /// xᵢ = [qᵢ vᵢ] of model instance i. (Here qᵢ ⊆ q and vᵢ ⊆ v.)
  /// @pre Finalize() was already called on `this` plant.
  /// @throws std::exception if called before Finalize().
  /// @throws std::exception if the model instance does not exist.
  const systems::OutputPort<T>& get_state_output_port(
      ModelInstanceIndex model_instance) const;

  /// Returns a constant reference to the output port for generalized
  /// accelerations v̇ of the model.
  ///
  /// In a discrete-time plant, the use_sampled_output_ports setting affects the
  /// output of this port.  See @ref output_port_sampling "Output port sampling"
  /// for details. When sampling is enabled and the plant has not yet taken a
  /// step, the output value will be all zeros.
  ///
  /// @pre Finalize() was already called on `this` plant.
  /// @throws std::exception if called before Finalize().
  const systems::OutputPort<T>& get_generalized_acceleration_output_port()
      const;

  /// Returns a constant reference to the output port for the generalized
  /// accelerations v̇ᵢ ⊆ v̇ for model instance i.
  ///
  /// In a discrete-time plant, the use_sampled_output_ports setting affects the
  /// output of this port.  See @ref output_port_sampling "Output port sampling"
  /// for details. When sampling is enabled and the plant has not yet taken a
  /// step, the output value will be all zeros.
  ///
  /// @pre Finalize() was already called on `this` plant.
  /// @throws std::exception if called before Finalize().
  /// @throws std::exception if the model instance does not exist.
  const systems::OutputPort<T>& get_generalized_acceleration_output_port(
      ModelInstanceIndex model_instance) const;

  /// Returns a constant reference to the output port of generalized contact
  /// forces for a specific model instance.
  ///
  /// In a discrete-time plant, the use_sampled_output_ports setting affects the
  /// output of this port.  See @ref output_port_sampling "Output port sampling"
  /// for details. When sampling is enabled and the plant has not yet taken a
  /// step, the output value will be all zeros.
  ///
  /// @pre Finalize() was already called on `this` plant.
  /// @throws std::exception if called before Finalize().
  /// @throws std::exception if the model instance does not exist.
  const systems::OutputPort<T>& get_generalized_contact_forces_output_port(
      ModelInstanceIndex model_instance) const;

  /// Returns the port for joint reaction forces.
  /// A Joint models the kinematical relationship which characterizes the
  /// possible relative motion between two bodies. In Drake, a joint connects a
  /// frame `Jp` on _parent_ body P with a frame `Jc` on a _child_ body C. This
  /// usage of the terms _parent_ and _child_ is just a convention and implies
  /// nothing about the inboard-outboard relationship between the bodies. Since
  /// a Joint imposes a kinematical relationship which characterizes the
  /// possible relative motion between frames Jp and Jc, reaction forces on each
  /// body are established. That is, we could cut the model at the joint and
  /// replace it with equivalent forces equal to these reaction forces in order
  /// to attain the same motions of the mechanical system.
  ///
  /// This output port allows to evaluate the reaction force `F_CJc_Jc` on the
  /// _child_ body C, at `Jc`, and expressed in Jc for all joints in the model.
  /// This port evaluates to a vector of type std::vector<SpatialForce<T>> and
  /// size num_joints() indexed by JointIndex, see Joint::index(). Each entry
  /// corresponds to the spatial force `F_CJc_Jc` applied on the joint's child
  /// body C (Joint::child_body()), at the joint's child frame `Jc`
  /// (Joint::frame_on_child()) and expressed in frame `Jc`.
  ///
  /// In a discrete-time plant, the use_sampled_output_ports setting affects the
  /// output of this port.  See @ref output_port_sampling "Output port sampling"
  /// for details. When sampling is enabled and the plant has not yet taken a
  /// step, the output value will be all zeros.
  ///
  /// @note PD controllers are not considered for actuators on locked joints,
  /// see Joint::Lock(). Therefore they do not contribute to this port if the
  /// joint is locked.
  /// @throws std::exception if called pre-finalize.
  const systems::OutputPort<T>& get_reaction_forces_output_port() const;

  /// Returns a constant reference to the port that outputs ContactResults.
  ///
  /// In a discrete-time plant, the use_sampled_output_ports setting affects the
  /// output of this port.  See @ref output_port_sampling "Output port sampling"
  /// for details. When sampling is enabled and the plant has not yet taken a
  /// step, the output value will be empty (no contacts).
  ///
  /// @throws std::exception if called pre-finalize, see Finalize().
  const systems::OutputPort<T>& get_contact_results_output_port() const;

  /// Returns the output port of frames' poses to communicate with a
  /// SceneGraph.
  const systems::OutputPort<T>& get_geometry_pose_output_port() const;

  /// Returns the output port for vertex positions (configurations), measured
  /// and expressed in the World frame, of the deformable bodies in `this` plant
  /// as a GeometryConfigurationVector.
  const systems::OutputPort<T>& get_deformable_body_configuration_output_port()
      const;
  /// @} <!-- Input and output ports -->

  /// @anchor mbp_construction
  /// @name                   Construction
  /// To add modeling elements like bodies, joints, force elements, constraints,
  /// etc. to a %MultibodyPlant, use one of the following construction methods.
  /// Once _all_ modeling elements have been added, the Finalize() method
  /// **must** be called. A call to any construction method **after** a call to
  /// Finalize() causes an exception to be thrown.
  ///  After calling Finalize(), you may invoke %MultibodyPlant
  /// methods that perform computations. See Finalize() for details.
  /// @{

  /// This constructor creates a plant with a single "world" body.
  /// Therefore, right after creation, num_bodies() returns one.
  ///
  /// %MultibodyPlant offers two different modalities to model mechanical
  /// systems in time. These are:
  ///  1. As a discrete system with periodic updates, `time_step` is strictly
  ///     greater than zero.
  ///  2. As a continuous system, `time_step` equals exactly zero.
  ///
  /// Currently the discrete model is preferred for simulation given its
  /// robustness and speed in problems with frictional contact. However this
  /// might change as we work towards developing better strategies to model
  /// contact.
  /// See @ref multibody_simulation for further details.
  ///
  /// @warning Users should be aware of current limitations in either modeling
  /// modality. While the discrete model is often the preferred option for
  /// problems with frictional contact given its robustness and speed, it might
  /// become unstable when using large feedback gains, high damping or large
  /// external forcing. %MultibodyPlant will throw an exception whenever the
  /// discrete solver is detected to fail.
  /// Conversely, the continuous modality has the potential to leverage the
  /// robustness and accuracy control provide by Drake's integrators. However
  /// thus far this has proved difficult in practice and especially due to poor
  /// performance.
  ///
  /// <!-- TODO(amcastro-tri): Update the @warning messages in these docs if the
  ///      best practices advice changes as our solvers evolve. -->
  ///
  /// @param[in] time_step
  ///   Indicates whether `this` plant is modeled as a continuous system
  ///   (`time_step = 0`) or as a discrete system with periodic updates of
  ///   period `time_step > 0`. See @ref multibody_simulation for further
  ///   details.
  ///
  /// @warning Currently the continuous modality with `time_step = 0` does not
  /// support joint limits for simulation, these are ignored. %MultibodyPlant
  /// prints a warning to console if joint limits are provided. If your
  /// simulation requires joint limits currently you must use a discrete
  /// %MultibodyPlant model.
  ///
  /// @throws std::exception if `time_step` is negative.
  explicit MultibodyPlant(double time_step);

  /// Scalar-converting copy constructor.  See @ref system_scalar_conversion.
  template <typename U>
  explicit MultibodyPlant(const MultibodyPlant<U>& other);

  ~MultibodyPlant() override;

  /// (Advanced) For a discrete-time plant, configures whether the output ports
  /// are sampled (the default) or live (opt-in).
  /// See @ref output_port_sampling "Output port sampling" for details.
  /// @throws std::exception if the plant is already finalized.
  /// @throws std::exception if `use_sampled_output_ports` is `true` but `this`
  /// %MultibodyPlant is not a discrete model (is_discrete() == false).
  void SetUseSampledOutputPorts(bool use_sampled_output_ports);

  /// Creates a rigid body with the provided name and spatial inertia.  This
  /// method returns a constant reference to the body just added, which will
  /// remain valid for the lifetime of `this` %MultibodyPlant.
  ///
  /// Example of usage:
  /// @code
  ///   MultibodyPlant<T> plant;
  ///   // ... Code to define spatial_inertia, a SpatialInertia<T> object ...
  ///   ModelInstanceIndex model_instance = plant.AddModelInstance("instance");
  ///   const RigidBody<T>& body =
  ///     plant.AddRigidBody("BodyName", model_instance, spatial_inertia);
  /// @endcode
  ///
  /// @param[in] name
  ///   A string that identifies the new body to be added to `this` model. A
  ///   std::runtime_error is thrown if a body named `name` already is part of
  ///   @p model_instance. See HasBodyNamed(), RigidBody::name().
  /// @param[in] model_instance
  ///   A model instance index which this body is part of.
  /// @param[in] M_BBo_B
  ///   The SpatialInertia of the new rigid body to be added to `this`
  ///   %MultibodyPlant, computed about the body frame origin `Bo` and expressed
  ///   in the body frame B. When not provided, defaults to zero.
  /// @returns A constant reference to the new RigidBody just added, which will
  ///          remain valid for the lifetime of `this` %MultibodyPlant.
  const RigidBody<T>& AddRigidBody(
      const std::string& name, ModelInstanceIndex model_instance,
      const SpatialInertia<double>& M_BBo_B = SpatialInertia<double>::Zero()) {
    DRAKE_MBP_THROW_IF_FINALIZED();
    // Add the actual rigid body to the model.
    const RigidBody<T>& body =
        this->mutable_tree().AddRigidBody(name, model_instance, M_BBo_B);
    // Each entry of visual_geometries_, ordered by body index, contains a
    // std::vector of geometry ids for that body. The emplace_back() below
    // resizes visual_geometries_ to store the geometry ids for the body we
    // just added.
    // Similarly for the collision_geometries_ vector.
    DRAKE_DEMAND(visual_geometries_.size() == body.index());
    visual_geometries_.emplace_back();
    DRAKE_DEMAND(collision_geometries_.size() == body.index());
    collision_geometries_.emplace_back();
    RegisterRigidBodyWithSceneGraph(body);
    return body;
  }

  /// Creates a rigid body with the provided name and spatial inertia.  This
  /// method returns a constant reference to the body just added, which will
  /// remain valid for the lifetime of `this` %MultibodyPlant.  The body will
  /// use the default model instance
  /// (@ref model_instance "more on model instances").
  ///
  /// Example of usage:
  /// @code
  ///   MultibodyPlant<T> plant;
  ///   // ... Code to define spatial_inertia, a SpatialInertia<T> object ...
  ///   const RigidBody<T>& body =
  ///     plant.AddRigidBody("BodyName", spatial_inertia);
  /// @endcode
  ///
  /// @param[in] name
  ///   A string that identifies the new body to be added to `this` model. A
  ///   std::runtime_error is thrown if a body named `name` already is part of
  ///   the model in the default model instance. See HasBodyNamed(),
  ///   RigidBody::name().
  /// @param[in] M_BBo_B
  ///   The SpatialInertia of the new rigid body to be added to `this`
  ///   %MultibodyPlant, computed about the body frame origin `Bo` and expressed
  ///   in the body frame B. When not provided, defaults to zero.
  /// @returns A constant reference to the new RigidBody just added, which will
  ///          remain valid for the lifetime of `this` %MultibodyPlant.
  /// @throws std::exception if additional model instances have been created
  ///                        beyond the world and default instances.
  const RigidBody<T>& AddRigidBody(
      const std::string& name,
      const SpatialInertia<double>& M_BBo_B = SpatialInertia<double>::Zero()) {
    if (num_model_instances() != 2) {
      throw std::logic_error(
          "This model has more model instances than the default.  Please "
          "call AddRigidBody with an explicit model instance.");
    }

    return AddRigidBody(name, default_model_instance(), M_BBo_B);
  }

  /// This method adds a Frame of type `FrameType<T>`. For more information,
  /// please see the corresponding constructor of `FrameType`.
  /// @tparam FrameType Template which will be instantiated on `T`.
  /// @param frame Unique pointer frame instance.
  /// @returns A constant reference to the new Frame just added, which will
  ///          remain valid for the lifetime of `this` %MultibodyPlant.
  template <template <typename> class FrameType>
  const FrameType<T>& AddFrame(std::unique_ptr<FrameType<T>> frame) {
    return this->mutable_tree().AddFrame(std::move(frame));
  }

  /// This method adds a Joint of type `JointType` between two bodies.
  /// For more information, see the below overload of `AddJoint<>`.
  template <template <typename Scalar> class JointType>
  const JointType<T>& AddJoint(std::unique_ptr<JointType<T>> joint) {
    static_assert(std::is_convertible_v<JointType<T>*, Joint<T>*>,
                  "JointType must be a sub-class of Joint<T>.");
    DRAKE_MBP_THROW_IF_FINALIZED();
    const JointType<T>& result =
        this->mutable_tree().AddJoint(std::move(joint));
    return result;
  }

  // clang-format off (to preserve link to image)
  /// This method adds a Joint of type `JointType` between two bodies.
  /// The two bodies connected by this Joint object are referred to as _parent_
  /// and _child_ bodies. The parent/child ordering defines the sign conventions
  /// for the generalized coordinates and the coordinate ordering for multi-DOF
  /// joints.
  ///
  /// <!-- NOLINTNEXTLINE(whitespace/line_length) -->
  /// @image html drake/multibody/plant/images/BodyParentChildJointCM.png
  /// width=50%
  ///
  /// Note: The previous figure also shows Pcm which is body P's center of mass
  /// and point Bcm which is body B's center of mass.
  ///
  /// As explained in the Joint class's documentation, in Drake we define a
  /// frame F attached to the parent body P with pose `X_PF` and a frame M
  /// attached to the child body B with pose `X_BM`. This method helps creating
  /// a joint between two bodies with fixed poses `X_PF` and `X_BM`.
  /// Refer to the Joint class's documentation for more details.
  ///
  /// @param name
  ///   A string that uniquely identifies the new joint to be added to `this`
  ///   model. A std::runtime_error is thrown if a joint named `name` already is
  ///   part of the model. See HasJointNamed(), Joint::name().
  /// @param[in] parent
  ///   The parent body connected by the new joint.
  /// @param[in] X_PF
  ///   The fixed pose of frame F attached to the parent body, measured in
  ///   the frame P of that body. `X_PF` is an optional parameter; empty curly
  ///   braces `{}` imply that frame F **is** the same body frame P. If instead
  ///   your intention is to make a frame F with pose `X_PF` equal to the
  ///   identity pose, provide `RigidTransform<double>::Identity()` as your
  ///   input. When non-nullopt, adds a FixedOffsetFrame named `{name}_parent`.
  /// @param[in] child
  ///   The child body connected by the new joint.
  /// @param[in] X_BM
  ///   The fixed pose of frame M attached to the child body, measured in
  ///   the frame B of that body. `X_BM` is an optional parameter; empty curly
  ///   braces `{}` imply that frame M **is** the same body frame B. If instead
  ///   your intention is to make a frame M with pose `X_BM` equal to the
  ///   identity pose, provide `RigidTransform<double>::Identity()` as your
  ///   input.  When non-nullopt, adds a FixedOffsetFrame named `{name}_child`.
  /// @param[in] args
  ///   Zero or more parameters provided to the constructor of the new joint. It
  ///   must be the case that
  ///   `JointType<T>(
  ///   const std::string&, const Frame<T>&, const Frame<T>&, args)` is a valid
  ///   constructor.
  /// @tparam JointType The type of the Joint to add.
  /// @returns A constant reference to the new joint just added, of type
  ///   `JointType<T>` specialized on the scalar type T of `this`
  ///   %MultibodyPlant. It will remain valid for the lifetime of `this`
  ///   %MultibodyPlant.
  ///
  /// Example of usage:
  /// @code
  ///   MultibodyPlant<T> plant;
  ///   // Code to define bodies serving as the joint's parent and child bodies.
  ///   const RigidBody<double>& body_1 =
  ///     plant.AddRigidBody("Body1", SpatialInertia<double>(...));
  ///   const RigidBody<double>& body_2 =
  ///     plant.AddRigidBody("Body2", SpatialInertia<double>(...));
  ///   // RigidBody 1 serves as parent, RigidBody 2 serves as child.
  ///   // Define the pose X_BM of a frame M rigidly attached to child body B.
  ///   const RevoluteJoint<double>& elbow =
  ///     plant.AddJoint<RevoluteJoint>(
  ///       "Elbow",                /* joint name */
  ///       body_1,                 /* parent body */
  ///       {},                     /* frame F IS the parent body frame P */
  ///       body_2,                 /* child body, the pendulum */
  ///       X_BM,                   /* pose of frame M in the body frame B */
  ///       Vector3d::UnitZ());     /* revolute axis in this case */
  /// @endcode
  ///
  /// @throws std::exception if `this` %MultibodyPlant already contains a joint
  ///     with the given `name`.  See HasJointNamed(), Joint::name().
  /// @throws std::exception if parent and child are the same body or if
  ///     they are not both from `this` %MultibodyPlant.
  ///
  /// @see The Joint class's documentation for further details on how a Joint
  /// is defined.
  template <template <typename> class JointType, typename... Args>
  const JointType<T>& AddJoint(
      const std::string& name, const RigidBody<T>& parent,
      const std::optional<math::RigidTransform<double>>& X_PF,
      const RigidBody<T>& child,
      const std::optional<math::RigidTransform<double>>& X_BM, Args&&... args) {
    // TODO(Mitiguy) Per discussion in PR# 13961 and issues #12789 and #13040,
    //  consider changing frame F to frame Jp and changing frame M to frame Jc.
    static_assert(std::is_base_of_v<Joint<T>, JointType<T>>,
                  "JointType<T> must be a sub-class of Joint<T>.");
    DRAKE_MBP_THROW_IF_FINALIZED();
    const JointType<T>& result =
        this->mutable_tree().template AddJoint<JointType>(
            name, parent, X_PF, child, X_BM, std::forward<Args>(args)...);
    return result;
  }
  // clang-format on

  /// Removes and deletes `joint` from this %MultibodyPlant. Any existing
  /// references to `joint` will become invalid, and future calls to
  /// `get_joint(joint_index)` will throw an exception. Other elements
  /// of the plant may depend on `joint` at the time of removal and should
  /// be removed first. For example, a JointActuator that depends on `joint`
  /// should be removed with RemoveJointActuator(). Currently, we do not
  /// provide joint dependency tracking for force elements or constraints,
  /// so this function will throw an exception if there are *any* user-added
  /// force elements or constraints in the plant.
  ///
  /// @throws std::exception if the plant is already finalized.
  /// @throws std::exception if the plant contains a non-zero number of
  /// user-added force elements or user-added constraints.
  /// @throws std::exception if `joint` has a dependent JointActuator.
  /// @see AddJoint()
  /// @note It is important to note that the JointIndex assigned to a joint is
  /// immutable. New joint indices are assigned in increasing order, even if a
  /// joint with a lower index has been removed. This has the consequence that
  /// when a joint is removed from the plant, the sequence `[0, num_joints())`
  /// is not necessarily the correct set of un-removed joint indices in the
  /// plant. Thus, it is important *NOT* to loop over joint indices sequentially
  /// from `0` to `num_joints() - 1`. Instead users should use the provided
  /// GetJointIndices() and GetJointIndices(ModelIndex) functions:
  /// ```
  /// for (JointIndex index : plant.GetJointIndices()) {
  ///   const Joint<double>& joint = plant.get_joint(index);
  ///   ...
  ///  }
  /// ```
  void RemoveJoint(const Joint<T>& joint);

  /// Welds `frame_on_parent_F` and `frame_on_child_M` with relative pose
  /// `X_FM`. That is, the pose of frame M in frame F is fixed, with value
  /// `X_FM`.  If `X_FM` is omitted, the identity transform will be used. The
  /// call to this method creates and adds a new WeldJoint to the model.  The
  /// new WeldJoint is named as:
  ///     frame_on_parent_F.name() + "_welds_to_" + frame_on_child_M.name().
  /// @returns a constant reference to the WeldJoint welding frames
  /// F and M.
  /// @throws std::exception if the weld produces a duplicate joint name.
  const WeldJoint<T>& WeldFrames(const Frame<T>& frame_on_parent_F,
                                 const Frame<T>& frame_on_child_M,
                                 const math::RigidTransform<double>& X_FM =
                                     math::RigidTransform<double>::Identity());

  /// Adds a new force element model of type `ForceElementType` to `this`
  /// %MultibodyPlant.  The arguments to this method `args` are forwarded to
  /// `ForceElementType`'s constructor.
  /// @param[in] args
  ///   Zero or more parameters provided to the constructor of the new force
  ///   element. It must be the case that
  ///   `ForceElementType<T>(args)` is a valid constructor.
  /// @tparam ForceElementType The type of the ForceElement to add.  As there
  /// is always a UniformGravityFieldElement present (accessible through
  /// gravity_field()), an exception will be thrown if this function is called
  /// to add another UniformGravityFieldElement.
  /// @returns A constant reference to the new ForceElement just added, of type
  ///   `ForceElementType<T>` specialized on the scalar type T of `this`
  ///   %MultibodyPlant. It will remain valid for the lifetime of `this`
  ///   %MultibodyPlant.
  /// @see The ForceElement class's documentation for further details on how a
  /// force element is defined.
  template <template <typename Scalar> class ForceElementType, typename... Args>
  const ForceElementType<T>& AddForceElement(Args&&... args) {
    DRAKE_MBP_THROW_IF_FINALIZED();
    return this->mutable_tree().template AddForceElement<ForceElementType>(
        std::forward<Args>(args)...);
  }

  /// Creates and adds a JointActuator model for an actuator acting on a given
  /// `joint`.
  /// This method returns a constant reference to the actuator just added, which
  /// will remain valid for the lifetime of `this` plant.
  ///
  /// @param[in] name
  ///   A string that uniquely identifies the new actuator to be added to `this`
  ///   model. A std::runtime_error is thrown if an actuator with the same name
  ///   already exists in the model. See HasJointActuatorNamed().
  /// @param[in] joint
  ///   The Joint to be actuated by the new JointActuator.
  /// @param[in] effort_limit
  ///   The maximum effort for the actuator. It must be strictly positive,
  ///   otherwise an std::exception is thrown. If +∞, the actuator has no limit,
  ///   which is the default. The effort limit has physical units in accordance
  ///   to the joint type it actuates. For instance, it will have units of
  ///   N⋅m (torque) for revolute joints while it will have units of N (force)
  ///   for prismatic joints.
  /// @returns A constant reference to the new JointActuator just added, which
  /// will remain valid for the lifetime of `this` plant or until the
  /// JointActuator has been removed from the plant with RemoveJointActuator().
  /// @throws std::exception if `joint.num_velocities() > 1` since for now we
  /// only support actuators for single dof joints.
  /// @see RemoveJointActuator()
  const JointActuator<T>& AddJointActuator(
      const std::string& name, const Joint<T>& joint,
      double effort_limit = std::numeric_limits<double>::infinity());

  /// Removes and deletes `actuator` from this %MultibodyPlant. Any existing
  /// references to `actuator` will become invalid, and future calls to
  /// `get_joint_actuator(actuator_index)` will throw an exception.
  ///
  /// @throws std::exception if the plant is already finalized.
  /// @see AddJointActuator()
  void RemoveJointActuator(const JointActuator<T>& actuator);

  /// Removes the effort limits on all joint actuators. (In other words, sets
  /// all effort limits to +∞.) This is a convenient way to obtain a plant
  /// without any built-in effort limits, in case models loaded by the Parser
  /// have unwanted limits.
  void RemoveAllJointActuatorEffortLimits();

  /// Creates a new model instance.  Returns the index for the model
  /// instance.
  ///
  /// @param[in] name
  ///   A string that uniquely identifies the new instance to be added to `this`
  ///   model. An exception is thrown if an instance with the same name
  ///   already exists in the model. See HasModelInstanceNamed().
  ModelInstanceIndex AddModelInstance(const std::string& name) {
    return this->mutable_tree().AddModelInstance(name);
  }

  /// Renames an existing model instance.
  ///
  /// @param[in] model_instance
  ///   The instance to rename.
  /// @param[in] name
  ///   A string that uniquely identifies the instance within `this` model.
  /// @throws std::exception if called after Finalize().
  /// @throws std::exception if `model_instance` is not a valid index.
  /// @throws std::exception if HasModelInstanceNamed(`name`) is true.
  void RenameModelInstance(ModelInstanceIndex model_instance,
                           const std::string& name);

  /// Sets the type of joint to be used by Finalize() to connect any otherwise
  /// unconnected bodies to World. Bodies connected by a joint to World are
  /// called _base bodies_ and are determined during Finalize() when we build
  /// a forest structure to model the multibody system for efficient
  /// computation.
  /// See @ref mbp_working_with_free_bodies "Working with free bodies"
  /// for a longer discussion.
  ///
  /// This can be set globally or for a particular model instance. Global
  /// options are used for any model elements that belong to model instances for
  /// which no options have been set explicitly. The default is to use a
  /// quaternion floating joint.
  ///
  /// | BaseBodyJointType::      | Notes                                  |
  /// | ------------------------ | -------------------------------------- |
  /// | kQuaternionFloatingJoint | 6 dofs, unrestricted orientation       |
  /// | kRpyFloatingJoint †      | 6 dofs, uses 3 angles for orientation  |
  /// | kWeldJoint               | 0 dofs, welded to World ("anchored")   |
  ///
  /// † The 3-angle orientation representation used by RpyFloatingJoint can be
  ///   easier to work with than a quaternion (especially for optimization) but
  ///   has a singular orientation which must be avoided (pitch angle near 90°).
  ///
  /// @note Reminder: if you aren't satisfied with the particular selection of
  /// joints here, you can always add an explicit joint to World with
  /// AddJoint().
  ///
  /// @param[in] joint_type The joint type to be used for base bodies.
  /// @param[in] model_instance (optional) the index of the model instance to
  ///   which `joint_type` is to be applied.
  /// @throws std::exception if called after Finalize().
  void SetBaseBodyJointType(
      BaseBodyJointType joint_type,
      std::optional<ModelInstanceIndex> model_instance = {});

  /// Returns the currently-set choice for base body joint type, either for
  /// the global setting or for a specific model instance if provided.
  /// If a model instance is provided for which no explicit choice has been
  /// made, the global setting is returned. Any model instance index is
  /// acceptable here; if not recognized then the global setting is returned.
  /// This can be called any time -- pre-finalize it returns the joint type
  /// that will be used by Finalize(); post-finalize it returns the joint type
  /// that _was_ used if there were any base bodies in need of a joint.
  /// @see SetBaseBodyJointType()
  BaseBodyJointType GetBaseBodyJointType(
      std::optional<ModelInstanceIndex> model_instance = {}) const;

  /// This method must be called after all elements in the model (joints,
  /// bodies, force elements, constraints, etc.) are added and before any
  /// computations are performed.
  /// It essentially compiles all the necessary "topological information", i.e.
  /// how bodies, joints and, any other elements connect with each other, and
  /// performs all the required pre-processing to enable computations at a
  /// later stage.
  ///
  /// If the finalize stage is successful, the topology of this %MultibodyPlant
  /// is valid, meaning that the topology is up-to-date after this call.
  /// No more multibody elements can be added after a call to Finalize().
  ///
  /// At Finalize(), state and input/output ports for `this` plant are declared.
  ///
  /// For a full account of the effects of Finalize(), see
  /// @ref mbp_finalize_stage "Finalize() stage".
  ///
  /// @see is_finalized(), @ref mbp_finalize_stage "Finalize() stage".
  ///
  /// @throws std::exception if the %MultibodyPlant has already been
  /// finalized.
  void Finalize();
  /// @}

  /// @anchor mbp_constraints
  /// @name                      Constraints
  ///
  /// Set of APIs to define constraints. To mention a few important examples,
  /// constraints can be used to couple the motion of joints, to create
  /// kinematic loops, or to weld bodies together.
  ///
  /// Currently constraints are only supported for discrete %MultibodyPlant
  /// models and not for all discrete solvers, see
  /// get_discrete_contact_solver(). If the model contains constraints not
  /// supported by the discrete solver, the plant will throw an exception at
  /// Finalize() time. At this point the user has the option to select a contact
  /// model approximation that uses a solver that supports constraints, or to
  /// re-define the model so that such a constraint is not needed. A contact
  /// model approximation can be set with set_discrete_contact_approximation()
  /// or in the MultibodyPlantConfig.
  ///
  /// Each constraint is identified with a MultibodyConstraintId returned
  /// by the function used to add it (e.g. AddCouplerConstraint()). It is
  /// possible to recover constraint specification parameters for each
  /// constraint with various introspection functions (e.g.
  /// get_coupler_constraint_specs()). Each constraint has an "active" status
  /// that is set to true by default. Query a constraint's active status with
  /// GetConstraintActiveStatus() and set its active status with
  /// SetConstraintActiveStatus().
  ///
  /// @warning Adding constraints to a continuous time plant is allowed at
  /// configuration time, but will raise exceptions at run time for results
  /// that should have been affected by the constraints.
  /// <!-- TODO(#23759,#23760,#23762,#23763,#23992): revisit this documentation
  /// as constraints are implemented for CENIC. -->
  /// <!-- TODO(joemasterjohn): As different constraint types are added in a
  /// piecemeal fashion, the burden of managing and maintaining these different
  /// constraints becomes cumbersome for the plant. Consider a new
  /// MultibodyConstraintManager class to consolidate constraint management. -->
  /// @{

  /// Returns the total number of constraints specified by the user.
  int num_constraints() const {
    return num_coupler_constraints() + num_distance_constraints() +
           num_ball_constraints() + num_weld_constraints() +
           num_tendon_constraints();
  }

  /// Returns a list of all constraint identifiers. The returned vector becomes
  /// invalid after any calls to Add*Constraint() or RemoveConstraint().
  std::vector<MultibodyConstraintId> GetConstraintIds() const;

  /// Returns the total number of coupler constraints specified by the user.
  int num_coupler_constraints() const {
    return ssize(coupler_constraints_specs_);
  }

  /// Returns the total number of distance constraints specified by the user.
  int num_distance_constraints() const {
    return ssize(distance_constraints_params_);
  }

  /// Returns the total number of ball constraints specified by the user.
  int num_ball_constraints() const { return ssize(ball_constraints_specs_); }

  /// Returns the total number of weld constraints specified by the user.
  int num_weld_constraints() const { return ssize(weld_constraints_specs_); }

  /// Returns the total number of tendon constraints specified by the
  /// user.
  int num_tendon_constraints() const {
    return ssize(tendon_constraints_specs_);
  }

  /// (Internal use only) Returns the coupler constraint specification
  /// corresponding to `id`
  /// @throws if `id` is not a valid identifier for a coupler constraint.
  const internal::CouplerConstraintSpec& get_coupler_constraint_specs(
      MultibodyConstraintId id) const {
    DRAKE_THROW_UNLESS(coupler_constraints_specs_.contains(id));
    return coupler_constraints_specs_.at(id);
  }

  /// (Internal use only)  Returns the ball constraint specification
  /// corresponding to `id`
  /// @throws if `id` is not a valid identifier for a ball constraint.
  const internal::BallConstraintSpec& get_ball_constraint_specs(
      MultibodyConstraintId id) const {
    DRAKE_THROW_UNLESS(ball_constraints_specs_.contains(id));
    return ball_constraints_specs_.at(id);
  }

  /// (Internal use only)  Returns the weld constraint specification
  /// corresponding to `id`
  /// @throws if `id` is not a valid identifier for a weld constraint.
  const internal::WeldConstraintSpec& get_weld_constraint_specs(
      MultibodyConstraintId id) const {
    DRAKE_THROW_UNLESS(weld_constraints_specs_.contains(id));
    return weld_constraints_specs_.at(id);
  }

  /// (Internal use only)  Returns the tendon constraint specification
  /// corresponding to `id`
  /// @throws if `id` is not a valid identifier for a tendon constraint.
  const internal::TendonConstraintSpec& get_tendon_constraint_specs(
      MultibodyConstraintId id) const {
    DRAKE_THROW_UNLESS(tendon_constraints_specs_.contains(id));
    return tendon_constraints_specs_.at(id);
  }

  /// (Internal use only)  Returns a reference to the all of the coupler
  /// constraints in this plant as a map from MultibodyConstraintId to
  /// CouplerConstraintSpec.
  const std::map<MultibodyConstraintId, internal::CouplerConstraintSpec>&
  get_coupler_constraint_specs() const {
    return coupler_constraints_specs_;
  }

  /// (Internal use only) Returns a reference to all of the ball constraints in
  /// this plant as a map from MultibodyConstraintId to BallConstraintSpec.
  const std::map<MultibodyConstraintId, internal::BallConstraintSpec>&
  get_ball_constraint_specs() const {
    return ball_constraints_specs_;
  }

  /// (Internal use only) Returns a reference to the all of the weld constraints
  /// in this plant as a map from MultibodyConstraintId to WeldConstraintSpec.
  const std::map<MultibodyConstraintId, internal::WeldConstraintSpec>&
  get_weld_constraint_specs() const {
    return weld_constraints_specs_;
  }

  /// (Internal use only) Returns a reference to the all of the tendon
  /// constraints in this plant as a map from MultibodyConstraintId to
  /// TendonConstraintSpec.
  const std::map<MultibodyConstraintId, internal::TendonConstraintSpec>&
  get_tendon_constraint_specs() const {
    return tendon_constraints_specs_;
  }

  /// Returns the active status of the constraint given by `id` in `context`.
  /// @throws std::exception if the %MultibodyPlant has not been finalized.
  /// @throws std::exception if `id` does not belong to any multibody constraint
  /// in `context`.
  bool GetConstraintActiveStatus(const systems::Context<T>& context,
                                 MultibodyConstraintId id) const;

  /// Sets the active status of the constraint given by `id` in `context`.
  /// @throws std::exception if the %MultibodyPlant has not been finalized.
  /// @throws std::exception if `context` == nullptr
  /// @throws std::exception if `id` does not belong to any multibody constraint
  /// in `context`.
  void SetConstraintActiveStatus(systems::Context<T>* context,
                                 MultibodyConstraintId id, bool status) const;

  /// Defines a holonomic constraint between two single-dof joints `joint0`
  /// and `joint1` with positions q₀ and q₁, respectively, such that q₀ = ρ⋅q₁ +
  /// Δq, where ρ is the gear ratio and Δq is a fixed offset. The gear ratio
  /// can have units if the units of q₀ and q₁ are different. For instance,
  /// between a prismatic and a revolute joint the gear ratio will specify the
  /// "pitch" of the resulting mechanism. As defined, `offset` has units of
  /// `q₀`.
  ///
  /// @note joint0 and/or joint1 can still be actuated, regardless of whether we
  /// have coupler constraint among them. That is, one or both of these joints
  /// can have external actuation applied to them.
  ///
  /// @note Generally, to couple (q0, q1, q2), the user would define a coupler
  /// between (q0, q1) and a second coupler between (q1, q2), or any
  /// combination therein.
  ///
  /// @throws if joint0 and joint1 are not both single-dof joints.
  /// @throws std::exception if the %MultibodyPlant has already been finalized.
  /// @throws std::exception if `this` %MultibodyPlant's underlying contact
  /// solver is not SAP. (i.e. get_discrete_contact_solver() !=
  /// DiscreteContactSolver::kSap)
  MultibodyConstraintId AddCouplerConstraint(const Joint<T>& joint0,
                                             const Joint<T>& joint1,
                                             double gear_ratio,
                                             double offset = 0.0);

  /// Defines a distance constraint between a point P on a body A and a point Q
  /// on a body B.
  ///
  /// This constraint can be compliant, modeling a spring with free length
  /// `distance` and given `stiffness` and `damping` parameters between points P
  /// and Q. For d = ‖p_PQ‖, then a compliant distance constraint models a
  /// spring with force along p_PQ given by:
  ///
  ///    f = −stiffness ⋅ d − damping ⋅ ḋ
  ///
  /// @param[in] body_A RigidBody to which point P is rigidly attached.
  /// @param[in] p_AP Position of point P in body A's frame.
  /// @param[in] body_B RigidBody to which point Q is rigidly attached.
  /// @param[in] p_BQ Position of point Q in body B's frame.
  /// @param[in] distance Fixed length of the distance constraint, in meters. It
  /// must be strictly positive.
  /// @param[in] stiffness For modeling a spring with free length equal to
  /// `distance`, the stiffness parameter in N/m. Optional, with its default
  /// value being infinite to model a rigid massless rod of length `distance`
  /// connecting points A and B.
  /// @param[in] damping For modeling a spring with free length equal to
  /// `distance`, damping parameter in N⋅s/m. Optional, with its default value
  /// being zero for a non-dissipative constraint.
  /// @returns the id of the newly added constraint.
  ///
  /// @warning Currently, it is the user's responsibility to initialize the
  /// model's context in a configuration compatible with the newly added
  /// constraint.
  ///
  /// @warning A distance constraint is the wrong modeling choice if the
  /// distance needs to go through zero. To constrain two points to be
  /// coincident we need a 3-dof ball constraint, the 1-dof distance constraint
  /// is singular in this case. Therefore we require the distance parameter to
  /// be strictly positive.
  ///
  /// @note When a new context is created, a DistanceConstraintParams is
  /// initialized to store the parameters passed to this function. Parameters in
  /// the context can be modified with calls to SetDistanceConstraintParams().
  ///
  /// @throws std::exception if bodies A and B are the same body.
  /// @throws std::exception if `distance` is not strictly positive.
  /// @throws std::exception if `stiffness` is not positive or zero.
  /// @throws std::exception if `damping` is not positive or zero.
  /// @throws std::exception if the %MultibodyPlant has already been finalized.
  /// @throws std::exception if `this` %MultibodyPlant's underlying contact
  /// solver is not SAP. (i.e. get_discrete_contact_solver() !=
  /// DiscreteContactSolver::kSap)
  MultibodyConstraintId AddDistanceConstraint(
      const RigidBody<T>& body_A, const Vector3<double>& p_AP,
      const RigidBody<T>& body_B, const Vector3<double>& p_BQ, double distance,
      double stiffness = std::numeric_limits<double>::infinity(),
      double damping = 0.0);

  /// Returns all default distance constraint parameters, as registered via
  /// AddDistanceConstraint(). See GetDistanceConstraintParams() and
  /// SetDistanceConstraintParams() for working with parameters stored in a
  /// context.
  const std::map<MultibodyConstraintId, DistanceConstraintParams>&
  GetDefaultDistanceConstraintParams() const;

  /// Returns all distance constraint parameters currently stored in `context`.
  const std::map<MultibodyConstraintId, DistanceConstraintParams>&
  GetDistanceConstraintParams(const systems::Context<T>& context) const;

  /// Returns a constant reference to the parameters for the distance constraint
  /// that corresponds to identifier `id`.
  /// @throws if `id` is not a valid identifier for a distance constraint.
  const DistanceConstraintParams& GetDistanceConstraintParams(
      const systems::Context<T>& context, MultibodyConstraintId id) const;

  /// Stores in `context` the parameters `params` for the distance constraint
  /// with identifier `id`.
  ///
  /// @param[in, out] context The plant's context. On output it stores `params`
  ///                         for the requested distance constraint.
  /// @param[in]  id          Unique identifier of the constraint.
  /// @param[in]  params      The new set of parameters to be stored in
  ///                         `context`.
  ///
  /// @throws if `id` is not a valid identifier for a distance constraint.
  /// @throws if params.bodyA() or params.bodyB() do not correspond to rigid
  /// bodies in `this` %MultibodyPlant.
  void SetDistanceConstraintParams(systems::Context<T>* context,
                                   MultibodyConstraintId id,
                                   DistanceConstraintParams params) const;

  /// Defines a constraint such that point P affixed to body A is coincident at
  /// all times with point Q affixed to body B, effectively modeling a
  /// ball-and-socket joint.
  ///
  /// @param[in] body_A RigidBody to which point P is rigidly attached.
  /// @param[in] p_AP Position of point P in body A's frame.
  /// @param[in] body_B RigidBody to which point Q is rigidly attached.
  /// @param[in] p_BQ (optional) Position of point Q in body B's frame. If p_BQ
  /// is std::nullopt, then p_BQ will be computed so that the constraint is
  /// satisfied for the default configuration at Finalize() time; subsequent
  /// changes to the default configuration will not change the computed p_BQ.
  /// @returns the id of the newly added constraint.
  ///
  /// @throws std::exception if bodies A and B are the same body.
  /// @throws std::exception if the %MultibodyPlant has already been finalized.
  /// @throws std::exception if `this` %MultibodyPlant's underlying contact
  /// solver is not SAP. (i.e. get_discrete_contact_solver() !=
  /// DiscreteContactSolver::kSap)
  MultibodyConstraintId AddBallConstraint(
      const RigidBody<T>& body_A, const Vector3<double>& p_AP,
      const RigidBody<T>& body_B,
      const std::optional<Vector3<double>>& p_BQ = {});

  /// Defines a constraint such that frame P affixed to body A is coincident at
  /// all times with frame Q affixed to body B, effectively modeling a weld
  /// joint.
  ///
  /// @param[in] body_A RigidBody to which frame P is rigidly attached.
  /// @param[in] X_AP Pose of frame P in body A's frame.
  /// @param[in] body_B RigidBody to which frame Q is rigidly attached.
  /// @param[in] X_BQ Pose of frame Q in body B's frame.
  /// @returns the id of the newly added constraint.
  ///
  /// @throws std::exception if bodies A and B are the same body.
  /// @throws std::exception if the %MultibodyPlant has already been finalized.
  /// @throws std::exception if `this` %MultibodyPlant's underlying contact
  /// solver is not SAP. (i.e. get_discrete_contact_solver() !=
  /// DiscreteContactSolver::kSap)
  MultibodyConstraintId AddWeldConstraint(
      const RigidBody<T>& body_A, const math::RigidTransform<double>& X_AP,
      const RigidBody<T>& body_B, const math::RigidTransform<double>& X_BQ);

  /// Defines a set of unilateral constraints on the length of an abstract
  /// tendon defined as:
  ///
  ///   l(q) = aᵀ⋅q + offset ∈ ℝ
  ///
  /// where **q** is the configuration of the model, **a** is a vector of
  /// coefficients, and **offset** a scalar offset. This constraint imposes:
  ///
  ///   lₗ ≤ l(q) ≤ lᵤ
  ///
  /// where **lₗ** and **lᵤ** are lower and upper bounds, respectively. Both
  /// limits are not strictly required. At most one of **lₗ** or **lᵤ** may be
  /// infinite (−∞ for **lₗ** and ∞ for **lᵤ**), indicating no lower or upper
  /// limit, respectively.
  ///
  /// For finite `stiffness` and `damping`, this constraint is modeled by
  /// compliant spring-like forces:
  ///
  ///  fₗ = −stiffness⋅(l - lₗ) − damping⋅dl(q)/dt \n
  ///  fᵤ = −stiffness⋅(lᵤ - l) + damping⋅dl(q)/dt
  ///
  /// that act to keep the length within bounds. If the user provided stiffness
  /// is either omitted or set to ∞, this constraint is modeled as close to
  /// rigid as possible by the underlying solver.
  ///
  /// @note The coefficients in a are expected to have units such that the
  /// abstract length l(q) has consistent units (either meters or radians) and
  /// it is up to the user to maintain consistency in these units. The
  /// (optionally user provided) `stiffness` and `damping` are expected to have
  /// consistent units such that their products have units of the corresponding
  /// generalized force. E.g. N/m for `stiffness` and N⋅s/m for `damping` when l
  /// has units of m, so that **fₗ** and **fᵤ** have units of N.
  ///
  /// @note Any joint involved in this constraint can still be actuated.
  ///
  /// @note See the MuJoCo model documentation for details the equivalent
  /// concept of a "fixed" tendon:
  /// https://mujoco.readthedocs.io/en/stable/XMLreference.html#tendon-fixed
  ///
  /// @param[in] joints Non-empty vector of single-dof joint indices where the
  /// configuration, qᵢ, of joints[i] corresponds to the entry a[i].
  /// @param[in] a Non-empty vector of coefficients where a[i]
  /// corresponds to the configuration, qᵢ, of joints[i].
  /// @param[in] offset (optional) Scalar length offset in either [m] or [rad].
  /// If std::nullopt, it is set to 0.
  /// @param[in] lower_limit (optional) Lower bound on l in either [m] or [rad].
  /// If std::nullopt, it is set to −∞.
  /// @param[in] upper_limit Upper bound on l in either [m] or [rad]. If
  /// std::nullopt, it is set to ∞.
  /// @param[in] stiffness (optional) Constraint stiffness in either [N/m] or
  /// [N⋅m/rad]. If std::nullopt, its default value is set to ∞ to model a rigid
  /// constraint.
  /// @param[in] damping (optional) Constraint damping in either [N⋅s/m] or
  /// [N⋅m⋅rad/s]. If std::nullopt, it is set to 0 to model a non-dissipative
  /// constraint.
  ///
  /// @warning Because of a restriction in the SAP solver, **at most** two
  /// kinematic trees can be represented by the joints in `joints`. This
  /// violation is only detected after the simulation has been started, in which
  /// case the solver will throw an exception when trying to add the constraint.
  ///
  /// @pre `joints.size() > 0`
  /// @pre `joints` contains no duplicates.
  /// @pre `a.size() == joints.size()`
  /// @pre `index ∈ joints` is a valid (non-removed) index to a joint in this
  /// plant.
  /// @pre `get_joint(index).%num_velocities() == 1` for each index in `joints`.
  /// @pre `lower_limit < ∞` (if not std::nullopt).
  /// @pre `upper_limit > -∞` (if not std::nullopt).
  /// @pre At least one of `lower_limit` and `upper_limit` are finite.
  /// @pre `lower_limit ≤ upper_limit` (if not std::nullopt).
  /// @pre `stiffness > 0` (if not std::nullopt).
  /// @pre `damping >= 0` (if not std::nullopt).
  ///
  /// @throws std::exception if the %MultibodyPlant has already been finalized.
  /// @throws std::exception if `this` %MultibodyPlant's underlying contact
  /// solver is not SAP. (i.e. get_discrete_contact_solver() !=
  /// DiscreteContactSolver::kSap).
  MultibodyConstraintId AddTendonConstraint(std::vector<JointIndex> joints,
                                            std::vector<double> a,
                                            std::optional<double> offset,
                                            std::optional<double> lower_limit,
                                            std::optional<double> upper_limit,
                                            std::optional<double> stiffness,
                                            std::optional<double> damping);

  /// Removes the constraint `id` from the plant. Note that this will _not_
  /// remove constraints registered directly with DeformableModel.
  ///
  /// @throws std::exception if the %MultibodyPlant has already been finalized.
  /// @throws std::exception if `id` does not identify any multibody constraint
  /// in this plant.
  void RemoveConstraint(MultibodyConstraintId id);

  /// <!-- TODO(#18732): Add getters to interrogate existing constraints.
  /// -->

  /// @}

  /// @anchor mbp_geometry
  /// @name                      Geometry
  ///
  /// The following geometry methods provide a convenient means for associating
  /// geometries with bodies. Ultimately, the geometries are owned by
  /// @ref geometry::SceneGraph "SceneGraph". These methods do the work of
  /// registering the requested geometries with SceneGraph and maintaining a
  /// mapping between the body and the registered data. Particularly, SceneGraph
  /// knows nothing about the concepts inherent in the %MultibodyPlant. These
  /// methods account for those differences as documented below.
  ///
  /// <h4>Geometry registration with roles</h4>
  ///
  /// Geometries can be associated with bodies via the `RegisterFooGeometry`
  /// family of methods. In SceneGraph, geometries have @ref geometry_roles
  /// "roles". The `RegisterCollisionGeometry()` methods register geometry with
  /// SceneGraph and assign it the proximity role. The
  /// `RegisterVisualGeometry()` methods do the same, but assign the
  /// illustration and/or perception role, depending on how the API is
  /// exercised (see below).
  ///
  /// All geometry registration methods return a @ref geometry::GeometryId
  /// GeometryId. This is how SceneGraph refers to the geometries. The
  /// properties of an individual geometry can be accessed with its id and
  /// geometry::SceneGraphInspector and geometry::QueryObject (for its
  /// state-dependent pose in world).
  ///
  /// <h4>%Body frames and SceneGraph frames</h4>
  ///
  /// The first time a geometry registration method is called on a particular
  /// body, that body's frame B is registered with SceneGraph. As SceneGraph
  /// knows nothing about bodies, in the SceneGraph domain, the frame is simply
  /// notated as F; this is merely an alias for the body frame. Thus, the pose
  /// of the geometry G in the SceneGraph frame F is the same as the pose of the
  /// geometry in the body frame B; `X_FG = X_BG`.
  ///
  /// The model instance index of the body is passed to the SceneGraph frame as
  /// its "frame group". This can be retrieved from the
  /// geometry::SceneGraphInspector::GetFrameGroup(FrameId) method.
  ///
  /// Given a GeometryId, SceneGraph cannot report what _body_ it is affixed to.
  /// It can only report the SceneGraph alias frame F. But the following idiom
  /// can report the body:
  ///
  /// ```
  /// const MultibodyPlant<T>& plant = ...;
  /// const SceneGraphInspector<T>& inspector =  ...;
  /// const GeometryId g_id = id_from_some_query;
  /// const FrameId f_id = inspector.GetFrameId(g_id);
  /// const RigidBody<T>* body = plant.GetBodyFromFrameId(f_id);
  /// ```
  /// See documentation of geometry::SceneGraphInspector on where to get an
  /// inspector.
  ///
  /// <h4>%MultibodyPlant names vs. SceneGraph names</h4>
  ///
  /// In %MultibodyPlant, frame names only have to be unique in a single
  /// model instance. However, SceneGraph knows nothing of model instances. So,
  /// to generate unique names for the corresponding frames in SceneGraph,
  /// when %MultibodyPlant registers the corresponding SceneGraph frame, it is
  /// named with a "scoped name". This is a concatenation of
  /// `[model instance name]::[body name]`. Searching for a frame with just the
  /// name `body name` will fail. (See RigidBody::name() and
  /// GetModelInstanceName() for those values.)
  ///
  /// Geometry names get scoped in the same way. The name passed to a
  /// RegisterXXGeometry becomes the scoped name `[model instance name]::[name]`
  /// in SceneGraph.
  ///
  /// <h4>Registering visual roles</h4>
  ///
  /// Drake has two visual roles: illustration and perception. Unless otherwise
  /// indicated, the RegisterVisualGeometry() APIs register the given geometry
  /// with both roles. One API allows specific control over which roles are
  /// assigned.
  /// @note This "assign-all-roles" behavior may change in the future and code
  /// that directly relies on it will break.
  ///
  /// Furthermore, unless the ("label", "id") property has already
  /// been defined, the PerceptionProperties will be assigned that property
  /// with a geometry::RenderLabel whose value is equal to the body's index.
  /// @{

  /// Registers `this` plant to serve as a source for an instance of
  /// SceneGraph. This registration allows %MultibodyPlant to
  /// register geometry with `scene_graph` for visualization and/or
  /// collision queries.  The string returned by `this->get_name()` is passed
  /// to SceneGraph's RegisterSource, so it is highly recommended that you give
  /// the plant a recognizable name before calling this.
  /// Successive registration calls with SceneGraph **must** be performed on
  /// the same instance to which the pointer argument `scene_graph` points
  /// to. Failure to do so will result in runtime exceptions.
  /// @param scene_graph
  ///   A valid non nullptr to the SceneGraph instance for which
  ///   `this` plant will sever as a source, see SceneGraph documentation
  ///   for further details.
  /// @returns the SourceId of `this` plant in `scene_graph`. It can also
  /// later on be retrieved with get_source_id().
  /// @throws std::exception if called post-finalize.
  /// @throws std::exception if `scene_graph` is the nullptr.
  /// @throws std::exception if called more than once.
  geometry::SourceId RegisterAsSourceForSceneGraph(
      geometry::SceneGraph<T>* scene_graph);

  /// Registers geometry in a SceneGraph with a given geometry::Shape to be
  /// used for visualization of a given `body`. The perception properties are a
  /// copy of the given `properties`. If the resulting perception properties
  /// do not include ("label", "id"), then it is added as documented above.
  ///
  /// See @ref mbp_geometry "the overview" for more details.
  ///
  /// @param[in] body
  ///   The body for which geometry is being registered.
  /// @param[in] X_BG
  ///   The fixed pose of the geometry frame G in the body frame B.
  /// @param[in] shape
  ///   The geometry::Shape used for visualization. E.g.: geometry::Sphere,
  ///   geometry::Cylinder, etc.
  /// @param[in] name
  ///   The name for the geometry. It must satisfy the requirements defined in
  ///   drake::geometry::GeometryInstance.
  /// @param[in] properties
  ///   The illustration properties for this geometry.
  /// @throws std::exception if called post-finalize.
  /// @returns the id for the registered geometry.
  geometry::GeometryId RegisterVisualGeometry(
      const RigidBody<T>& body, const math::RigidTransform<double>& X_BG,
      const geometry::Shape& shape, const std::string& name,
      const geometry::IllustrationProperties& properties);

  /// Registers the given `geometry_instance` in a SceneGraph to be used for
  /// visualization of a given `body`.
  ///
  /// The roles that `geometry_instance` gets assigned (illustration/perception)
  /// in SceneGraph depend solely on the properties that have _already_ been
  /// assigned to `geometry_instance`. If _any_ visual roles have been assigned,
  /// those will be the only roles used. If _no_ visual roles have been
  /// assigned, then both roles will be assigned using the default set of
  /// property values.
  ///
  /// If the registered geometry has the perception role, it will have the
  /// ("label", "id") property. Possibly assigned as documented above.
  ///
  /// See @ref mbp_geometry "the overview" for more details.
  ///
  /// @param[in] body
  ///   The body for which geometry is being registered.
  /// @param[in] geometry_instance
  ///   The geometry to associate with the visual appearance of `body`.
  /// @throws std::exception if `geometry_instance` is null.
  /// @throws std::exception if called post-finalize.
  /// @returns the id for the registered geometry.
  geometry::GeometryId RegisterVisualGeometry(
      const RigidBody<T>& body,
      std::unique_ptr<geometry::GeometryInstance> geometry_instance);

  /// Overload for visual geometry registration. The following properties are
  /// set:
  ///   - ("phong", "diffuse") = `diffuse_color` in both sets of properties.
  ///   - ("label", "id") in perception properties as documented above.
  ///
  /// See @ref mbp_geometry "the overview" for more details.
  geometry::GeometryId RegisterVisualGeometry(
      const RigidBody<T>& body, const math::RigidTransform<double>& X_BG,
      const geometry::Shape& shape, const std::string& name,
      const Vector4<double>& diffuse_color);

  /// Overload for visual geometry registration. The ("label", "id")  property
  /// is set in the perception properties (as documented above).
  ///
  /// See @ref mbp_geometry "the overview" for more details.
  geometry::GeometryId RegisterVisualGeometry(
      const RigidBody<T>& body, const math::RigidTransform<double>& X_BG,
      const geometry::Shape& shape, const std::string& name);

  /// Returns an array of GeometryId's identifying the different visual
  /// geometries for `body` previously registered with a SceneGraph.
  /// @note This method can be called at any time during the lifetime of `this`
  /// plant, either pre- or post-finalize, see Finalize().
  /// Post-finalize calls will always return the same value.
  /// @see RegisterVisualGeometry(), Finalize()
  const std::vector<geometry::GeometryId>& GetVisualGeometriesForBody(
      const RigidBody<T>& body) const;

  /// Registers geometry in a SceneGraph with a given geometry::Shape to be
  /// used for the contact modeling of a given `body`.
  /// More than one geometry can be registered with a body, in which case the
  /// body's contact geometry is the union of all geometries registered to that
  /// body.
  ///
  /// @param[in] body
  ///   The body for which geometry is being registered.
  /// @param[in] X_BG
  ///   The fixed pose of the geometry frame G in the body frame B.
  /// @param[in] shape
  ///   The geometry::Shape used for collision and contact. E.g.:
  ///   geometry::Sphere, geometry::Cylinder, etc.
  /// @param[in] properties
  ///   The proximity properties associated with the collision geometry.
  /// @throws std::exception if called post-finalize.
  geometry::GeometryId RegisterCollisionGeometry(
      const RigidBody<T>& body, const math::RigidTransform<double>& X_BG,
      const geometry::Shape& shape, const std::string& name,
      geometry::ProximityProperties properties);

  // TODO(SeanCurtis-TRI): Deprecate this in favor of simply passing properties.
  /// Overload which specifies a single property: coulomb_friction.
  geometry::GeometryId RegisterCollisionGeometry(
      const RigidBody<T>& body, const math::RigidTransform<double>& X_BG,
      const geometry::Shape& shape, const std::string& name,
      const CoulombFriction<double>& coulomb_friction);

  /// Returns an array of GeometryId's identifying the different contact
  /// geometries for `body` previously registered with a SceneGraph.
  /// @note This method can be called at any time during the lifetime of `this`
  /// plant, either pre- or post-finalize, see Finalize().
  /// Post-finalize calls will always return the same value.
  /// @see RegisterCollisionGeometry(), Finalize()
  const std::vector<geometry::GeometryId>& GetCollisionGeometriesForBody(
      const RigidBody<T>& body) const;

  /// Excludes the collision geometries between two given collision filter
  /// groups.
  /// @pre RegisterAsSourceForSceneGraph() has been called.
  /// @pre Finalize() has *not* been called.
  void ExcludeCollisionGeometriesWithCollisionFilterGroupPair(
      const std::pair<std::string, geometry::GeometrySet>&
          collision_filter_group_a,
      const std::pair<std::string, geometry::GeometrySet>&
          collision_filter_group_b);

  /// For each of the provided `bodies`, collects up all geometries that have
  /// been registered to that body. Intended to be used in conjunction with
  /// CollisionFilterDeclaration and
  /// CollisionFilterManager::Apply() to filter collisions between the
  /// geometries registered to the bodies.
  ///
  /// For example:
  /// ```
  /// // Don't report on collisions between geometries affixed to `body1`,
  /// // `body2`, or `body3`.
  /// std::vector<const RigidBody<T>*> bodies{&body1, &body2, &body3};
  /// geometry::GeometrySet set = plant.CollectRegisteredGeometries(bodies);
  /// scene_graph.collision_filter_manager().Apply(
  ///     CollisionFilterDeclaration().ExcludeWithin(set));
  /// ```
  ///
  /// @note There is a *very* specific order of operations:
  ///
  /// 1. Bodies and geometries must be added to the %MultibodyPlant.
  /// 2. Create GeometrySet instances from bodies (via this method).
  /// 3. Invoke SceneGraph::ExcludeCollisions*() to filter collisions.
  /// 4. Allocate context.
  ///
  /// Changing the order will cause exceptions to be thrown.
  ///
  /// @throws std::exception if `this` %MultibodyPlant was not
  /// registered with a SceneGraph.
  geometry::GeometrySet CollectRegisteredGeometries(
      const std::vector<const RigidBody<T>*>& bodies) const;

  /// Given a geometry frame identifier, returns a pointer to the body
  /// associated with that id (nullptr if there is no such body).
  const RigidBody<T>* GetBodyFromFrameId(geometry::FrameId frame_id) const {
    const auto it = frame_id_to_body_index_.find(frame_id);
    if (it == frame_id_to_body_index_.end()) return nullptr;
    return &internal_tree().get_body(it->second);
  }

  /// If the body with `body_index` belongs to the called plant, it returns
  /// the geometry::FrameId associated with it. Otherwise, it returns nullopt.
  std::optional<geometry::FrameId> GetBodyFrameIdIfExists(
      BodyIndex body_index) const {
    const auto it = body_index_to_frame_id_.find(body_index);
    if (it == body_index_to_frame_id_.end()) {
      return {};
    }
    return it->second;
  }

  /// If the body with `body_index` belongs to the called plant, it returns
  /// the geometry::FrameId associated with it. Otherwise this method throws
  /// an exception.
  /// @throws std::exception if the called plant does not have the body
  /// indicated by `body_index`.
  geometry::FrameId GetBodyFrameIdOrThrow(BodyIndex body_index) const {
    const auto it = body_index_to_frame_id_.find(body_index);
    if (it == body_index_to_frame_id_.end()) {
      throw std::logic_error("Body '" +
                             internal_tree().get_body(body_index).name() +
                             "' does not have geometry registered with it.");
    }
    return it->second;
  }

  /// Returns the inspector from the `context` for the SceneGraph associated
  /// with this plant, via this plant's "geometry_query" input port. (In the
  /// future, the inspector might come from a different context source that
  /// is more efficient than the "geometry_query" input port.)
  const geometry::SceneGraphInspector<T>& EvalSceneGraphInspector(
      const systems::Context<T>& context) const;
  /// @} <!-- Geometry -->

  /// @anchor mbp_contact_modeling
  /// @name                    Contact modeling
  /// Use methods in this section to choose the contact model and to provide
  /// parameters for that model. Currently Drake supports an advanced compliant
  /// model of continuous surface patches we call _Hydroelastic contact_ and a
  /// compliant point contact model as a reliable fallback. Please refer to
  /// @ref compliant_contact "Modeling Compliant Contact" for details.
  ///
  /// @{

  /// Sets the contact model to be used by `this` %MultibodyPlant, see
  /// ContactModel for available options.
  /// The default contact model is ContactModel::kHydroelasticWithFallback.
  /// @throws std::exception iff called post-finalize.
  void set_contact_model(ContactModel model);

  /// Returns the contact solver type used for discrete %MultibodyPlant models.
  DiscreteContactSolver get_discrete_contact_solver() const;

  /// Sets the discrete contact model approximation.
  ///
  /// @note Calling this method also sets the contact solver type (see
  /// get_discrete_contact_solver()) according to:
  /// - DiscreteContactApproximation::kTamsi sets the solver to
  ///   DiscreteContactSolver::kTamsi.
  /// - DiscreteContactApproximation::kSap,
  ///   DiscreteContactApproximation::kSimilar and
  ///   DiscreteContactApproximation::kLagged set the solver to
  ///   DiscreteContactSolver::kSap.
  ///
  /// @throws iff `this` plant is continuous (i.e. is_discrete() is `false`.)
  /// @throws std::exception iff called post-finalize.
  void set_discrete_contact_approximation(
      DiscreteContactApproximation approximation);

  /// @returns the discrete contact solver approximation.
  DiscreteContactApproximation get_discrete_contact_approximation() const;

  /// Non-negative dimensionless number typically in the range [0.0, 1.0],
  /// though larger values are allowed even if uncommon. This parameter controls
  /// the "near rigid" regime of the SAP solver, β in section V.B of [Castro et
  /// al., 2021]. It essentially controls a threshold value for the maximum
  /// amount of stiffness SAP can handle robustly. Beyond this value, stiffness
  /// saturates as explained in [Castro et al., 2021]. A value of 1.0 is a
  /// conservative choice to avoid ill-conditioning that might lead to softer
  /// than expected contact. If this is your case, consider turning off this
  /// approximation by setting this parameter to zero. For difficult cases where
  /// ill-conditioning is a problem, a small but non-zero number can be used,
  /// e.g. 1.0e-3.
  /// @throws std::exception if near_rigid_threshold is negative.
  /// @throws std::exception if called post-finalize.
  void set_sap_near_rigid_threshold(
      double near_rigid_threshold =
          MultibodyPlantConfig{}.sap_near_rigid_threshold);

  /// @returns the SAP near rigid regime threshold.
  /// @see See set_sap_near_rigid_threshold().
  double get_sap_near_rigid_threshold() const;

  /// Return the default value for contact representation, given the desired
  /// time step. Discrete systems default to use polygons; continuous systems
  /// default to use triangles.
  static geometry::HydroelasticContactRepresentation
  GetDefaultContactSurfaceRepresentation(double time_step) {
    // Maintainers should keep this function consistent with defaults chosen in
    // MultibodyPlantConfig.
    if (time_step == 0.0) {
      return geometry::HydroelasticContactRepresentation::kTriangle;
    }
    return geometry::HydroelasticContactRepresentation::kPolygon;
  }

  /// Sets the representation of contact surfaces to be used by `this`
  /// %MultibodyPlant. See geometry::HydroelasticContactRepresentation for
  /// available options. See GetDefaultContactSurfaceRepresentation() for
  /// explanation of default values.
  /// @throws std::exception if called post-finalize.
  void set_contact_surface_representation(
      geometry::HydroelasticContactRepresentation representation) {
    DRAKE_MBP_THROW_IF_FINALIZED();
    contact_surface_representation_ = representation;
  }

  /// Gets the current representation of contact surfaces used by `this`
  /// %MultibodyPlant.
  geometry::HydroelasticContactRepresentation
  get_contact_surface_representation() const {
    return contact_surface_representation_;
  }

  /// Sets whether to apply collision filters to topologically adjacent bodies
  /// at Finalize() time.  Filters are applied when there exists a joint
  /// between bodies, except in the case of 6-dof joints or joints in which the
  /// parent body is `world`.
  /// @throws std::exception iff called post-finalize.
  void set_adjacent_bodies_collision_filters(bool value) {
    DRAKE_MBP_THROW_IF_FINALIZED();
    adjacent_bodies_collision_filters_ = value;
  }

  /// Returns whether to apply collision filters to topologically adjacent
  /// bodies at Finalize() time.
  bool get_adjacent_bodies_collision_filters() const {
    return adjacent_bodies_collision_filters_;
  }

  /// For use only by advanced developers wanting to try out their custom time
  /// stepping strategies, including contact resolution.
  ///
  /// @experimental
  ///
  /// With this method MultibodyPlant takes ownership of `manager`.
  ///
  /// @note Setting a contact manager bypasses the mechanism to set a different
  /// contact solver with SetContactSolver(). Use only one of these two
  /// experimental mechanisms but never both.
  ///
  /// @param manager
  ///   After this call the new manager is used to advance discrete states.
  /// @pre this %MultibodyPlant is discrete.
  /// @pre manager != nullptr.
  /// @throws std::exception if called pre-finalize. See Finalize().
  void SetDiscreteUpdateManager(
      std::unique_ptr<internal::DiscreteUpdateManager<T>> manager);

#ifndef DRAKE_DOXYGEN_CXX
  // (For testing only) Adds a DummyPhysicalModel to this plant and returns the
  // added model if successful. With this method, MultibodyPlant takes ownership
  // of `model` and calls its DeclareSystemResources() method at Finalize(),
  // allowing the DummyPhysicalModel to declare the system resources it needs.
  //
  // @param model After this call the model is owned by `this` MultibodyPlant.
  // @returns a mutable reference to the added DummyPhysicalModel that's valid
  // for the life time of this MultibodyPlant.
  // @throws std::exception if called post-finalize. See Finalize().
  // @throws std::exception if model is nullptr or a DummyPhysicalModel is
  // already added.
  internal::DummyPhysicalModel<T>& AddDummyModel(
      std::unique_ptr<internal::DummyPhysicalModel<T>> model);

  // (Internal only) Returns a vector of pointers to all physical models
  // registered with `this` MultibodyPlant.
  std::vector<const PhysicalModel<T>*> physical_models() const;
#endif

  /// Returns the DeformableModel owned by this plant.
  /// @experimental
  const DeformableModel<T>& deformable_model() const {
    const DeformableModel<T>* model = physical_models_->deformable_model();
    // A DeformableModel is always added to the plant at construction time.
    DRAKE_DEMAND(model != nullptr);
    return *model;
  }

  /// Returns a mutable reference to the DeformableModel owned by this plant.
  /// @throws std::exception if the plant is finalized.
  /// @experimental
  DeformableModel<T>& mutable_deformable_model() {
    DRAKE_MBP_THROW_IF_FINALIZED();
    DeformableModel<T>* model = physical_models_->mutable_deformable_model();
    // A DeformableModel is always added to the plant at construction time.
    DRAKE_DEMAND(model != nullptr);
    return *model;
  }

  // TODO(amcastro-tri): deprecate. Defaults should always come from
  // DefaultProximityProperties.
  /// Sets a penetration allowance used to estimate the point contact stiffness
  /// and Hunt & Crossley dissipation parameters. Refer to the
  /// section @ref point_contact_defaults "Point Contact Default Parameters"
  /// for further details.
  ///
  /// @warning This will be deprecated. Prefer using defaults specified in
  /// geometry::DefaultProximityProperties.
  ///
  /// @warning Values provided in geometry::DefaultProximityProperties have
  /// precedence. If values estimated based on penetration allowance are
  /// desired, set defaults in geometry::DefaultProximityProperties to
  /// std::nullopt.
  ///
  /// @throws std::exception if penetration_allowance is not positive.
  void set_penetration_allowance(
      double penetration_allowance =
          MultibodyPlantConfig{}.penetration_allowance);

  // TODO(amcastro-tri): deprecate. Defaults should always come from
  // DefaultProximityProperties.
  /// Returns a time-scale estimate `tc` based on the requested penetration
  /// allowance δ set with set_penetration_allowance().
  /// For the compliant contact model to enforce non-penetration, this time
  /// scale relates to the time it takes the relative normal velocity between
  /// two bodies to go to zero. This time scale `tc` is a global estimate of the
  /// dynamics introduced by the compliant contact model and goes to zero in the
  /// limit to ideal rigid contact. Since numerical integration methods for
  /// continuum systems must be able to resolve a system's dynamics, the time
  /// step used by an integrator must in general be much smaller than the time
  /// scale `tc`. How much smaller will depend on the details of the problem and
  /// the convergence characteristics of the integrator and should be tuned
  /// appropriately.
  /// Another factor to take into account for setting up the simulation's time
  /// step is the speed of the objects in your simulation. If `vn` represents a
  /// reference velocity scale for the normal relative velocity between bodies,
  /// the new time scale `tn = δ / vn` represents the time it would take for the
  /// distance between two bodies approaching with relative normal velocity `vn`
  /// to decrease by the penetration_allowance δ. In this case a user should
  /// choose a time step for simulation that can resolve the smallest of the two
  /// time scales `tc` and `tn`.
  double get_contact_penalty_method_time_scale() const {
    DRAKE_MBP_THROW_IF_NOT_FINALIZED();
    return penalty_method_contact_parameters_.time_scale;
  }

  /// @anchor mbp_stribeck_model
  /// ###               Stribeck model of friction
  ///
  /// Currently %MultibodyPlant uses the Stribeck approximation to model dry
  /// friction. The Stribeck model of friction is an approximation to Coulomb's
  /// law of friction that allows using continuous time integration without the
  /// need to specify complementarity constraints. While this results in a
  /// simpler model immediately tractable with standard numerical methods for
  /// integration of ODE's, it often leads to stiff dynamics that require
  /// an explicit integrator to take very small time steps. It is therefore
  /// recommended to use error controlled integrators when using this model or
  /// the discrete time stepping (see @ref multibody_simulation).
  /// See @ref stribeck_approximation for a detailed discussion of the Stribeck
  /// model.
  ///
  /// Sets the stiction tolerance `v_stiction` for the Stribeck model, where
  /// `v_stiction` must be specified in m/s (meters per second.)
  /// `v_stiction` defaults to a value of 1 millimeter per second.
  /// In selecting a value for `v_stiction`, you must ask yourself the question,
  /// "When two objects are ostensibly in stiction, how much slip am I willing
  /// to allow?" There are two opposing design issues in picking a value for
  /// vₛ. On the one hand, small values of vₛ make the problem numerically
  /// stiff during stiction, potentially increasing the integration cost. On the
  /// other hand, it should be picked to be appropriate for the scale of the
  /// problem. For example, a car simulation could allow a "large" value for vₛ
  /// of 1 cm/s (1×10⁻² m/s), but reasonable stiction for grasping a 10 cm box
  /// might require limiting residual slip to 1×10⁻³ m/s or less. Ultimately,
  /// picking the largest viable value will allow your simulation to run
  /// faster and more robustly.
  /// Note that `v_stiction` is the slip velocity that we'd have when we are at
  /// edge of the friction cone. For cases when the friction force is well
  /// within the friction cone the slip velocity will always be smaller than
  /// this value.
  /// See also @ref stribeck_approximation.
  /// @throws std::exception if `v_stiction` is non-positive.
  void set_stiction_tolerance(
      double v_stiction = MultibodyPlantConfig{}.stiction_tolerance) {
    friction_model_.set_stiction_tolerance(v_stiction);
  }

  /// @returns the stiction tolerance parameter, in m/s.
  /// @see set_stiction_tolerance.
  double stiction_tolerance() const {
    return friction_model_.stiction_tolerance();
  }

  /// @} <!-- Contact modeling -->

  /// @anchor mbp_state_accessors_and_mutators
  /// @name               State accessors and mutators
  /// The following state methods allow getting and setting the kinematic state
  /// variables `[q; v]`, where `q` is the vector of generalized positions and
  /// `v` is the vector of generalized velocities. The state resides in a
  /// @ref systems::Context "Context" that is supplied
  /// as the first argument to every method.
  ///
  /// @note State vectors for the full system are returned as live references
  /// into the Context, not independent copies. In contrast, state vectors
  /// for individual model instances are returned as copies because the state
  /// associated with a model instance is generally not contiguous in a Context.
  ///
  /// There are also utilities for accessing and mutating portions of state
  /// or actuation arrays corresponding to just a single model instance.
  /// @{

  /// Returns a const vector reference `[q; v]` to the generalized positions q
  /// and generalized velocities v in a given Context.
  /// @note This method returns a reference to existing data, exhibits constant
  ///       i.e., O(1) time complexity, and runs very quickly.
  /// @throws std::exception if `context` does not correspond to the Context
  /// for a multibody model.
  Eigen::VectorBlock<const VectorX<T>> GetPositionsAndVelocities(
      const systems::Context<T>& context) const {
    this->ValidateContext(context);
    return internal_tree().get_positions_and_velocities(context);
  }

  /// Returns a vector `[q; v]` containing the generalized positions q and
  /// generalized velocities v of a specified model instance in a given Context.
  /// @note Returns a dense vector of dimension
  ///       `num_positions(model_instance) + num_velocities(model_instance)`
  ///       associated with `model_instance` by copying from `context`.
  /// @throws std::exception if `context` does not correspond to the Context
  /// for a multibody model or `model_instance` is invalid.
  VectorX<T> GetPositionsAndVelocities(
      const systems::Context<T>& context,
      ModelInstanceIndex model_instance) const {
    this->ValidateContext(context);
    return internal_tree().GetPositionsAndVelocities(context, model_instance);
  }

  /// (Advanced) Populates output vector qv_out representing the generalized
  /// positions q and generalized velocities v of a specified model instance
  /// in a given Context.
  /// @note qv_out is a dense vector of dimensions
  ///       `num_positions(model_instance) + num_velocities(model_instance)`
  ///       associated with `model_instance` and is populated by copying from
  ///       `context`.
  /// @note This function is guaranteed to allocate no heap.
  /// @throws std::exception if `context` does not correspond to the Context
  ///         for a multibody model or `model_instance` is invalid.
  /// @throws std::exception if qv_out does not have size
  ///         `num_positions(model_instance) + num_velocities(model_instance)`
  void GetPositionsAndVelocities(const systems::Context<T>& context,
                                 ModelInstanceIndex model_instance,
                                 EigenPtr<VectorX<T>> qv_out) const {
    this->ValidateContext(context);
    internal_tree().GetPositionsAndVelocities(context, model_instance, qv_out);
  }

  /// Sets generalized positions q and generalized velocities v in a given
  /// Context from a given vector [q; v]. Prefer this method over
  /// GetMutablePositionsAndVelocities().
  /// @throws std::exception if `context` is nullptr, if `context` does
  /// not correspond to the context for a multibody model, if the length of
  /// `q_v` is not equal to `num_positions() + num_velocities()`, or if `q_v`
  /// contains non-finite values.
  void SetPositionsAndVelocities(
      systems::Context<T>* context,
      const Eigen::Ref<const VectorX<T>>& q_v) const {
    this->ValidateContext(context);
    DRAKE_THROW_UNLESS(q_v.size() == (num_positions() + num_velocities()));
    DRAKE_THROW_UNLESS(AllFinite(q_v));
    internal_tree().GetMutablePositionsAndVelocities(context) = q_v;
  }

  /// Sets generalized positions q and generalized velocities v from a given
  /// vector [q; v] for a specified model instance in a given Context.
  /// @throws std::exception if `context` is nullptr, if `context` does
  /// not correspond to the Context for a multibody model, if the model instance
  /// index is invalid, if the length of `q_v` is not equal to
  /// `num_positions(model_instance) + num_velocities(model_instance)`, or if
  /// `q_v` contains non-finite values.
  void SetPositionsAndVelocities(
      systems::Context<T>* context, ModelInstanceIndex model_instance,
      const Eigen::Ref<const VectorX<T>>& q_v) const {
    this->ValidateContext(context);
    DRAKE_THROW_UNLESS(q_v.size() == (num_positions(model_instance) +
                                      num_velocities(model_instance)));
    DRAKE_THROW_UNLESS(AllFinite(q_v));
    internal_tree().SetPositionsAndVelocities(model_instance, q_v, context);
  }

  /// Returns a const vector reference to the vector of generalized positions
  /// q in a given Context.
  /// @note This method returns a reference to existing data, exhibits constant
  ///       i.e., O(1) time complexity, and runs very quickly.
  /// @throws std::exception if `context` does not correspond to the Context for
  /// a multibody model.
  Eigen::VectorBlock<const VectorX<T>> GetPositions(
      const systems::Context<T>& context) const {
    this->ValidateContext(context);
    return internal_tree().get_positions(context);
  }

  /// Returns a vector containing the generalized positions q of a specified
  /// model instance in a given Context.
  /// @note Returns a dense vector of dimension `num_positions(model_instance)`
  ///       associated with `model_instance` by copying from `context`.
  /// @throws std::exception if `context` does not correspond to the Context
  /// for a multibody model or `model_instance` is invalid.
  VectorX<T> GetPositions(const systems::Context<T>& context,
                          ModelInstanceIndex model_instance) const {
    this->ValidateContext(context);
    return internal_tree().GetPositionsFromArray(
        model_instance, internal_tree().get_positions(context));
  }

  /// (Advanced) Populates output vector q_out with the generalized positions q
  /// of a specified model instance in a given Context.
  /// @note q_out is a dense vector of dimension
  ///       `num_positions(model_instance)` associated with `model_instance`
  ///       and is populated by copying from `context`.
  /// @note This function is guaranteed to allocate no heap.
  /// @throws std::exception if `context` does not correspond to the Context
  ///         for a multibody model or `model_instance` is invalid.
  void GetPositions(const systems::Context<T>& context,
                    ModelInstanceIndex model_instance,
                    EigenPtr<VectorX<T>> q_out) const {
    this->ValidateContext(context);
    internal_tree().GetPositionsFromArray(
        model_instance, internal_tree().get_positions(context), q_out);
  }

  /// Sets the generalized positions q in a given Context from a given vector.
  /// Prefer this method over GetMutablePositions().
  /// @throws std::exception if `context` is nullptr, if `context` does not
  /// correspond to the Context for a multibody model, if the length of `q`
  /// is not equal to `num_positions()`, or if `q` contains non-finite values.
  void SetPositions(systems::Context<T>* context,
                    const Eigen::Ref<const VectorX<T>>& q) const {
    this->ValidateContext(context);
    DRAKE_THROW_UNLESS(q.size() == num_positions());
    DRAKE_THROW_UNLESS(AllFinite(q));
    internal_tree().GetMutablePositions(context) = q;
  }

  /// Sets the generalized positions q for a particular model instance in a
  /// given Context from a given vector.
  /// @throws std::exception if the `context` is nullptr, if `context` does
  /// not correspond to the Context for a multibody model, if the model instance
  /// index is invalid, if the length of `q_instance` is not equal to
  /// `num_positions(model_instance)`, or if `q_instance` contains non-finite
  /// values.
  void SetPositions(systems::Context<T>* context,
                    ModelInstanceIndex model_instance,
                    const Eigen::Ref<const VectorX<T>>& q_instance) const {
    this->ValidateContext(context);
    DRAKE_THROW_UNLESS(q_instance.size() == num_positions(model_instance));
    DRAKE_THROW_UNLESS(AllFinite(q_instance));
    Eigen::VectorBlock<VectorX<T>> q =
        internal_tree().GetMutablePositions(context);
    internal_tree().SetPositionsInArray(model_instance, q_instance, &q);
  }

  /// (Advanced) Sets the generalized positions q for a particular model
  /// instance in a given State from a given vector.
  /// @note No cache invalidation occurs.
  /// @throws std::exception if the `context` is nullptr, if `context` does
  /// not correspond to the Context for a multibody model, if the model instance
  /// index is invalid, if the length of `q_instance` is not equal to
  /// `num_positions(model_instance)`, or if `q_instance` contains non-finite
  /// values.
  /// @pre `state` comes from this MultibodyPlant.
  void SetPositions(const systems::Context<T>& context,
                    systems::State<T>* state, ModelInstanceIndex model_instance,
                    const Eigen::Ref<const VectorX<T>>& q_instance) const {
    this->ValidateContext(context);
    this->ValidateCreatedForThisSystem(state);
    DRAKE_THROW_UNLESS(q_instance.size() == num_positions(model_instance));
    DRAKE_THROW_UNLESS(AllFinite(q_instance));
    Eigen::VectorBlock<VectorX<T>> q =
        internal_tree().get_mutable_positions(state);
    internal_tree().SetPositionsInArray(model_instance, q_instance, &q);
  }

  /// Gets the default positions for the plant, which can be changed via
  /// SetDefaultPositions().
  /// @throws std::exception if the plant is not finalized.
  VectorX<T> GetDefaultPositions() const;

  /// Gets the default positions for the plant for a given model instance,
  /// which can be changed via SetDefaultPositions().
  /// @throws std::exception if the plant is not finalized, or if the
  /// model_instance is invalid,
  VectorX<T> GetDefaultPositions(ModelInstanceIndex model_instance) const;

  /// Sets the default positions for the plant.  Calls to CreateDefaultContext
  /// or SetDefaultContext/SetDefaultState will return a Context populated with
  /// these position values. They have no other effects on the dynamics of the
  /// system.
  /// @throws std::exception if the plant is not finalized, if q is not of size
  /// num_positions(), or `q` contains non-finite values.
  void SetDefaultPositions(const Eigen::Ref<const Eigen::VectorXd>& q);

  /// Sets the default positions for the model instance.  Calls to
  /// CreateDefaultContext or SetDefaultContext/SetDefaultState will return a
  /// Context populated with these position values. They have no other effects
  /// on the dynamics of the system.
  /// @throws std::exception if the plant is not
  /// finalized, if the model_instance is invalid, if the length of `q_instance`
  /// is not equal to `num_positions(model_instance)`, or if `q_instance`
  /// contains non-finite values.
  void SetDefaultPositions(ModelInstanceIndex model_instance,
                           const Eigen::Ref<const Eigen::VectorXd>& q_instance);

  /// Returns a const vector reference to the generalized velocities v in a
  /// given Context.
  /// @note This method returns a reference to existing data, exhibits constant
  ///       i.e., O(1) time complexity, and runs very quickly.
  /// @throws std::exception if `context` does not correspond to the Context
  /// for a multibody model.
  Eigen::VectorBlock<const VectorX<T>> GetVelocities(
      const systems::Context<T>& context) const {
    this->ValidateContext(context);
    return internal_tree().get_velocities(context);
  }

  /// Returns a vector containing the generalized velocities v of a specified
  /// model instance in a given Context.
  /// @note returns a dense vector of dimension `num_velocities(model_instance)`
  ///       associated with `model_instance` by copying from `context`.
  /// @throws std::exception if `context` does not correspond to the Context
  /// for a multibody model or `model_instance` is invalid.
  VectorX<T> GetVelocities(const systems::Context<T>& context,
                           ModelInstanceIndex model_instance) const {
    this->ValidateContext(context);
    return internal_tree().GetVelocitiesFromArray(
        model_instance, internal_tree().get_velocities(context));
  }

  /// (Advanced) Populates output vector v_out with the generalized
  /// velocities v of a specified model instance in a given Context.
  /// @note v_out is a dense vector of dimension
  ///       `num_velocities(model_instance)` associated with `model_instance`
  ///       and is populated by copying from `context`.
  /// @note This function is guaranteed to allocate no heap.
  /// @throws std::exception if `context` does not correspond to the Context
  ///         for a multibody model or `model_instance` is invalid.
  void GetVelocities(const systems::Context<T>& context,
                     ModelInstanceIndex model_instance,
                     EigenPtr<VectorX<T>> v_out) const {
    this->ValidateContext(context);
    internal_tree().GetVelocitiesFromArray(
        model_instance, internal_tree().get_velocities(context), v_out);
  }

  /// Sets the generalized velocities v in a given Context from a given
  /// vector. Prefer this method over GetMutableVelocities().
  /// @throws std::exception if the `context` is nullptr, if the context does
  /// not correspond to the context for a multibody model, if the length of
  /// `v` is not equal to `num_velocities()`, or if `v` contains non-finite
  /// values.
  void SetVelocities(systems::Context<T>* context,
                     const Eigen::Ref<const VectorX<T>>& v) const {
    this->ValidateContext(context);
    DRAKE_THROW_UNLESS(v.size() == num_velocities());
    DRAKE_THROW_UNLESS(AllFinite(v));
    internal_tree().GetMutableVelocities(context) = v;
  }

  /// Sets the generalized velocities v for a particular model instance in a
  /// given Context from a given vector.
  /// @throws std::exception if the `context` is nullptr, if `context` does
  /// not correspond to the Context for a multibody model, if the model instance
  /// index is invalid, if the length of `v_instance` is not equal to
  /// `num_velocities(model_instance)`, or if `v_instance` contains non-finite
  /// values.
  void SetVelocities(systems::Context<T>* context,
                     ModelInstanceIndex model_instance,
                     const Eigen::Ref<const VectorX<T>>& v_instance) const {
    this->ValidateContext(context);
    DRAKE_THROW_UNLESS(v_instance.size() == num_velocities(model_instance));
    DRAKE_THROW_UNLESS(AllFinite(v_instance));
    Eigen::VectorBlock<VectorX<T>> v =
        internal_tree().GetMutableVelocities(context);
    internal_tree().SetVelocitiesInArray(model_instance, v_instance, &v);
  }

  /// (Advanced) Sets the generalized velocities v for a particular model
  /// instance in a given State from a given vector.
  /// @note No cache invalidation occurs.
  /// @throws std::exception if the `context` is nullptr, if `context` does
  /// not correspond to the Context for a multibody model, if the model instance
  /// index is invalid, if the length of `v_instance` is not equal to
  /// `num_velocities(model_instance)`, or if `v_instance` contains non-finite
  /// values.
  /// @pre `state` comes from this MultibodyPlant.
  void SetVelocities(const systems::Context<T>& context,
                     systems::State<T>* state,
                     ModelInstanceIndex model_instance,
                     const Eigen::Ref<const VectorX<T>>& v_instance) const {
    this->ValidateContext(context);
    this->ValidateCreatedForThisSystem(state);
    DRAKE_THROW_UNLESS(v_instance.size() == num_velocities(model_instance));
    DRAKE_THROW_UNLESS(AllFinite(v_instance));
    Eigen::VectorBlock<VectorX<T>> v =
        internal_tree().get_mutable_velocities(state);
    internal_tree().SetVelocitiesInArray(model_instance, v_instance, &v);
  }

  /// Sets `state` according to defaults set by the user for joints (e.g.
  /// RevoluteJoint::set_default_angle()) and free bodies
  /// (SetDefaultFreeBodyPose()). If the user does not specify defaults, the
  /// state corresponds to zero generalized positions and velocities.
  /// @throws std::exception if called pre-finalize. See Finalize().
  void SetDefaultState(const systems::Context<T>& context,
                       systems::State<T>* state) const override {
    DRAKE_MBP_THROW_IF_NOT_FINALIZED();
    this->ValidateContext(context);
    this->ValidateCreatedForThisSystem(state);
    internal_tree().SetDefaultState(context, state);
    deformable_model().SetDefaultState(context, state);
  }

  /// Assigns random values to all elements of the state, by drawing samples
  /// independently for each joint/free body (coming soon: and then
  /// solving a mathematical program to "project" these samples onto the
  /// registered system constraints). If a random distribution is not specified
  /// for a joint/free body, the default state is used.
  ///
  /// @see @ref stochastic_systems
  void SetRandomState(const systems::Context<T>& context,
                      systems::State<T>* state,
                      RandomGenerator* generator) const override {
    DRAKE_MBP_THROW_IF_NOT_FINALIZED();
    this->ValidateContext(context);
    this->ValidateCreatedForThisSystem(state);
    internal_tree().SetRandomState(context, state, generator);
  }

  /// Returns a list of string names corresponding to each element of the
  /// position vector. These strings take the form
  /// `{model_instance_name}_{joint_name}_{joint_position_suffix}`, but the
  /// prefix and suffix may optionally be withheld using @p
  /// add_model_instance_prefix and @p always_add_suffix.
  ///
  /// @param always_add_suffix (optional). If true, then the suffix is always
  /// added. If false, then the suffix is only added for joints that have more
  /// than one position (in this case, not adding would lead to ambiguity).
  ///
  /// The returned names are guaranteed to be unique if @p
  /// add_model_instance_prefix is `true` (the default).
  ///
  /// @throws std::exception if the plant is not finalized.
  std::vector<std::string> GetPositionNames(
      bool add_model_instance_prefix = true,
      bool always_add_suffix = true) const;

  /// Returns a list of string names corresponding to each element of the
  /// position vector. These strings take the form
  /// `{model_instance_name}_{joint_name}_{joint_position_suffix}`, but the
  /// prefix and suffix may optionally be withheld using @p
  /// add_model_instance_prefix and @p always_add_suffix.
  ///
  /// @param always_add_suffix (optional). If true, then the suffix is always
  /// added. If false, then the suffix is only added for joints that have more
  /// than one position (in this case, not adding would lead to ambiguity).
  ///
  /// The returned names are guaranteed to be unique.
  ///
  /// @throws std::exception if the plant is not finalized or if the @p
  /// model_instance is invalid.
  std::vector<std::string> GetPositionNames(
      ModelInstanceIndex model_instance, bool add_model_instance_prefix = false,
      bool always_add_suffix = true) const;

  /// Returns a list of string names corresponding to each element of the
  /// velocity vector. These strings take the form
  /// `{model_instance_name}_{joint_name}_{joint_velocity_suffix}`, but the
  /// prefix and suffix may optionally be withheld using @p
  /// add_model_instance_prefix and @p always_add_suffix.
  ///
  /// @param always_add_suffix (optional). If true, then the suffix is always
  /// added. If false, then the suffix is only added for joints that have more
  /// than one position (in this case, not adding would lead to ambiguity).
  ///
  /// The returned names are guaranteed to be unique if @p
  /// add_model_instance_prefix is `true` (the default).
  ///
  /// @throws std::exception if the plant is not finalized.
  std::vector<std::string> GetVelocityNames(
      bool add_model_instance_prefix = true,
      bool always_add_suffix = true) const;

  /// Returns a list of string names corresponding to each element of the
  /// velocity vector. These strings take the form
  /// `{model_instance_name}_{joint_name}_{joint_velocity_suffix}`, but the
  /// prefix and suffix may optionally be withheld using @p
  /// add_model_instance_prefix and @p always_add_suffix.
  ///
  /// @param always_add_suffix (optional). If true, then the suffix is always
  /// added. If false, then the suffix is only added for joints that have more
  /// than one position (in this case, not adding would lead to ambiguity).
  ///
  /// The returned names are guaranteed to be unique.
  ///
  /// @throws std::exception if the plant is not finalized or if the
  /// @p model_instance is invalid.
  std::vector<std::string> GetVelocityNames(
      ModelInstanceIndex model_instance, bool add_model_instance_prefix = false,
      bool always_add_suffix = true) const;

  /// Returns a list of string names corresponding to each element of the
  /// multibody state vector. These strings take the form
  /// `{model_instance_name}_{joint_name}_{joint_position_suffix |
  /// joint_velocity_suffix}`, but the prefix may optionally be withheld using
  /// @p add_model_instance_prefix.
  ///
  /// The returned names are guaranteed to be unique if @p
  /// add_model_instance_prefix is `true` (the default).
  ///
  /// @throws std::exception if the plant is not finalized.
  std::vector<std::string> GetStateNames(
      bool add_model_instance_prefix = true) const;

  /// Returns a list of string names corresponding to each element of the
  /// multibody state vector. These strings take the form
  /// `{model_instance_name}_{joint_name}_{joint_position_suffix |
  /// joint_velocity_suffix}`, but the prefix may optionally be withheld using
  /// @p add_model_instance_prefix.
  ///
  /// The returned names are guaranteed to be unique.
  ///
  /// @throws std::exception if the plant is not finalized or if the @p
  /// model_instance is invalid.
  std::vector<std::string> GetStateNames(
      ModelInstanceIndex model_instance,
      bool add_model_instance_prefix = false) const;

  /// Returns a list of string names corresponding to each element of the
  /// actuation vector. These strings take the form
  /// `{model_instance_name}_{joint_actuator_name}`, but the prefix may
  /// optionally be withheld using @p add_model_instance_prefix.
  ///
  /// The returned names are guaranteed to be unique if @p
  /// add_model_instance_prefix is `true` (the default).
  ///
  /// @throws std::exception if the plant is not finalized.
  std::vector<std::string> GetActuatorNames(
      bool add_model_instance_prefix = true) const;

  /// Returns a list of string names corresponding to each element of the
  /// actuation vector. These strings take the form
  /// `{model_instance_name}_{joint_actuator_name}`, but the prefix may
  /// optionally be withheld using @p add_model_instance_prefix.
  ///
  /// The returned names are guaranteed to be unique.
  ///
  /// @throws std::exception if the plant is not finalized or if the
  /// @p model_instance is invalid.
  std::vector<std::string> GetActuatorNames(
      ModelInstanceIndex model_instance,
      bool add_model_instance_prefix = false) const;

  /// Returns a vector of actuation values for `model_instance` from a vector
  /// `u` of actuation values for the entire plant model. Refer to @ref
  /// mbp_actuation "Actuation" for further details.
  /// @param[in] u Actuation values for the entire model. The actuation value
  ///   in `u` for a particular actuator must be found at offset
  ///   JointActuator::input_start().
  /// @returns Actuation values for `model_instance`, ordered by monotonically
  ///   increasing @ref JointActuatorIndex.
  /// @throws std::exception if `u` is not of size
  ///   MultibodyPlant::num_actuated_dofs().
  VectorX<T> GetActuationFromArray(
      ModelInstanceIndex model_instance,
      const Eigen::Ref<const VectorX<T>>& u) const {
    return internal_tree().GetActuationFromArray(model_instance, u);
  }

  /// Given actuation values `u_instance` for the actuators in `model_instance`,
  /// this function updates the actuation vector u for the entire plant model to
  /// which this actuator belongs to. Refer to @ref mbp_actuation "Actuation"
  /// for further details.
  /// @param[in] u_instance Actuation values for the model instance. Values are
  ///   ordered by monotonically increasing @ref JointActuatorIndex within the
  ///   model instance.
  /// @param[in,out] u Actuation values for the entire plant model. The
  ///   actuation value in `u` for a particular actuator must be found at offset
  ///   JointActuator::input_start(). Only values corresponding to
  ///   `model_instance` are changed.
  /// @throws std::exception if the size of `u_instance` is not equal to the
  ///   number of actuation inputs for the joints of `model_instance`.
  void SetActuationInArray(ModelInstanceIndex model_instance,
                           const Eigen::Ref<const VectorX<T>>& u_instance,
                           EigenPtr<VectorX<T>> u) const {
    DRAKE_DEMAND(u != nullptr);
    internal_tree().SetActuationInArray(model_instance, u_instance, u);
  }

  /// Returns a vector of generalized positions for `model_instance` from a
  /// vector `q_array` of generalized positions for the entire model
  /// model.  This method throws an exception if `q` is not of size
  /// MultibodyPlant::num_positions().
  VectorX<T> GetPositionsFromArray(
      ModelInstanceIndex model_instance,
      const Eigen::Ref<const VectorX<T>>& q) const {
    return internal_tree().GetPositionsFromArray(model_instance, q);
  }

  /// (Advanced) Populates output vector q_out and with the generalized
  /// positions for `model_instance` from a vector `q` of generalized
  /// positions for the entire model.  This method throws an exception
  /// if `q` is not of size MultibodyPlant::num_positions().
  void GetPositionsFromArray(ModelInstanceIndex model_instance,
                             const Eigen::Ref<const VectorX<T>>& q,
                             EigenPtr<VectorX<T>> q_out) const {
    internal_tree().GetPositionsFromArray(model_instance, q, q_out);
  }

  /// Sets the vector of generalized positions for `model_instance` in
  /// `q` using `q_instance`, leaving all other elements in the array
  /// untouched. This method throws an exception if `q` is not of size
  /// MultibodyPlant::num_positions() or `q_instance` is not of size
  /// `MultibodyPlant::num_positions(model_instance)`.
  void SetPositionsInArray(ModelInstanceIndex model_instance,
                           const Eigen::Ref<const VectorX<T>>& q_instance,
                           EigenPtr<VectorX<T>> q) const {
    DRAKE_DEMAND(q != nullptr);
    internal_tree().SetPositionsInArray(model_instance, q_instance, q);
  }

  /// Returns a vector of generalized velocities for `model_instance` from a
  /// vector `v` of generalized velocities for the entire MultibodyPlant
  /// model.  This method throws an exception if the input array is not of size
  /// MultibodyPlant::num_velocities().
  VectorX<T> GetVelocitiesFromArray(
      ModelInstanceIndex model_instance,
      const Eigen::Ref<const VectorX<T>>& v) const {
    return internal_tree().GetVelocitiesFromArray(model_instance, v);
  }

  /// (Advanced) Populates output vector v_out with the generalized
  /// velocities for `model_instance` from a vector `v` of generalized
  /// velocities for the entire model.  This method throws an exception
  /// if `v` is not of size MultibodyPlant::num_velocities().
  void GetVelocitiesFromArray(ModelInstanceIndex model_instance,
                              const Eigen::Ref<const VectorX<T>>& v,
                              EigenPtr<VectorX<T>> v_out) const {
    internal_tree().GetVelocitiesFromArray(model_instance, v, v_out);
  }

  /// Sets the vector of generalized velocities for `model_instance` in
  /// `v` using `v_instance`, leaving all other elements in the array
  /// untouched. This method throws an exception if `v` is not of size
  /// MultibodyPlant::num_velocities(), `v_instance` is not of size
  /// `MultibodyPlant::num_positions(model_instance)`, or `v_instance` contains
  /// non-finite values.
  void SetVelocitiesInArray(ModelInstanceIndex model_instance,
                            const Eigen::Ref<const VectorX<T>>& v_instance,
                            EigenPtr<VectorX<T>> v) const {
    DRAKE_DEMAND(v != nullptr);
    DRAKE_THROW_UNLESS(AllFinite(v_instance));
    internal_tree().SetVelocitiesInArray(model_instance, v_instance, v);
  }
  /// @} <!-- State accessors and mutators -->

  /// @anchor mbp_working_with_free_bodies
  /// @name           Working with free and floating base bodies
  ///
  /// In robotics it is natural to think of some bodies as "floating", in the
  /// sense that they may be posed independently of any other body. Manipulands
  /// (objects to be manipulated by a robot) are the most obvious example since
  /// they aren't connected to anything else. A mobile base or humanoid torso
  /// can also be posed freely, though other bodies will move along with them.
  /// %MultibodyPlant recognizes floating bodies at Finalize() by the fact that
  /// they have no user-provided joint connecting them to any parent body. They
  /// are given six degrees of freedom relative to the World frame and referred
  /// to as _floating base bodies_.
  ///
  /// We use the term _free body_ for _any_ body that has six degrees of freedom
  /// relative to its parent. Floating base bodies are a special case of free
  /// bodies, with World as the assumed parent. The distinction is where the
  /// degrees of freedom come from: floating base bodies get theirs from
  /// Finalize(); all other free bodies get theirs from user-defined joints. The
  /// APIs below depend on that distinction. Those with "FreeBody" in their
  /// names work on all free bodies. Those with "FloatingBaseBody" only work on
  /// floating base bodies.
  ///
  /// To implement a floating base body at %Finalize(), %MultibodyPlant
  /// automatically adds a floating joint between that body's frame and the
  /// World frame. Prior to that there is no joint so we provide a
  /// pre-Finalize() API here to set the default pose of a floating base body in
  /// World. (See SetDefaultFloatingBaseBodyPose() below.) The default pose is
  /// used to initialize the floating joint's coordinates once that joint has
  /// been added. After %Finalize(), you can use the Joint API by accessing the
  /// automatically-added floating joint (see below), or continue to use the
  /// APIs in this group.
  ///
  /// Post-Finalize() there are a few additional APIs that apply only to
  /// floating base bodies. For example, you can query whether a RigidBody is a
  /// floating base body with RigidBody::is_floating_base_body(), and can
  /// request a list of all floating base bodies with GetFloatingBaseBodies().
  /// The relevant joint coordinate entries q and v in the multibody state
  /// vector can be obtained with RigidBody::floating_positions_start() and
  /// RigidBody::floating_velocities_start_in_v().
  ///
  /// If there is a user-provided joint mobilizing a free body, use the Joint
  /// API as you would for any other joint. You can also use the Joint API to
  /// work with floating base bodies after %Finalize(), by accessing the
  /// automatically-added floating joint. Use GetJointByName() with the name of
  /// the floating base body (see RigidBody::name()). (In the rare case that
  /// there is already some unrelated joint with that name, we prepend
  /// underscores to the joint's name until it is unique.)
  /// @{

  /// Returns the set of body indices corresponding to the floating base
  /// bodies in the model, in no particular order.
  /// See @ref mbp_working_with_free_bodies "above for details".
  /// @throws std::exception if called pre-finalize, see Finalize().
  std::unordered_set<BodyIndex> GetFloatingBaseBodies() const;

  /// Provisionally records a default World pose for `body`, to be used in case
  /// `body` turns out to be a floating base body after Finalize().
  ///
  /// This may be called pre- or post-Finalize(). Pre-Finalize() this is the
  /// only way to set the default pose of a floating base body. Post-Finalize(),
  /// a floating base body's default pose may be set either by this function or
  /// by setting the default pose directly through the Joint API applied to the
  /// automatically-added floating joint. The most recent value set by either
  /// method will be used to initialize the floating joint's coordinates in
  /// subsequently-created Contexts.
  ///
  /// @warning If this is called on a `body` that does _not_ turn out to be a
  /// floating base body after Finalize(), it will have no effect other than to
  /// be echoed back in GetDefaultFloatingBaseBodyPose(); in particular it will
  /// not affect the initial state in a subsequently-created Context. Use the
  /// Joint API to set the default pose for any body that has an
  /// explicitly-defined joint to its parent body.
  ///
  /// See @ref mbp_working_with_free_bodies "above for details".
  /// @param[in] body
  ///   RigidBody whose default pose will be set if it turns out to be a
  ///   floating base body.
  /// @param[in] X_WB
  ///   Default pose of the floating base body in the World frame.
  void SetDefaultFloatingBaseBodyPose(
      const RigidBody<T>& body, const math::RigidTransform<double>& X_WB) {
    this->mutable_tree().SetDefaultFloatingBaseBodyPose(body, X_WB);
  }

  /// Gets the provisional default pose of `body` as set by
  /// SetDefaultFloatingBaseBodyPose(). If no pose was specified for `body`,
  /// returns the identity pose. This may be called pre- or post-Finalize().
  ///
  /// @warning This value is only meaningful for bodies that turn out to be
  /// floating base bodies after Finalize(). If called on any other body, the
  /// result simply echoes whatever provisional pose was set in
  /// SetDefaultFloatingBaseBodyPose() but has no other effect. Use the Joint
  /// API to get the default pose for any body that has an explicitly-defined
  /// joint to its parent body.
  ///
  /// @note Post-Finalize(), a floating base body's default pose may be set
  /// either by SetDefaultFloatingBaseBodyPose() or by setting the default pose
  /// directly through the Joint API applied to the automatically-added floating
  /// joint. GetDefaultFloatingBaseBodyPose() will return the most-recent value
  /// set by either method.
  ///
  /// See @ref mbp_working_with_free_bodies "above for details".
  /// @param[in] body
  ///   RigidBody whose default pose will be retrieved.
  /// @retval X_WB The default pose of the floating base body B in World. Not
  ///   meaningful if `body` is not a floating base body.
  math::RigidTransform<double> GetDefaultFloatingBaseBodyPose(
      const RigidBody<T>& body) const {
    return internal_tree().GetDefaultFloatingBaseBodyPose(body);
  }

  /// Updates `context` to store the pose `X_WB` of a given floating base body
  /// B's body frame in the World frame W.
  ///
  /// See @ref mbp_working_with_free_bodies "above for details".
  /// @throws std::exception if called pre-finalize.
  /// @throws std::exception if `body` is not a floating base body.
  void SetFloatingBaseBodyPoseInWorldFrame(
      systems::Context<T>* context, const RigidBody<T>& body,
      const math::RigidTransform<T>& X_WB) const;

  /// Updates `context` to store the World-frame pose of floating base body B,
  /// given its pose `X_FB` in an arbitrary anchored frame F.
  ///
  /// Frame F must be _anchored_, meaning that it is either on a body which is
  /// directly welded to a frame on the World body, or more generally, that it
  /// is on a body for which there is a kinematic path between that body and the
  /// world body that only includes weld joints.
  ///
  /// @warning The World-frame pose is calculated here and stored in `context`.
  /// Moving F subsequently will not change the stored pose unless you call this
  /// method again.
  ///
  /// See @ref mbp_working_with_free_bodies "above for details".
  /// @throws std::exception if called pre-finalize.
  /// @throws std::exception if frame F is not anchored to the world.
  /// @throws std::exception if `body` is not a floating base body.
  void SetFloatingBaseBodyPoseInAnchoredFrame(
      systems::Context<T>* context, const Frame<T>& frame_F,
      const RigidBody<T>& body, const math::RigidTransform<T>& X_FB) const;

  /// If there is a single base body in the model given by `model_instance`,
  /// and that body is a floating base body, returns that floating base body.
  /// Otherwise, throws an exception. Use HasUniqueFloatingBaseBody() to check
  /// first.
  ///
  /// See @ref mbp_working_with_free_bodies "above for details".
  /// @throws std::exception if called pre-finalize.
  /// @throws std::exception if `model_instance` is not valid.
  /// @throws std::exception if !HasUniqueFloatingBaseBody(model_instance).
  /// @see HasUniqueFloatingBaseBody(), GetFloatingBaseBodies()
  const RigidBody<T>& GetUniqueFloatingBaseBodyOrThrow(
      ModelInstanceIndex model_instance) const {
    DRAKE_MBP_THROW_IF_NOT_FINALIZED();
    return internal_tree().GetUniqueFloatingBaseBodyOrThrowImpl(model_instance);
  }

  /// Returns true if there is a single base body in the model given by
  /// `model_instance`, and that body is a floating base body.
  ///
  /// See @ref mbp_working_with_free_bodies "above for details".
  /// @throws std::exception if called pre-finalize.
  /// @throws std::exception if `model_instance` is not valid.
  /// @see GetUniqueFloatingBaseBodyOrThrow()
  bool HasUniqueFloatingBaseBody(ModelInstanceIndex model_instance) const {
    DRAKE_MBP_THROW_IF_NOT_FINALIZED();
    return internal_tree().HasUniqueFloatingBaseBodyImpl(model_instance);
  }

  /// For any free body's 6-dof joint, gets the pose X_JpJc of the child frame
  /// Jc in its parent frame Jp.
  ///
  /// @note Unless `body` is a floating base body, the parent frame Jp is not
  /// necessarily the World frame W, and the child frame Jc is not necessarily
  /// the body frame B.
  ///
  /// See @ref mbp_working_with_free_bodies "above for details".
  /// @retval X_JpJc The current pose of child frame Jc in its parent frame Jp.
  ///                Returns X_WB if `body` B is a floating base body.
  /// @throws std::exception if called pre-finalize.
  /// @throws std::exception if `body` is not a free body.
  math::RigidTransform<T> GetFreeBodyPose(const systems::Context<T>& context,
                                          const RigidBody<T>& body) const {
    this->ValidateContext(context);
    return internal_tree().GetFreeBodyPoseOrThrow(context, body);
  }

  /// For any free body's 6-dof joint, sets `context` to store the pose
  /// X_JpJc of child frame Jc in its parent frame Jp.
  ///
  /// @note Unless `body` is a floating base body, the parent frame Jp is not
  /// necessarily the World frame W, and the child frame Jc is not necessarily
  /// the body frame B. For a floating base body B, this method sets X_WB, the
  /// pose of body B in World.
  ///
  /// See @ref mbp_working_with_free_bodies "above for details".
  /// @throws std::exception if called pre-finalize.
  /// @throws std::exception if `body` is not a free body.
  void SetFreeBodyPose(systems::Context<T>* context, const RigidBody<T>& body,
                       const math::RigidTransform<T>& X_JpJc) const {
    this->ValidateContext(context);
    internal_tree().SetFreeBodyPoseOrThrow(body, X_JpJc, context);
  }

  /// (Advanced) Variant of SetFreeBodyPose() that writes to a given `state`
  /// rather than directly to the Context.
  /// @pre `state` comes from this %MultibodyPlant.
  void SetFreeBodyPose(const systems::Context<T>& context,
                       systems::State<T>* state, const RigidBody<T>& body,
                       const math::RigidTransform<T>& X_JpJc) const {
    this->ValidateContext(context);
    this->ValidateCreatedForThisSystem(state);
    internal_tree().SetFreeBodyPoseOrThrow(body, X_JpJc, context, state);
  }

  /// For any free body's 6-dof joint, sets `context` to store the spatial
  /// velocity V_JpJc of child frame Jc in its parent frame Jp.
  ///
  /// @note Unless `body` is a floating base body, the parent frame Jp is not
  /// necessarily the World frame W, and the child frame Jc is not necessarily
  /// the body frame B. For a floating base body B, this method sets V_WB, the
  /// spatial velocity of body B in World.
  ///
  /// See @ref mbp_working_with_free_bodies "above for details".
  /// @throws std::exception if called pre-finalize.
  /// @throws std::exception if `body` is not a free body.
  void SetFreeBodySpatialVelocity(systems::Context<T>* context,
                                  const RigidBody<T>& body,
                                  const SpatialVelocity<T>& V_JpJc) const {
    this->ValidateContext(context);
    internal_tree().SetFreeBodySpatialVelocityOrThrow(body, V_JpJc, context);
  }

  /// (Advanced) Variant of SetFreeBodySpatialVelocity() that writes to a given
  /// `state` rather than directly to the Context.
  /// @pre `state` comes from this %MultibodyPlant.
  void SetFreeBodySpatialVelocity(const systems::Context<T>& context,
                                  systems::State<T>* state,
                                  const RigidBody<T>& body,
                                  const SpatialVelocity<T>& V_JpJc) const {
    this->ValidateContext(context);
    this->ValidateCreatedForThisSystem(state);
    internal_tree().SetFreeBodySpatialVelocityOrThrow(body, V_JpJc, context,
                                                      state);
  }

  /// For any free body's 6-dof joint, sets the distribution used by
  /// SetRandomState() to populate the x-y-z `translation` of its child frame Jc
  /// with respect to its parent frame Jp.
  ///
  /// @note Unless `body` is a floating base body, the parent frame Jp is not
  /// necessarily the World frame W, and the child frame Jc is not necessarily
  /// the body frame B. For a floating base body B, this method sets the
  /// distribution of p_WBo, the position of body B's frame origin Bo in World.
  ///
  /// See @ref mbp_working_with_free_bodies "above for details".
  /// @throws std::exception if called pre-finalize.
  /// @throws std::exception if `body` is not a free body.
  void SetFreeBodyRandomTranslationDistribution(
      const RigidBody<T>& body,
      const Vector3<symbolic::Expression>& translation) {
    this->mutable_tree().SetFreeBodyRandomTranslationDistributionOrThrow(
        body, translation);
  }

  /// For any free body's 6-dof joint, sets the distribution used by
  /// SetRandomState() to populate the orientation of its child frame Jc with
  /// respect to its parent frame Jp, expressed as a quaternion. Requires that
  /// the free body is modeled using a QuaternionFloatingJoint.
  ///
  /// @note Unless `body` is a floating base body, the parent frame Jp is not
  /// necessarily the World frame W, and the child frame Jc is not necessarily
  /// the body frame B. For a floating base body B, this method sets the
  /// distribution of R_WB, the orientation of body B's frame in World (as a
  /// quaternion).
  ///
  /// @note This distribution is not necessarily uniform over the sphere
  /// reachable by the quaternion; that depends on the quaternion expression
  /// provided in `rotation`. See
  /// SetFreeBodyRandomRotationDistributionToUniform() for a uniform
  /// alternative.
  ///
  /// See @ref mbp_working_with_free_bodies "above for details".
  /// @throws std::exception if called pre-finalize.
  /// @throws std::exception if `body` is not a free body.
  /// @throws std::exception if the free body is not modeled with a
  ///   QuaternionFloatingJoint.
  /// @see SetFreeBodyRandomAnglesDistribution() for a free body that is
  ///   modeled using an RpyFloatingJoint.
  /// @see SetBaseBodyJointType() for control over the type of automatically-
  ///   added joints.
  void SetFreeBodyRandomRotationDistribution(
      const RigidBody<T>& body,
      const Eigen::Quaternion<symbolic::Expression>& rotation) {
    this->mutable_tree().SetFreeBodyRandomRotationDistributionOrThrow(body,
                                                                      rotation);
  }

  /// For any free body's 6-dof joint, sets the distribution used by
  /// SetRandomState() to populate the orientation of its child frame Jc with
  /// respect to its parent frame Jp using uniformly random rotations (expressed
  /// as a quaternion). Requires that the free body is modeled using a
  /// QuaternionFloatingJoint.
  ///
  /// @note Unless `body` is a floating base body, the parent frame Jp is not
  /// necessarily the World frame W, and the child frame Jc is not necessarily
  /// the body frame B. For a floating base body B, this method sets the
  /// distribution of R_WB, the orientation of body B's frame in World (as a
  /// quaternion).
  ///
  /// See @ref mbp_working_with_free_bodies "above for details".
  /// @throws std::exception if called pre-finalize.
  /// @throws std::exception if `body` is not a free body in the model.
  /// @throws std::exception if the free body is not modeled with a
  ///   QuaternionFloatingJoint.
  /// @see SetFreeBodyRandomAnglesDistribution() for a free body that is
  ///   modeled using an RpyFloatingJoint.
  /// @see SetBaseBodyJointType() for control over the type of automatically-
  ///   added joints.
  void SetFreeBodyRandomRotationDistributionToUniform(const RigidBody<T>& body);

  /// For any free body's 6-dof joint, sets the distribution used by
  /// SetRandomState() to populate the orientation of its child frame Jc with
  /// respect to its parent frame Jp, expressed with roll-pitch-yaw angles.
  /// Requires that the free body is modeled using an RpyFloatingJoint.
  ///
  /// @note Unless `body` is a floating base body, the parent frame Jp is not
  /// necessarily the World frame W, and the child frame Jc is not necessarily
  /// the body frame B. For a floating base body B, this method sets the
  /// distribution of R_WB, the orientation of body B's frame in World (as
  /// roll-pitch-yaw angles).
  ///
  /// @note This distribution is not uniform over the sphere reachable by the
  /// three angles. For a uniform alternative, switch the joint to
  /// QuaternionFloatingJoint and use
  /// SetFreeBodyRandomRotationDistributionToUniform().
  ///
  /// See @ref mbp_working_with_free_bodies "above for details".
  /// @throws std::exception if called pre-finalize.
  /// @throws std::exception if `body` is not a free body in the model.
  /// @throws std::exception if the free body is not modeled with an
  ///   RpyFloatingJoint.
  /// @see SetFreeBodyRandomRotationDistribution() for a free body that is
  ///   modeled using a QuaternionFloatingJoint.
  /// @see SetBaseBodyJointType() for control over the type of automatically-
  ///   added joints.
  void SetFreeBodyRandomAnglesDistribution(
      const RigidBody<T>& body,
      const math::RollPitchYaw<symbolic::Expression>& angles) {
    this->mutable_tree().SetFreeBodyRandomAnglesDistributionOrThrow(body,
                                                                    angles);
  }

  DRAKE_DEPRECATED("2026-06-01",
                   "Use SetFloatingBaseBodyPoseInWorldFrame() instead.")
  void SetFreeBodyPoseInWorldFrame(systems::Context<T>* context,
                                   const RigidBody<T>& body,
                                   const math::RigidTransform<T>& X_WB) const {
    SetFloatingBaseBodyPoseInWorldFrame(context, body, X_WB);
  }

  DRAKE_DEPRECATED("2026-06-01",
                   "Use SetFloatingBaseBodyPoseInAnchoredFrame() instead.")
  void SetFreeBodyPoseInAnchoredFrame(
      systems::Context<T>* context, const Frame<T>& frame_F,
      const RigidBody<T>& body, const math::RigidTransform<T>& X_FB) const {
    SetFloatingBaseBodyPoseInAnchoredFrame(context, frame_F, body, X_FB);
  }

  DRAKE_DEPRECATED("2026-06-01",
                   "Use GetUniqueFloatingBaseBodyOrThrow() instead.")
  const RigidBody<T>& GetUniqueFreeBaseBodyOrThrow(
      ModelInstanceIndex model_instance) const {
    return GetUniqueFloatingBaseBodyOrThrow(model_instance);
  }

  DRAKE_DEPRECATED("2026-06-01",
                   "Use SetDefaultFloatingBaseBodyPose() instead.")
  void SetDefaultFreeBodyPose(const RigidBody<T>& body,
                              const math::RigidTransform<double>& X_PB) {
    SetDefaultFloatingBaseBodyPose(body, X_PB);
  }

  DRAKE_DEPRECATED("2026-06-01",
                   "Use GetDefaultFloatingBaseBodyPose() instead.")
  math::RigidTransform<double> GetDefaultFreeBodyPose(
      const RigidBody<T>& body) const {
    return GetDefaultFloatingBaseBodyPose(body);
  }

  DRAKE_DEPRECATED("2026-06-01", "Use HasUniqueFloatingBaseBody() instead.")
  bool HasUniqueFreeBaseBody(ModelInstanceIndex model_instance) const {
    DRAKE_MBP_THROW_IF_NOT_FINALIZED();
    return internal_tree().HasUniqueFloatingBaseBodyImpl(model_instance);
  }

  /// @} <!-- Working with free bodies -->

  /// @anchor mbp_kinematic_and_dynamic_computations
  /// @name             Kinematic and dynamic computations
  /// These methods return kinematic results for the state supplied in the given
  /// @ref systems::Context "Context". Methods whose names being with `Eval`
  /// return a reference
  /// into the Context's cache, performing computation first only if the
  /// relevant state has changed. Methods beginning with `Calc` perform
  /// computation unconditionally and return a result without updating the
  /// cache.
  /// @{

  /// Evaluate the pose `X_WB` of a body B in the world frame W.
  /// @param[in] context
  ///   The context storing the state of the model.
  /// @param[in] body_B
  ///   The body B for which the pose is requested.
  /// @retval X_WB
  ///   The pose of body frame B in the world frame W.
  /// @throws std::exception if Finalize() was not called on `this` model or
  ///   if `body_B` does not belong to this model.
  const math::RigidTransform<T>& EvalBodyPoseInWorld(
      const systems::Context<T>& context, const RigidBody<T>& body_B) const {
    this->ValidateContext(context);
    return internal_tree().EvalBodyPoseInWorld(context, body_B);
  }

  /// Evaluates V_WB, body B's spatial velocity in the world frame W.
  /// @param[in] context The context storing the state of the model.
  /// @param[in] body_B  The body B for which the spatial velocity is requested.
  /// @retval V_WB_W RigidBody B's spatial velocity in the world frame W,
  ///   expressed in W (for point Bo, the body's origin).
  /// @throws std::exception if Finalize() was not called on `this` model or
  ///   if `body_B` does not belong to this model.
  const SpatialVelocity<T>& EvalBodySpatialVelocityInWorld(
      const systems::Context<T>& context, const RigidBody<T>& body_B) const {
    this->ValidateContext(context);
    return internal_tree().EvalBodySpatialVelocityInWorld(context, body_B);
  }

  /// Evaluates A_WB, body B's spatial acceleration in the world frame W.
  /// @param[in] context The context storing the state of the model.
  /// @param[in] body_B  The body for which spatial acceleration is requested.
  /// @retval A_WB_W RigidBody B's spatial acceleration in the world frame W,
  ///   expressed in W (for point Bo, the body's origin).
  /// @throws std::exception if Finalize() was not called on `this` model or
  ///   if `body_B` does not belong to this model.
  /// @note When cached values are out of sync with the state stored in context,
  /// this method performs an expensive forward dynamics computation, whereas
  /// once evaluated, successive calls to this method are inexpensive.
  const SpatialAcceleration<T>& EvalBodySpatialAccelerationInWorld(
      const systems::Context<T>& context, const RigidBody<T>& body_B) const;

  /// Calculates the rigid transform (pose) `X_AB` relating frame A and frame B.
  /// @param[in] context
  ///    The state of the multibody system, which includes the system's
  ///    generalized positions q.  Note: `X_AB` is a function of q.
  /// @param[in] frame_A
  ///    The frame A designated in the rigid transform `X_AB`.
  /// @param[in] frame_B
  ///    The frame G designated in the rigid transform `X_AB`.
  /// @retval X_AB
  ///    The RigidTransform relating frame A and frame B.
  math::RigidTransform<T> CalcRelativeTransform(
      const systems::Context<T>& context, const Frame<T>& frame_A,
      const Frame<T>& frame_B) const {
    this->ValidateContext(context);
    return internal_tree().CalcRelativeTransform(context, frame_A, frame_B);
  }

  /// Calculates the rotation matrix `R_AB` relating frame A and frame B.
  /// @param[in] context
  ///    The state of the multibody system, which includes the system's
  ///    generalized positions q.  Note: `R_AB` is a function of q.
  /// @param[in] frame_A
  ///    The frame A designated in the rigid transform `R_AB`.
  /// @param[in] frame_B
  ///    The frame G designated in the rigid transform `R_AB`.
  /// @retval R_AB
  ///    The RigidTransform relating frame A and frame B.
  math::RotationMatrix<T> CalcRelativeRotationMatrix(
      const systems::Context<T>& context, const Frame<T>& frame_A,
      const Frame<T>& frame_B) const {
    this->ValidateContext(context);
    return internal_tree().CalcRelativeRotationMatrix(context, frame_A,
                                                      frame_B);
  }

  /// Given the positions `p_BQi` for a set of points `Qi` measured and
  /// expressed in a frame B, this method computes the positions `p_AQi(q)` of
  /// each point `Qi` in the set as measured and expressed in another frame A,
  /// as a function of the generalized positions q of the model.
  ///
  /// Example of usage: Given two points Q0 and Q1 that are fixed to a frame B,
  /// the code below calculates their positions from the world frame origin,
  /// expressed in the world frame W.
  ///
  /// @code
  ///  constexpr int num_position_vectors = 2;
  ///  MatrixX<double> p_BQi(3, num_position_vectors);
  ///  p_BQi.col(0) = Vector3<double>(1.1, 2.2, 3.3);
  ///  p_BQi.col(1) = Vector3<double>(-9.8, 7.6, -5.43);
  ///  MatrixX<double> p_WQi(3, num_position_vectors);
  ///  const Frame<double>& frame_W = plant.world_frame();
  ///  plant.CalcPointsPositions(*context_, frame_B, p_BQi, frame_W, &p_WQi);
  /// @endcode
  ///
  /// @param[in] context
  ///   The context containing the state of the model. It stores the
  ///   generalized positions q of the model.
  /// @param[in] frame_B
  ///   The frame B in which the positions `p_BQi` of a set of points `Qi` are
  ///   given.
  /// @param[in] p_BQi
  ///   The input positions of each point `Qi` in frame B. `p_BQi ∈ ℝ³ˣⁿ` with
  ///   `n` the number of points in the set. Each column of `p_BQi` corresponds
  ///   to a vector in ℝ³ holding the position of one of the points in the set
  ///   as measured and expressed in frame B. Each column of p_BQi is a position
  ///   vector associated with one point Qi, and the number of columns in p_BQi
  ///   is the number n of points.
  /// @param[in] frame_A
  ///   The frame A in which it is desired to compute the positions `p_AQi` of
  ///   each point `Qi` in the set.
  /// @param[out] p_AQi
  ///   The output positions of each point `Qi` now computed as measured and
  ///   expressed in frame A. The output `p_AQi` **must** have the same size as
  ///   the input `p_BQi` or otherwise this method aborts. That is `p_AQi`
  ///   **must** be in `ℝ³ˣⁿ`. Each column of p_AQi is a position vector
  ///   associated with one point Qi, and the number of columns in p_BQi is the
  ///   number n of points.
  ///
  /// @note Both `p_BQi` and `p_AQi` must have three rows. Otherwise this
  /// method will throw a std::exception. This method also throws a
  /// std::exception if `p_BQi` and `p_AQi` differ in the number of columns.
  void CalcPointsPositions(const systems::Context<T>& context,
                           const Frame<T>& frame_B,
                           const Eigen::Ref<const MatrixX<T>>& p_BQi,
                           const Frame<T>& frame_A,
                           EigenPtr<MatrixX<T>> p_AQi) const {
    this->ValidateContext(context);
    DRAKE_DEMAND(p_AQi != nullptr);
    return internal_tree().CalcPointsPositions(context, frame_B, p_BQi, frame_A,
                                               p_AQi);
  }

  /// For a set of n points Qi (i = 0, ... n-1) that are regarded as fixed on a
  /// frame B, calculates the velocities v_AQi_E of Qi measured in a frame A and
  /// expressed in a frame E.
  ///
  /// Example of usage: Given two points Q0 and Q1 that are fixed to a frame B,
  /// the code below calculates their velocities measured and expressed in the
  /// world frame W.
  ///
  /// @code
  ///  constexpr int num_position_vectors = 2;
  ///  MatrixX<double> p_BQi(3, num_position_vectors);
  ///  p_BQi.col(0) = Vector3<double>(1.1, 2.2, 3.3);
  ///  p_BQi.col(1) = Vector3<double>(-9.8, 7.6, -5.43);
  ///  MatrixX<double> v_WQi_W(3, num_position_vectors);
  ///  const Frame<double>& frame_W = plant.world_frame();
  ///  plant.CalcPointsVelocities(*context_, frame_B, p_BQi, frame_W, frame_W,
  ///                             &v_WQi_W);
  /// @endcode
  ///
  /// @param[in] context Contains the state of the multibody system, including
  /// the generalized positions q and the generalized velocities v.
  /// @param[in] frame_B The frame B in which each point Qi is fixed and whose
  /// frame origin Bo is the starting point for position vectors in p_BQi.
  /// frame_B is also the expressed-in-frame for position vectors p_BQi.
  /// @param[in] p_BQi Position vectors from Bo (frame B's origin) to each
  /// point Qi (i = 0, ... n-1), expressed in frame B. Each column of p_BQi
  /// is a position vector associated with one point Qi, and the number of
  /// columns in p_BQi is the number n of points.
  /// @param[in] frame_A The frame in which the velocities are to be measured.
  /// @param[in] frame_E The frame in which the velocities are to be expressed.
  /// @param[out] v_AQi_E The velocities of each point Qi (i = 0, ... n-1)
  /// measured in frame A and expressed in frame E. Each column of v_AQi_E is a
  /// translational velocity vector associated with one point Qi, and the
  /// number of columns in v_AQi_E is the number n of points.
  /// @throws std::exception if p_BQi and v_AQi_E do not have three rows (are
  /// not 3 element vectors) or do not have the same number (n > 0) of columns.
  void CalcPointsVelocities(const systems::Context<T>& context,
                            const Frame<T>& frame_B,
                            const Eigen::Ref<const MatrixX<T>>& p_BQi,
                            const Frame<T>& frame_A, const Frame<T>& frame_E,
                            EigenPtr<MatrixX<T>> v_AQi_E) const {
    this->ValidateContext(context);
    return internal_tree().CalcPointsVelocities(context, frame_B, p_BQi,
                                                frame_A, frame_E, v_AQi_E);
  }

  /// Calculates the total mass of all bodies in this MultibodyPlant.
  /// @param[in] context Contains the state of the model.
  /// @retval The total mass of all bodies or 0 if there are none.
  /// @note The mass of the world_body() does not contribute to the total mass.
  T CalcTotalMass(const systems::Context<T>& context) const {
    this->ValidateContext(context);
    return internal_tree().CalcTotalMass(context);
  }

  /// Calculates the total mass of all bodies contained in model_instances.
  /// @param[in] context Contains the state of the model.
  /// @param[in] model_instances Vector of selected model instances. This method
  /// does not distinguish between welded, joint connected, or floating bodies.
  /// @retval The total mass of all bodies belonging to a model instance in
  ///   model_instances or 0 if model_instances is empty.
  /// @note The mass of the world_body() does not contribute to the total mass
  ///   and each body only contributes to the total mass once, even if the body
  ///   has repeated occurrence (instance) in model_instances.
  T CalcTotalMass(
      const systems::Context<T>& context,
      const std::vector<ModelInstanceIndex>& model_instances) const {
    this->ValidateContext(context);
    return internal_tree().CalcTotalMass(context, model_instances);
  }

  /// Calculates the position vector from the world origin Wo to the center of
  /// mass of all bodies in this MultibodyPlant, expressed in the world frame W.
  /// @param[in] context Contains the state of the model.
  /// @retval p_WoScm_W position vector from Wo to Scm expressed in world frame
  /// W, where Scm is the center of mass of the system S stored by `this` plant.
  /// @throws std::exception if `this` has no body except world_body().
  /// @throws std::exception if mₛ ≤ 0 (where mₛ is the mass of system S).
  /// @note The world_body() is ignored.  p_WoScm_W = ∑ (mᵢ pᵢ) / mₛ, where
  /// mₛ = ∑ mᵢ, mᵢ is the mass of the iᵗʰ body, and pᵢ is Bᵢcm's position from
  /// Wo expressed in frame W (Bᵢcm is the center of mass of the iᵗʰ body).
  Vector3<T> CalcCenterOfMassPositionInWorld(
      const systems::Context<T>& context) const {
    this->ValidateContext(context);
    return internal_tree().CalcCenterOfMassPositionInWorld(context);
  }

  /// Calculates the position vector from the world origin Wo to the center of
  /// mass of all non-world bodies contained in model_instances, expressed in
  /// the world frame W.
  /// @param[in] context Contains the state of the model.
  /// @param[in] model_instances Vector of selected model instances.  If a model
  /// instance is repeated in the vector (unusual), it is only counted once.
  /// @retval p_WoScm_W position vector from world origin Wo to Scm expressed in
  /// the world frame W, where Scm is the center of mass of the system S of
  /// non-world bodies contained in model_instances.
  /// @throws std::exception if model_instances is empty or only has world body.
  /// @throws std::exception if mₛ ≤ 0 (where mₛ is the mass of system S).
  /// @note The world_body() is ignored.  p_WoScm_W = ∑ (mᵢ pᵢ) / mₛ, where
  /// mₛ = ∑ mᵢ, mᵢ is the mass of the iᵗʰ body contained in model_instances,
  /// and pᵢ is Bᵢcm's position vector from Wo expressed in frame W
  /// (Bᵢcm is the center of mass of the iᵗʰ body).
  Vector3<T> CalcCenterOfMassPositionInWorld(
      const systems::Context<T>& context,
      const std::vector<ModelInstanceIndex>& model_instances) const {
    this->ValidateContext(context);
    return internal_tree().CalcCenterOfMassPositionInWorld(context,
                                                           model_instances);
  }

  /// Returns M_SFo_F, the spatial inertia of a set S of bodies about point Fo
  /// (the origin of a frame F), expressed in frame F. You may regard M_SFo_F as
  /// measuring spatial inertia as if the set S of bodies were welded into a
  /// single composite body at the configuration specified in the `context`.
  /// @param[in] context Contains the configuration of the set S of bodies.
  /// @param[in] frame_F specifies the about-point Fo (frame_F's origin) and
  ///  the expressed-in frame for the returned spatial inertia.
  /// @param[in] body_indexes Array of selected bodies.  This method does not
  ///  distinguish between welded bodies, joint-connected bodies, etc.
  /// @throws std::exception if body_indexes contains an invalid BodyIndex or
  ///  if there is a repeated BodyIndex.
  /// @note The mass and inertia of the world_body() does not contribute to the
  ///  the returned spatial inertia.
  SpatialInertia<T> CalcSpatialInertia(
      const systems::Context<T>& context, const Frame<T>& frame_F,
      const std::vector<BodyIndex>& body_indexes) const {
    this->ValidateContext(context);
    return internal_tree().CalcSpatialInertia(context, frame_F, body_indexes);
  }

  /// Calculates system center of mass translational velocity in world frame W.
  /// @param[in] context The context contains the state of the model.
  /// @retval v_WScm_W Scm's translational velocity in frame W, expressed in W,
  /// where Scm is the center of mass of the system S stored by `this` plant.
  /// @throws std::exception if `this` has no body except world_body().
  /// @throws std::exception if mₛ ≤ 0 (where mₛ is the mass of system S).
  /// @note The world_body() is ignored.  v_WScm_W = ∑ (mᵢ vᵢ) / mₛ, where
  /// mₛ = ∑ mᵢ, mᵢ is the mass of the iᵗʰ body, and vᵢ is Bᵢcm's velocity in
  /// world W (Bᵢcm is the center of mass of the iᵗʰ body).
  Vector3<T> CalcCenterOfMassTranslationalVelocityInWorld(
      const systems::Context<T>& context) const {
    DRAKE_MBP_THROW_IF_NOT_FINALIZED();
    this->ValidateContext(context);
    return internal_tree().CalcCenterOfMassTranslationalVelocityInWorld(
        context);
  }

  /// For the system S contained in this %MultibodyPlant, calculates Scm's
  /// translational acceleration in the world frame W expressed in W, where
  /// Scm is the center of mass of S.
  /// @param[in] context The context contains the state of the model.
  /// @retval a_WScm_W Scm's translational acceleration in the world frame W
  /// expressed in the world frame W.
  /// @throws std::exception if `this` has no body except world_body().
  /// @throws std::exception if mₛ ≤ 0, where mₛ is the mass of system S.
  /// @note The world_body() is ignored.  a_WScm_W = ∑ (mᵢ aᵢ) / mₛ, where
  /// mₛ = ∑ mᵢ is the mass of system S, mᵢ is the mass of the iᵗʰ body, and
  /// aᵢ is the translational acceleration of Bᵢcm in world W expressed in W
  /// (Bᵢcm is the center of mass of the iᵗʰ body).
  /// @note When cached values are out of sync with the state stored in context,
  /// this method performs an expensive forward dynamics computation, whereas
  /// once evaluated, successive calls to this method are inexpensive.
  Vector3<T> CalcCenterOfMassTranslationalAccelerationInWorld(
      const systems::Context<T>& context) const {
    DRAKE_MBP_THROW_IF_NOT_FINALIZED();
    this->ValidateContext(context);
    return internal_tree().CalcCenterOfMassTranslationalAccelerationInWorld(
        context);
  }

  /// For the system S containing the selected model instances, calculates
  /// Scm's translational acceleration in the world frame W expressed in W,
  /// where Scm is the center of mass of S.
  /// @param[in] context The context contains the state of the model.
  /// @param[in] model_instances Vector of selected model instances.  If a model
  /// instance is repeated in the vector (unusual), it is only counted once.
  /// @retval a_WScm_W Scm's translational acceleration in the world frame W
  /// expressed in the world frame W.
  /// @throws std::exception if model_instances is empty or only has world body.
  /// @throws std::exception if mₛ ≤ 0, where mₛ is the mass of system S.
  /// @note The world_body() is ignored.  a_WScm_W = ∑ (mᵢ aᵢ) / mₛ, where
  /// mₛ = ∑ mᵢ is the mass of system S, mᵢ is the mass of the iᵗʰ body in
  /// model_instances, and aᵢ is the translational acceleration of Bᵢcm in
  /// world W expressed in W (Bᵢcm is the center of mass of the iᵗʰ body).
  /// @note When cached values are out of sync with the state stored in context,
  /// this method performs an expensive forward dynamics computation, whereas
  /// once evaluated, successive calls to this method are inexpensive.
  Vector3<T> CalcCenterOfMassTranslationalAccelerationInWorld(
      const systems::Context<T>& context,
      const std::vector<ModelInstanceIndex>& model_instances) const {
    DRAKE_MBP_THROW_IF_NOT_FINALIZED();
    this->ValidateContext(context);
    return internal_tree().CalcCenterOfMassTranslationalAccelerationInWorld(
        context, model_instances);
  }

  /// Calculates system center of mass translational velocity in world frame W.
  /// @param[in] context The context contains the state of the model.
  /// @param[in] model_instances Vector of selected model instances.  If a model
  /// instance is repeated in the vector (unusual), it is only counted once.
  /// @retval v_WScm_W Scm's translational velocity in frame W, expressed in W,
  /// where Scm is the center of mass of the system S of non-world bodies
  /// contained in model_instances.
  /// @throws std::exception if model_instances is empty or only has world body.
  /// @throws std::exception if mₛ ≤ 0 (where mₛ is the mass of system S).
  /// @note The world_body() is ignored.  v_WScm_W = ∑ (mᵢ vᵢ) / mₛ, where
  /// mₛ = ∑ mᵢ, mᵢ is the mass of the iᵗʰ body contained in model_instances,
  /// and vᵢ is Bᵢcm's velocity in world W expressed in frame W
  /// (Bᵢcm is the center of mass of the iᵗʰ body).
  Vector3<T> CalcCenterOfMassTranslationalVelocityInWorld(
      const systems::Context<T>& context,
      const std::vector<ModelInstanceIndex>& model_instances) const {
    DRAKE_MBP_THROW_IF_NOT_FINALIZED();
    this->ValidateContext(context);
    return internal_tree().CalcCenterOfMassTranslationalVelocityInWorld(
        context, model_instances);
  }

  /// This method returns the spatial momentum of `this` MultibodyPlant in the
  /// world frame W, about a designated point P, expressed in the world frame W.
  /// @param[in] context Contains the state of the model.
  /// @param[in] p_WoP_W Position from Wo (origin of the world frame W) to an
  ///            arbitrary point P, expressed in the world frame W.
  /// @retval L_WSP_W, spatial momentum of the system S represented by `this`
  ///   plant, measured in the world frame W, about point P, expressed in W.
  /// @note To calculate the spatial momentum of this system S in W about Scm
  /// (the system's center of mass), use something like: <pre>
  ///   MultibodyPlant<T> plant;
  ///   // ... code to load a model ....
  ///   const Vector3<T> p_WoScm_W =
  ///     plant.CalcCenterOfMassPositionInWorld(context);
  ///   const SpatialMomentum<T> L_WScm_W =
  ///     plant.CalcSpatialMomentumInWorldAboutPoint(context, p_WoScm_W);
  /// </pre>
  SpatialMomentum<T> CalcSpatialMomentumInWorldAboutPoint(
      const systems::Context<T>& context, const Vector3<T>& p_WoP_W) const {
    this->ValidateContext(context);
    return internal_tree().CalcSpatialMomentumInWorldAboutPoint(context,
                                                                p_WoP_W);
  }

  /// This method returns the spatial momentum of a set of model instances in
  /// the world frame W, about a designated point P, expressed in frame W.
  /// @param[in] context Contains the state of the model.
  /// @param[in] model_instances Set of selected model instances.
  /// @param[in] p_WoP_W Position from Wo (origin of the world frame W) to an
  ///            arbitrary point P, expressed in the world frame W.
  /// @retval L_WSP_W, spatial momentum of the system S represented by the
  /// model_instances, measured in world frame W, about point P, expressed in W.
  /// @note To calculate the spatial momentum of this system S in W about Scm
  /// (the system's center of mass), use something like: <pre>
  ///   MultibodyPlant<T> plant;
  ///   // ... code to create a set of selected model instances, e.g., ...
  ///   const ModelInstanceIndex gripper_model_instance =
  ///     plant.GetModelInstanceByName("gripper");
  ///   const ModelInstanceIndex robot_model_instance =
  ///     plant.GetBodyByName("end_effector").model_instance();
  ///   const std::vector<ModelInstanceIndex> model_instances{
  ///     gripper_model_instance, robot_model_instance};
  ///   const Vector3<T> p_WoScm_W =
  ///     plant.CalcCenterOfMassPositionInWorld(context, model_instances);
  ///   SpatialMomentum<T> L_WScm_W =
  ///     plant.CalcSpatialMomentumInWorldAboutPoint(context, model_instances,
  ///                                                p_WoScm_W);
  /// </pre>
  /// @throws std::exception if model_instances contains an invalid
  ///         ModelInstanceIndex.
  SpatialMomentum<T> CalcSpatialMomentumInWorldAboutPoint(
      const systems::Context<T>& context,
      const std::vector<ModelInstanceIndex>& model_instances,
      const Vector3<T>& p_WoP_W) const {
    this->ValidateContext(context);
    return internal_tree().CalcSpatialMomentumInWorldAboutPoint(
        context, model_instances, p_WoP_W);
  }

  /// Given the state of this model in `context` and a known vector
  /// of generalized accelerations `known_vdot`, this method computes the
  /// spatial acceleration `A_WB` for each body as measured and expressed in the
  /// world frame W.
  ///
  /// @param[in] context
  ///   The context containing the state of this model.
  /// @param[in] known_vdot
  ///   A vector with the generalized accelerations for the full model.
  /// @param[out] A_WB_array
  ///   A pointer to a valid, non nullptr, vector of spatial accelerations
  ///   containing the spatial acceleration `A_WB` for each body. It must be of
  ///   size equal to the number of bodies in the model. On output,
  ///   entries will be ordered by BodyIndex.
  /// @throws std::exception if A_WB_array is not of size `num_bodies()`.
  void CalcSpatialAccelerationsFromVdot(
      const systems::Context<T>& context, const VectorX<T>& known_vdot,
      std::vector<SpatialAcceleration<T>>* A_WB_array) const;

  /// Given the state of this model in `context` and a known vector
  /// of generalized accelerations `vdot`, this method computes the
  /// set of generalized forces `tau` that would need to be applied in order to
  /// attain the specified generalized accelerations.
  /// Mathematically, this method computes: <pre>
  ///   tau = M(q)v̇ + C(q, v)v - tau_app - ∑ J_WBᵀ(q) Fapp_Bo_W
  /// </pre>
  /// where `M(q)` is the model's mass matrix (including rigid body mass
  /// properties and @ref reflected_inertia "reflected inertias"), `C(q, v)v` is
  /// the bias term for Coriolis and gyroscopic effects and `tau_app` consists
  /// of a vector applied generalized forces. The last term is a summation over
  /// all bodies in the model where `Fapp_Bo_W` is an applied spatial force on
  /// body B at `Bo` which gets projected into the space of generalized forces
  /// with the transpose of `Jv_V_WB(q)` (where `Jv_V_WB` is B's spatial
  /// velocity Jacobian in W with respect to generalized velocities v).
  /// Note: B's spatial velocity in W can be written as `V_WB = Jv_V_WB * v`.
  ///
  /// This method does not compute explicit expressions for the mass matrix nor
  /// for the bias term, which would be of at least `O(n²)` complexity, but it
  /// implements an `O(n)` Newton-Euler recursive algorithm, where n is the
  /// number of bodies in the model. The explicit formation of the
  /// mass matrix `M(q)` would require the calculation of `O(n²)` entries while
  /// explicitly forming the product `C(q, v) * v` could require up to `O(n³)`
  /// operations (see [Featherstone 1987, §4]), depending on the implementation.
  /// The recursive Newton-Euler algorithm is the most efficient currently known
  /// general method for solving inverse dynamics [Featherstone 2008].
  ///
  /// @param[in] context
  ///   The context containing the state of the model.
  /// @param[in] known_vdot
  ///   A vector with the known generalized accelerations `vdot` for the full
  ///   model. Use the provided Joint APIs in order to access entries into this
  ///   array.
  /// @param[in] external_forces
  ///   A set of forces to be applied to the system either as body spatial
  ///   forces `Fapp_Bo_W` or generalized forces `tau_app`, see MultibodyForces
  ///   for details.
  ///
  /// @returns the vector of generalized forces that would need to be applied to
  /// the mechanical system in order to achieve the desired acceleration given
  /// by `known_vdot`.
  VectorX<T> CalcInverseDynamics(
      const systems::Context<T>& context, const VectorX<T>& known_vdot,
      const MultibodyForces<T>& external_forces) const {
    this->ValidateContext(context);
    return internal_tree().CalcInverseDynamics(context, known_vdot,
                                               external_forces);
  }

#ifdef DRAKE_DOXYGEN_CXX
  // MultibodyPlant uses the NVI implementation of
  // CalcImplicitTimeDerivativesResidual from
  // MultibodyTreeSystem::DoCalcImplicitTimeDerivativesResidual.  We provide the
  // public facing documentation for it here.

  /// MultibodyPlant implements the
  /// systems::System::CalcImplicitTimeDerivativesResidual method when the plant
  /// is modeled as a continuous-time system, returning one residual for each
  /// multibody state.  In particular, the first num_positions() residuals are
  /// given by <pre>
  ///   q̇_proposed - N(q)⋅v
  /// </pre>
  /// and the final num_velocities() residuals are given by <pre>
  ///   CalcInverseDynamics(context, v_proposed)
  /// </pre>
  /// including all actuator and applied forces.
  /// @see systems::System::CalcImplicitTimeDerivativesResidual for more
  /// details.
  void CalcImplicitTimeDerivativesResidual(
      const systems::Context<T>& context,
      const systems::ContinuousState<T>& proposed_derivatives,
      EigenPtr<VectorX<T>> residual) const;
#endif

  /// Computes the combined force contribution of ForceElement objects in the
  /// model. A ForceElement can apply forces as a spatial force per body or as
  /// generalized forces, depending on the ForceElement model.
  /// ForceElement contributions are a function of the state and time only.
  /// The output from this method can immediately be used as input to
  /// CalcInverseDynamics() to include the effect of applied forces by force
  /// elements.
  ///
  /// @param[in] context
  ///   The context containing the state of this model.
  /// @param[out] forces
  ///   A pointer to a valid, non nullptr, multibody forces object. On output
  ///   `forces` will store the forces exerted by all the ForceElement
  ///   objects in the model.
  /// @throws std::exception if `forces` is null or not compatible with this
  ///   model, per MultibodyForces::CheckInvariants().
  void CalcForceElementsContribution(const systems::Context<T>& context,
                                     MultibodyForces<T>* forces) const;

  /// Computes the generalized forces `tau_g(q)` due to gravity as a function
  /// of the generalized positions `q` stored in the input `context`.
  /// The vector of generalized forces due to gravity `tau_g(q)` is defined such
  /// that it appears on the right hand side of the equations of motion together
  /// with any other generalized forces, like so:
  /// <pre>
  ///   Mv̇ + C(q, v)v = tau_g(q) + tau_app
  /// </pre>
  /// where `tau_app` includes any other generalized forces applied on the
  /// system.
  ///
  /// @param[in] context
  ///   The context storing the state of the model.
  /// @returns tau_g
  ///   A vector containing the generalized forces due to gravity.
  ///   The generalized forces are consistent with the vector of
  ///   generalized velocities `v` for `this` so that
  ///   the inner product `v⋅tau_g` corresponds to the power applied by the
  ///   gravity forces on the mechanical system. That is, `v⋅tau_g > 0`
  ///   corresponds to potential energy going into the system, as either
  ///   mechanical kinetic energy, some other potential energy, or heat, and
  ///   therefore to a decrease of the gravitational potential energy.
  VectorX<T> CalcGravityGeneralizedForces(
      const systems::Context<T>& context) const {
    this->ValidateContext(context);
    return internal_tree().CalcGravityGeneralizedForces(context);
  }

  /// Computes the generalized forces result of a set of MultibodyForces applied
  /// to this model.
  ///
  /// MultibodyForces stores applied forces as both generalized forces τ and
  /// spatial forces F on each body, refer to documentation in MultibodyForces
  /// for details. Users of MultibodyForces will use
  /// MultibodyForces::mutable_generalized_forces() to mutate the stored
  /// generalized forces directly and will use RigidBody::AddInForceInWorld() to
  /// append spatial forces.
  ///
  /// For a given set of forces stored as MultibodyForces, this method will
  /// compute the total generalized forces on this model. More precisely, if
  /// J_WBo is the Jacobian (with respect to velocities) for this model,
  /// including all bodies, then this method computes: <pre>
  ///   τᵣₑₛᵤₗₜ = τ + J_WBo⋅F
  /// </pre>
  ///
  /// @param[in] context Context that stores the state of the model.
  /// @param[in] forces Set of multibody forces, including both generalized
  /// forces and per-body spatial forces.
  /// @param[out] generalized_forces The total generalized forces on the model
  /// that would result from applying `forces`. In other words, `forces` can be
  /// replaced by the equivalent `generalized_forces`. On output,
  /// `generalized_forces` is resized to num_velocities().
  ///
  /// @throws std::exception if `forces` is null or not compatible with this
  /// model.
  /// @throws std::exception if `generalized_forces` is not a valid non-null
  /// pointer.
  void CalcGeneralizedForces(const systems::Context<T>& context,
                             const MultibodyForces<T>& forces,
                             VectorX<T>* generalized_forces) const;

  /// Returns true iff the generalized velocity v is exactly the time
  /// derivative q̇ of the generalized coordinates q. In this case
  /// MapQDotToVelocity() and MapVelocityToQDot() implement the identity map.
  /// This method is, in the worst case, O(n), where n is the number of joints.
  bool IsVelocityEqualToQDot() const {
    // TODO(sherm1): Consider caching this value.
    return internal_tree().IsVelocityEqualToQDot();
  }

  // Preserve access to base overload from this class.
  using systems::System<T>::MapVelocityToQDot;

  /// Transforms generalized velocities v to time derivatives `qdot` of the
  /// generalized positions vector `q` (stored in `context`). `v` and `qdot`
  /// are related linearly by `q̇ = N(q)⋅v`.
  /// Using the configuration `q` stored in the given `context` this method
  /// calculates `q̇ = N(q)⋅v`.
  ///
  /// @param[in] context
  ///   The context containing the state of the model.
  /// @param[in] v
  ///   A vector of generalized velocities for this model.
  ///   This method aborts if v is not of size num_velocities().
  /// @param[out] qdot
  ///   A valid (non-null) pointer to a vector in `ℝⁿ` with n being the number
  ///   of generalized positions in this model,
  ///   given by `num_positions()`. This method aborts if `qdot` is nullptr
  ///   or if it is not of size num_positions().
  ///
  /// @see MapQDotToVelocity()
  /// @see Mobilizer::MapVelocityToQDot()
  void MapVelocityToQDot(const systems::Context<T>& context,
                         const Eigen::Ref<const VectorX<T>>& v,
                         EigenPtr<VectorX<T>> qdot) const {
    this->ValidateContext(context);
    DRAKE_DEMAND(qdot != nullptr);
    return internal_tree().MapVelocityToQDot(context, v, qdot);
  }

  // Preserve access to base overload from this class.
  using systems::System<T>::MapQDotToVelocity;

  /// Transforms the time derivative `qdot` of the generalized positions vector
  /// `q` (stored in `context`) to generalized velocities `v`. `v` and `q̇`
  /// are related linearly by `q̇ = N(q)⋅v`. Although `N(q)` is not
  /// necessarily square, its left pseudo-inverse `N⁺(q)` can be used to
  /// invert that relationship without residual error, provided that `qdot` is
  /// in the range space of `N(q)` (that is, if it *could* have been produced as
  /// `q̇ = N(q)⋅v` for some `v`).
  /// Using the configuration `q` stored in the given `context` this method
  /// calculates `v = N⁺(q)⋅q̇`.
  ///
  /// @param[in] context
  ///   The context containing the state of the model.
  /// @param[in] qdot
  ///   A vector containing the time derivatives of the generalized positions.
  ///   This method aborts if `qdot` is not of size num_positions().
  /// @param[out] v
  ///   A valid (non-null) pointer to a vector in `ℛⁿ` with n the number of
  ///   generalized velocities. This method aborts if v is nullptr or if it
  ///   is not of size num_velocities().
  ///
  /// @see MapVelocityToQDot()
  /// @see Mobilizer::MapQDotToVelocity()
  void MapQDotToVelocity(const systems::Context<T>& context,
                         const Eigen::Ref<const VectorX<T>>& qdot,
                         EigenPtr<VectorX<T>> v) const {
    this->ValidateContext(context);
    DRAKE_DEMAND(v != nullptr);
    internal_tree().MapQDotToVelocity(context, qdot, v);
  }

  /// Returns the matrix `N(q)`, which maps `q̇ = N(q)⋅v`, as described in
  /// MapVelocityToQDot(). Prefer calling MapVelocityToQDot() directly; this
  /// entry point is provided to support callers that require the explicit
  /// linear form (once q is given) of the relationship. Do not take the
  /// (pseudo-)inverse of `N(q)`; call MakeQDotToVelocityMap instead. This
  /// method is, in the worst case, O(n), where n is the number of joints.
  ///
  /// @param[in] context
  ///   The context containing the state of the model.
  ///
  /// @see MapVelocityToQDot()
  Eigen::SparseMatrix<T> MakeVelocityToQDotMap(
      const systems::Context<T>& context) const {
    this->ValidateContext(context);
    return internal_tree().MakeVelocityToQDotMap(context);
  }

  /// Returns the matrix `N⁺(q)`, which maps `v = N⁺(q)⋅q̇`, as described in
  /// MapQDotToVelocity(). Prefer calling MapQDotToVelocity() directly; this
  /// entry point is provided to support callers that require the explicit
  /// linear form (once q is given) of the relationship. This method is, in the
  /// worst case, O(n), where n is the number of joints.
  ///
  /// @param[in] context
  ///   The context containing the state of the model.
  ///
  /// @see MapVelocityToQDot()
  Eigen::SparseMatrix<T> MakeQDotToVelocityMap(
      const systems::Context<T>& context) const {
    this->ValidateContext(context);
    return internal_tree().MakeQDotToVelocityMap(context);
  }
  /// @} <!-- Kinematic and dynamic computations -->

  /// @anchor mbp_system_matrix_computations
  /// @name                System matrix computations
  /// Methods in this section compute and return various matrices that
  /// appear in the system equations of motion. For better performance, prefer
  /// to use direct computations where available rather than work with explicit
  /// matrices. See
  /// @ref mbp_kinematic_and_dynamic_computations
  /// "Kinematic and dynamics computations" for available computations. For
  /// example, you can obtain the mass matrix, Coriolis, centripetal, and
  /// gyroscopic "bias" terms, and a variety of Jacobian and actuation matrices.
  /// @{

  /// Computes the mass matrix `M(q)` of the model using a slow method (inverse
  /// dynamics). The generalized positions q are taken from the given `context`.
  /// M includes the mass properties of rigid bodies and @ref reflected_inertia
  /// "reflected inertias" as provided with JointActuator specifications.
  ///
  /// Use CalcMassMatrix() for a faster implementation using the Composite %Body
  /// Algorithm.
  ///
  /// @param[in] context
  ///   The Context containing the state of the model from which generalized
  ///   coordinates q are extracted.
  /// @param[out] M
  ///   A pointer to a square matrix in `ℛⁿˣⁿ` with n the number of generalized
  ///   velocities (num_velocities()) of the model. Although symmetric, the
  ///   matrix is filled in completely on return.
  ///
  /// @pre M is non-null and has the right size.
  ///
  /// The algorithm used to build `M(q)` consists in computing one column of
  /// `M(q)` at a time using inverse dynamics. The result from inverse dynamics,
  /// with no applied forces, is the vector of generalized forces: <pre>
  ///   tau = M(q)v̇ + C(q, v)v
  /// </pre>
  /// where q and v are the generalized positions and velocities, respectively.
  /// When `v = 0` the Coriolis and gyroscopic forces term `C(q, v)v` is zero.
  /// Therefore the `i-th` column of `M(q)` can be obtained performing inverse
  /// dynamics with an acceleration vector `v̇ = eᵢ`, with `eᵢ` the standard
  /// (or natural) basis of `ℛⁿ` with n the number of generalized velocities.
  /// We write this as: <pre>
  ///   M.ᵢ(q) = M(q) * e_i
  /// </pre>
  /// where `M.ᵢ(q)` (notice the dot for the rows index) denotes the `i-th`
  /// column in M(q).
  ///
  /// @warning This is an O(n²) algorithm. Avoid the explicit computation of the
  /// mass matrix whenever possible.
  /// @see CalcMassMatrix(), CalcInverseDynamics()
  void CalcMassMatrixViaInverseDynamics(const systems::Context<T>& context,
                                        EigenPtr<MatrixX<T>> M) const {
    this->ValidateContext(context);
    DRAKE_DEMAND(M != nullptr);
    internal_tree().CalcMassMatrixViaInverseDynamics(context, M);
  }

  /// Efficiently computes the mass matrix `M(q)` of the model. The generalized
  /// positions q are taken from the given `context`. M includes the mass
  /// properties of rigid bodies and @ref reflected_inertia "reflected inertias"
  /// as provided with JointActuator specifications.
  ///
  /// This method employs the Composite %Body Algorithm, which we believe to be
  /// the fastest O(n²) algorithm to compute the mass matrix of a multibody
  /// system.
  ///
  /// @param[in] context
  ///   The Context containing the state of the model from which generalized
  ///   coordinates q are extracted.
  /// @param[out] M
  ///   A pointer to a square matrix in `ℛⁿˣⁿ` with n the number of generalized
  ///   velocities (num_velocities()) of the model. Although symmetric, the
  ///   matrix is filled in completely on return.
  ///
  /// @pre M is non-null and has the right size.
  ///
  /// @warning This is an O(n²) algorithm. Avoid the explicit computation of the
  /// mass matrix whenever possible.
  /// @see CalcMassMatrixViaInverseDynamics() (slower)
  void CalcMassMatrix(const systems::Context<T>& context,
                      EigenPtr<MatrixX<T>> M) const {
    this->ValidateContext(context);
    DRAKE_DEMAND(M != nullptr);
    internal_tree().CalcMassMatrix(context, M);
  }

  /// This method allows users to map the state of `this` model, x, into a
  /// vector of selected state xₛ with a given preferred ordering.
  /// The mapping, or selection, is returned in the form of a selector matrix
  /// Sx such that `xₛ = Sx⋅x`. The size nₛ of xₛ is always smaller or equal
  /// than the size of the full state x. That is, a user might be interested in
  /// only a given portion of the full state x.
  ///
  /// This selection matrix is particularly useful when adding PID control
  /// on a portion of the state, see systems::controllers::PidController.
  ///
  /// A user specifies the preferred order in xₛ via `user_to_joint_index_map`.
  /// The selected state is built such that selected positions are followed
  /// by selected velocities, as in `xₛ = [qₛ, vₛ]`.
  /// The positions in qₛ are a concatenation of the positions for each joint
  /// in the order they appear in `user_to_joint_index_map`. That is, the
  /// positions for `user_to_joint_index_map[0]` are first, followed by the
  /// positions for `user_to_joint_index_map[1]`, etc. Similarly for the
  /// selected velocities vₛ.
  ///
  /// @throws std::exception if there are repeated indices in
  /// `user_to_joint_index_map`.
  MatrixX<double> MakeStateSelectorMatrix(
      const std::vector<JointIndex>& user_to_joint_index_map) const {
    // TODO(amcastro-tri): consider having an extra `free_body_index_map`
    // so that users could also re-order free bodies if they wanted to.
    return internal_tree().MakeStateSelectorMatrix(user_to_joint_index_map);
  }

  /// This method allows user to map a vector `uₛ` containing the actuation
  /// for a set of selected actuators into the vector u containing the actuation
  /// values for `this` full model.
  /// The mapping, or selection, is returned in the form of a selector matrix
  /// Su such that `u = Su⋅uₛ`. The size nₛ of uₛ is always smaller or equal
  /// than the size of the full vector of actuation values u. That is, a user
  /// might be interested in only a given subset of actuators in the model.
  ///
  /// This selection matrix is particularly useful when adding PID control
  /// on a portion of the state, see systems::controllers::PidController.
  ///
  /// A user specifies the preferred order in uₛ via
  /// `user_to_actuator_index_map`. The actuation values in uₛ are a
  /// concatenation of the values for each actuator in the order they appear in
  /// `user_to_actuator_index_map`. The actuation value in the full vector of
  /// actuation values `u` for a particular actuator can be found at offset
  /// JointActuator::input_start().
  MatrixX<double> MakeActuatorSelectorMatrix(
      const std::vector<JointActuatorIndex>& user_to_actuator_index_map) const {
    return internal_tree().MakeActuatorSelectorMatrix(
        user_to_actuator_index_map);
  }

  /// This method creates an actuation matrix B mapping a vector of actuation
  /// values u into generalized forces `tau_u = B * u`, where B is a matrix of
  /// size `nv x nu` with `nu` equal to num_actuated_dofs() and `nv` equal to
  /// num_velocities().
  /// The vector u of actuation values is of size num_actuated_dofs(). For a
  /// given JointActuator, `u[JointActuator::input_start()]` stores the value
  /// for the external actuation corresponding to that actuator. `tau_u` on the
  /// other hand is indexed by generalized velocity indices according to
  /// `Joint::velocity_start()`.
  /// @warning B is a permutation matrix. While making a permutation has
  /// `O(n)` complexity, making a full B matrix has `O(n²)` complexity. For most
  /// applications this cost can be neglected but it could become significant
  /// for very large systems.
  MatrixX<T> MakeActuationMatrix() const;

  /// Creates the pseudoinverse of the actuation matrix B directly (without
  /// requiring an explicit inverse calculation). See MakeActuationMatrix().
  ///
  /// Notably, when B is full row rank (the system is fully actuated), then the
  /// pseudoinverse is a true inverse.
  Eigen::SparseMatrix<double> MakeActuationMatrixPseudoinverse() const;

  /// Alternative signature to build an actuation selector matrix `Su` such
  /// that `u = Su⋅uₛ`, where u is the vector of actuation values for the full
  /// model (see get_actuation_input_port()) and uₛ is a vector of actuation
  /// values for the actuators acting on the joints listed by
  /// `user_to_joint_index_map`. It is assumed that all joints referenced by
  /// `user_to_joint_index_map` are actuated. See
  /// MakeActuatorSelectorMatrix(const std::vector<JointActuatorIndex>&) for
  /// details.
  /// @throws std::exception if any of the joints in
  /// `user_to_joint_index_map` does not have an actuator.
  MatrixX<double> MakeActuatorSelectorMatrix(
      const std::vector<JointIndex>& user_to_joint_index_map) const {
    return internal_tree().MakeActuatorSelectorMatrix(user_to_joint_index_map);
  }
  /// @} <!-- System matrix computations -->

  /// @anchor Jacobian_functions
  /// @name Jacobian functions
  /// Herein, a Jacobian is a matrix that contains the partial derivatives of a
  /// vector with respect to a list of scalars. The vector may be a position
  /// vector, translational velocity, angular velocity, or spatial velocity and
  /// the scalars may be the system's generalized positions q or "speeds" 𝑠
  /// where 𝑠 is either q̇ (time-derivative of generalized positions) or
  /// v (generalized velocities).
  ///
  /// JAq_p_PQ denotes the Jacobian in a frame A of the position vector from
  /// point P to point Q with respect to the generalized positions q. It is
  /// calculated with CalcJacobianPositionVector().
  ///
  /// J𝑠_w_AB denotes the angular velocity Jacobian in a frame A of a frame B
  /// with respect to "speeds" 𝑠. It is calculated with
  /// CalcJacobianAngularVelocity().
  ///
  /// J𝑠_V_ABp denotes the spatial velocity Jacobian in a frame A of a point Bp
  /// of frame B with respect to "speeds" 𝑠. It is calculated with
  /// CalcJacobianSpatialVelocity().
  ///
  /// J𝑠_v_ABp denotes the translational velocity Jacobian in a frame A of a
  /// point Bp of frame B with respect to "speeds" 𝑠. It is calculated with
  /// CalcJacobianTranslationalVelocity().
  ///
  /// J𝑠_v_AScm_E denotes the translational velocity Jacobian in a frame A of a
  /// point Scm with respect to "speeds" 𝑠, where point Scm is the center of
  /// mass of a system S. It is calculated with
  /// CalcJacobianCenterOfMassTranslationalVelocity()
  ///@{

  /// (Internal use only) Returns the System Jacobian Jv_V_WB(q) in block form.
  /// Each block is dense and corresponds to one Tree of the as-built
  /// internal::SpanningForest. The blocks follow the Tree ordering defined by
  /// the SpanningForest, so are in TreeIndex order. The block for Treeᵢ is a
  /// MatrixX of size 6nᵢ x mᵢ, where nᵢ is the number of mobilized bodies in
  /// Treeᵢ and mᵢ is the total number of mobilizer velocity degrees of freedom
  /// (mobilities) in the Tree. Every Tree has an entry even if it has no
  /// mobilities (in that case mᵢ=0). World is not part of any Tree so there
  /// is no block corresponding to World here.
  ///
  /// To be precise: the iᵗʰ block Jvi_V_WB ≡ ∂Vi_WB/∂vᵢ where Vi_WB is the
  /// stacked spatial velocities for each mobilized body in Treeᵢ (in order of
  /// MobodIndex), and vᵢ is the vector of generalized velocities associated
  /// with those mobilized bodies, in the same order. Thus Jvi_V_WB⋅v for some
  /// set of mᵢ generalized velocities v, returns the spatial velocities for
  /// each body in Treeᵢ that would result from velocities v.
  ///
  /// Note that locking and unlocking mobilizers does not affect the Jacobian;
  /// the Jacobian reflects what would happen if a velocity variable changed
  /// regardless of whether it can currently do so.
  /// @see CalcJacobianSpatialVelocity(), CalcFullSystemJacobian()
  const std::vector<Eigen::MatrixX<T>>& EvalBlockSystemJacobian(
      const systems::Context<T>& context) const {
    const internal::BlockSystemJacobianCache<T>& sjc =
        this->EvalBlockSystemJacobianCache(context);
    return sjc.block_system_jacobian();
  }

  /// (Internal use only) Evaluates the block system Jacobian, then uses it to
  /// fill in an equivalent full matrix of size 6n x m where n is the number of
  /// mobilized bodies and m the number of generalized velocities (mobilities).
  /// Each mobilized body generates a 6 x m strip of the Jacobian (6 rows) and
  /// those are ordered by MobodIndex. Note that World is the 0th mobilized body
  /// so to keep the numbering consistent the first 6 rows of the Jacobian
  /// correspond to the World Mobod (and are thus all zero).
  ///
  /// This is most useful for testing; it is more efficient to use
  /// EvalBlockSystemJacobian() and to work with the individual blocks.
  /// @see EvalBlockSystemJacobian(), CalcJacobianSpatialVelocity()
  Eigen::MatrixX<T> CalcFullSystemJacobian(
      const systems::Context<T>& context) const {
    const internal::BlockSystemJacobianCache<T>& sjc =
        this->EvalBlockSystemJacobianCache(context);
    return sjc.ToFullMatrix();
  }

  /// For one point Bp fixed/welded to a frame B, calculates J𝑠_V_ABp, Bp's
  /// spatial velocity Jacobian in frame A with respect to "speeds" 𝑠.
  /// <pre>
  ///      J𝑠_V_ABp ≜ [ ∂(V_ABp)/∂𝑠₁,  ...  ∂(V_ABp)/∂𝑠ₙ ]    (n is j or k)
  ///      V_ABp = J𝑠_V_ABp ⋅ 𝑠          V_ABp is linear in 𝑠 ≜ [𝑠₁ ... 𝑠ₙ]ᵀ
  /// </pre>
  /// `V_ABp` is Bp's spatial velocity in frame A and "speeds" 𝑠 is either
  /// q̇ ≜ [q̇₁ ... q̇ⱼ]ᵀ (time-derivatives of j generalized positions) or
  /// v ≜ [v₁ ... vₖ]ᵀ (k generalized velocities).
  ///
  /// @param[in] context The state of the multibody system.
  /// @param[in] with_respect_to Enum equal to JacobianWrtVariable::kQDot or
  /// JacobianWrtVariable::kV, indicating whether the Jacobian `J𝑠_V_ABp` is
  /// partial derivatives with respect to 𝑠 = q̇ (time-derivatives of generalized
  /// positions) or with respect to 𝑠 = v (generalized velocities).
  /// @param[in] frame_B The frame on which point Bp is fixed/welded.
  /// @param[in] p_BoBp_B A position vector from Bo (frame_B's origin) to point
  /// Bp (regarded as fixed/welded to B), expressed in frame_B.
  /// @param[in] frame_A The frame that measures `v_ABp` (Bp's velocity in A).
  /// Note: It is natural to wonder why there is no parameter p_AoAp_A (similar
  /// to the parameter p_BoBp_B for frame_B).  There is no need for p_AoAp_A
  /// because Bp's velocity in A is defined as the derivative in frame A of
  /// Bp's position vector from _any_ point fixed to A.
  /// @param[in] frame_E The frame in which `v_ABp` is expressed on input and
  /// the frame in which the Jacobian `J𝑠_V_ABp` is expressed on output.
  /// @param[out] Js_V_ABp_E Point Bp's spatial velocity Jacobian in frame A
  /// with respect to speeds 𝑠 (which is either q̇ or v), expressed in frame E.
  /// `J𝑠_V_ABp_E` is a `6 x n` matrix, where n is the number of elements in 𝑠.
  /// The Jacobian is a function of only generalized positions q (which are
  /// pulled from the context).
  /// @note The returned `6 x n` matrix stores frame B's angular velocity
  /// Jacobian in A in rows 1-3 and stores point Bp's translational velocity
  /// Jacobian in A in rows 4-6, i.e., <pre>
  ///     J𝑠_w_AB_E = J𝑠_V_ABp_E.topRows<3>();
  ///     J𝑠_v_ABp_E = J𝑠_V_ABp_E.bottomRows<3>();
  /// </pre>
  /// @note Consider CalcJacobianTranslationalVelocity() for multiple points
  /// fixed to frame B and consider CalcJacobianAngularVelocity() to calculate
  /// frame B's angular velocity Jacobian.
  /// @see See @ref Jacobian_functions "Jacobian functions" for related
  /// functions.
  /// @throws std::exception if `J𝑠_V_ABp_E` is nullptr or not sized `6 x n`.
  void CalcJacobianSpatialVelocity(const systems::Context<T>& context,
                                   JacobianWrtVariable with_respect_to,
                                   const Frame<T>& frame_B,
                                   const Eigen::Ref<const Vector3<T>>& p_BoBp_B,
                                   const Frame<T>& frame_A,
                                   const Frame<T>& frame_E,
                                   EigenPtr<MatrixX<T>> Js_V_ABp_E) const {
    this->ValidateContext(context);
    DRAKE_DEMAND(Js_V_ABp_E != nullptr);
    internal_tree().CalcJacobianSpatialVelocity(context, with_respect_to,
                                                frame_B, p_BoBp_B, frame_A,
                                                frame_E, Js_V_ABp_E);
  }

  /// Calculates J𝑠_w_AB, a frame B's angular velocity Jacobian in a frame A
  /// with respect to "speeds" 𝑠.
  /// <pre>
  ///      J𝑠_w_AB ≜ [ ∂(w_AB)/∂𝑠₁,  ...  ∂(w_AB)/∂𝑠ₙ ]    (n is j or k)
  ///      w_AB = J𝑠_w_AB ⋅ 𝑠          w_AB is linear in 𝑠 ≜ [𝑠₁ ... 𝑠ₙ]ᵀ
  /// </pre>
  /// `w_AB` is B's angular velocity in frame A and "speeds" 𝑠 is either
  /// q̇ ≜ [q̇₁ ... q̇ⱼ]ᵀ (time-derivatives of j generalized positions) or
  /// v ≜ [v₁ ... vₖ]ᵀ (k generalized velocities).
  ///
  /// @param[in] context The state of the multibody system.
  /// @param[in] with_respect_to Enum equal to JacobianWrtVariable::kQDot or
  /// JacobianWrtVariable::kV, indicating whether the Jacobian `J𝑠_w_AB` is
  /// partial derivatives with respect to 𝑠 = q̇ (time-derivatives of generalized
  /// positions) or with respect to 𝑠 = v (generalized velocities).
  /// @param[in] frame_B The frame B in `w_AB` (B's angular velocity in A).
  /// @param[in] frame_A The frame A in `w_AB` (B's angular velocity in A).
  /// @param[in] frame_E The frame in which `w_AB` is expressed on input and
  /// the frame in which the Jacobian `J𝑠_w_AB` is expressed on output.
  /// @param[out] Js_w_AB_E Frame B's angular velocity Jacobian in frame A with
  /// respect to speeds 𝑠 (which is either q̇ or v), expressed in frame E.
  /// The Jacobian is a function of only generalized positions q (which are
  /// pulled from the context).  The previous definition shows `J𝑠_w_AB_E` is
  /// a matrix of size `3 x n`, where n is the number of elements in 𝑠.
  /// @see See @ref Jacobian_functions "Jacobian functions" for related
  /// functions.
  /// @throws std::exception if `J𝑠_w_AB_E` is nullptr or not of size `3 x n`.
  void CalcJacobianAngularVelocity(const systems::Context<T>& context,
                                   const JacobianWrtVariable with_respect_to,
                                   const Frame<T>& frame_B,
                                   const Frame<T>& frame_A,
                                   const Frame<T>& frame_E,
                                   EigenPtr<Matrix3X<T>> Js_w_AB_E) const {
    this->ValidateContext(context);
    DRAKE_DEMAND(Js_w_AB_E != nullptr);
    return internal_tree().CalcJacobianAngularVelocity(
        context, with_respect_to, frame_B, frame_A, frame_E, Js_w_AB_E);
  }

  /// For each point Bi affixed/welded to a frame B, calculates J𝑠_v_ABi, Bi's
  /// translational velocity Jacobian in frame A with respect to "speeds" 𝑠.
  /// <pre>
  ///      J𝑠_v_ABi ≜ [ ∂(v_ABi)/∂𝑠₁,  ...  ∂(v_ABi)/∂𝑠ₙ ]    (n is j or k)
  ///      v_ABi = J𝑠_v_ABi ⋅ 𝑠          v_ABi is linear in 𝑠 ≜ [𝑠₁ ... 𝑠ₙ]ᵀ
  /// </pre>
  /// `v_ABi` is Bi's translational velocity in frame A and "speeds" 𝑠 is either
  /// q̇ ≜ [q̇₁ ... q̇ⱼ]ᵀ (time-derivatives of j generalized positions) or
  /// v ≜ [v₁ ... vₖ]ᵀ (k generalized velocities).
  ///
  /// @param[in] context The state of the multibody system.
  /// @param[in] with_respect_to Enum equal to JacobianWrtVariable::kQDot or
  /// JacobianWrtVariable::kV, indicating whether the Jacobian `J𝑠_v_ABi` is
  /// partial derivatives with respect to 𝑠 = q̇ (time-derivatives of generalized
  /// positions) or with respect to 𝑠 = v (generalized velocities).
  /// @param[in] frame_B The frame on which point Bi is affixed/welded.
  /// @param[in] p_BoBi_B A position vector or list of p position vectors from
  /// Bo (frame_B's origin) to points Bi (regarded as affixed to B), where each
  /// position vector is expressed in frame_B.
  /// @param[in] frame_A The frame that measures `v_ABi` (Bi's velocity in A).
  /// Note: It is natural to wonder why there is no parameter p_AoAi_A (similar
  /// to the parameter p_BoBi_B for frame_B).  There is no need for p_AoAi_A
  /// because Bi's velocity in A is defined as the derivative in frame A of
  /// Bi's position vector from _any_ point affixed to A.
  /// @param[in] frame_E The frame in which `v_ABi` is expressed on input and
  /// the frame in which the Jacobian `J𝑠_v_ABi` is expressed on output.
  /// @param[out] Js_v_ABi_E Point Bi's velocity Jacobian in frame A with
  /// respect to speeds 𝑠 (which is either q̇ or v), expressed in frame E.
  /// `J𝑠_v_ABi_E` is a `3*p x n` matrix, where p is the number of points Bi and
  /// n is the number of elements in 𝑠.  The Jacobian is a function of only
  /// generalized positions q (which are pulled from the context).
  /// @throws std::exception if `J𝑠_v_ABi_E` is nullptr or not sized `3*p x n`.
  /// @note When 𝑠 = q̇, `Jq̇_v_ABi = Jq_p_AoBi`.  In other words, point Bi's
  /// velocity Jacobian in frame A with respect to q̇ is equal to point Bi's
  /// position Jacobian from Ao (A's origin) in frame A with respect to q. <pre>
  /// [∂(v_ABi)/∂q̇₁,  ...  ∂(v_ABi)/∂q̇ⱼ] = [∂(p_AoBi)/∂q₁,  ...  ∂(p_AoBi)/∂qⱼ]
  /// </pre>
  /// Note: Each partial derivative of p_AoBi is taken in frame A.
  /// @see CalcJacobianPositionVector() for details on Jq_p_AoBi.
  /// @see See @ref Jacobian_functions "Jacobian functions" for related
  /// functions.
  void CalcJacobianTranslationalVelocity(
      const systems::Context<T>& context, JacobianWrtVariable with_respect_to,
      const Frame<T>& frame_B, const Eigen::Ref<const Matrix3X<T>>& p_BoBi_B,
      const Frame<T>& frame_A, const Frame<T>& frame_E,
      EigenPtr<MatrixX<T>> Js_v_ABi_E) const {
    // TODO(amcastro-tri): provide the Jacobian-times-vector operation.  For
    // some applications it is all we need and it is more efficient to compute.
    this->ValidateContext(context);
    DRAKE_DEMAND(Js_v_ABi_E != nullptr);
    internal_tree().CalcJacobianTranslationalVelocity(
        context, with_respect_to, frame_B, frame_B, p_BoBi_B, frame_A, frame_E,
        Js_v_ABi_E);
  }

  /// For each point Bi affixed/welded to a frame B, calculates Jq_p_AoBi, Bi's
  /// position vector Jacobian in frame A with respect to the generalized
  /// positions q ≜ [q₁ ... qₙ]ᵀ as
  /// <pre>
  ///      Jq_p_AoBi ≜ [ ᴬ∂(p_AoBi)/∂q₁,  ...  ᴬ∂(p_AoBi)/∂qₙ ]
  /// </pre>
  /// where p_AoBi is Bi's position vector from point Ao (frame A's origin) and
  /// ᴬ∂(p_AoBi)/∂qᵣ denotes the partial derivative in frame A of p_AoBi with
  /// respect to the generalized position qᵣ, where qᵣ is one of q₁ ... qₙ.
  /// @param[in] context The state of the multibody system.
  /// @param[in] frame_B The frame on which point Bi is affixed/welded.
  /// @param[in] p_BoBi_B A position vector or list of k position vectors from
  /// Bo (frame_B's origin) to points Bi (Bi is regarded as affixed to B), where
  /// each position vector is expressed in frame_B.
  /// @param[in] frame_A The frame in which partial derivatives are calculated
  /// and the frame in which point Ao is affixed.
  /// @param[in] frame_E The frame in which the Jacobian Jq_p_AoBi is expressed
  /// on output.
  /// @param[out] Jq_p_AoBi_E Point Bi's position vector Jacobian in frame A
  /// with generalized positions q, expressed in frame E. Jq_p_AoBi_E is a
  /// `3*k x n` matrix, where k is the number of points Bi and n is the number
  /// of elements in q.  The Jacobian is a function of only generalized
  /// positions q (which are pulled from the context).
  /// @throws std::exception if Jq_p_AoBi_E is nullptr or not sized `3*k x n`.
  /// @note Jq̇_v_ABi = Jq_p_AoBi.  In other words, point Bi's velocity Jacobian
  /// in frame A with respect to q̇ is equal to point Bi's position vector
  /// Jacobian in frame A with respect to q.
  /// <pre>
  /// [∂(v_ABi)/∂q̇₁, ... ∂(v_ABi)/∂q̇ₙ] = [ᴬ∂(p_AoBi)/∂q₁, ... ᴬ∂(p_AoBi)/∂qₙ]
  /// </pre>
  /// @see CalcJacobianTranslationalVelocity() for details on Jq̇_v_ABi.
  /// Note: Jq_p_AaBi = Jq_p_AoBi, where point Aa is _any_ point fixed/welded to
  /// frame A, i.e., this calculation's result is the same if point Ao is
  /// replaced with any point fixed on frame A.
  /// @see See @ref Jacobian_functions "Jacobian functions" for related
  /// functions.
  void CalcJacobianPositionVector(const systems::Context<T>& context,
                                  const Frame<T>& frame_B,
                                  const Eigen::Ref<const Matrix3X<T>>& p_BoBi_B,
                                  const Frame<T>& frame_A,
                                  const Frame<T>& frame_E,
                                  EigenPtr<MatrixX<T>> Jq_p_AoBi_E) const {
    // TODO(mitiguy) Consider providing the Jacobian-times-vector operation.
    //  Sometimes it is all that is needed and more efficient to compute.
    this->ValidateContext(context);
    DRAKE_DEMAND(Jq_p_AoBi_E != nullptr);
    internal_tree().CalcJacobianTranslationalVelocity(
        context, JacobianWrtVariable::kQDot, frame_B, frame_B, p_BoBi_B,
        frame_A, frame_E, Jq_p_AoBi_E);
  }

  /// Calculates J𝑠_v_AScm_E, point Scm's translational velocity Jacobian in
  /// frame A with respect to "speeds" 𝑠, expressed in frame E, where point Scm
  /// is the center of mass of the system S of all non-world bodies contained in
  /// `this` MultibodyPlant.
  /// @param[in] context contains the state of the model.
  /// @param[in] with_respect_to Enum equal to JacobianWrtVariable::kQDot or
  /// JacobianWrtVariable::kV, indicating whether the Jacobian `J𝑠_v_AScm_E` is
  /// partial derivatives with respect to 𝑠 = q̇ (time-derivatives of generalized
  /// positions) or with respect to 𝑠 = v (generalized velocities).
  /// @param[in] frame_A The frame in which the translational velocity
  /// v_AScm and its Jacobian J𝑠_v_AScm are measured.
  /// @param[in] frame_E The frame in which the Jacobian J𝑠_v_AScm is
  /// expressed on output.
  /// @param[out] Js_v_AScm_E Point Scm's translational velocity Jacobian in
  /// frame A with respect to speeds 𝑠 (𝑠 = q̇ or 𝑠 = v), expressed in frame E.
  /// J𝑠_v_AScm_E is a 3 x n matrix, where n is the number of elements in 𝑠.
  /// The Jacobian is a function of only generalized positions q (which are
  /// pulled from the context).
  /// @throws std::exception if Scm does not exist, which occurs if there
  /// are no massive bodies in MultibodyPlant (except world_body()).
  /// @throws std::exception if mₛ ≤ 0 (where mₛ is the mass of all non-world
  /// bodies contained in `this` MultibodyPlant).
  /// @see See @ref Jacobian_functions "Jacobian functions" for related
  /// functions.
  void CalcJacobianCenterOfMassTranslationalVelocity(
      const systems::Context<T>& context, JacobianWrtVariable with_respect_to,
      const Frame<T>& frame_A, const Frame<T>& frame_E,
      EigenPtr<Matrix3X<T>> Js_v_AScm_E) const {
    this->ValidateContext(context);
    DRAKE_DEMAND(Js_v_AScm_E != nullptr);
    internal_tree().CalcJacobianCenterOfMassTranslationalVelocity(
        context, with_respect_to, frame_A, frame_E, Js_v_AScm_E);
  }

  /// Calculates J𝑠_v_AScm_E, point Scm's translational velocity Jacobian in
  /// frame A with respect to "speeds" 𝑠, expressed in frame E, where point Scm
  /// is the center of mass of the system S of all non-world bodies contained in
  /// model_instances.
  /// @param[in] context contains the state of the model.
  /// @param[in] model_instances Vector of selected model instances.  If a model
  /// instance is repeated in the vector (unusual), it is only counted once.
  /// @param[in] with_respect_to Enum equal to JacobianWrtVariable::kQDot or
  /// JacobianWrtVariable::kV, indicating whether the Jacobian `J𝑠_v_AScm_E` is
  /// partial derivatives with respect to 𝑠 = q̇ (time-derivatives of generalized
  /// positions) or with respect to 𝑠 = v (generalized velocities).
  /// @param[in] frame_A The frame in which the translational velocity
  /// v_AScm and its Jacobian J𝑠_v_AScm are measured.
  /// @param[in] frame_E The frame in which the Jacobian J𝑠_v_AScm is
  /// expressed on output.
  /// @param[out] Js_v_AScm_E Point Scm's translational velocity Jacobian in
  /// frame A with respect to speeds 𝑠 (𝑠 = q̇ or 𝑠 = v), expressed in frame E.
  /// J𝑠_v_AScm_E is a 3 x n matrix, where n is the number of elements in 𝑠.
  /// The Jacobian is a function of only generalized positions q (which are
  /// pulled from the context).
  /// @throws std::exception if mₛ ≤ 0 (where mₛ is the mass of all non-world
  /// bodies contained in model_instances).
  /// @throws std::exception if model_instances is empty or only has world body.
  /// @note The world_body() is ignored.  J𝑠_v_AScm_ = ∑ (mᵢ Jᵢ) / mₛ, where
  /// mₛ = ∑ mᵢ, mᵢ is the mass of the iᵗʰ body contained in model_instances,
  /// and Jᵢ is Bᵢcm's translational velocity Jacobian in frame A, expressed in
  /// frame E (Bᵢcm is the center of mass of the iᵗʰ body).
  /// @see See @ref Jacobian_functions "Jacobian functions" for related
  /// functions.
  void CalcJacobianCenterOfMassTranslationalVelocity(
      const systems::Context<T>& context,
      const std::vector<ModelInstanceIndex>& model_instances,
      JacobianWrtVariable with_respect_to, const Frame<T>& frame_A,
      const Frame<T>& frame_E, EigenPtr<Matrix3X<T>> Js_v_AScm_E) const {
    this->ValidateContext(context);
    DRAKE_DEMAND(Js_v_AScm_E != nullptr);
    internal_tree().CalcJacobianCenterOfMassTranslationalVelocity(
        context, model_instances, with_respect_to, frame_A, frame_E,
        Js_v_AScm_E);
  }
  /// @} <!-- Jacobian_functions -->

  /// @anchor bias_acceleration_functions
  /// @name Bias acceleration functions
  /// The name a𝑠Bias_AP denotes a point P's bias translational acceleration
  /// with respect to "speeds" 𝑠 measured in a frame A, where 𝑠 is either q̇
  /// (time-derivatives of generalized positions) or v (generalized velocities).
  /// a𝑠Bias_AP includes the terms in a_AP (P's translational acceleration in A)
  /// that depend on q, q̇, v, but not terms that depend on 𝑠̇, i.e.,
  /// a𝑠Bias_AP = a_AP when 𝑠̇ = 0. The proof below starts with v_AP (point P's
  /// translational velocity in frame A) written in terms of J𝑠_v_AP (point P's
  /// translational velocity Jacobian in frame A for s).  <pre>
  ///   v_AP = J𝑠_v_AP ⋅ 𝑠         which upon vector differentiation in A gives
  ///   a_AP = Ĵ𝑠_v_AP ⋅ 𝑠  +  J𝑠_v_AP ⋅ 𝑠̇                 setting 𝑠̇ = 0, gives
  ///   a𝑠Bias_AP = Ĵ𝑠_v_AP ⋅ 𝑠                               is quadratic in s.
  /// </pre>
  /// Note: Since Ĵ𝑠_v_AP (the time-derivative of J𝑠_v_AP in frame A) is linear
  /// in s, a𝑠Bias_AP = Ĵ𝑠_v_AP ⋅ 𝑠  is quadratic in 𝑠.
  ///
  /// Similarly, A𝑠Bias_AB denotes a frame B's bias spatial acceleration with
  /// respect to speeds 𝑠 measured in frame A. It can be written in terms of the
  /// time-derivative of J𝑠_V_AB (B's spatial velocity Jacobian in frame A for
  /// speeds 𝑠) as <pre>
  ///   A𝑠Bias_AB = Ĵ𝑠_V_AB ⋅ 𝑠                       is quadratic in s. </pre>
  /// @see CalcJacobianSpatialVelocity() for details on J𝑠_V_AB.
  ///@{

  /// Computes the bias term `C(q, v) v` containing Coriolis, centripetal, and
  /// gyroscopic effects in the multibody equations of motion: <pre>
  ///   M(q) v̇ + C(q, v) v = tau_app + ∑ (Jv_V_WBᵀ(q) ⋅ Fapp_Bo_W)
  /// </pre>
  /// where `M(q)` is the multibody model's mass matrix (including rigid body
  /// mass properties and @ref reflected_inertia "reflected inertias") and
  /// `tau_app` is a vector of applied generalized forces. The last term is a
  /// summation over all bodies of the dot-product of `Fapp_Bo_W` (applied
  /// spatial force on body B at Bo) with `Jv_V_WB(q)` (B's spatial Jacobian in
  /// world W with respect to generalized velocities v).
  /// Note: B's spatial velocity in W can be written `V_WB = Jv_V_WB * v`.
  /// @param[in] context Contains the state of the multibody system, including
  /// the generalized positions q and the generalized velocities v.
  /// @param[out] Cv On output, `Cv` will contain the product `C(q, v)v`. It
  /// must be a valid (non-null) pointer to a column vector in `ℛⁿ` with n the
  /// number of generalized velocities (num_velocities()) of the model. This
  /// method aborts if Cv is nullptr or if it does not have the proper size.
  void CalcBiasTerm(const systems::Context<T>& context,
                    EigenPtr<VectorX<T>> Cv) const {
    this->ValidateContext(context);
    DRAKE_DEMAND(Cv != nullptr);
    internal_tree().CalcBiasTerm(context, Cv);
  }

  /// For each point Bi affixed/welded to a frame B, calculates a𝑠Bias_ABi, Bi's
  /// translational acceleration bias in frame A with respect to "speeds" 𝑠,
  /// expressed in frame E, where speeds 𝑠 is either q̇ or v.
  /// @param[in] context Contains the state of the multibody system.
  /// @param[in] with_respect_to Enum equal to JacobianWrtVariable::kQDot or
  /// JacobianWrtVariable::kV, indicating whether the translational
  /// acceleration bias is with respect to 𝑠 = q̇ or 𝑠 = v. Currently, an
  /// exception is thrown if with_respect_to is JacobianWrtVariable::kQDot.
  /// @param[in] frame_B The frame on which points Bi are affixed/welded.
  /// @param[in] p_BoBi_B A position vector or list of p position vectors from
  /// Bo (frame_B's origin) to points Bi (regarded as affixed to B), where each
  /// position vector is expressed in frame_B.  Each column in the `3 x p`
  /// matrix p_BoBi_B corresponds to a position vector.
  /// @param[in] frame_A The frame in which a𝑠Bias_ABi is measured.
  /// @param[in] frame_E The frame in which a𝑠Bias_ABi is expressed on output.
  /// @returns a𝑠Bias_ABi_E Point Bi's translational acceleration bias in
  /// frame A with respect to speeds 𝑠 (𝑠 = q̇ or 𝑠 = v), expressed in frame E.
  /// a𝑠Bias_ABi_E is a `3 x p` matrix, where p is the number of points Bi.
  /// @see CalcJacobianTranslationalVelocity() to compute J𝑠_v_ABi, point Bi's
  /// translational velocity Jacobian in frame A with respect to 𝑠.
  /// @pre p_BoBi_B must have 3 rows.
  /// @throws std::exception if with_respect_to is JacobianWrtVariable::kQDot.
  /// @note See @ref bias_acceleration_functions "Bias acceleration functions"
  /// for theory and details.
  Matrix3X<T> CalcBiasTranslationalAcceleration(
      const systems::Context<T>& context, JacobianWrtVariable with_respect_to,
      const Frame<T>& frame_B, const Eigen::Ref<const Matrix3X<T>>& p_BoBi_B,
      const Frame<T>& frame_A, const Frame<T>& frame_E) const {
    // TODO(Mitiguy) Allow with_respect_to to be JacobianWrtVariable::kQDot.
    this->ValidateContext(context);
    return internal_tree().CalcBiasTranslationalAcceleration(
        context, with_respect_to, frame_B, p_BoBi_B, frame_A, frame_E);
  }

  /// For one point Bp affixed/welded to a frame B, calculates A𝑠Bias_ABp, Bp's
  /// spatial acceleration bias in frame A with respect to "speeds" 𝑠, expressed
  /// in frame E, where speeds 𝑠 is either q̇ or v.
  /// @param[in] context Contains the state of the multibody system.
  /// @param[in] with_respect_to Enum equal to JacobianWrtVariable::kQDot or
  /// JacobianWrtVariable::kV, indicating whether the spatial accceleration bias
  /// is with respect to 𝑠 = q̇ or 𝑠 = v. Currently, an exception is thrown if
  /// with_respect_to is JacobianWrtVariable::kQDot.
  /// @param[in] frame_B The frame on which point Bp is affixed/welded.
  /// @param[in] p_BoBp_B Position vector from Bo (frame_B's origin) to point Bp
  /// (regarded as affixed/welded to B), expressed in frame_B.
  /// @param[in] frame_A The frame in which A𝑠Bias_ABp is measured.
  /// @param[in] frame_E The frame in which A𝑠Bias_ABp is expressed on output.
  /// @returns A𝑠Bias_ABp_E Point Bp's spatial acceleration bias in frame A
  /// with respect to speeds 𝑠 (𝑠 = q̇ or 𝑠 = v), expressed in frame E.
  /// @see CalcJacobianSpatialVelocity() to compute J𝑠_V_ABp, point Bp's
  /// spatial velocity Jacobian in frame A with respect to 𝑠.
  /// @throws std::exception if with_respect_to is JacobianWrtVariable::kQDot.
  /// @note Use CalcBiasTranslationalAcceleration() to efficiently calculate
  /// bias translational accelerations for a list of points (each fixed to
  /// frame B). This function returns only one bias spatial acceleration, which
  /// contains both frame B's bias angular acceleration and point Bp's bias
  /// translational acceleration.
  /// @note See @ref bias_acceleration_functions "Bias acceleration functions"
  /// for theory and details.
  SpatialAcceleration<T> CalcBiasSpatialAcceleration(
      const systems::Context<T>& context, JacobianWrtVariable with_respect_to,
      const Frame<T>& frame_B, const Eigen::Ref<const Vector3<T>>& p_BoBp_B,
      const Frame<T>& frame_A, const Frame<T>& frame_E) const {
    // TODO(Mitiguy) Allow with_respect_to to be JacobianWrtVariable::kQDot.
    this->ValidateContext(context);
    return internal_tree().CalcBiasSpatialAcceleration(
        context, with_respect_to, frame_B, p_BoBp_B, frame_A, frame_E);
  }

  /// For the system S of all bodies other than the world body, calculates
  /// a𝑠Bias_AScm_E, Scm's translational acceleration bias in frame A with
  /// respect to "speeds" 𝑠, expressed in frame E, where Scm is the center of
  /// mass of S and speeds 𝑠 is either q̇ or v.
  /// @param[in] context Contains the state of the multibody system.
  /// @param[in] with_respect_to Enum equal to JacobianWrtVariable::kQDot or
  /// JacobianWrtVariable::kV, indicating whether the accceleration bias is
  /// with respect to 𝑠 = q̇ or 𝑠 = v. Currently, an exception is thrown if
  /// with_respect_to is JacobianWrtVariable::kQDot.
  /// @param[in] frame_A The frame in which a𝑠Bias_AScm is measured.
  /// @param[in] frame_E The frame in which a𝑠Bias_AScm is expressed on output.
  /// @returns a𝑠Bias_AScm_E Point Scm's translational acceleration bias in
  /// frame A with respect to speeds 𝑠 (𝑠 = q̇ or 𝑠 = v), expressed in frame E.
  /// @throws std::exception if `this` has no body except world_body().
  /// @throws std::exception if mₛ ≤ 0, where mₛ is the mass of system S.
  /// @throws std::exception if with_respect_to is JacobianWrtVariable::kQDot.
  /// @see CalcJacobianCenterOfMassTranslationalVelocity() to compute J𝑠_v_Scm,
  /// point Scm's translational velocity Jacobian in frame A with respect to 𝑠.
  /// @note The world_body() is ignored. asBias_AScm_E = ∑ (mᵢ aᵢ) / mₛ, where
  /// mₛ = ∑ mᵢ is the mass of system S, mᵢ is the mass of the iᵗʰ body, and
  /// aᵢ is the translational bias acceleration of Bᵢcm in frame A expressed in
  /// frame E for speeds 𝑠 (Bᵢcm is the center of mass of the iᵗʰ body).
  /// @note See @ref bias_acceleration_functions "Bias acceleration functions"
  /// for theory and details.
  Vector3<T> CalcBiasCenterOfMassTranslationalAcceleration(
      const systems::Context<T>& context, JacobianWrtVariable with_respect_to,
      const Frame<T>& frame_A, const Frame<T>& frame_E) const {
    // TODO(Mitiguy) Allow with_respect_to to be JacobianWrtVariable::kQDot.
    this->ValidateContext(context);
    return internal_tree().CalcBiasCenterOfMassTranslationalAcceleration(
        context, with_respect_to, frame_A, frame_E);
  }

  /// For the system S containing the selected model instances, calculates
  /// a𝑠Bias_AScm_E, Scm's translational acceleration bias in frame A with
  /// respect to "speeds" 𝑠, expressed in frame E, where Scm is the center of
  /// mass of S and speeds 𝑠 is either q̇ or v.
  /// @param[in] context Contains the state of the multibody system.
  /// @param[in] model_instances Vector of selected model instances.  If a model
  /// instance is repeated in the vector (unusual), it is only counted once.
  /// @param[in] with_respect_to Enum equal to JacobianWrtVariable::kQDot or
  /// JacobianWrtVariable::kV, indicating whether the accceleration bias is
  /// with respect to 𝑠 = q̇ or 𝑠 = v. Currently, an exception is thrown if
  /// with_respect_to is JacobianWrtVariable::kQDot.
  /// @param[in] frame_A The frame in which a𝑠Bias_AScm is measured.
  /// @param[in] frame_E The frame in which a𝑠Bias_AScm is expressed on output.
  /// @returns a𝑠Bias_AScm_E Point Scm's translational acceleration bias in
  /// frame A with respect to speeds 𝑠 (𝑠 = q̇ or 𝑠 = v), expressed in frame E.
  /// @throws std::exception if `this` has no body except world_body().
  /// @throws std::exception if mₛ ≤ 0, where mₛ is the mass of system S.
  /// @throws std::exception if with_respect_to is JacobianWrtVariable::kQDot.
  /// @see CalcJacobianCenterOfMassTranslationalVelocity() to compute J𝑠_v_Scm,
  /// point Scm's translational velocity Jacobian in frame A with respect to 𝑠.
  /// @note The world_body() is ignored. asBias_AScm_E = ∑ (mᵢ aᵢ) / mₛ, where
  /// mₛ = ∑ mᵢ is the mass of system S, mᵢ is the mass of the iᵗʰ body, and
  /// aᵢ is the translational bias acceleration of Bᵢcm in frame A expressed in
  /// frame E for speeds 𝑠 (Bᵢcm is the center of mass of the iᵗʰ body).
  /// @note See @ref bias_acceleration_functions "Bias acceleration functions"
  /// for theory and details.
  Vector3<T> CalcBiasCenterOfMassTranslationalAcceleration(
      const systems::Context<T>& context,
      const std::vector<ModelInstanceIndex>& model_instances,
      JacobianWrtVariable with_respect_to, const Frame<T>& frame_A,
      const Frame<T>& frame_E) const {
    // TODO(Mitiguy) Allow with_respect_to to be JacobianWrtVariable::kQDot.
    this->ValidateContext(context);
    return internal_tree().CalcBiasCenterOfMassTranslationalAcceleration(
        context, model_instances, with_respect_to, frame_A, frame_E);
  }
  /// @} <!-- Bias acceleration functions -->

  /// @anchor mbp_introspection
  /// @name                    Introspection
  /// These methods allow a user to query whether a given multibody element is
  /// part of this plant's model. These queries can be performed at any time
  /// during the lifetime of a %MultibodyPlant model, i.e. there is no
  /// restriction on whether they must be called before or after Finalize().
  /// These queries can be performed while new multibody elements are
  /// being added to the model.
  /// These methods allow a user to retrieve a reference to a multibody element
  /// by its name. An exception is thrown if there is no element with the
  /// requested name.
  ///
  /// If the named element is present in more than one model instance and a
  /// model instance is not explicitly specified, std::logic_error is thrown.
  /// @{

  /// The time step (or period) used to model `this` plant as a discrete system
  /// with periodic updates. Returns 0 (zero) if the plant is modeled as a
  /// continuous system.
  /// This property of the plant is specified at construction and therefore this
  /// query can be performed either pre- or post-finalize, see Finalize().
  /// @see MultibodyPlant::MultibodyPlant(double)
  double time_step() const { return time_step_; }

  /// Returns `true` if this %MultibodyPlant was finalized with a call to
  /// Finalize().
  /// @see Finalize().
  bool is_finalized() const { return internal_tree().is_finalized(); }

  /// (Advanced) If `this` plant is continuous (i.e., is_discrete() is `false`),
  /// returns false. If `this` plant is discrete, returns whether or not the
  /// output ports are sampled (change only at a time step boundary) or
  /// live (instantaneously reflect changes to the input ports).
  /// See @ref output_port_sampling "Output port sampling" for details.
  bool has_sampled_output_ports() const { return use_sampled_output_ports_; }

  /// Returns a constant reference to the *world* body.
  const RigidBody<T>& world_body() const {
    return internal_tree().world_body();
  }

  /// Returns a constant reference to the *world* frame.
  const RigidBodyFrame<T>& world_frame() const {
    return internal_tree().world_frame();
  }

  /// Returns the number of RigidBody elements in the model, including the
  /// "world" RigidBody, which is always part of the model.
  /// @see AddRigidBody().
  int num_bodies() const { return internal_tree().num_bodies(); }

  /// Returns `true` if plant has a rigid body with unique index `body_index`.
  bool has_body(BodyIndex body_index) const {
    return internal_tree().has_body(body_index);
  }

  /// Returns a constant reference to the body with unique index `body_index`.
  /// @throws std::exception if `body_index` does not correspond to a body in
  /// this model.
  const RigidBody<T>& get_body(BodyIndex body_index) const {
    return internal_tree().get_body(body_index);
  }

  /// Returns `true` if @p body is anchored (i.e. the kinematic path between
  /// @p body and the world only contains weld joints.)
  /// @throws std::exception if called pre-finalize.
  bool IsAnchored(const RigidBody<T>& body) const {
    DRAKE_MBP_THROW_IF_NOT_FINALIZED();
    return internal_tree().graph().link_by_index(body.index()).is_anchored();
  }

  /// @returns `true` if a body named `name` was added to the %MultibodyPlant.
  /// @see AddRigidBody().
  ///
  /// @throws std::exception if the body name occurs in multiple model
  /// instances.
  bool HasBodyNamed(std::string_view name) const {
    return internal_tree().HasBodyNamed(name);
  }

  /// @returns The total number of bodies (across all model instances) with the
  /// given name.
  int NumBodiesWithName(std::string_view name) const {
    return internal_tree().NumBodiesWithName(name);
  }

  /// @returns `true` if a body named `name` was added to the %MultibodyPlant
  /// in @p model_instance.
  /// @see AddRigidBody().
  ///
  /// @throws std::exception if @p model_instance is not valid for this model.
  bool HasBodyNamed(std::string_view name,
                    ModelInstanceIndex model_instance) const {
    return internal_tree().HasBodyNamed(name, model_instance);
  }

  /// Returns a constant reference to a body that is identified
  /// by the string `name` in `this` %MultibodyPlant.
  /// @throws std::exception if there is no body with the requested name.
  /// @throws std::exception if the body name occurs in multiple model
  /// instances.
  /// @see HasBodyNamed() to query if there exists a body in `this`
  /// %MultibodyPlant with a given specified name.
  const RigidBody<T>& GetBodyByName(std::string_view name) const {
    return internal_tree().GetRigidBodyByName(name);
  }

  /// Returns a constant reference to the body that is uniquely identified
  /// by the string `name` and @p model_instance in `this` %MultibodyPlant.
  /// @throws std::exception if there is no body with the requested name.
  /// @see HasBodyNamed() to query if there exists a body in `this`
  /// %MultibodyPlant with a given specified name.
  const RigidBody<T>& GetBodyByName(std::string_view name,
                                    ModelInstanceIndex model_instance) const {
    return internal_tree().GetRigidBodyByName(name, model_instance);
  }

  /// Returns a list of body indices associated with `model_instance`.
  std::vector<BodyIndex> GetBodyIndices(
      ModelInstanceIndex model_instance) const {
    return internal_tree().GetBodyIndices(model_instance);
  }

  /// Returns a constant reference to a rigid body that is identified
  /// by the string `name` in `this` model.
  /// @throws std::exception if there is no body with the requested name.
  /// @throws std::exception if the body name occurs in multiple model
  /// instances.
  /// @throws std::exception if the requested body is not a RigidBody.
  /// @see HasBodyNamed() to query if there exists a body in `this` model with a
  /// given specified name.
  const RigidBody<T>& GetRigidBodyByName(std::string_view name) const {
    return internal_tree().GetRigidBodyByName(name);
  }

  /// Returns a constant reference to the rigid body that is uniquely identified
  /// by the string `name` in @p model_instance.
  /// @throws std::exception if there is no body with the requested name.
  /// @throws std::exception if the requested body is not a RigidBody.
  /// @throws std::exception if @p model_instance is not valid for this
  ///         model.
  /// @see HasBodyNamed() to query if there exists a body in `this` model with a
  /// given specified name.
  const RigidBody<T>& GetRigidBodyByName(
      std::string_view name, ModelInstanceIndex model_instance) const {
    return internal_tree().GetRigidBodyByName(name, model_instance);
  }

  /// Returns all bodies that are transitively welded, or rigidly affixed, to
  /// `body`, per these two definitions:
  ///
  /// 1. A body is always considered welded to itself.
  /// 2. Two unique bodies are considered welded together exclusively by the
  /// presence of a weld joint, not by other constructs that prevent mobility
  /// (e.g. constraints).
  ///
  /// This method can be called at any time during the lifetime of `this` plant,
  /// either pre- or post-finalize, see Finalize().
  ///
  /// Meant to be used with `CollectRegisteredGeometries`.
  ///
  /// The following example demonstrates filtering collisions between all
  /// bodies rigidly affixed to a door (which could be moving) and all bodies
  /// rigidly affixed to the world:
  /// @code
  /// GeometrySet g_world = plant.CollectRegisteredGeometries(
  ///     plant.GetBodiesWeldedTo(plant.world_body()));
  /// GeometrySet g_door = plant.CollectRegisteredGeometries(
  ///     plant.GetBodiesWeldedTo(plant.GetBodyByName("door")));
  /// scene_graph.ExcludeCollisionsBetweeen(g_world, g_door);
  /// @endcode
  /// @note Usages akin to this example may introduce redundant collision
  /// filtering; this will not have a functional impact, but may have a minor
  /// performance impact.
  ///
  /// @returns all bodies rigidly fixed to `body`. This does not return the
  /// bodies in any prescribed order.
  /// @throws std::exception if `body` is not part of this plant.
  std::vector<const RigidBody<T>*> GetBodiesWeldedTo(
      const RigidBody<T>& body) const;

  /// Returns all bodies whose kinematics are transitively affected by the given
  /// vector of Joints. This is a _kinematic_ relationship rather than a
  /// dynamic one. It is is inherently a query on the topology of the plant's
  /// modeled tree. Constraints are likewise not considered.
  ///
  /// The affected bodies are returned in increasing order of body indices. A
  /// body is included in the output if that body's spatial velocity is
  /// affected by the generalized velocities v of one of the indicated joints.
  ///
  /// As such, there are some notable implications:
  ///
  ///   1. If a body has an inboard free (6 dof) joint, it will be
  ///      _kinematically_ affected by joints further inboard, even though there
  ///      might not be any dynamic influence on that body.
  ///   2. If the set of joints have no velocities (i.e., they are all weld (0
  ///      dof) joints), then, by definition, no bodies will be affected.
  ///
  /// This function can be only be called post-finalize, see Finalize().
  /// @throws std::exception if any of the given joint has an invalid index,
  /// doesn't correspond to a mobilizer, or is welded.
  std::vector<BodyIndex> GetBodiesKinematicallyAffectedBy(
      const std::vector<JointIndex>& joint_indexes) const;

  /// Returns the number of joints in the model.
  /// @see AddJoint().
  int num_joints() const { return internal_tree().num_joints(); }

  /// Returns true if plant has a joint with unique index `joint_index`. The
  /// value could be false if the joint was removed using RemoveJoint().
  bool has_joint(JointIndex joint_index) const {
    return internal_tree().has_joint(joint_index);
  }

  /// Returns a constant reference to the joint with unique index `joint_index`.
  /// @throws std::exception when `joint_index` does not correspond to a
  /// joint in this model.
  const Joint<T>& get_joint(JointIndex joint_index) const {
    return internal_tree().get_joint(joint_index);
  }

  /// @returns `true` if a joint named `name` was added to this model.
  /// @see AddJoint().
  /// @throws std::exception if the joint name occurs in multiple model
  /// instances.
  bool HasJointNamed(std::string_view name) const {
    return internal_tree().HasJointNamed(name);
  }

  /// @returns `true` if a joint named `name` was added to @p model_instance.
  /// @see AddJoint().
  /// @throws std::exception if @p model_instance is not valid for this model.
  bool HasJointNamed(std::string_view name,
                     ModelInstanceIndex model_instance) const {
    return internal_tree().HasJointNamed(name, model_instance);
  }

  /// Returns a mutable reference to the joint with unique index `joint_index`.
  /// @throws std::exception when `joint_index` does not correspond to a
  /// joint in this model.
  Joint<T>& get_mutable_joint(JointIndex joint_index) {
    return this->mutable_tree().get_mutable_joint(joint_index);
  }

  /// Returns a list of all joint indices. The vector is ordered by
  /// monotonically increasing @ref JointIndex, but the indices will in
  /// general not be consecutive due to joints that were removed.
  const std::vector<JointIndex>& GetJointIndices() const {
    return internal_tree().GetJointIndices();
  }

  /// Returns a list of joint indices associated with `model_instance`.
  std::vector<JointIndex> GetJointIndices(
      ModelInstanceIndex model_instance) const {
    return internal_tree().GetJointIndices(model_instance);
  }

  /// Returns a list of all joint actuator indices. The vector is ordered by
  /// monotonically increasing @ref JointActuatorIndex, but the indices will in
  /// general not be consecutive due to actuators that were removed.
  const std::vector<JointActuatorIndex>& GetJointActuatorIndices() const {
    return internal_tree().GetJointActuatorIndices();
  }

  /// Returns a list of joint actuator indices associated with `model_instance`.
  /// The vector is ordered by monotonically increasing @ref JointActuatorIndex.
  /// @throws std::exception if called pre-finalize.
  std::vector<JointActuatorIndex> GetJointActuatorIndices(
      ModelInstanceIndex model_instance) const {
    return internal_tree().GetJointActuatorIndices(model_instance);
  }

  /// Returns a list of actuated joint indices associated with `model_instance`.
  /// @throws std::exception if called pre-finalize.
  std::vector<JointIndex> GetActuatedJointIndices(
      ModelInstanceIndex model_instance) const {
    return internal_tree().GetActuatedJointIndices(model_instance);
  }

  /// Returns a constant reference to a joint that is identified
  /// by the string `name` in `this` %MultibodyPlant.  If the optional
  /// template argument is supplied, then the returned value is downcast to
  /// the specified `JointType`.
  /// @tparam JointType The specific type of the Joint to be retrieved. It must
  /// be a subclass of Joint.
  /// @throws std::exception if the named joint is not of type `JointType` or
  /// if there is no Joint with that name.
  /// @throws std::exception if @p model_instance is not valid for this model.
  /// @see HasJointNamed() to query if there exists a joint in `this`
  /// %MultibodyPlant with a given specified name.
  template <template <typename> class JointType = Joint>
  const JointType<T>& GetJointByName(
      std::string_view name,
      std::optional<ModelInstanceIndex> model_instance = std::nullopt) const {
    return internal_tree().template GetJointByName<JointType>(name,
                                                              model_instance);
  }

  /// A version of GetJointByName that returns a mutable reference.
  /// @see GetJointByName.
  template <template <typename> class JointType = Joint>
  JointType<T>& GetMutableJointByName(
      std::string_view name,
      std::optional<ModelInstanceIndex> model_instance = std::nullopt) {
    return this->mutable_tree().template GetMutableJointByName<JointType>(
        name, model_instance);
  }

  /// Returns the number of Frame objects in this model.
  /// Frames include body frames associated with each of the bodies,
  /// including the _world_ body. This means the minimum number of frames is
  /// one.
  int num_frames() const { return internal_tree().num_frames(); }

  /// Returns a constant reference to the frame with unique index `frame_index`.
  /// @throws std::exception if `frame_index` does not correspond to a frame in
  /// this plant.
  const Frame<T>& get_frame(FrameIndex frame_index) const {
    return internal_tree().get_frame(frame_index);
  }

  /// @returns `true` if a frame named `name` was added to the model.
  /// @see AddFrame().
  /// @throws std::exception if the frame name occurs in multiple model
  /// instances.
  bool HasFrameNamed(std::string_view name) const {
    return internal_tree().HasFrameNamed(name);
  }

  /// @returns `true` if a frame named `name` was added to @p model_instance.
  /// @see AddFrame().
  /// @throws std::exception if @p model_instance is not valid for this model.
  bool HasFrameNamed(std::string_view name,
                     ModelInstanceIndex model_instance) const {
    return internal_tree().HasFrameNamed(name, model_instance);
  }

  /// Returns a constant reference to a frame that is identified by the
  /// string `name` in `this` model.
  /// @throws std::exception if there is no frame with the requested name.
  /// @throws std::exception if the frame name occurs in multiple model
  /// instances.
  /// @see HasFrameNamed() to query if there exists a frame in `this` model with
  /// a given specified name.
  const Frame<T>& GetFrameByName(std::string_view name) const {
    return internal_tree().GetFrameByName(name);
  }

  /// Returns a constant reference to the frame that is uniquely identified
  /// by the string `name` in @p model_instance.
  /// @throws std::exception if there is no frame with the requested name.
  /// @throws std::exception if @p model_instance is not valid for this
  ///         model.
  /// @see HasFrameNamed() to query if there exists a frame in `this` model with
  /// a given specified name.
  const Frame<T>& GetFrameByName(std::string_view name,
                                 ModelInstanceIndex model_instance) const {
    return internal_tree().GetFrameByName(name, model_instance);
  }

  /// Returns a list of frame indices associated with `model_instance`.
  std::vector<FrameIndex> GetFrameIndices(
      ModelInstanceIndex model_instance) const {
    return internal_tree().GetFrameIndices(model_instance);
  }

  /// Returns the number of joint actuators in the model.
  /// @see AddJointActuator().
  int num_actuators() const { return internal_tree().num_actuators(); }

  /// Returns the number of actuators for a specific model instance.
  /// @throws std::exception if called pre-finalize.
  int num_actuators(ModelInstanceIndex model_instance) const {
    return internal_tree().num_actuators(model_instance);
  }

  /// Returns the total number of actuated degrees of freedom.
  /// That is, the vector of actuation values u has this size.
  /// See AddJointActuator().
  int num_actuated_dofs() const { return internal_tree().num_actuated_dofs(); }

  /// Returns the total number of actuated degrees of freedom for a specific
  /// model instance.  That is, the vector of actuation values u has this size.
  /// See AddJointActuator().
  /// @throws std::exception if called pre-finalize.
  int num_actuated_dofs(ModelInstanceIndex model_instance) const {
    return internal_tree().num_actuated_dofs(model_instance);
  }

  /// Returns true if plant has a joint actuator with unique index
  /// `actuator_index`. The value could be false if the actuator was removed
  /// using RemoveJointActuator().
  bool has_joint_actuator(JointActuatorIndex actuator_index) const {
    return internal_tree().has_joint_actuator(actuator_index);
  }

  /// Returns a constant reference to the joint actuator with unique index
  /// `actuator_index`.
  /// @throws std::exception if `actuator_index` does not correspond to a joint
  /// actuator in this tree.
  const JointActuator<T>& get_joint_actuator(
      JointActuatorIndex actuator_index) const {
    return internal_tree().get_joint_actuator(actuator_index);
  }

  /// Returns a mutable reference to the joint actuator with unique index
  /// `actuator_index`.
  /// @throws std::exception if `actuator_index` does not correspond to a joint
  /// actuator in this tree.
  JointActuator<T>& get_mutable_joint_actuator(
      JointActuatorIndex actuator_index) {
    return mutable_tree().get_mutable_joint_actuator(actuator_index);
  }

  /// @returns `true` if an actuator named `name` was added to this model.
  /// @see AddJointActuator().
  /// @throws std::exception if the actuator name occurs in multiple model
  /// instances.
  bool HasJointActuatorNamed(std::string_view name) const {
    return internal_tree().HasJointActuatorNamed(name);
  }

  /// @returns `true` if an actuator named `name` was added to
  /// @p model_instance.
  /// @see AddJointActuator().
  /// @throws std::exception if @p model_instance is not valid for this model.
  bool HasJointActuatorNamed(std::string_view name,
                             ModelInstanceIndex model_instance) const {
    return internal_tree().HasJointActuatorNamed(name, model_instance);
  }

  /// Returns a constant reference to an actuator that is identified
  /// by the string `name` in `this` %MultibodyPlant.
  /// @throws std::exception if there is no actuator with the requested name.
  /// @throws std::exception if the actuator name occurs in multiple model
  /// instances.
  /// @see HasJointActuatorNamed() to query if there exists an actuator in
  /// `this` %MultibodyPlant with a given specified name.
  const JointActuator<T>& GetJointActuatorByName(std::string_view name) const {
    return internal_tree().GetJointActuatorByName(name);
  }

  /// Returns a constant reference to the actuator that is uniquely identified
  /// by the string `name` and @p model_instance in `this` %MultibodyPlant.
  /// @throws std::exception if there is no actuator with the requested name.
  /// @throws std::exception if @p model_instance is not valid for this model.
  /// @see HasJointActuatorNamed() to query if there exists an actuator in
  /// `this` %MultibodyPlant with a given specified name.
  const JointActuator<T>& GetJointActuatorByName(
      std::string_view name, ModelInstanceIndex model_instance) const {
    return internal_tree().GetJointActuatorByName(name, model_instance);
  }

  /// Returns the number of ForceElement objects.
  /// @see AddForceElement().
  int num_force_elements() const {
    return internal_tree().num_force_elements();
  }

  /// Returns a constant reference to the force element with unique index
  /// `force_element_index`.
  /// @throws std::exception when `force_element_index` does not correspond
  /// to a force element in this model.
  const ForceElement<T>& get_force_element(
      ForceElementIndex force_element_index) const {
    return internal_tree().get_force_element(force_element_index);
  }

  /// Returns a constant reference to a force element identified by its unique
  /// index in `this` %MultibodyPlant.  If the optional template argument is
  /// supplied, then the returned value is downcast to the specified
  /// `ForceElementType`.
  /// @tparam ForceElementType The specific type of the ForceElement to be
  /// retrieved. It must be a subclass of ForceElement.
  /// @throws std::exception if the force element is not of type
  /// `ForceElementType` or if there is no ForceElement with that index.
  template <template <typename> class ForceElementType = ForceElement>
  const ForceElementType<T>& GetForceElement(
      ForceElementIndex force_element_index) const {
    return internal_tree().template GetForceElement<ForceElementType>(
        force_element_index);
  }

  /// @returns `true` iff gravity is enabled for `model_instance`.
  /// @see set_gravity_enabled().
  /// @throws std::exception if the model instance is invalid.
  bool is_gravity_enabled(ModelInstanceIndex model_instance) const;

  /// Sets is_gravity_enabled() for `model_instance` to `is_enabled`.
  /// The effect of `is_enabled = false` is effectively equivalent to disabling
  /// (or making zero) gravity for all bodies in the specified model instance.
  /// By default is_gravity_enabled() equals `true` for all model instances.
  /// @throws std::exception if called post-finalize.
  /// @throws std::exception if the model instance is invalid.
  void set_gravity_enabled(ModelInstanceIndex model_instance, bool is_enabled);

  /// An accessor to the current gravity field.
  const UniformGravityFieldElement<T>& gravity_field() const {
    return internal_tree().gravity_field();
  }

  // TODO(amastro-tri): mutation of the model should only be allowed
  // pre-finalize.
  /// A mutable accessor to the current gravity field.
  UniformGravityFieldElement<T>& mutable_gravity_field() {
    return this->mutable_tree().mutable_gravity_field();
  }

  /// Returns the number of model instances in the model.
  /// @see AddModelInstance().
  int num_model_instances() const {
    return internal_tree().num_model_instances();
  }

  /// Returns the name of a `model_instance`.
  /// @throws std::exception when `model_instance` does not correspond to a
  /// model in this model.
  const std::string& GetModelInstanceName(
      ModelInstanceIndex model_instance) const {
    return internal_tree().GetModelInstanceName(model_instance);
  }

  /// @returns `true` if a model instance named `name` was added to this model.
  /// @see AddModelInstance().
  bool HasModelInstanceNamed(std::string_view name) const {
    return internal_tree().HasModelInstanceNamed(name);
  }

  /// Returns the index to the model instance that is uniquely identified
  /// by the string `name` in `this` %MultibodyPlant.
  /// @throws std::exception if there is no instance with the requested name.
  /// @see HasModelInstanceNamed() to query if there exists an instance in
  /// `this` %MultibodyPlant with a given specified name.
  ModelInstanceIndex GetModelInstanceByName(std::string_view name) const {
    return internal_tree().GetModelInstanceByName(name);
  }

  /// Returns a Graphviz string describing the topology of this plant.
  /// To render the string, use the Graphviz tool, `dot`.
  /// http://www.graphviz.org/
  ///
  /// Note: this method can be called either before or after `Finalize()`.
  std::string GetTopologyGraphvizString() const;

  /// Returns the size of the generalized position vector q for this model.
  /// @throws std::exception if called pre-finalize.
  int num_positions() const { return internal_tree().num_positions(); }

  /// Returns the size of the generalized position vector qᵢ for model
  /// instance i.
  /// @throws std::exception if called pre-finalize.
  int num_positions(ModelInstanceIndex model_instance) const {
    return internal_tree().num_positions(model_instance);
  }

  /// Returns the size of the generalized velocity vector v for this model.
  /// @throws std::exception if called pre-finalize.
  int num_velocities() const { return internal_tree().num_velocities(); }

  /// Returns the size of the generalized velocity vector vᵢ for model
  /// instance i.
  /// @throws std::exception if called pre-finalize.
  int num_velocities(ModelInstanceIndex model_instance) const {
    return internal_tree().num_velocities(model_instance);
  }

  // N.B. The state in the Context may at some point contain values such as
  // integrated power and other discrete states, hence the specific name.
  /// Returns the size of the multibody system state vector x = [q v]. This
  /// will be `num_positions()` plus `num_velocities()`.
  /// @throws std::exception if called pre-finalize.
  int num_multibody_states() const { return internal_tree().num_states(); }

  /// Returns the size of the multibody system state vector xᵢ = [qᵢ vᵢ] for
  /// model instance i. (Here qᵢ ⊆ q and vᵢ ⊆ v.)
  /// will be `num_positions(model_instance)` plus
  /// `num_velocities(model_instance)`.
  /// @throws std::exception if called pre-finalize.
  int num_multibody_states(ModelInstanceIndex model_instance) const {
    return internal_tree().num_states(model_instance);
  }

  /// Returns a vector of size `num_positions()` containing the lower position
  /// limits for every generalized position coordinate. These include joint and
  /// free body coordinates. Any unbounded or unspecified limits will be
  /// -infinity.
  /// @throws std::exception if called pre-finalize.
  VectorX<double> GetPositionLowerLimits() const {
    return internal_tree().GetPositionLowerLimits();
  }

  /// Upper limit analog of GetPositionLowerLimits(), where any unbounded or
  /// unspecified limits will be +infinity.
  /// @see GetPositionLowerLimits() for more information.
  VectorX<double> GetPositionUpperLimits() const {
    return internal_tree().GetPositionUpperLimits();
  }

  /// Returns a vector of size `num_velocities()` containing the lower velocity
  /// limits for every generalized velocity coordinate. These include joint and
  /// free body coordinates. Any unbounded or unspecified limits will be
  /// -infinity.
  /// @throws std::exception if called pre-finalize.
  VectorX<double> GetVelocityLowerLimits() const {
    return internal_tree().GetVelocityLowerLimits();
  }

  /// Upper limit analog of GetVelocitysLowerLimits(), where any unbounded or
  /// unspecified limits will be +infinity.
  /// @see GetVelocityLowerLimits() for more information.
  VectorX<double> GetVelocityUpperLimits() const {
    return internal_tree().GetVelocityUpperLimits();
  }

  /// Returns a vector of size `num_velocities()` containing the lower
  /// acceleration limits for every generalized velocity coordinate. These
  /// include joint and free body coordinates. Any unbounded or unspecified
  /// limits will be -infinity.
  /// @throws std::exception if called pre-finalize.
  VectorX<double> GetAccelerationLowerLimits() const {
    return internal_tree().GetAccelerationLowerLimits();
  }

  /// Upper limit analog of GetAccelerationsLowerLimits(), where any unbounded
  /// or unspecified limits will be +infinity.
  /// @see GetAccelerationLowerLimits() for more information.
  VectorX<double> GetAccelerationUpperLimits() const {
    return internal_tree().GetAccelerationUpperLimits();
  }

  /// Returns a vector of size `num_actuated_dofs()` containing the lower effort
  /// limits for every actuator. Any unbounded or unspecified limits will be
  /// -∞. The returned vector is indexed by @ref JointActuatorIndex, see
  /// JointActuator::index().
  /// @see GetEffortUpperLimits()
  /// @throws std::exception if called pre-finalize.
  VectorX<double> GetEffortLowerLimits() const {
    DRAKE_MBP_THROW_IF_NOT_FINALIZED();
    return internal_tree().GetEffortLowerLimits();
  }

  /// Returns a vector of size `num_actuated_dofs()` containing the upper effort
  /// limits for every actuator. Any unbounded or unspecified limits will be
  /// +∞. The returned vector is indexed by @ref JointActuatorIndex, see
  /// JointActuator::index().
  /// @see GetEffortLowerLimits()
  /// @throws std::exception if called pre-finalize.
  VectorX<double> GetEffortUpperLimits() const {
    DRAKE_MBP_THROW_IF_NOT_FINALIZED();
    return internal_tree().GetEffortUpperLimits();
  }

  /// Returns the model used for contact. See documentation for ContactModel.
  ContactModel get_contact_model() const;

  /// Returns the number of geometries registered for visualization.
  /// This method can be called at any time during the lifetime of `this` plant,
  /// either pre- or post-finalize, see Finalize().
  /// Post-finalize calls will always return the same value.
  int num_visual_geometries() const { return num_visual_geometries_; }

  /// Returns the number of geometries registered for contact modeling.
  /// This method can be called at any time during the lifetime of `this` plant,
  /// either pre- or post-finalize, see Finalize().
  /// Post-finalize calls will always return the same value.
  int num_collision_geometries() const;

  /// Returns the unique id identifying `this` plant as a source for a
  /// SceneGraph.
  /// Returns `nullopt` if `this` plant did not register any geometry.
  /// This method can be called at any time during the lifetime of `this` plant
  /// to query if `this` plant has been registered with a SceneGraph, either
  /// pre- or post-finalize, see Finalize(). However, a geometry::SourceId is
  /// only assigned once at the first call of any of this plant's geometry
  /// registration methods, and it does not change after that.
  /// Post-finalize calls will always return the same value.
  std::optional<geometry::SourceId> get_source_id() const { return source_id_; }

  /// Returns `true` if `this` %MultibodyPlant was registered with a
  /// SceneGraph.
  /// This method can be called at any time during the lifetime of `this` plant
  /// to query if `this` plant has been registered with a SceneGraph, either
  /// pre- or post-finalize, see Finalize().
  bool geometry_source_is_registered() const {
    if (source_id_) {
      if (!is_finalized()) {
        DRAKE_DEMAND(scene_graph_ != nullptr);
      }
      return true;
    } else {
      return false;
    }
  }

  // Note: As discussed in #18351, we've used CamelCase here in case we need to
  // change its implementation down the road to a more expensive lookup.
  /// (Internal use only) Returns a mutable pointer to the SceneGraph that this
  /// plant is registered as a source for. This method can only be used
  /// pre-Finalize.
  ///
  /// @throws std::exception if is_finalized() == true ||
  ///           geometry_source_is_registered() == false
  geometry::SceneGraph<T>* GetMutableSceneGraphPreFinalize() {
    DRAKE_THROW_UNLESS(!is_finalized());
    DRAKE_THROW_UNLESS(geometry_source_is_registered());
    return scene_graph_;
  }

  /// (Internal use only) Provides access to the internal::LinkJointGraph.
  /// You can use graph().forest() to access the as-built
  /// internal::SpanningForest if you've already called Finalize().
  const internal::LinkJointGraph& graph() const {
    return internal_tree().graph();
  }

  /// @} <!-- Introspection -->

#ifndef DRAKE_DOXYGEN_CXX
  // Internal-only access to LinkJointGraph::FindSubgraphsOfWeldedBodies();
  // TODO(calderpg-tri) Properly expose this method (docs/tests/bindings).
  std::vector<std::set<BodyIndex>> FindSubgraphsOfWeldedBodies() const;

#endif

  using internal::MultibodyTreeSystem<T>::is_discrete;
  using internal::MultibodyTreeSystem<T>::EvalPositionKinematics;
  using internal::MultibodyTreeSystem<T>::EvalVelocityKinematics;

 private:
  using internal::MultibodyTreeSystem<T>::internal_tree;
  using internal::MultibodyTreeSystem<T>::mutable_tree;

  // Allow different specializations to access each other's private data for
  // scalar conversion.
  template <typename U>
  friend class MultibodyPlant;

  // Friend class to facilitate testing.
  friend class MultibodyPlantTester;

  // Friend attorney class to provide private access to those internal::
  // implementations that need it.
  friend class internal::MultibodyPlantDiscreteUpdateManagerAttorney<T>;
  friend class internal::MultibodyPlantIcfAttorney<T>;
  friend class internal::MultibodyPlantModelAttorney<T>;

  // This struct stores in one single place the index of all of our inputs.
  // The order of the items matches our Doxygen system overview figure.
  // Unless otherwise noted, all ports are declared during Finalize().
  struct InputPortIndices {
    systems::InputPortIndex actuation;
    systems::InputPortIndex applied_generalized_force;
    systems::InputPortIndex applied_spatial_force;
    struct Instance {
      systems::InputPortIndex actuation;
      systems::InputPortIndex desired_state;
    };
    std::vector<Instance> instance;
    systems::InputPortIndex geometry_query;  // Declared in ctor, not Finalize.
  };

  // This struct stores in one single place the index of all of our outputs.
  // The order of the items matches our Doxygen system overview figure.
  // Unless otherwise noted, all ports are declared during Finalize().
  struct OutputPortIndices {
    systems::OutputPortIndex state;
    systems::OutputPortIndex body_poses;
    systems::OutputPortIndex body_spatial_velocities;
    systems::OutputPortIndex body_spatial_accelerations;
    systems::OutputPortIndex generalized_acceleration;
    systems::OutputPortIndex net_actuation;
    systems::OutputPortIndex reaction_forces;
    systems::OutputPortIndex contact_results;
    struct Instance {
      systems::OutputPortIndex state;
      systems::OutputPortIndex generalized_acceleration;
      systems::OutputPortIndex generalized_contact_forces;
      systems::OutputPortIndex net_actuation;
    };
    std::vector<Instance> instance;
    systems::OutputPortIndex geometry_pose;  // Declared in ctor, not Finalize.
    // N.B. The deformable_body_configuration port is owned by DeformableModel,
    // so is not tracked here.
  };

  // This struct stores in one single place all indices related to specific
  // MultibodyPlant cache entries. These are initialized at Finalize().
  struct CacheIndices {
    systems::CacheIndex geometry_contact_data;
    systems::CacheIndex joint_locking;
    systems::CacheIndex actuation_input_without_effort_limit;
    systems::CacheIndex actuation_input_with_effort_limit;
    systems::CacheIndex desired_state_input;

    // This is only valid for a continuous-time, hydroelastic-contact plant.
    systems::CacheIndex hydroelastic_contact_forces_continuous;

    // These are only valid for a continuous-time plant.
    systems::CacheIndex contact_results_point_pair_continuous;
    systems::CacheIndex spatial_contact_forces_continuous;
    systems::CacheIndex generalized_contact_forces_continuous;
  };

  // This struct stores in one single place all indices related to
  // MultibodyPlant parameters. These are initialized at Finalize()
  // when the plant declares parameters.
  struct ParameterIndices {
    systems::AbstractParameterIndex constraint_active_status;
    systems::AbstractParameterIndex distance_constraints;
  };

  // Constructor to bridge testing from MultibodyTree to MultibodyPlant.
  // WARNING: This may *not* result in a plant with a valid state. Use
  // sparingly to test forwarding methods when the overhead is high to
  // reproduce the testing (e.g. benchmarks).
  explicit MultibodyPlant(std::unique_ptr<internal::MultibodyTree<T>> tree_in,
                          double time_step = 0);

  // Helper method for throwing an exception within public methods that should
  // not be called post-finalize. The invoking method should pass its name so
  // that the error message can include that detail.
  void ThrowIfFinalized(const char* source_method) const;

  // Helper method for throwing an exception within public methods that should
  // not be called pre-finalize. The invoking method should pass it's name so
  // that the error message can include that detail.
  void ThrowIfNotFinalized(const char* source_method) const;

  // Returns `true` if the vector `v` contains only finite values.
  // @param v The vector to test.
  static boolean<T> AllFinite(const Eigen::Ref<const VectorX<T>>& v) {
    return all_of(v, [](const T& t) {
      using std::isfinite;
      return isfinite(t);
    });
  }

  // Helper method that is used to finalize the plant's internals after
  // MultibodyTree::Finalize() was called.
  void FinalizePlantOnly();

  // Consolidates calls to Eval on the geometry query input port to have a
  // consistent and helpful error message in the situation where the
  // geometry_query_input_port is not connected, but is expected to be.
  //
  // Public APIs that ultimately depend on the query object input port should
  // invoke ValidateGeometryInput() to guard against failed access in the depths
  // of the code. As a safety net, all invocations of *this* method should
  // pass the function that invoked it (via __func__), so that if any usage
  // slips through the curated net, some insight will be provided as to what was
  // attempting to access the disconnected port.
  const geometry::QueryObject<T>& EvalGeometryQueryInput(
      const systems::Context<T>& context, std::string_view caller) const;

  // These functions provide a mechanism to provide early warning when a
  // calculation depends on the QueryObject input port. The goal is to provide
  // as much feedback to the caller as to *why* the input port is required.
  // Therefore, it should be called as "high" in the callstack as possible with
  // an explanation of the need (e.g., computing forward dynamics).
  //
  // Note: the connection is only tested if MbP knows about collision
  // geometries. This is correlated with the fact that the operations that
  // depend on the geometry input port only do so based on that same condition.
  void ValidateGeometryInput(const systems::Context<T>& context,
                             std::string_view explanation) const;

  // A validation overload that automatically constructs an explanation when
  // the reason is due to evaluating an output port.
  void ValidateGeometryInput(const systems::Context<T>& context,
                             const systems::OutputPort<T>& output_port) const;

  // Reports if the geometry input is "valid", i.e., either unnecessary or
  // connected.
  bool IsValidGeometryInput(const systems::Context<T>& context) const;

  // Helper to acquire per-geometry contact parameters from SG.
  // Returns the pair (stiffness, dissipation)
  // Defaults to heuristically computed parameter if the given geometry
  // isn't assigned that parameter.
  std::pair<T, T> GetPointContactParameters(
      geometry::GeometryId id,
      const geometry::SceneGraphInspector<T>& inspector) const;

  // Helper to acquire per-geometry Coulomb friction coefficients from
  // SceneGraph.
  const CoulombFriction<double>& GetCoulombFriction(
      geometry::GeometryId id,
      const geometry::SceneGraphInspector<T>& inspector) const;

  // Helper method to apply default collision filters. By default, we don't
  // consider collisions:
  // * between rigid geometries affixed to bodies connected by a joint
  // * within subgraphs of welded-together rigid bodies
  // Note that collisions involving deformable bodies are not filtered by
  // default.
  void ApplyDefaultCollisionFilters();

  // For discrete models, MultibodyPlant uses a penalty method to impose joint
  // limits. In this penalty method a force law of the form:
  //   τ = -k(q - qᵤ) - cv if q > qᵤ
  //   τ = -k(q - qₗ) - cv if q < qₗ
  // is used to limit the position q to be within the lower/upper limits
  // (qₗ, qᵤ).
  // The penalty parameters k (stiffness) and c (damping) are estimated using
  // a harmonic oscillator model of the form ẍ + 2ζω₀ ẋ + ω₀² x = 0, with
  // x = (q - qᵤ) near the upper limit when q > qᵤ and x = (q - qₗ) near the
  // lower limit when q < qₗ and where ω₀² = k / m̃ is the characteristic
  // numerical stiffness frequency and m̃ is an inertia term that for prismatic
  // joints reduces to a simple function of the mass of the bodies adjacent to
  // a particular joint. For revolute joints m̃ relates to the rotational inertia
  // of the adjacent bodies to a joint. See the implementation notes for further
  // details. Both ω₀ and ζ are non-negative numbers.
  // The characteristic frequency ω₀ is entirely a function the time step of the
  // discrete model so that, from a stability analysis of the simplified
  // harmonic oscillator model, we guarantee the resulting time stepping is
  // stable. That is, the numerical stiffness of the method is such that it
  // corresponds to the largest penalty parameter (smaller violation errors)
  // that still guarantees stability.
  void SetUpJointLimitsParameters();

  // Some constraints support std::optional specs, which implies that the
  // kinematics should be used to compute values such that the constraint is
  // satisfied by the default context at the moment Finalize() is called. This
  // method computes those values and stores them in the constraint specs.
  void FinalizeConstraints();

  // Declares any input ports that haven't yet been declared.
  // This happens during Finalize().
  void DeclareInputPorts();

  // Declares the system-level parameters specific to MultibodyPlant.
  // This happens during Finalize().
  void DeclareParameters();

  // Declares the system-level cache entries specific to MultibodyPlant.
  // This happens during Finalize().
  void DeclareCacheEntries();

  // Declares the state-update events.
  // This happens during Finalize().
  void DeclareStateUpdate();

  // Declares any input ports that haven't yet been declared.
  // This happens during Finalize().
  void DeclareOutputPorts();

  // Declares an output port that is sampled (or else direct feedthrough) based
  // on the use_sampled_output_ports_ option. Note that the name "declare
  // sampled ..." is slightly misleading: the port is sampled if and only if the
  // option is set to true.
  //
  // The `model_value` is passed along to the systems framework in support of
  // allocating storage for type erasure. If the port should be vector-valued
  // instead of abstract, pass an `int` for the `model_value`, containing the
  // size of the vector.
  //
  // The two `calc_...` callbacks should both be member function pointers,
  // typically to a member function templated on `<bool sampled>`. They should
  // accept the signature `(const Context<T>&, ModelValue*) const` or
  // `(ModelInstanceIndex, const Context<T>&, ModelValue*) const`.
  //
  // The `prerequisites_of_unsampled` provides the dependency tickets for
  // `calc_unsampled`. The dependency ticket for `calc_sampled` is always
  // just the DiscreteStepMemory abstract state.
  //
  // If the optional `ModelInstanceIndex model_instance` last argument is
  // given, then it will be passed along as the first argument of the calc
  // callbacks (in front of the context).
  template <typename ModelValue, typename CalcFunction,
            typename MaybeModelInstanceIndex = void*>
  DRAKE_NO_EXPORT systems::OutputPortIndex DeclareSampledOutputPort(
      const std::string& name, const ModelValue& model_value,
      const CalcFunction& calc_sampled, const CalcFunction& calc_unsampled,
      const std::set<systems::DependencyTicket>& prerequisites_of_unsampled,
      MaybeModelInstanceIndex model_instance = {});

  // Estimates a global set of point contact parameters given a
  // `penetration_allowance`. See `set_penetration_allowance()` for details.
  // TODO(amcastro-tri): Once #13064 is resolved, make this a method outside MBP
  // with signature:
  // EstimatePointContactParameters(double penetration_allowance,
  //                                MultibodyPlant<double>* plant)
  // We will document the heuristics used by this method thoroughly so that we
  // have a place we can refer users to for details.
  void EstimatePointContactParameters(double penetration_allowance);

  // Methods that assemble actuation input vector from the appropriate ports.
  // The actuation value for a particular actuator can be found at offset
  // JointActuator::input_start() in the returned vector (see
  // MultibodyPlant::get_actuation_input_port()). N.B. this does not include
  // actuation due to the desired_state input ports; this is only the
  // feedforward actuation. When `effort_limit` is true, the actuation will be
  // clamped by each joint's `effort_lmit()`; when false, the limit is ignored.
  const VectorX<T>& EvalActuationInput(const systems::Context<T>& context,
                                       bool effort_limit) const;
  void CalcActuationInputWithoutEffortLimit(const systems::Context<T>& context,
                                            VectorX<T>* actuation_input) const;
  void CalcActuationInputWithEffortLimit(const systems::Context<T>& context,
                                         VectorX<T>* actuation_input) const;

  // Calc method for the "net_actuation" output port.
  template <bool sampled>
  void CalcNetActuationOutput(const systems::Context<T>& context,
                              systems::BasicVector<T>* output) const;

  // Calc method for the "{model_instance_name}_net_actuation" output ports.
  template <bool sampled>
  void CalcInstanceNetActuationOutput(ModelInstanceIndex model_instance,
                                      const systems::Context<T>& context,
                                      systems::BasicVector<T>* output) const;

  // These methods evaluate the desired state input ports and return them as a
  // DesiredStateInput.
  const internal::DesiredStateInput<T>& EvalDesiredStateInput(
      const systems::Context<T>& context) const;
  void CalcDesiredStateInput(
      const systems::Context<T>& context,
      internal::DesiredStateInput<T>* desired_state_input) const;

  // Throws if the plant uses features not supported by continuous time
  // calculations.
  void ThrowIfUnsupportedContinuousTimeDynamics(
      const systems::Context<T>& context) const;

  // Computes all non-contact applied forces including:
  //  - Force elements.
  //  - Joint actuation.
  //  - Externally applied spatial forces.
  //  - Joint limits.
  // @pre The plant is continuous.
  void CalcNonContactForcesContinuous(const drake::systems::Context<T>& context,
                                      MultibodyForces<T>* forces) const;

  // Collects up forces from input ports (actuator, generalized, and spatial
  // forces) and contact forces (from compliant contact models). Does not
  // include ForceElement forces which are accounted for elsewhere.
  // @pre The plant is continuous.
  void AddInForcesContinuous(const systems::Context<T>& context,
                             MultibodyForces<T>* forces) const override;

  // Discrete system version of CalcForwardDynamics(). This method does not use
  // O(n) forward dynamics but a discrete solver according to the discrete
  // contact solver specified.
  // @see get_discrete_contact_solver().
  void DoCalcForwardDynamicsDiscrete(
      const drake::systems::Context<T>& context,
      internal::AccelerationKinematicsCache<T>* ac) const override;

  // If the plant is modeled as a discrete system with periodic updates (see
  // is_discrete()) with use_sampled_output_ports set to `false`, then this
  // method computes the periodic updates of the state using a semi-explicit
  // Euler strategy, that is:
  //   vⁿ⁺¹ = vⁿ + dt v̇ⁿ
  //   qⁿ⁺¹ = qⁿ + dt N(qⁿ) vⁿ⁺¹
  // This semi-explicit update inherits some of the nice properties of the
  // semi-implicit Euler scheme (which uses v̇ⁿ⁺¹ for the v updated instead) when
  // there are no velocity-dependent forces (including Coriolis and gyroscopic
  // terms). The semi-implicit Euler scheme is a symplectic integrator, which
  // for a Hamiltonian system has the nice property of nearly conserving energy
  // (in many cases we can write a "modified energy functional" which can be
  // shown to be exactly conserved and to be within O(dt) of the real energy of
  // the mechanical system.)
  // TODO(amcastro-tri): Update this docs when contact is added.
  systems::EventStatus CalcStepDiscrete(
      const systems::Context<T>& context0,
      systems::DiscreteValues<T>* next_discrete_state) const;

  // If the plant is modeled as a discrete system with periodic updates (see
  // is_discrete()) with use_sampled_output_ports set to `true`, then this
  // method computes the periodic updates of the state. The function performs
  // the same update as CalcStepDiscrete() does for the discrete values, but
  // also updates the abstract state that holds the sample.
  systems::EventStatus CalcStepUnrestricted(
      const systems::Context<T>& context0, systems::State<T>* next_state) const;

  // Accesses the AccelerationKinematicsCache for use by possibly-sampled
  // output port calculations:
  //
  // - When `sampled` is true, the result will be all-zero when the plant has
  //   not yet taken a step. Otherwise, it will refer to the sampled
  //   acceleration kinematics from the most recent step.
  //
  // - When `sampled` is false, the result will be an instantaneously up-to-date
  //   function of the current context.
  //
  // Note that MbTS::EvalForwardDynamics is the non-sampled flavor of this eval
  // function. When sampled is false, we just call that.
  template <bool sampled>
  const internal::AccelerationKinematicsCache<T>&
  EvalAccelerationKinematicsCacheForOutputPortCalc(
      const systems::Context<T>& context) const;

  // Data will be resized on output according to the documentation for
  // JointLockingCacheData.
  void CalcJointLocking(const systems::Context<T>& context,
                        internal::JointLockingCacheData<T>* data) const;

  // Eval version of the method CalcJointLocking().
  const internal::JointLockingCacheData<T>& EvalJointLocking(
      const systems::Context<T>& context) const {
    return this->get_cache_entry(cache_indices_.joint_locking)
        .template Eval<internal::JointLockingCacheData<T>>(context);
  }

  // Calc function for the like-named cache entry (geometry_contact_data).
  void CalcGeometryContactData(const systems::Context<T>& context,
                               internal::GeometryContactData<T>* result) const;

  // Eval function for the like-named cache entry (geometry_contact_data).
  const internal::GeometryContactData<T>& EvalGeometryContactData(
      const systems::Context<T>& context) const;

  // Helper method to fill in the ContactResults given the current context when
  // the model is continuous.
  // @param[out] contact_results is fully overwritten
  void CalcContactResultsContinuous(const systems::Context<T>& context,
                                    ContactResults<T>* contact_results) const;

  // Calc function for the like-named cache entry
  // (contact_results_point_pair_continuous).
  void CalcContactResultsPointPairContinuous(
      const systems::Context<T>& context,
      std::vector<PointPairContactInfo<T>>*
          contact_results_point_pair_continuous) const;

  // Eval function for the like-named cache entry
  // (contact_results_point_pair_continuous).
  const std::vector<PointPairContactInfo<T>>&
  EvalContactResultsPointPairContinuous(
      const systems::Context<T>& context) const;

  // Helper used by CalcContactResultsContinuous() to fill a ContactResults.
  // This function computes the continuous-time hydroelastic forces as a
  // function of the state stored in `context`.
  // @param[out] contact_results_hydroelastic is fully overwritten
  void CalcContactResultsHydroelasticContinuous(
      const systems::Context<T>& context,
      std::vector<HydroelasticContactInfo<T>>* contact_results_hydroelastic)
      const
    requires scalar_predicate<T>::is_bool;

  // Calc method for the "reaction_forces" output port.
  // This is responsible for handling the sampled vs unsampled branching logic.
  template <bool sampled>
  void CalcReactionForcesOutput(const systems::Context<T>& context,
                                std::vector<SpatialForce<T>>* output) const;

  // Helper method used by CalcReactionForcesOutput().
  // This is responsible for the actual (unsampled) computation of the dynamics.
  void CalcReactionForces(const systems::Context<T>& context,
                          std::vector<SpatialForce<T>>* output) const;

  // Collect joint actuator forces and externally provided spatial and
  // generalized forces.
  void AddInForcesFromInputPorts(const drake::systems::Context<T>& context,
                                 MultibodyForces<T>* forces) const;

  // Add contribution of generalized forces passed in through our
  // applied_generalized_force input port.
  void AddAppliedExternalGeneralizedForces(const systems::Context<T>& context,
                                           MultibodyForces<T>* forces) const;

  // Add contribution of body spatial forces passed in through our
  // applied_spatial_force input port.
  void AddAppliedExternalSpatialForces(const systems::Context<T>& context,
                                       MultibodyForces<T>* forces) const;

  // Add contribution of external actuation forces passed in through our
  // actuation input ports (there is a separate port for each model instance).
  void AddJointActuationForces(const systems::Context<T>& context,
                               VectorX<T>* forces) const;

  // Helper method to register geometry for a given body, either visual or
  // collision. The registration includes:
  // 1. Register geometry for the corresponding FrameId associated with `body`.
  // 2. Update the geometry_id_to_body_index_ map associating the new GeometryId
  //    to the BodyIndex of `body`.
  // This assumes:
  // 1. Finalize() was not called on `this` plant.
  // 2. RegisterAsSourceForSceneGraph() was called on `this` plant.
  // 3. `scene_graph` points to the same SceneGraph instance previously
  //    passed to RegisterAsSourceForSceneGraph().
  geometry::GeometryId RegisterGeometry(
      const RigidBody<T>& body,
      std::unique_ptr<geometry::GeometryInstance> instance);

  // Registers a geometry frame for every body. If the body already has a
  // geometry frame, it is unchanged. This registration is part of finalization.
  // This requires RegisterAsSourceForSceneGraph() was called on `this` plant.
  void RegisterGeometryFramesForAllBodies();

  bool body_has_registered_frame(const RigidBody<T>& body) const {
    return body_index_to_frame_id_.find(body.index()) !=
           body_index_to_frame_id_.end();
  }

  // Registers the given body with this plant's SceneGraph instance (if it has
  // one).
  void RegisterRigidBodyWithSceneGraph(const RigidBody<T>& body);

  // Calc method for the "state" output port.
  void CalcStateOutput(const systems::Context<T>& context,
                       systems::BasicVector<T>* output) const;

  // Calc method for the "{model_instance_name}_output" output ports.
  void CalcInstanceStateOutput(ModelInstanceIndex model_instance,
                               const systems::Context<T>& context,
                               systems::BasicVector<T>* output) const;

  // Calc method for the "{model_instance_name}_generalized_acceleration" output
  // ports.
  template <bool sampled>
  void CalcInstanceGeneralizedContactForcesOutput(
      ModelInstanceIndex model_instance, const systems::Context<T>& context,
      systems::BasicVector<T>* output) const;

  // Calc method for the "body_poses" output port.
  void CalcBodyPosesOutput(const systems::Context<T>& context,
                           std::vector<math::RigidTransform<T>>* output) const;

  // Calc method for the "body_spatial_velocities" output port.
  void CalcBodySpatialVelocitiesOutput(
      const systems::Context<T>& context,
      std::vector<SpatialVelocity<T>>* output) const;

  // Calc method for the "body_spatial_accelerations" output port.
  template <bool sampled>
  void CalcBodySpatialAccelerationsOutput(
      const systems::Context<T>& context,
      std::vector<SpatialAcceleration<T>>* output) const;

  // Calc method for the "generalized_acceleration" output port.
  template <bool sampled>
  void CalcGeneralizedAccelerationOutput(const systems::Context<T>& context,
                                         systems::BasicVector<T>* output) const;

  // Calc method for the "{model_instance_name}_generalized_acceleration"
  // output ports.
  template <bool sampled>
  void CalcInstanceGeneralizedAccelerationOutput(
      ModelInstanceIndex model_instance, const systems::Context<T>& context,
      systems::BasicVector<T>* output) const;

  // Method to compute spatial contact forces for continuous plants.
  // @note This version zeros out the forces in @p F_BBo_W_array before adding
  // in contact force.
  // @see CalcAndAddSpatialContactForcesContinuous() for the version of this
  // method that does not zero out the forces.
  void CalcSpatialContactForcesContinuous(
      const drake::systems::Context<T>& context,
      std::vector<SpatialForce<T>>* F_BBo_W_array) const;

  // Eval version of CalcSpatialContactForcesContinuous().
  const std::vector<SpatialForce<T>>& EvalSpatialContactForcesContinuous(
      const systems::Context<T>& context) const;

  // Method to compute spatial contact forces for continuous plants.
  // @note This version does *not* zero out the forces in @p F_BBo_W_array.
  // @see CalcSpatialContactForcesContinuous() for the version of this method
  // that zeros out @p F_BBo_W_array before adding in contact forces.
  void CalcAndAddSpatialContactForcesContinuous(
      const drake::systems::Context<T>& context,
      std::vector<SpatialForce<T>>* F_BBo_W_array) const;

  // Method to compute generalized contact forces for continuous plants.
  void CalcGeneralizedContactForcesContinuous(
      const drake::systems::Context<T>& context, VectorX<T>* tau_contact) const;

  // Eval version of CalcGeneralizedContactForcesContinuous().
  const VectorX<T>& EvalGeneralizedContactForcesContinuous(
      const systems::Context<T>& context) const;

  // Helper method to declare output ports used by this plant to communicate
  // with a SceneGraph.
  void DeclareSceneGraphPorts();

  // Calc method for the "geometry_pose" output port.
  void CalcGeometryPoseOutput(const systems::Context<T>& context,
                              geometry::FramePoseVector<T>* output) const;

  // Calc method for the "contact_results" output port.
  template <bool sampled>
  void CalcContactResultsOutput(const systems::Context<T>& context,
                                ContactResults<T>* output) const;

  // (Advanced) Helper method to compute contact forces in the normal direction
  // using a penalty method.
  void CalcAndAddPointContactForcesContinuous(
      const systems::Context<T>& context,
      std::vector<SpatialForce<T>>* F_BBo_W_array) const;

  // Helper method to compute continuous-time contact forces using the
  // hydroelastic model for continuous plants.
  void CalcHydroelasticContactForcesContinuous(
      const systems::Context<T>& context,
      internal::HydroelasticContactForcesContinuousCacheData<T>* output) const
    requires scalar_predicate<T>::is_bool;

  // Eval version of CalcHydroelasticContactForces().
  const internal::HydroelasticContactForcesContinuousCacheData<T>&
  EvalHydroelasticContactForcesContinuous(
      const systems::Context<T>& context) const
    requires scalar_predicate<T>::is_bool;

  // Helper method to apply penalty forces that enforce joint limits.
  // At each joint with joint limits this penalty method applies a force law of
  // the form:
  //   τ = min(-k(q - qᵤ) - cv, 0) if q > qᵤ
  //   τ = max(-k(q - qₗ) - cv, 0) if q < qₗ
  // is used to limit the position q to be within the lower/upper limits
  // (qₗ, qᵤ).
  // The penalty parameters k (stiffness) and c (damping) are estimated using
  // a harmonic oscillator model within SetUpJointLimitsParameters().
  void AddJointLimitsPenaltyForces(const systems::Context<T>& context,
                                   MultibodyForces<T>* forces) const;

  // Given a GeometryId, return the corresponding BodyIndex or throw if the
  // GeometryId is invalid or unknown to this plant.
  BodyIndex FindBodyByGeometryId(geometry::GeometryId) const;

  // Gets the parameter corresponding to constraint active status.
  const std::map<MultibodyConstraintId, bool>& GetConstraintActiveStatus(
      const systems::Context<T>& context) const {
    return context.get_parameters()
        .template get_abstract_parameter<internal::ConstraintActiveStatusMap>(
            parameter_indices_.constraint_active_status)
        .map;
  }

  // Gets the mutable parameter corresponding to constraint active status.
  std::map<MultibodyConstraintId, bool>& GetMutableConstraintActiveStatus(
      systems::Context<T>* context) const {
    return context->get_mutable_parameters()
        .template get_mutable_abstract_parameter<
            internal::ConstraintActiveStatusMap>(
            parameter_indices_.constraint_active_status)
        .map;
  }

  // Helper to get mutable parameters for all distance constraints.
  std::map<MultibodyConstraintId, DistanceConstraintParams>&
  GetMutableDistanceConstraintParams(systems::Context<T>* context) const;

  // Removes `this` MultibodyPlant's ability to convert to the scalar types
  // unsupported by the given `component`.
  void RemoveUnsupportedScalars(
      const internal::ScalarConvertibleComponent<T>& component);

  // Adds a DeformableModel to this plant. The added DeformableModel is owned
  // by `this` MultibodyPlant and calls its `DeclareSystemResources()` method
  // when `this` MultibodyPlant is finalized to declare the system resources it
  // needs.
  // @returns a mutable reference to the added DeformableModel that's valid for
  // the life time of this MultibodyPlant.
  // @throws std::exception if called post-finalize. See Finalize().
  // @throws std::exception if a DeformableModel is already added.
  // @note DeformableModel only meaningfully supports double as a scalar type.
  // Adding a non-double DeformableModel is allowed, but registering
  // deformable bodies with non-double scalar types is not supported yet.
  DeformableModel<T>& AddDeformableModel();

  // Geometry source identifier for this system to interact with geometry
  // system. It is made optional for plants that do not register geometry
  // (dynamics only).
  std::optional<geometry::SourceId> source_id_{std::nullopt};

  internal::ContactByPenaltyMethodParameters penalty_method_contact_parameters_;

  // Penetration allowance used to estimate ContactByPenaltyMethodParameters.
  // See set_penetration_allowance() for details.
  double penetration_allowance_{MultibodyPlantConfig{}.penetration_allowance};

  // Stribeck model of friction.
  class StribeckModel {
   public:
    DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(StribeckModel);

    /// Creates an uninitialized Stribeck model with an invalid value (negative)
    /// of the stiction tolerance.
    StribeckModel() = default;

    /// Computes the friction coefficient based on the tangential *speed*
    /// `speed_BcAc` of the contact point `Ac` on A relative to the
    /// contact point `Bc` on B. That is, `speed_BcAc = ‖vt_BcAc‖`, where
    /// `vt_BcAc` is the tangential component of the velocity `v_BcAc` of
    /// contact point `Ac` relative to point `Bc`.
    ///
    /// See contact_model_doxygen.h @section tangent_force for details.
    T ComputeFrictionCoefficient(const T& speed_BcAc,
                                 const CoulombFriction<double>& friction) const;

    /// Evaluates an S-shaped quintic curve, f(x), mapping the domain [0, 1] to
    /// the range [0, 1] where f(0) = f''(0) = f''(1) = f'(0) = f'(1) = 0 and
    /// f(1) = 1.
    static T step5(const T& x);

    /// Sets the stiction tolerance `v_stiction` for the Stribeck model, where
    /// `v_stiction` must be specified in m/s (meters per second.)
    /// @throws std::exception if `v_stiction` is non-positive.
    void set_stiction_tolerance(double v_stiction) {
      DRAKE_THROW_UNLESS(v_stiction > 0);
      v_stiction_tolerance_ = v_stiction;
      inv_v_stiction_tolerance_ = 1.0 / v_stiction;
    }

    /// Returns the value of the stiction tolerance for `this` %MultibodyPlant.
    /// It returns a negative value when the stiction tolerance has not been set
    /// previously with set_stiction_tolerance().
    double stiction_tolerance() const { return v_stiction_tolerance_; }

   private:
    // Stiction velocity tolerance for the Stribeck model.
    double v_stiction_tolerance_{MultibodyPlantConfig{}.stiction_tolerance};
    // Note: this is the *inverse* of the v_stiction_tolerance_ parameter to
    // optimize for the division.
    double inv_v_stiction_tolerance_{1.0 / v_stiction_tolerance_};
  };
  StribeckModel friction_model_;

  // This structure aids in the bookkeeping of parameters associated with joint
  // limits and the penalty method parameters used to enforce them.
  struct JointLimitsParameters {
    // list of joints that have limits. These are all single-dof joints.
    std::vector<JointIndex> joints_with_limits;
    // Position lower/upper bounds for each joint in joints_with_limits. The
    // Units depend on the particular joint type. For instance, radians for
    // RevoluteJoint or meters for PrismaticJoint.
    std::vector<double> lower_limit;
    std::vector<double> upper_limit;
    // Penalty parameters. These are defined in accordance to the penalty force
    // internally implemented by MultibodyPlant in
    // AddJointLimitsPenaltyForces().
    std::vector<double> stiffness;
    std::vector<double> damping;
    // If these joint limits will be ignored (because the plant uses continuous
    // time) and we have not yet warned the user about that fact, this contains
    // the warning message to be printed. Marked mutable because it's not part
    // of our dynamics, so that we can clear it from a const method.
    mutable std::string pending_warning_message;
  } joint_limits_parameters_;

  // Iteration order on this map DOES matter, and therefore we use an std::map.
  std::map<BodyIndex, geometry::FrameId> body_index_to_frame_id_;

  // Data to get back from a SceneGraph-reported frame id to its associated
  // body.
  std::unordered_map<geometry::FrameId, BodyIndex> frame_id_to_body_index_;

  // Map from GeometryId to BodyIndex. During contact queries, it allows to find
  // out to which body a given geometry corresponds to.
  std::unordered_map<geometry::GeometryId, BodyIndex>
      geometry_id_to_body_index_;

  // Per-body arrays of visual geometries indexed by BodyIndex.
  // That is, visual_geometries_[body_index] corresponds to the array of visual
  // geometries for body with index body_index.
  std::vector<std::vector<geometry::GeometryId>> visual_geometries_;

  // The total number of GeometryId values within visual_geometries_.
  int num_visual_geometries_{0};

  // Per-body arrays of collision geometries indexed by BodyIndex.
  // That is, collision_geometries_[body_index] corresponds to the array of
  // collision geometries for body with index body_index.
  std::vector<std::vector<geometry::GeometryId>> collision_geometries_;

  // The total number of GeometryId values within collision_geometries_.
  int num_collision_geometries_{0};

  // The model used by the plant to compute contact forces. Keep this in sync
  // with the default value in multibody_plant_config.h; there are already
  // assertions in the cc file that enforce this.
  ContactModel contact_model_{ContactModel::kHydroelasticWithFallback};

  // The contact model approximation used by discrete MultibodyPlant models.
  DiscreteContactApproximation discrete_contact_approximation_{
      DiscreteContactApproximation::kLagged};

  // Near rigid regime parameter from [Castro et al., 2021]. Refer to
  // set_near_rigid_threshold() for details.
  double sap_near_rigid_threshold_{
      MultibodyPlantConfig{}.sap_near_rigid_threshold};

  // User's choice of the representation of contact surfaces in discrete
  // systems. The default value is dependent on whether the system is
  // continuous or discrete, so the constructor will set it. See
  // GetDefaultContactSurfaceRepresentation().
  geometry::HydroelasticContactRepresentation contact_surface_representation_{};

  // For geometry registration with a GS, we save a pointer to the GS instance
  // on which this plants calls RegisterAsSourceForSceneGraph(). This will be
  // set to `nullptr` after finalization, to mirror constraints presented by
  // scalar conversion (where we cannot easily obtain a reference to the
  // scalar-converted scene graph).
  geometry::SceneGraph<T>* scene_graph_{nullptr};

  // If the plant is modeled as a discrete system with periodic updates,
  // time_step_ corresponds to the period of those updates. Otherwise, if the
  // plant is modeled as a continuous system, it is exactly zero.
  double time_step_{0};

  bool use_sampled_output_ports_{};

  // This manager class is used to advance discrete states.
  // Post-finalize, it is never null (for a discrete-time plant).
  // TODO(amcastro-tri): migrate the entirety of computations related to contact
  // resolution into a default contact manager.
  std::unique_ptr<internal::DiscreteUpdateManager<T>> discrete_update_manager_;

  // (Experimental) The collection of all physical models owned by
  // this MultibodyPlant.
  std::unique_ptr<internal::PhysicalModelCollection<T>> physical_models_{
      std::make_unique<internal::PhysicalModelCollection<T>>()};

  // Map of coupler constraints specifications.
  std::map<MultibodyConstraintId, internal::CouplerConstraintSpec>
      coupler_constraints_specs_;

  // Map of default distance constraints parameters.
  std::map<MultibodyConstraintId, DistanceConstraintParams>
      distance_constraints_params_;

  // Map of ball constraint specifications.
  std::map<MultibodyConstraintId, internal::BallConstraintSpec>
      ball_constraints_specs_;

  // Map of weld constraint specifications.
  std::map<MultibodyConstraintId, internal::WeldConstraintSpec>
      weld_constraints_specs_;

  // Map of tendon constraint specifications.
  std::map<MultibodyConstraintId, internal::TendonConstraintSpec>
      tendon_constraints_specs_;

  // Whether to apply collsion filters to adjacent bodies at Finalize().
  bool adjacent_bodies_collision_filters_{
      MultibodyPlantConfig{}.adjacent_bodies_collision_filters};

  // When use_sampled_output_ports_ is true, then during Finalize() we populate
  // this with all-zero data, for use when the State has not yet been stepped.
  std::unique_ptr<internal::AccelerationKinematicsCache<T>>
      zero_acceleration_kinematics_placeholder_;

  InputPortIndices input_port_indices_;
  OutputPortIndices output_port_indices_;
  CacheIndices cache_indices_;
  ParameterIndices parameter_indices_;
};

/// @cond
// Undef macros defined at the top of the file. From the GSG:
// "Exporting macros from headers (i.e. defining them in a header without
// #undefing them before the end of the header) is extremely strongly
// discouraged."
// This will require us to re-define them in the .cc file.
#undef DRAKE_MBP_THROW_IF_FINALIZED
#undef DRAKE_MBP_THROW_IF_NOT_FINALIZED
/// @endcond

// Forward declare to permit exclusive friendship for construction.
template <typename T>
struct AddMultibodyPlantSceneGraphResult;

/// Makes a new MultibodyPlant with discrete update period `time_step` and
/// adds it to a diagram builder together with the provided SceneGraph instance,
/// connecting the geometry ports.
/// @note Usage examples in @ref add_multibody_plant_scene_graph
/// "AddMultibodyPlantSceneGraph".
///
/// @param[in,out] builder
///   Builder to add to.
/// @param[in] time_step
///   The discrete update period for the new MultibodyPlant to be added.
///   Please refer to the documentation provided in
///   MultibodyPlant::MultibodyPlant(double) for further details on the
///   parameter `time_step`.
/// @param[in] scene_graph (optional)
///   Constructed scene graph. If none is provided, one will be created and
///   used.
/// @return Pair of the registered plant and scene graph.
/// @pre `builder` must be non-null.
/// @tparam_default_scalar
/// @relates MultibodyPlant
template <typename T>
AddMultibodyPlantSceneGraphResult<T> AddMultibodyPlantSceneGraph(
    systems::DiagramBuilder<T>* builder, double time_step,
    std::unique_ptr<geometry::SceneGraph<T>> scene_graph = nullptr);

/// Adds a MultibodyPlant and a SceneGraph instance to a diagram
/// builder, connecting the geometry ports.
/// @note Usage examples in @ref add_multibody_plant_scene_graph
/// "AddMultibodyPlantSceneGraph".
///
/// @param[in,out] builder
///   Builder to add to.
/// @param[in] plant
///   Plant to be added to the builder.
/// @param[in] scene_graph (optional)
///   Constructed scene graph. If none is provided, one will be created and
///   used.
/// @return Pair of the registered plant and scene graph.
/// @pre `builder` and `plant` must be non-null.
/// @tparam_default_scalar
/// @relates MultibodyPlant
template <typename T>
AddMultibodyPlantSceneGraphResult<T> AddMultibodyPlantSceneGraph(
    systems::DiagramBuilder<T>* builder,
    std::unique_ptr<MultibodyPlant<T>> plant,
    std::unique_ptr<geometry::SceneGraph<T>> scene_graph = nullptr);

namespace internal {
// Adds a MultibodyPlant and a SceneGraph instance via shared pointers to a
// diagram builder, connecting the geometry ports.
//
// The shared pointer signature is useful for implementing pydrake memory
// management, because it permits supplying a custom deleter. The systems are
// not *actually* shared. They are logically owned by the builder, and
// eventually by the diagram.
template <typename T>
AddMultibodyPlantSceneGraphResult<T> AddMultibodyPlantSceneGraphFromShared(
    systems::DiagramBuilder<T>* builder,
    std::shared_ptr<MultibodyPlant<T>> plant,
    std::shared_ptr<geometry::SceneGraph<T>> scene_graph);
}  // namespace internal

/// Temporary result from `AddMultibodyPlantSceneGraph`. This cannot be
/// constructed outside of this method.
/// @warning Do NOT use this as a function argument or member variable. The
/// lifetime of this object should be as short as possible.
/// @tparam_default_scalar
template <typename T>
struct AddMultibodyPlantSceneGraphResult final {
  MultibodyPlant<T>& plant;
  geometry::SceneGraph<T>& scene_graph;

  /// For assignment to a plant reference (ignoring the scene graph).
  operator MultibodyPlant<T>&() { return plant; }

  /// For assignment to a std::tie of pointers.
  operator std::tuple<MultibodyPlant<T>*&, geometry::SceneGraph<T>*&>() {
    return std::tie(plant_ptr, scene_graph_ptr);
  }

#ifndef DRAKE_DOXYGEN_CXX
  // Returns the N-th member referenced by this struct.
  // If N = 0, returns the reference to the MultibodyPlant.
  // If N = 1, returns the reference to the geometry::SceneGraph.
  // Provided to support C++'s structured binding.
  template <std::size_t N>
  decltype(auto) get() const {
    if constexpr (N == 0) {
      return plant;
    } else if constexpr (N == 1) {
      return scene_graph;
    }
  }
#endif

#ifndef DRAKE_DOXYGEN_CXX
  // Only the move constructor is enabled; copy/assign/move-assign are deleted.
  AddMultibodyPlantSceneGraphResult(AddMultibodyPlantSceneGraphResult&&) =
      default;
  AddMultibodyPlantSceneGraphResult(const AddMultibodyPlantSceneGraphResult&) =
      delete;
  void operator=(const AddMultibodyPlantSceneGraphResult&) = delete;
  void operator=(AddMultibodyPlantSceneGraphResult&&) = delete;
#endif

 private:
  // Deter external usage by hiding construction.
  friend AddMultibodyPlantSceneGraphResult
  internal::AddMultibodyPlantSceneGraphFromShared<T>(
      systems::DiagramBuilder<T>*, std::shared_ptr<MultibodyPlant<T>>,
      std::shared_ptr<geometry::SceneGraph<T>>);

  AddMultibodyPlantSceneGraphResult(MultibodyPlant<T>* plant_in,
                                    geometry::SceneGraph<T>* scene_graph_in)
      : plant(*plant_in),
        scene_graph(*scene_graph_in),
        plant_ptr(plant_in),
        scene_graph_ptr(scene_graph_in) {}

  // Pointers to enable implicit casts for `std::tie()` assignments using
  // `T*&`.
  MultibodyPlant<T>* plant_ptr{};
  geometry::SceneGraph<T>* scene_graph_ptr{};
};

namespace internal {
// Combines the contact stiffness and dissipation parameters from two bodies in
// contact to create the contact stiffness and dissipation to be used for the
// contact pair.
// @tparam_default_scalar
template <typename T>
std::pair<T, T> CombinePointContactParameters(const T& k1, const T& k2,
                                              const T& d1, const T& d2) {
  // Simple utility to detect 0 / 0. As it is used in this method, denom
  // can only be zero if num is also zero, so we'll simply return zero.
  auto safe_divide = [](const T& num, const T& denom) {
    return denom == 0.0 ? 0.0 : num / denom;
  };
  return std::pair(
      safe_divide(k1 * k2, k1 + k2),                                   // k
      safe_divide(k2, k1 + k2) * d1 + safe_divide(k1, k1 + k2) * d2);  // d
}
}  // namespace internal
}  // namespace multibody
}  // namespace drake

#ifndef DRAKE_DOXYGEN_CXX
// Specializations provided to support C++'s structured binding for
// AddMultibodyPlantSceneGraphResult.
namespace std {
// The GCC standard library defines tuple_size as class and struct which
// triggers a warning here.
// We found this solution in: https://github.com/nlohmann/json/issues/1401
#if defined(__clang__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmismatched-tags"
#endif
template <typename T>
struct tuple_size<drake::multibody::AddMultibodyPlantSceneGraphResult<T>>
    : std::integral_constant<std::size_t, 2> {};

template <typename T>
struct tuple_element<0,
                     drake::multibody::AddMultibodyPlantSceneGraphResult<T>> {
  using type = drake::multibody::MultibodyPlant<T>;
};

template <typename T>
struct tuple_element<1,
                     drake::multibody::AddMultibodyPlantSceneGraphResult<T>> {
  using type = drake::geometry::SceneGraph<T>;
};
#if defined(__clang__)
#pragma GCC diagnostic pop
#endif
}  // namespace std
#endif

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class drake::multibody::MultibodyPlant);
DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    struct drake::multibody::AddMultibodyPlantSceneGraphResult);
