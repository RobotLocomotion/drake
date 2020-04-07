#pragma once

#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <tuple>
#include <type_traits>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_deprecated.h"
#include "drake/common/nice_type_name.h"
#include "drake/common/random.h"
#include "drake/geometry/scene_graph.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/hydroelastics/hydroelastic_engine.h"
#include "drake/multibody/plant/contact_jacobians.h"
#include "drake/multibody/plant/contact_results.h"
#include "drake/multibody/plant/coulomb_friction.h"
#include "drake/multibody/plant/tamsi_solver.h"
#include "drake/multibody/plant/tamsi_solver_results.h"
#include "drake/multibody/topology/multibody_graph.h"
#include "drake/multibody/tree/force_element.h"
#include "drake/multibody/tree/multibody_tree-inl.h"
#include "drake/multibody/tree/multibody_tree_system.h"
#include "drake/multibody/tree/rigid_body.h"
#include "drake/multibody/tree/weld_joint.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/scalar_conversion_traits.h"

namespace drake {
namespace multibody {
namespace internal {

// Data stored in the cache entry for the hydroelastic with fallback contact
// model.
template <typename T>
struct HydroelasticFallbackCacheData {
  std::vector<geometry::ContactSurface<T>> contact_surfaces;
  std::vector<geometry::PenetrationAsPointPair<T>> point_pairs;
};

// Structure used in the calculation of hydroelastic contact forces.
template <typename T>
struct HydroelasticContactInfoAndBodySpatialForces {
  explicit HydroelasticContactInfoAndBodySpatialForces(int num_bodies) {
    F_BBo_W_array.resize(num_bodies);
  }

  // Forces from hydroelastic contact applied to the origin of each body
  // (indexed by BodyNodeIndex) in the MultibodyPlant.
  std::vector<SpatialForce<T>> F_BBo_W_array;

  // Information used for contact reporting collected through the evaluation
  // of the hydroelastic model.
  std::vector<HydroelasticContactInfo<T>> contact_info;
};

}  // namespace internal

// TODO(amcastro-tri): Add a section on contact models in
// contact_model_doxygen.h.
/// Enumeration for contact model options.
enum class ContactModel {
  /// Contact forces are computed using the Hydroelastic model. Conctact between
  /// unsupported geometries will cause a runtime exception.
  kHydroelasticsOnly,

  /// Contact forces are computed using a point contact model, see @ref
  /// point_contact_approximation "Numerical Approximation of Point Contact".
  kPointContactOnly,

  /// Contact forces are computed using the hydroelastic model, where possible.
  /// For most other unsupported colliding pairs, the point model from
  /// kPointContactOnly is used. See
  /// geometry::QueryObject:ComputeContactSurfacesWithFallback for more
  /// details.
  kHydroelasticWithFallback
};

/// @cond
// Helper macro to throw an exception within methods that should not be called
// post-finalize.
#define DRAKE_MBP_THROW_IF_FINALIZED() ThrowIfFinalized(__func__)

// Helper macro to throw an exception within methods that should not be called
// pre-finalize.
#define DRAKE_MBP_THROW_IF_NOT_FINALIZED() ThrowIfNotFinalized(__func__)
/// @endcond

// TODO(sherm1) Rename "continuous_state" output ports to just "state" since
//              they can be discrete. However see issue #12214.
/// %MultibodyPlant is a Drake system framework representation (see
/// systems::System) for the model of a physical system consisting of a
/// collection of interconnected bodies.  See @ref multibody for an overview of
/// concepts/notation.
///
/// @system{MultibodyPlant,
///   @input_port{applied_generalized_force}
///   @input_port{applied_spatial_force}
///   @input_port{<em style="color:gray">
///     model_instance_name[i]</em>_actuation}
///   @input_port{<span style="color:green">geometry_query</span>},
///   @output_port{continuous_state}
///   @output_port{generalized_acceleration}
///   @output_port{reaction_forces}
///   @output_port{contact_results}
///   @output_port{<em style="color:gray">
///     model_instance_name[i]</em>_continuous_state}
///   @output_port{<em style="color:gray">
///     model_instance_name[i]</em>_generalized_acceleration}
///   @output_port{<em style="color:gray">
///     model_instance_name[i]</em>_generalized_contact_forces}
///   @output_port{<span style="color:green">geometry_pose</span>}
/// }
///
/// The ports whose names begin with <em style="color:gray">
/// model_instance_name[i]</em> represent groups of ports, one for each of the
/// @ref model_instances "model instances", with i ∈ {0, ..., N-1} for the N
/// model instances. If a model instance does not contain any data of the
/// indicated type the port will still be present but its value will be a
/// zero-length vector. (Model instances `world_model_instance()` and
/// `default_model_instance()` always exist.)
///
/// The ports shown in <span style="color:green">
/// green</span> are for communication with Drake's
/// @ref geometry::SceneGraph "SceneGraph" system for dealing with geometry.
///
/// %MultibodyPlant provides a user-facing API for:
///
/// - @ref mbp_input_and_output_ports "Ports":
///   Access input and output ports.
/// - @ref mbp_construction "Construction":
///   Add bodies, joints, frames, force elements, and actuators.
/// - @ref mbp_geometry "Geometry":
///   Register geometries to a provided SceneGraph instance.
/// - @ref mbp_contact_modeling "Contact modeling":
///   Select and parameterize contact models.
/// - @ref mbp_state_accessors_and_mutators "State access and modification":
///   Obtain and manipulate position and velocity state variables.
/// - @ref mbp_working_with_free_bodies "Free bodies":
///   Work conveniently with free (floating) bodies.
/// - @ref mbp_kinematic_and_dynamic_computations "Kinematics and dynamics":
///   Perform @ref systems::Context "Context"-dependent kinematic and dynamic
///   queries.
/// - @ref mbp_system_matrix_computations "System matrices":
///   Explicitly form matrices that appear in the equations of motion.
/// - @ref mbp_introspection "Introspection":
///   Perform introspection to find out what's in the %MultibodyPlant.
///
/// @anchor model_instances
///                         ### Model Instances
///
/// A MultiBodyPlant may contain multiple model instances. Each model instance
/// corresponds to a
/// set of bodies and their connections (joints). Model instances provide
/// methods to get or set the state of the set of bodies (e.g., through
/// GetPositionsAndVelocities() and SetPositionsAndVelocities()), connecting
/// controllers (through get_state_output_port()
/// and get_actuation_input_port()), and organizing duplicate models (read
/// through a parser). In fact, many %MultibodyPlant methods are overloaded
/// to allow operating on the entire plant or just the subset corresponding to
/// the model instance; for example, one GetPositions() method obtains the
/// generalized positions for the entire plant while another GetPositions()
/// method obtains the generalized positions for model instance.
///
/// Model instances are frequently defined through SDF files
/// (using the `model` tag) and are automatically created when SDF
/// files are parsed (by Parser). There are two special
/// multibody::ModelInstanceIndex values. The world body is always
/// multibody::ModelInstanceIndex 0. multibody::ModelInstanceIndex 1 is
/// reserved for all elements with no explicit model instance and
/// is generally only relevant for elements
/// created programmatically (and only when a model instance is not explicitly
/// specified). Note that Parser creates model instances (resulting in a
/// multibody::ModelInstanceIndex ≥ 2) as needed.
///
/// See num_model_instances(),
/// num_positions(),
/// num_velocities(), num_actuated_dofs(),
/// AddModelInstance() GetPositionsAndVelocities(),
/// GetPositions(), GetVelocities(),
/// SetPositionsAndVelocities(),
/// SetPositions(), SetVelocities(),
/// GetPositionsFromArray(), GetVelocitiesFromArray(),
/// SetPositionsInArray(), SetVelocitiesInArray(), SetActuationInArray(),
/// HasModelInstanceNamed(), GetModelInstanceName(),
/// get_state_output_port(),
/// get_actuation_input_port().
///
/// @anchor mbp_equations_of_motion
///                         ### System dynamics
///
/// <!-- TODO(amcastro-tri): Update this documentation to include:
///      - Bilateral constraints.
///      - Unilateral constraints and contact. -->
///
/// The state of a multibody system `x = [q; v]` is given by its generalized
/// positions vector q, of size `nq` (see num_positions()), and by its
/// generalized velocities vector v, of size `nv` (see num_velocities()).
/// As a Drake @ref systems::System "System", %MultibodyPlant implements the
/// governing equations for a
/// multibody dynamical system in the form `ẋ = f(t, x, u)` with t being
/// time and u the actuation forces. The governing equations for
/// the dynamics of a multibody system modeled with %MultibodyPlant are
/// [Featherstone 2008, Jain 2010]: <pre>
///          q̇ = N(q)v
///   (1)    M(q)v̇ + C(q, v)v = τ
/// </pre>
/// where `M(q)` is the mass matrix of the multibody system, `C(q, v)v`
/// contains Coriolis, centripetal, and gyroscopic terms and
/// `N(q)` is the kinematic coupling matrix describing the relationship between
/// q̇ (the time derivatives of the generalized positions) and the generalized
/// velocities v, [Seth 2010]. `N(q)` is an `nq x nv` matrix.
/// The vector `τ ∈ ℝⁿᵛ` on the right hand side of Eq. (1) is
/// the system's generalized forces. These incorporate gravity, springs,
/// externally applied body forces, constraint forces, and contact forces.
///
/// @anchor sdf_loading
///                  ### Loading models from SDF files
///
/// Drake has the capability to load multibody models from SDF and URDF
/// files.  Consider the example below which loads an acrobot model:
/// @code
///   MultibodyPlant<T> acrobot;
///   SceneGraph<T> scene_graph;
///   Parser parser(&acrobot, &scene_graph);
///   const std::string relative_name =
///     "drake/multibody/benchmarks/acrobot/acrobot.sdf";
///   const std::string full_name = FindResourceOrThrow(relative_name);
///   parser.AddModelFromFile(full_name);
/// @endcode
/// As in the example above, for models including visual geometry, collision
/// geometry or both, the user must specify a SceneGraph for geometry handling.
/// You can find a full example of the LQR controlled acrobot in
/// examples/multibody/acrobot/run_lqr.cc.
///
/// AddModelFromFile() can be invoked multiple times on the same plant in order
/// to load multiple model instances.  Other methods are available on Parser
/// such as AddAllModelsFromFile() which allows creating model instances per
/// each `<model>` tag found in the file. Please refer to each of these
/// methods' documentation for further details.
///
/// @anchor add_multibody_plant_scene_graph
///   ### Adding a %MultibodyPlant connected to a %SceneGraph to your %Diagram
///
/// Probably the simplest way to add and wire up a MultibodyPlant with
/// a SceneGraph in your Diagram is using AddMultibodyPlantSceneGraph().
///
/// Recommended usages:
///
/// Assign to a MultibodyPlant reference (ignoring the SceneGraph):
/// @code
///   MultibodyPlant<double>& plant =
///       AddMultibodyPlantSceneGraph(&builder, 0.0 /* time_step */);
///   plant.DoFoo(...);
/// @endcode
/// This flavor is the simplest, when the SceneGraph is not explicitly needed.
/// (It can always be retrieved later via GetSubsystemByName("scene_graph").)
///
/// Assign to auto, and use the named public fields:
/// @code
///   auto items = AddMultibodyPlantSceneGraph(&builder, 0.0 /* time_step */);
///   items.plant.DoFoo(...);
///   items.scene_graph.DoBar(...);
/// @endcode
/// or taking advantage of C++17's structured binding
/// @code
///   auto [plant, scene_graph] = AddMultibodyPlantSceneGraph(&builder);
///   ...
///   plant.DoFoo(...);
///   scene_graph.DoBar(...);
/// @endcode
/// This is the easiest way to use both the plant and scene_graph.
///
/// Assign to already-declared pointer variables:
/// @code
///   MultibodyPlant<double>* plant{};
///   SceneGraph<double>* scene_graph{};
///   std::tie(plant, scene_graph) =
///       AddMultibodyPlantSceneGraph(&builder, 0.0 /* time_step */);
///   plant->DoFoo(...);
///   scene_graph->DoBar(...);
/// @endcode
/// This flavor is most useful when the pointers are class member fields
/// (and so perhaps cannot be references).
///
/// @anchor mbp_adding_elements
///                    ### Adding modeling elements
///
/// <!-- TODO(amcastro-tri): Update this section to add force elements and
///      constraints. -->
///
/// Add multibody elements to a %MultibodyPlant with methods like:
///
/// - Bodies: AddRigidBody()
/// - Joints: AddJoint()
/// - see @ref mbp_construction "Construction" for more.
///
/// All modeling elements **must** be added before Finalize() is called.
/// See @ref mbp_finalize_stage "Finalize stage" for a discussion.
///
/// @anchor mbp_geometry_registration
///               ### Registering geometry with a SceneGraph
///
/// %MultibodyPlant users can register geometry with a SceneGraph for
/// essentially two purposes; a) visualization and, b) contact modeling.
///
/// <!--TODO(SeanCurtis-TRI): update this comment as the number of SceneGraph
///     roles changes. -->
///
/// Before any geometry registration takes place, a user **must** first make a
/// call to RegisterAsSourceForSceneGraph() in order to register the
/// %MultibodyPlant as a client of a SceneGraph instance, point at which the
/// plant will have assigned a valid geometry::SourceId.
/// At Finalize(), %MultibodyPlant will declare input/output ports as
/// appropriate to communicate with the SceneGraph instance on which
/// registrations took place. All geometry registration **must** be performed
/// pre-finalize.
///
/// If %MultibodyPlant registers geometry with a SceneGraph via calls to
/// RegisterCollisionGeometry(), an input port for geometric queries will be
/// declared at Finalize() time, see get_geometry_query_input_port(). Users must
/// connect this input port to the output port for geometric queries of the
/// SceneGraph used for registration, which can be obtained with
/// SceneGraph::get_query_output_port().
/// In summary, if %MultibodyPlant registers collision geometry, the setup
/// process will include:
///
/// 1. Call to RegisterAsSourceForSceneGraph().
/// 2. Calls to RegisterCollisionGeometry(), as many as needed.
/// 3. Call to Finalize(), user is done specifying the model.
/// 4. Connect SceneGraph::get_query_output_port() to
///    get_geometry_query_input_port().
///
/// Refer to the documentation provided in each of the methods above for further
/// details.
///
/// @anchor mbp_modeling_contact
///                           ### Modeling contact
///
/// Please refer to @ref drake_contacts "Contact Modeling in Drake" for details
/// on the available approximations, setup, and considerations for a multibody
/// simulation with frictional contact.
///
/// @anchor mbp_energy_and_power
///                         ### Energy and Power
/// <!-- TODO(sherm1) Update this as issue #12942 gets resolved. -->
/// %MultibodyPlant implements the System energy and power methods, with
/// some limitations.
/// - Kinetic energy: fully implemented.
/// - Potential energy and conservative power: currently include only gravity
///   and contributions from ForceElement objects; potential energy from
///   compliant contact and joint limits are not included.
/// - Nonconservative power: currently includes only contributions from
///   ForceElement objects; actuation and input port forces, joint damping,
///   and dissipation from joint limits, friction, and contact dissipation
///   are not included.
///
/// See Drake issue #12942 for more discussion.
///
/// @anchor mbp_finalize_stage
///                            ### %Finalize() stage
///
/// Once the user is done adding modeling elements and registering geometry, a
/// call to Finalize() must be performed. This call will:
/// - Build the underlying MultibodyTree topology, see MultibodyTree::Finalize()
///   for details,
/// - declare the plant's state,
/// - declare the plant's input and output ports,
/// - declare input and output ports for communication with a SceneGraph.
///
/// <!-- TODO(amcastro-tri): Consider making the actual geometry registration
///      with GS AFTER Finalize() so that we can tell if there are any bodies
///      welded to the world to which we could just assign anchored geometry
///      instead of dynamic geometry. This is an optimization and the API, and
///      pre/post-finalize conditions should not change. -->
///
/// @anchor mbp_table_of_contents
///
/// @anchor mbp_references
///                            ### References
///
/// - [Featherstone 2008] Featherstone, R., 2008.
///     Rigid body dynamics algorithms. Springer.
/// - [Jain 2010] Jain, A., 2010.
///     Robot and multibody dynamics: analysis and algorithms.
///     Springer Science & Business Media.
/// - [Seth 2010] Seth, A., Sherman, M., Eastman, P. and Delp, S., 2010.
///     Minimal formulation of joint motion for biomechanisms.
///     Nonlinear dynamics, 62(1), pp.291-303.
///
/// @tparam_default_scalar
/// @ingroup systems
template <typename T>
class MultibodyPlant : public internal::MultibodyTreeSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MultibodyPlant)

  /// @anchor mbp_input_and_output_ports
  /// @name                 Input and output ports
  /// These methods provide access to the Drake
  /// @ref systems::System "System" input and output ports
  /// as depicted in the MultibodyPlant class documentation.
  ///
  /// Actuation values can be provided through a single
  /// input port which describes the entire plant (in the case where only a
  /// single model instance has actuated dofs), or through multiple input ports
  /// which each provide the actuation values for a specific model instance.
  /// See AddJointActuator() and num_actuators().
  ///
  /// Output ports provide information about the entire %MultibodyPlant
  /// or its individual model instances.
  /// @{

  /// Returns a constant reference to the input port for external actuation for
  /// a specific model instance.  This input port is a vector valued port, which
  /// can be set with JointActuator::set_actuation_vector().
  /// @pre Finalize() was already called on `this` plant.
  /// @throws std::exception if called before Finalize().
  /// @throws std::exception if the model instance does not exist.
  const systems::InputPort<T>& get_actuation_input_port(
      ModelInstanceIndex model_instance) const;

  /// Returns a constant reference to the input port for external actuation for
  /// the case where only one model instance has actuated dofs.  This input
  /// port is a vector valued port, which can be set with
  /// JointActuator::set_actuation_vector().
  /// @pre Finalize() was already called on `this` plant.
  /// @throws std::exception if called before Finalize(), if the model does not
  /// contain any actuators, or if multiple model instances have actuated dofs.
  const systems::InputPort<T>& get_actuation_input_port() const;

  /// Returns a constant reference to the vector-valued input port for applied
  /// generalized forces, and the vector will be added directly into `tau` (see
  /// @ref mbp_equations_of_motion "System dynamics"). This vector is ordered
  /// using the same convention as the plant velocities: you can set the
  /// generalized forces that will be applied to model instance i using, e.g.,
  /// `SetVelocitiesInArray(i, model_forces, &force_array)`.
  /// @throws std::exception if called before Finalize().
  const systems::InputPort<T>& get_applied_generalized_force_input_port() const;

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
  /// @throws std::exception if this system was not registered with a
  /// SceneGraph.
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
  /// @pre Finalize() was already called on `this` plant.
  /// @throws std::exception if called before Finalize().
  const systems::OutputPort<T>& get_generalized_acceleration_output_port()
      const;

  /// Returns a constant reference to the output port for the generalized
  /// accelerations v̇ᵢ ⊆ v̇ for model instance i.
  /// @pre Finalize() was already called on `this` plant.
  /// @throws std::exception if called before Finalize().
  /// @throws std::exception if the model instance does not exist.
  const systems::OutputPort<T>& get_generalized_acceleration_output_port(
      ModelInstanceIndex model_instance) const;

  /// Returns a constant reference to the output port of generalized contact
  /// forces for a specific model instance.
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
  /// @throws std::exception if called pre-finalize.
  const systems::OutputPort<T>& get_reaction_forces_output_port() const;

  /// Returns a constant reference to the port that outputs ContactResults.
  /// @throws std::exception if called pre-finalize, see Finalize().
  const systems::OutputPort<T>& get_contact_results_output_port() const;

  /// Returns the output port of frames' poses to communicate with a
  /// SceneGraph.
  /// @throws std::exception if this system was not registered with a
  /// SceneGraph.
  const systems::OutputPort<T>& get_geometry_poses_output_port() const;
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

  /// Default constructor creates a plant modeled as a continuous system.
  /// Please refer to MultibodyPlant(double) for details.
  DRAKE_DEPRECATED("2020-05-01",
                   "Use MultibodyPlant(double) with time_step = 0.")
  MultibodyPlant() : MultibodyPlant(0.0) {}

  /// This constructor creates a plant with a single "world" body.
  /// Therefore, right after creation, num_bodies() returns one.
  ///
  /// %MultibodyPlant offers two different modalities to model mechanical sytems
  /// in time. These are:
  ///  1. As a discrete system with periodic updates, `time_step` is strictly
  ///     greater than zero.
  ///  2. As a continuous system, `time_step` equals exactly zero.
  ///
  /// Currently the discrete model is preferred for simulation given its
  /// robustness and speed in problems with frictional contact. However this
  /// might change as we work towards developing better strategies to model
  /// contact.
  /// See @ref time_advancement_strategy
  /// "Choice of Time Advancement Strategy" for further details.
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
  ///   period `time_step > 0`. See @ref time_advancement_strategy
  ///   "Choice of Time Advancement Strategy" for further details.
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
  explicit MultibodyPlant(const MultibodyPlant<U>& other)
      : internal::MultibodyTreeSystem<T>(
            systems::SystemTypeTag<MultibodyPlant>{},
            other.internal_tree().template CloneToScalar<T>(),
            other.is_discrete()) {
    DRAKE_THROW_UNLESS(other.is_finalized());
    time_step_ = other.time_step_;
    // Copy of all members related with geometry registration.
    source_id_ = other.source_id_;
    body_index_to_frame_id_ = other.body_index_to_frame_id_;
    frame_id_to_body_index_ = other.frame_id_to_body_index_;
    geometry_id_to_body_index_ = other.geometry_id_to_body_index_;
    geometry_id_to_visual_index_ = other.geometry_id_to_visual_index_;
    geometry_id_to_collision_index_ = other.geometry_id_to_collision_index_;
    default_coulomb_friction_ = other.default_coulomb_friction_;
    visual_geometries_ = other.visual_geometries_;
    collision_geometries_ = other.collision_geometries_;
    X_WB_default_list_ = other.X_WB_default_list_;
    contact_model_ = other.contact_model_;
    if (geometry_source_is_registered())
      DeclareSceneGraphPorts();

    // MultibodyTree::CloneToScalar() already called MultibodyTree::Finalize()
    // on the new MultibodyTree on U. Therefore we only Finalize the plant's
    // internals (and not the MultibodyTree).
    FinalizePlantOnly();
  }

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
  ///   @p model_instance. See HasBodyNamed(), Body::name().
  /// @param[in] model_instance
  ///   A model instance index which this body is part of.
  /// @param[in] M_BBo_B
  ///   The SpatialInertia of the new rigid body to be added to `this`
  ///   %MultibodyPlant, computed about the body frame origin `Bo` and expressed
  ///   in the body frame B.
  /// @returns A constant reference to the new RigidBody just added, which will
  ///          remain valid for the lifetime of `this` %MultibodyPlant.
  const RigidBody<T>& AddRigidBody(
      const std::string& name, ModelInstanceIndex model_instance,
      const SpatialInertia<double>& M_BBo_B) {
    DRAKE_MBP_THROW_IF_FINALIZED();
    // Make note in the graph.
    multibody_graph_.AddBody(name, model_instance);
    // Add the actual rigid body to the model.
    const RigidBody<T>& body = this->mutable_tree().AddRigidBody(
        name, model_instance, M_BBo_B);
    // Each entry of visual_geometries_, ordered by body index, contains a
    // std::vector of geometry ids for that body. The emplace_back() below
    // resizes visual_geometries_ to store the geometry ids for the body we
    // just added.
    // Similarly for the collision_geometries_ vector.
    DRAKE_DEMAND(visual_geometries_.size() == body.index());
    visual_geometries_.emplace_back();
    DRAKE_DEMAND(collision_geometries_.size() == body.index());
    collision_geometries_.emplace_back();
    DRAKE_DEMAND(X_WB_default_list_.size() == body.index());
    X_WB_default_list_.emplace_back();
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
  ///   Body::name().
  /// @param[in] M_BBo_B
  ///   The SpatialInertia of the new rigid body to be added to `this`
  ///   %MultibodyPlant, computed about the body frame origin `Bo` and expressed
  ///   in the body frame B.
  /// @returns A constant reference to the new RigidBody just added, which will
  ///          remain valid for the lifetime of `this` %MultibodyPlant.
  /// @throws std::logic_error if additional model instances have been created
  ///                          beyond the world and default instances.
  const RigidBody<T>& AddRigidBody(
      const std::string& name, const SpatialInertia<double>& M_BBo_B) {
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
  template <template<typename> class FrameType>
  const FrameType<T>& AddFrame(std::unique_ptr<FrameType<T>> frame) {
    return this->mutable_tree().AddFrame(std::move(frame));
  }

  /// This method adds a Joint of type `JointType` between two bodies.
  /// For more information, see the below overload of `AddJoint<>`, and the
  /// related `MultibodyTree::AddJoint<>` method.
  template <template<typename Scalar> class JointType>
  const JointType<T>& AddJoint(std::unique_ptr<JointType<T>> joint) {
    DRAKE_MBP_THROW_IF_FINALIZED();
    static_assert(std::is_convertible<JointType<T>*, Joint<T>*>::value,
                  "JointType must be a sub-class of Joint<T>.");
    RegisterJointInGraph(*joint);
    return this->mutable_tree().AddJoint(std::move(joint));
  }

  /// This method adds a Joint of type `JointType` between two bodies.
  /// The two bodies connected by this Joint object are referred to as the
  /// _parent_ and _child_ bodies. Although the terms _parent_ and _child_ are
  /// sometimes used synonymously to describe the relationship between inboard
  /// and outboard bodies in multibody models, this usage is wholly unrelated
  /// and implies nothing about the inboard-outboard relationship between the
  /// bodies.
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
  ///   input.
  /// @param[in] child
  ///   The child body connected by the new joint.
  /// @param[in] X_BM
  ///   The fixed pose of frame M attached to the child body, measured in
  ///   the frame B of that body. `X_BM` is an optional parameter; empty curly
  ///   braces `{}` imply that frame M **is** the same body frame B. If instead
  ///   your intention is to make a frame M with pose `X_BM` equal to the
  ///   identity pose, provide `RigidTransform<double>::Identity()` as your
  ///   input.
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
  ///   // Body 1 serves as parent, Body 2 serves as child.
  ///   // Define the pose X_BM of a frame M rigidly atached to child body B.
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
  /// with the given `name`.  See HasJointNamed(), Joint::name().
  ///
  /// @see The Joint class's documentation for further details on how a Joint
  /// is defined.
  template <template <typename> class JointType, typename... Args>
  const JointType<T>& AddJoint(
      const std::string& name, const Body<T>& parent,
      const std::optional<math::RigidTransform<double>>& X_PF,
      const Body<T>& child,
      const std::optional<math::RigidTransform<double>>& X_BM, Args&&... args) {
    static_assert(std::is_base_of<Joint<T>, JointType<T>>::value,
                  "JointType<T> must be a sub-class of Joint<T>.");

    const Frame<T>* frame_on_parent{nullptr};
    if (X_PF) {
      frame_on_parent = &this->AddFrame(
          std::make_unique<FixedOffsetFrame<T>>(parent, *X_PF));
    } else {
      frame_on_parent = &parent.body_frame();
    }

    const Frame<T>* frame_on_child{nullptr};
    if (X_BM) {
      frame_on_child = &this->AddFrame(
          std::make_unique<FixedOffsetFrame<T>>(child, *X_BM));
    } else {
      frame_on_child = &child.body_frame();
    }

    const JointType<T>& joint = AddJoint(
        std::make_unique<JointType<T>>(
            name,
            *frame_on_parent, *frame_on_child,
            std::forward<Args>(args)...));
    return joint;
  }

  /// Welds frames A and B with relative pose `X_AB`. That is, the pose of
  /// frame B in frame A is fixed, with value `X_AB`.
  /// The call to this method creates and adds a new WeldJoint to the model.
  /// The new WeldJoint is named as: A.name() + "_welds_to_" + B.name().
  /// @returns a constant reference to the WeldJoint welding frames A and B.
  const WeldJoint<T>& WeldFrames(const Frame<T>& A, const Frame<T>& B,
                                 const math::RigidTransform<double>& X_AB =
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
  template<template<typename Scalar> class ForceElementType, typename... Args>
  const ForceElementType<T>&
  AddForceElement(Args&&... args) {
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
  /// will remain valid for the lifetime of `this` plant.
  /// @throws std::exception if `joint.num_velocities() > 1` since for now we
  /// only support actuators for single dof joints.
  const JointActuator<T>& AddJointActuator(
      const std::string& name, const Joint<T>& joint,
      double effort_limit = std::numeric_limits<double>::infinity()) {
    DRAKE_THROW_UNLESS(joint.num_velocities() == 1);
    return this->mutable_tree().AddJointActuator(name, joint, effort_limit);
  }

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
  /// If `this` plant registered geometry with a SceneGraph, input and
  /// output ports to enable communication with that SceneGraph are declared
  /// as well.
  ///
  /// If geometry has been registered on a SceneGraph instance, that instance
  /// must be provided to the Finalize() method so that any geometric
  /// implications of the finalization process can be appropriately handled.
  ///
  /// @see is_finalized().
  ///
  /// @throws std::logic_error if the %MultibodyPlant has already been
  /// finalized.
  void Finalize();
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
  /// Geometries can be associated with bodies via the `RegisterXXXGeometry`
  /// family of methods. In SceneGraph, geometries have @ref geometry_roles
  /// "roles". The `RegisterCollisionGeometry()` methods register geometry with
  /// SceneGraph and assign it the proximity role. The
  /// `RegisterVisualGeometry()` methods do the same, but assign the
  /// illustration role.
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
  /// const Body<T>* body = plant.GetBodyFromFrameId(f_id);
  /// ```
  /// See documentation of geometry::SceneGraphInspector on where to get an
  /// inspector.
  ///
  /// In %MultibodyPlant, frame names only have to be unique in a single
  /// model instance. However, SceneGraph knows nothing of model instances. So,
  /// to generate unique names for the corresponding frames in SceneGraph,
  /// when %MultibodyPlant registers the corresponding SceneGraph frame, it is
  /// named with a "scoped name". This is a concatenation of
  /// `[model instance name]::[body name]`. Searching for a frame with just the
  /// name `body name` will fail. (See Body::name() and GetModelInstanceName()
  /// for those values.)
  /// @{

  /// Registers `this` plant to serve as a source for an instance of
  /// SceneGraph. This registration allows %MultibodyPlant to
  /// register geometry with `scene_graph` for visualization and/or
  /// collision queries.
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
  /// used for visualization of a given `body`.
  ///
  /// @note Currently, the visual geometry will _also_ be assigned a perception
  /// role. Its render label's value will be equal to the body's index and its
  /// perception color will be the same as its illustration color (defaulting to
  /// gray if no color is provided). This behavior will change in the near
  /// future and code that directly relies on this behavior will break.
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
  /// @throws std::exception if `scene_graph` does not correspond to the same
  /// instance with which RegisterAsSourceForSceneGraph() was called.
  /// @returns the id for the registered geometry.
  geometry::GeometryId RegisterVisualGeometry(
      const Body<T>& body, const math::RigidTransform<double>& X_BG,
      const geometry::Shape& shape, const std::string& name,
      const geometry::IllustrationProperties& properties);

  /// Overload for visual geometry registration; it converts the `diffuse_color`
  /// (RGBA with values in the range [0, 1]) into a
  /// geometry::ConnectDrakeVisualizer()-compatible set of
  /// geometry::IllustrationProperties.
  geometry::GeometryId RegisterVisualGeometry(
      const Body<T>& body, const math::RigidTransform<double>& X_BG,
      const geometry::Shape& shape, const std::string& name,
      const Vector4<double>& diffuse_color);

  /// Overload for visual geometry registration; it relies on the downstream
  /// geometry::IllustrationProperties _consumer_ to provide default parameter
  /// values (see @ref geometry_roles for details).
  geometry::GeometryId RegisterVisualGeometry(
      const Body<T>& body, const math::RigidTransform<double>& X_BG,
      const geometry::Shape& shape, const std::string& name);

  /// Returns an array of GeometryId's identifying the different visual
  /// geometries for `body` previously registered with a SceneGraph.
  /// @note This method can be called at any time during the lifetime of `this`
  /// plant, either pre- or post-finalize, see Finalize().
  /// Post-finalize calls will always return the same value.
  /// @see RegisterVisualGeometry(), Finalize()
  const std::vector<geometry::GeometryId>& GetVisualGeometriesForBody(
      const Body<T>& body) const;

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
  ///   The geometry::Shape used for visualization. E.g.: geometry::Sphere,
  ///   geometry::Cylinder, etc.
  /// @param[in] properties
  ///   The proximity properties associated with the collision geometry. They
  ///   *must* include the (`material`, `coulomb_friction`) property of type
  ///   CoulombFriction<double>.
  /// @throws std::exception if called post-finalize or if the properties are
  /// missing the coulomb friction property (or if it is of the wrong type).
  geometry::GeometryId RegisterCollisionGeometry(
      const Body<T>& body, const math::RigidTransform<double>& X_BG,
      const geometry::Shape& shape, const std::string& name,
      geometry::ProximityProperties properties);

  // TODO(SeanCurtis-TRI): Deprecate this in favor of simply passing properties.
  /// Overload which specifies a single property: coulomb_friction.
  geometry::GeometryId RegisterCollisionGeometry(
      const Body<T>& body, const math::RigidTransform<double>& X_BG,
      const geometry::Shape& shape, const std::string& name,
      const CoulombFriction<double>& coulomb_friction);

  /// Returns an array of GeometryId's identifying the different contact
  /// geometries for `body` previously registered with a SceneGraph.
  /// @note This method can be called at any time during the lifetime of `this`
  /// plant, either pre- or post-finalize, see Finalize().
  /// Post-finalize calls will always return the same value.
  /// @see RegisterCollisionGeometry(), Finalize()
  const std::vector<geometry::GeometryId>& GetCollisionGeometriesForBody(
      const Body<T>& body) const;

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
  /// SceneGraph::ExcludeCollisionsWithin() and
  /// SceneGraph::ExcludeCollisionsBetween() to filter collisions between the
  /// geometries registered to the bodies.
  ///
  /// For example:
  /// ```
  /// // Don't report on collisions between geometries affixed to `body1`,
  /// // `body2`, or `body3`.
  /// std::vector<const RigidBody<T>*> bodies{&body1, &body2, &body3};
  /// geometry::GeometrySet set = plant.CollectRegisteredGeometries(bodies);
  /// scene_graph.ExcludeCollisionsWithin(set);
  /// ```
  ///
  /// @note There is a *very* specific order of operations:
  ///
  /// 1. Bodies and geometries must be added to the %MultibodyPlant.
  /// 2. The %MultibodyPlant must be finalized (via Finalize()).
  /// 3. Create GeometrySet instances from bodies (via this method).
  /// 4. Invoke SceneGraph::ExcludeCollisions*() to filter collisions.
  /// 5. Allocate context.
  ///
  /// Changing the order will cause exceptions to be thrown.
  ///
  /// @throws std::exception if called pre-finalize.
  geometry::GeometrySet CollectRegisteredGeometries(
      const std::vector<const Body<T>*>& bodies) const;

  /// Given a geometry frame identifier, returns a pointer to the body
  /// associated with that id (nullptr if there is no such body).
  const Body<T>* GetBodyFromFrameId(geometry::FrameId frame_id) const {
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
      throw std::logic_error(
          "Body '" + internal_tree().get_body(body_index).name() +
              "' does not have geometry registered with it.");
    }
    return it->second;
  }
  /// @} <!-- Geometry -->

  /// @anchor mbp_contact_modeling
  /// @name                    Contact modeling
  /// Use methods in this section to choose the contact model and to provide
  /// parameters for that model. Currently Drake supports an advanced compliant
  /// contact model we call _Hydroelastic contact_ that is still experimental,
  /// and a penalty-based point contact model as a reliable fallback.
  ///
  /// @anchor mbp_hydroelastic_materials_properties
  ///                      #### Hydroelastic contact
  ///
  /// To understand how material properties enter into the modeling of contact
  /// traction in the hydroelastic model, the user is referred to [R. Elandt
  /// 2019] for details.
  /// For brevity, here we limit ourselves to state the relationship between the
  /// material properties and the computation of the normal traction or
  /// "pressure" `p(x)` at each point `x` in the contact patch.
  /// Given two bodies A and B, with elastic moduli `Eᵃ` and `Eᵇ` respectively
  /// and dissipation `dᵃ` and `dᵇ` respectively, we define the effective
  /// material properties of the pair according to: <pre>
  ///   E = Eᵃ⋅Eᵇ/(Eᵃ + Eᵇ),
  ///   d = E/Eᵃ⋅dᵃ + E/Eᵇ⋅dᵇ = Eᵇ/(Eᵃ+Eᵇ)⋅dᵃ + Eᵃ/(Eᵃ+Eᵇ)⋅dᵇ
  /// </pre>
  /// The effective modulus of elasticity is computed in accordance with the
  /// Hertz theory of contact. Dissipation is weighted in accordance with the
  /// fact that the softer material will deform more and faster and thus the
  /// softer material dissipation is given more importance. Elastic modulus has
  /// units of pressure, i.e. `Pa (N/m²)`. The elastic modulus is often
  /// estimated based on the Young's modulus of the material though in the
  /// hydroelastic model it represents an effective elastic property. For
  /// instance, [R. Elandt 2019] chooses to use `E = G`, with `G` the P-wave
  /// elastic modulus `G = (1-ν)/(1+ν)/(1-2ν)E`, with ν the Poisson
  /// ratio, consistent with the theory of layered solids in which plane
  /// sections remain planar after compression. Another possibility is to
  /// specify `E = E*`, with `E*` the effective elastic modulus given by the
  /// Hertz theory of contact, `E* = E/(1-ν²)`. In all of these cases a sound
  /// estimation of `elastic_modulus` starts with the Young's modulus of the
  /// material.
  ///
  /// We use a dissipation model inspired by the model in
  /// [Hunt and Crossley, 1975], parameterized by a dissipation constant with
  /// units of inverse of velocity, i.e. `s/m`.
  ///
  /// The elastic modulus and dissipation can be specified in one of two ways:
  ///
  /// - define them in an instance of geometry::ProximityProperties using
  ///   the function geometry::AddContactMaterial(), or
  /// - define them in an input URDF/SDF as detailed @ref sdf_contact_material
  ///   "here for SDF" or @ref urdf_contact_material "here for URDF".
  ///
  /// With the effective properties of the pair defined as above, the
  /// hydroelastic model pressure field is computed according to:
  /// <pre>
  ///   p(x) = E⋅ε(x)⋅(1 - d⋅vₙ(x))₊
  /// </pre>
  /// where we defined the effective strain: <pre>
  ///   ε(x) = εᵃ(x) + εᵇ(x)
  /// </pre>
  /// which relates to the quasi-static pressure field p₀(x) (i.e. when velocity
  /// is neglected) by: <pre>
  ///   p₀(x) = E⋅ε(x) = Eᵃ⋅εᵃ(x) = Eᵇ⋅εᵇ(x)
  /// </pre>
  /// that is, the hydroelastic model computes the contact patch assuming
  /// quasi-static equilibrium.
  /// The separation speed `vₙ(x)` is computed as the component in the
  /// direction of the contact surface's normal `n̂(x)` of the relative velocity
  /// between points `Ax` and `Bx` at point `x` instantaneously moving with body
  /// frames A and B respectively, i.e. `vₙ(x) = ᴬˣvᴮˣ⋅n̂(x)`, where the normal
  /// `n̂(x)` points from body A into body B.
  ///
  /// [Elandt 2019] R. Elandt, E. Drumwright, M. Sherman, and A. Ruina. A
  ///   pressure field model for fast, robust approximation of net contact force
  ///   and moment between nominally rigid objects. Proc. IEEE/RSJ Intl. Conf.
  ///   on Intelligent Robots and Systems (IROS), 2019.
  /// [Hunt and Crossley 1975] Hunt, KH and Crossley, FRE, 1975. Coefficient
  ///   of restitution interpreted as damping in vibroimpact. Journal of Applied
  ///   Mechanics, vol. 42, pp. 440–445.
  ///
  /// @anchor mbp_penalty_method
  ///                   #### Penalty method point contact
  ///
  /// Currently %MultibodyPlant uses a rigid contact model that is, bodies in
  /// the model are infinitely stiff or ideal rigid bodies. Therefore, the
  /// mathematical description of the rigid contact model needs to include
  /// non-penetration constraints among bodies in the formulation. There are
  /// several numerical methods to impose and solve these constraints.
  /// In a penalty method approach, we allow for a certain amount of
  /// interpenetration and we compute contact forces according to a simple law
  /// of the form: <pre>
  ///   fₙ = k(1+dẋ)x
  /// </pre>
  /// where the normal contact force `fₙ` is made a continuous function of the
  /// penetration distance x between the bodies (defined to be
  /// positive when the bodies are in contact) and the penetration distance
  /// rate ẋ (with ẋ > 0 meaning the penetration distance is increasing and
  /// therefore the interpenetration between the bodies is also increasing).
  /// k and d are the penalty method coefficients for stiffness and damping.
  /// These are ad-hoc parameters which need to be tuned as a trade-off between:
  /// - The accuracy of the numerical approximation to rigid contact, which
  ///   requires a stiffness that approaches infinity, and
  /// - the computational cost of the numerical integration, which will
  ///   require smaller time steps for stiffer systems.
  ///
  /// There is no exact procedure for choosing these coefficients, and
  /// estimating them manually can be cumbersome since in general they will
  /// depend on the scale of the problem including masses, speeds and even
  /// body sizes. However, %MultibodyPlant aids the estimation of these
  /// coefficients using a heuristic function based on a user-supplied
  /// "penetration allowance", see set_penetration_allowance(). The penetration
  /// allowance is a number in meters that specifies the order of magnitude of
  /// the average penetration between bodies in the system that the user is
  /// willing to accept as reasonable for the problem being solved. For
  /// instance, in the robotics manipulation of ordinary daily objects the user
  /// might set this number to 1 millimeter. However, the user might want to
  /// increase it for the simulation of heavy walking robots for which an
  /// allowance of 1 millimeter would result in a very stiff system.
  ///
  /// As for the damping coefficient in the simple law above, %MultibodyPlant
  /// chooses the damping coefficient d to model inelastic collisions and
  /// therefore sets it so that the penetration distance x behaves as a
  /// critically damped oscillator. That is, at the limit of ideal rigid contact
  /// (very stiff penalty coefficient k or equivalently the penetration
  /// allowance goes to zero), this method behaves as a unilateral constraint on
  /// the penetration distance, which models a perfect inelastic collision. For
  /// most applications, such as manipulation and walking, this is the desired
  /// behavior.
  ///
  /// When set_penetration_allowance() is called, %MultibodyPlant will estimate
  /// reasonable penalty method coefficients as a function of the input
  /// penetration allowance. Users will want to run their simulation a number of
  /// times and asses they are satisfied with the level of inter-penetration
  /// actually observed in the simulation; if the observed penetration is too
  /// large, the user will want to set a smaller penetration allowance. If the
  /// system is too stiff and the time integration requires very small time
  /// steps while at the same time the user can afford larger
  /// inter-penetrations, the user will want to increase the penetration
  /// allowance. Typically, the observed penetration will be
  /// proportional to the penetration allowance. Thus scaling the penetration
  /// allowance by say a factor of 0.5, would typically results in
  /// inter-penetrations being reduced by the same factor of 0.5.
  /// In summary, users should choose the largest penetration allowance that
  /// results in inter-penetration levels that are acceptable for the particular
  /// application (even when in theory this penetration should be zero for
  /// perfectly rigid bodies.)
  ///
  /// For a given penetration allowance, the contact interaction that takes two
  /// bodies with a non-zero approaching velocity to zero approaching velocity,
  /// takes place in a finite amount of time (for ideal rigid contact this time
  /// is zero.) A good estimate of this time period is given by a call to
  /// get_contact_penalty_method_time_scale(). Users might want to query this
  /// value to either set the maximum time step in error-controlled time
  /// integration or to set the time step for fixed time step integration.
  /// As a guidance, typical fixed time step integrators will become unstable
  /// for time steps larger than about a tenth of this time scale.
  ///
  /// For further details on contact modeling in Drake, please refer to the
  /// section @ref drake_contacts "Contact Modeling in Drake" of our
  /// documentation.
  /// @{

  /// Sets the contact model to be used by `this` %MultibodyPlant, see
  /// ContactModel for available options.
  /// The default contact model is ContactModel::kPointContactOnly.
  /// @throws std::exception iff called post-finalize.
  void set_contact_model(ContactModel model);

  /// Sets the penetration allowance used to estimate the coefficients in the
  /// penalty method used to impose non-penetration among bodies. Refer to the
  /// section @ref mbp_penalty_method "Contact by penalty method" for further
  /// details.
  void set_penetration_allowance(double penetration_allowance = 0.001);

  /// Returns a time-scale estimate `tc` based on the requested penetration
  /// allowance δ set with set_penetration_allowance().
  /// For the penalty method in use to enforce non-penetration, this time scale
  /// relates to the time it takes the relative normal velocity between two
  /// bodies to go to zero. This time scale `tc` is artificially introduced by
  /// the penalty method and goes to zero in the limit to ideal rigid contact.
  /// Since numerical integration methods for continuum systems must be able to
  /// resolve a system's dynamics, the time step used by an integrator must in
  /// general be much smaller than the time scale `tc`. How much smaller will
  /// depend on the details of the problem and the convergence characteristics
  /// of the integrator and should be tuned appropriately.
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
  /// the discrete time stepping (see @ref time_advancement_strategy
  /// "Choice of Time Advancement Strategy").
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
  void set_stiction_tolerance(double v_stiction = 0.001) {
    friction_model_.set_stiction_tolerance(v_stiction);
    // We allow calling this method post-finalize. Therefore, if the plant is
    // modeled as a discrete system, we must update the solver's stiction
    // parameter. Pre-Finalize the solver is not yet created and therefore we
    // check for nullptr.
    if (is_discrete() && tamsi_solver_ != nullptr) {
      TamsiSolverParameters solver_parameters =
          tamsi_solver_->get_solver_parameters();
      solver_parameters.stiction_tolerance =
          friction_model_.stiction_tolerance();
      tamsi_solver_->set_solver_parameters(solver_parameters);
    }
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
  /// There are also utilities for accessing and mutating portions of state
  /// or actuation arrays corresponding to just a single model instance.
  /// @{

  /// Returns a const vector reference containing the vector
  /// `[q; v]` with `q` the vector of generalized positions and
  /// `v` the vector of generalized velocities.
  /// @note This method returns a reference to existing data, exhibits constant
  ///       i.e., O(1) time complexity, and runs very quickly.
  /// @throws std::exception if the `context` does not
  /// correspond to the context for a multibody model.
  Eigen::VectorBlock<const VectorX<T>> GetPositionsAndVelocities(
      const systems::Context<T>& context) const {
    return internal_tree().GetPositionsAndVelocities(context);
  }

  /// Returns the vector `[q; v]`
  /// of the model with `q` the vector of generalized positions and `v` the
  /// vector of generalized velocities for model instance `model_instance`.
  /// @throws std::exception if the `context` does not correspond to the context
  /// for a multibody model or `model_instance` is invalid.
  /// @note returns a dense vector of dimension `q.size() + v.size()` associated
  ///          with `model_instance` in O(`q.size()`) time.
  VectorX<T> GetPositionsAndVelocities(
      const systems::Context<T>& context,
      ModelInstanceIndex model_instance) const {
    return internal_tree().GetPositionsAndVelocities(context, model_instance);
  }

  /// (Advanced) Returns a mutable vector containing the vector `[q; v]`
  /// of the model with `q` the vector of generalized positions and `v` the
  /// vector of generalized velocities (**see warning**).
  /// @warning You should use SetPositionsAndVelocities() instead of this method
  ///          unless you are fully aware of the interactions with the caching
  ///          mechanism (see @ref dangerous_get_mutable).
  /// @throws std::exception if the `context` is nullptr or if it does not
  /// correspond to the context for a multibody model.
  Eigen::VectorBlock<VectorX<T>> GetMutablePositionsAndVelocities(
      systems::Context<T>* context) const {
    return internal_tree().GetMutablePositionsAndVelocities(context);
  }

  /// Sets all generalized positions and velocities from the given vector
  /// [q; v].
  /// @throws std::exception if the `context` is nullptr, if the context does
  /// not correspond to the context for a multibody model, or if the length of
  /// `q_v` is not equal to `num_positions() + num_velocities()`.
  void SetPositionsAndVelocities(
      systems::Context<T>* context, const VectorX<T>& q_v) const {
    DRAKE_THROW_UNLESS(q_v.size() == (num_positions() + num_velocities()));
    internal_tree().GetMutablePositionsAndVelocities(context) = q_v;
  }

  /// Sets generalized positions and velocities from the given vector
  /// [q; v] for the specified model instance.
  /// @throws std::exception if the `context` is nullptr, if the context does
  /// not correspond to the context for a multibody model, if the model instance
  /// index is invalid, or if the length of `q_v` is not equal to
  /// `num_positions(model_instance) + num_velocities(model_instance)`.
  void SetPositionsAndVelocities(
      systems::Context<T>* context, ModelInstanceIndex model_instance,
      const VectorX<T>& q_v) const {
    DRAKE_THROW_UNLESS(
        q_v.size() ==
        (num_positions(model_instance) + num_velocities(model_instance)));
    internal_tree().SetPositionsAndVelocities(model_instance, q_v, context);
  }

  /// Returns a const vector reference containing the vector of
  /// generalized positions.
  /// @note This method returns a reference to existing data, exhibits constant
  ///       i.e., O(1) time complexity, and runs very quickly.
  /// @throws std::exception if the `context` does not
  /// correspond to the context for a multibody model.
  Eigen::VectorBlock<const VectorX<T>> GetPositions(
      const systems::Context<T>& context) const {
    // Note: the nestedExpression() is necessary to treat the VectorBlock<T>
    // returned from GetPositionsAndVelocities() as a VectorX<T> so that we can
    // call head() on it.
    return GetPositionsAndVelocities(context).nestedExpression().head(
        num_positions());
  }

  /// Returns an vector containing the generalized positions (`q`) for the
  /// given model instance.
  /// @throws std::exception if the `context` does not
  /// correspond to the context for a multibody model.
  /// @note returns a dense vector of dimension `q.size()` associated with
  ///          `model_instance` in O(`q.size()`) time.
  VectorX<T> GetPositions(
      const systems::Context<T>& context,
      ModelInstanceIndex model_instance) const {
    return internal_tree().GetPositionsFromArray(
        model_instance, GetPositions(context));
  }

  /// (Advanced) Returns a mutable vector reference containing the vector
  /// of generalized positions (**see warning**).
  /// @note This method returns a reference to existing data, exhibits constant
  ///       i.e., O(1) time complexity, and runs very quickly.
  /// @warning You should use SetPositions() instead of this method
  ///          unless you are fully aware of the possible interactions with the
  ///          caching mechanism (see @ref dangerous_get_mutable).
  /// @throws std::exception if the `context` is nullptr or if it does not
  /// correspond to the context for a multibody model.
  Eigen::VectorBlock<VectorX<T>> GetMutablePositions(
      systems::Context<T>* context) const {
    // Note: the nestedExpression() is necessary to treat the VectorBlock<T>
    // returned from GetMutablePositionsAndVelocities() as a VectorX<T> so that
    // we can call head() on it.
    return internal_tree().GetMutablePositionsAndVelocities(context)
        .nestedExpression().head(num_positions());
  }

  /// (Advanced) Returns a mutable vector reference containing the vector
  /// of generalized positions (**see warning**).
  /// @note This method returns a reference to existing data, exhibits constant
  ///       i.e., O(1) time complexity, and runs very quickly.
  /// @warning You should use SetPositions() instead of this method
  ///          unless you are fully aware of the possible interactions with the
  ///          caching mechanism (see @ref dangerous_get_mutable).
  /// @throws std::exception if the `state` is nullptr or if the context does
  ///         not correspond to the context for a multibody model.
  /// @pre `state` comes from this MultibodyPlant.
  Eigen::VectorBlock<VectorX<T>> GetMutablePositions(
      const systems::Context<T>& context, systems::State<T>* state) const {
    DRAKE_ASSERT_VOID(CheckValidState(state));
    // Note: the nestedExpression() is necessary to treat the VectorBlock<T>
    // returned from GetMutablePositionsAndVelocities() as a VectorX<T> so that
    // we can call head() on it.
    return internal_tree()
        .GetMutablePositionsAndVelocities(context, state)
        .nestedExpression()
        .head(num_positions());
  }

  /// Sets all generalized positions from the given vector.
  /// @throws std::exception if the `context` is nullptr, if the context does
  /// not correspond to the context for a multibody model, or if the length of
  /// `q` is not equal to `num_positions()`.
  void SetPositions(systems::Context<T>* context, const VectorX<T>& q) const {
    DRAKE_THROW_UNLESS(q.size() == num_positions());
    GetMutablePositions(context) = q;
  }

  /// Sets the positions for a particular model instance from the given vector.
  /// @throws std::exception if the `context` is nullptr, if the context does
  /// not correspond to the context for a multibody model, if the model instance
  /// index is invalid, or if the length of `q_instance` is not equal to
  /// `num_positions(model_instance)`.
  void SetPositions(
      systems::Context<T>* context,
      ModelInstanceIndex model_instance, const VectorX<T>& q_instance) const {
    DRAKE_THROW_UNLESS(q_instance.size() == num_positions(model_instance));
    Eigen::VectorBlock<VectorX<T>> q = GetMutablePositions(context);
    internal_tree().SetPositionsInArray(model_instance, q_instance, &q);
  }

  /// Sets the positions for a particular model instance from the given vector.
  /// @throws std::exception if the `state` is nullptr, if the context does
  /// not correspond to the context for a multibody model, if the model instance
  /// index is invalid, or if the length of `q_instance` is not equal to
  /// `num_positions(model_instance)`.
  /// @pre `state` comes from this MultibodyPlant.
  void SetPositions(const systems::Context<T>& context,
                    systems::State<T>* state, ModelInstanceIndex model_instance,
                    const VectorX<T>& q_instance) const {
    DRAKE_THROW_UNLESS(q_instance.size() == num_positions(model_instance));
    CheckValidState(state);
    Eigen::VectorBlock<VectorX<T>> q = GetMutablePositions(context, state);
    internal_tree().SetPositionsInArray(model_instance, q_instance, &q);
  }

  /// Returns a const vector reference containing the generalized velocities.
  /// @note This method returns a reference to existing data, exhibits constant
  ///       i.e., O(1) time complexity, and runs very quickly.
  Eigen::VectorBlock<const VectorX<T>> GetVelocities(
      const systems::Context<T>& context) const {
    // Note: the nestedExpression() is necessary to treat the VectorBlock<T>
    // returned from GetPositionsAndVelocities() as a VectorX<T> so that we can
    // call tail() on it.
    return GetPositionsAndVelocities(context).nestedExpression().tail(
        num_velocities());
  }

  /// Returns a vector containing the generalized velocities (`v`) for
  /// the given model instance.
  /// @throws std::exception if the `context` does not
  /// correspond to the context for a multibody model.
  /// @note returns a dense vector of dimension `v.size()` associated with
  ///          `model_instance` in O(`v.size()`) time.
  VectorX<T> GetVelocities(
      const systems::Context<T>& context,
      ModelInstanceIndex model_instance) const {
    return internal_tree().GetVelocitiesFromArray(
        model_instance, GetVelocities(context));
  }

  /// (Advanced) Returns a mutable vector reference containing the vector
  /// of generalized velocities (**see warning**).
  /// @note This method returns a reference to existing data, exhibits constant
  ///       i.e., O(1) time complexity, and runs very quickly.
  /// @warning You should use SetVelocities() instead of this method
  ///          unless you are fully aware of the possible interactions with the
  ///          caching mechanism (see @ref dangerous_get_mutable).
  /// @throws std::exception if the `context` is nullptr or the context does
  /// not correspond to the context for a multibody model.
  /// @pre `state` comes from this MultibodyPlant.
  Eigen::VectorBlock<VectorX<T>> GetMutableVelocities(
      const systems::Context<T>& context, systems::State<T>* state) const {
    DRAKE_ASSERT_VOID(CheckValidState(state));
    // Note: the nestedExpression() is necessary to treat the VectorBlock<T>
    // returned from GetMutablePositionsAndVelocities() as a VectorX<T> so that
    // we can call tail() on it.
    return internal_tree()
        .GetMutablePositionsAndVelocities(context, state)
        .nestedExpression()
        .tail(num_velocities());
  }

  /// See GetMutableVelocities() method above.
  Eigen::VectorBlock<VectorX<T>> GetMutableVelocities(
      systems::Context<T>* context) const {
    return GetMutableVelocities(*context, &context->get_mutable_state());
  }

  /// Sets all generalized velocities from the given vector.
  /// @throws std::exception if the `context` is nullptr, if the context does
  /// not correspond to the context for a multibody model, or if the length of
  /// `v` is not equal to `num_velocities()`.
  void SetVelocities(systems::Context<T>* context, const VectorX<T>& v) const {
    DRAKE_THROW_UNLESS(v.size() == num_velocities());
    GetMutableVelocities(context) = v;
  }

  /// Sets the generalized velocities for a particular model instance from the
  /// given vector.
  /// @throws std::exception if the `context` is nullptr, if the context does
  /// not correspond to the context for a multibody model, if the model instance
  /// index is invalid, or if the length of `v_instance` is not equal to
  /// `num_velocities(model_instance)`.
  /// @pre `state` comes from this MultibodyPlant.
  void SetVelocities(
      const systems::Context<T>& context, systems::State<T>* state,
      ModelInstanceIndex model_instance, const VectorX<T>& v_instance) const {
    DRAKE_THROW_UNLESS(v_instance.size() == num_velocities(model_instance));
    CheckValidState(state);
    Eigen::VectorBlock<VectorX<T>> v = GetMutableVelocities(context, state);
    internal_tree().SetVelocitiesInArray(model_instance, v_instance, &v);
  }

  /// Sets the generalized velocities for a particular model instance from the
  /// given vector.
  /// @throws std::exception if the `context` is nullptr, if the context does
  /// not correspond to the context for a multibody model, if the model instance
  /// index is invalid, or if the length of `v_instance` is not equal to
  /// `num_velocities(model_instance)`.
  void SetVelocities(
      systems::Context<T>* context,
      ModelInstanceIndex model_instance, const VectorX<T>& v_instance) const {
    DRAKE_THROW_UNLESS(v_instance.size() == num_velocities(model_instance));
    Eigen::VectorBlock<VectorX<T>> v = GetMutableVelocities(context);
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
    CheckValidState(state);
    internal_tree().SetDefaultState(context, state);
    for (const BodyIndex index : GetFloatingBaseBodies()) {
      SetFreeBodyPose(
          context, state, internal_tree().get_body(index),
          X_WB_default_list_[index].template cast<T>());
    }
  }

  /// Assigns random values to all elements of the state, by drawing samples
  /// independently for each joint/free body (coming soon: and then
  /// solving a mathematical program to "project" these samples onto the
  /// registered system constraints).
  ///
  /// @see @ref stochastic_systems
  void SetRandomState(const systems::Context<T>& context,
                      systems::State<T>* state,
                      RandomGenerator* generator) const override {
    DRAKE_MBP_THROW_IF_NOT_FINALIZED();
    CheckValidState(state);
    internal_tree().SetRandomState(context, state, generator);
  }

  /// Returns a vector of actuation values for `model_instance` from a
  /// vector `u` of actuation values for the entire model. This method throws an
  /// exception if `u` is not of size MultibodyPlant::num_actuated_dofs().
  VectorX<T> GetActuationFromArray(
      ModelInstanceIndex model_instance,
      const Eigen::Ref<const VectorX<T>>& u) const {
    return internal_tree().GetActuationFromArray(model_instance, u);
  }

  /// Given the actuation values `u_instance` for all actuators in
  /// `model_instance`, this method sets the actuation vector u for the entire
  /// model to which this actuator belongs to. This method throws
  /// an exception if the size of `u_instance` is not equal to the number of
  /// degrees of freedom of all of the actuated joints in `model_instance`.
  /// @param[in] u_instance Actuation values for the actuators. It must be of
  ///   size equal to the number of degrees of freedom of all of the actuated
  ///   joints in `model_instance`.
  /// @param[out] u
  ///   The vector containing the actuation values for the entire model.
  void SetActuationInArray(
      ModelInstanceIndex model_instance,
      const Eigen::Ref<const VectorX<T>>& u_instance,
      EigenPtr<VectorX<T>> u) const {
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

  /// Sets the vector of generalized positions for `model_instance` in
  /// `q` using `q_instance`, leaving all other elements in the array
  /// untouched. This method throws an exception if `q` is not of size
  /// MultibodyPlant::num_positions() or `q_instance` is not of size
  /// `MultibodyPlant::num_positions(model_instance)`.
  void SetPositionsInArray(
      ModelInstanceIndex model_instance,
      const Eigen::Ref<const VectorX<T>>& q_instance,
      EigenPtr<VectorX<T>> q) const {
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

  /// Sets the vector of generalized velocities for `model_instance` in
  /// `v` using `v_instance`, leaving all other elements in the array
  /// untouched. This method throws an exception if `v` is not of size
  /// MultibodyPlant::num_velocities() or `v_instance` is not of size
  /// `MultibodyPlant::num_positions(model_instance)`.
  void SetVelocitiesInArray(
      ModelInstanceIndex model_instance,
      const Eigen::Ref<const VectorX<T>>& v_instance,
      EigenPtr<VectorX<T>> v) const {
    internal_tree().SetVelocitiesInArray(model_instance, v_instance, v);
  }
  /// @} <!-- State accessors and mutators -->

  /// @anchor mbp_working_with_free_bodies
  /// @name                Working with free bodies
  ///
  /// A %MultibodyPlant user adds sets of Body and Joint objects to `this` plant
  /// to build a physical representation of a mechanical model.
  /// At Finalize(), %MultibodyPlant builds a mathematical representation of
  /// such system, consisting of a tree representation. In this
  /// representation each body is assigned a Mobilizer, which grants a certain
  /// number of degrees of freedom in accordance to the physical specification.
  /// In this regard, the modeling representation can be seen as a forest of
  /// tree structures each of which contains a single body at the root of the
  /// tree. If the root body has six degrees of freedom with respect to the
  /// world, it is called a "free body" (sometimes called a "floating body").
  /// A user can request the set of all free bodies with a call to
  /// GetFloatingBaseBodies(). Alternatively, a user can query whether a Body is
  /// free (floating) or not with Body::is_floating().
  /// For many applications, a user might need to work with indexes in the
  /// multibody state vector. For such applications,
  /// Body::floating_positions_start() and Body::floating_velocities_start()
  /// offer the additional level of introspection needed.
  /// @{

  /// Returns the set of body indexes corresponding to the free (floating)
  /// bodies in the model, in no particular order.
  /// @throws std::exception if called pre-finalize, see Finalize().
  std::unordered_set<BodyIndex> GetFloatingBaseBodies() const;

  /// Gets the pose of a given `body` in the world frame W.
  /// @note In general getting the pose of a body in the model would involve
  /// solving the kinematics. This method allows us to simplify this process
  /// when we know the body is free in space.
  /// @throws std::exception if `body` is not a free body in the model.
  /// @throws std::exception if called pre-finalize.
  math::RigidTransform<T> GetFreeBodyPose(const systems::Context<T>& context,
                                          const Body<T>& body) const {
    return internal_tree().GetFreeBodyPoseOrThrow(context, body);
  }

  /// Sets `context` to store the pose `X_WB` of a given `body` B in the world
  /// frame W.
  /// @note In general setting the pose and/or velocity of a body in the model
  /// would involve a complex inverse kinematics problem. This method allows us
  /// to simplify this process when we know the body is free in space.
  /// @throws std::exception if `body` is not a free body in the model.
  /// @throws std::exception if called pre-finalize.
  void SetFreeBodyPose(systems::Context<T>* context, const Body<T>& body,
                       const math::RigidTransform<T>& X_WB) const {
    internal_tree().SetFreeBodyPoseOrThrow(body, X_WB, context);
  }

  /// Sets `state` to store the pose `X_WB` of a given `body` B in the world
  /// frame W, for a given `context` of `this` model.
  /// @note In general setting the pose and/or velocity of a body in the model
  /// would involve a complex inverse kinematics problem. This method allows us
  /// to simplify this process when we know the body is free in space.
  /// @throws std::exception if `body` is not a free body in the model.
  /// @throws std::exception if called pre-finalize.
  /// @pre `state` comes from this MultibodyPlant.
  void SetFreeBodyPose(
      const systems::Context<T>& context, systems::State<T>* state,
      const Body<T>& body, const math::RigidTransform<T>& X_WB) const {
    CheckValidState(state);
    internal_tree().SetFreeBodyPoseOrThrow(body, X_WB, context, state);
  }

  /// Sets the default pose of `body`. If `body.is_floating()` is true, this
  /// will affect subsequent calls to SetDefaultState(); otherwise, this value
  /// is effectively ignored.
  /// @param[in] body
  ///   Body whose default pose will be set.
  /// @param[in] X_WB
  ///   Default pose of the body.
  void SetDefaultFreeBodyPose(
      const Body<T>& body, const math::RigidTransform<double>& X_WB) {
    X_WB_default_list_[body.index()] = X_WB;
  }

  /// Sets `context` to store the spatial velocity `V_WB` of a given `body` B in
  /// the world frame W.
  /// @note In general setting the pose and/or velocity of a body in the model
  /// would involve a complex inverse kinematics problem. This method allows us
  /// to simplify this process when we know the body is free in space.
  /// @throws std::exception if `body` is not a free body in the model.
  /// @throws std::exception if called pre-finalize.
  void SetFreeBodySpatialVelocity(
      systems::Context<T>* context, const Body<T>& body,
      const SpatialVelocity<T>& V_WB) const {
    internal_tree().SetFreeBodySpatialVelocityOrThrow(body, V_WB, context);
  }

  /// Sets `state` to store the spatial velocity `V_WB` of a given `body` B in
  /// the world frame W, for a given `context` of `this` model.
  /// @note In general setting the pose and/or velocity of a body in the model
  /// would involve a complex inverse kinematics problem. This method allows us
  /// to simplify this process when we know the body is free in space.
  /// @throws std::exception if `body` is not a free body in the model.
  /// @throws std::exception if called pre-finalize.
  /// @pre `state` comes from this MultibodyPlant.
  void SetFreeBodySpatialVelocity(
      const systems::Context<T>& context, systems::State<T>* state,
      const Body<T>& body, const SpatialVelocity<T>& V_WB) const {
    CheckValidState(state);
    internal_tree().SetFreeBodySpatialVelocityOrThrow(
        body, V_WB, context, state);
  }

  /// Sets the distribution used by SetRandomState() to populate the free
  /// body's x-y-z `position` with respect to World.
  /// @throws std::exception if `body` is not a free body in the model.
  /// @throws std::exception if called pre-finalize.
  void SetFreeBodyRandomPositionDistribution(
      const Body<T>& body, const Vector3<symbolic::Expression>& position) {
    this->mutable_tree().SetFreeBodyRandomPositionDistributionOrThrow(body,
                                                                      position);
  }

  /// Sets the distribution used by SetRandomState() to populate the free
  /// body's `rotation` with respect to World.
  /// @throws std::exception if `body` is not a free body in the model.
  /// @throws std::exception if called pre-finalize.
  void SetFreeBodyRandomRotationDistribution(
      const Body<T>& body,
      const Eigen::Quaternion<symbolic::Expression>& rotation) {
    this->mutable_tree().SetFreeBodyRandomRotationDistributionOrThrow(
        body, rotation);
  }

  /// Sets the distribution used by SetRandomState() to populate the free
  /// body's rotation with respect to World using uniformly random rotations.
  /// @throws std::exception if `body` is not a free body in the model.
  /// @throws std::exception if called pre-finalize.
  void SetFreeBodyRandomRotationDistributionToUniform(const Body<T>& body);

  /// Sets `context` to store the pose `X_WB` of a given `body` B in the world
  /// frame W.
  /// @param[in] context
  ///   The context to store the pose `X_WB` of `body_B`.
  /// @param[in] body_B
  ///   The body B corresponding to the pose `X_WB` to be stored in `context`.
  /// @retval X_WB
  ///   The pose of body frame B in the world frame W.
  /// @note In general setting the pose and/or velocity of a body in the model
  /// would involve a complex inverse kinematics problem. This method allows us
  /// to simplify this process when we know the body is free in space.
  /// @throws std::exception if `body` is not a free body in the model.
  /// @throws std::logic_error if called pre-finalize.
  void SetFreeBodyPoseInWorldFrame(
      systems::Context<T>* context,
      const Body<T>& body, const math::RigidTransform<T>& X_WB) const;

  /// Updates `context` to store the pose `X_FB` of a given `body` B in a frame
  /// F.
  /// Frame F must be anchored, meaning that it is either directly welded to the
  /// world frame W or, more generally, that there is a kinematic path between
  /// frame F and the world frame W that only includes weld joints.
  /// @throws std::logic_error if called pre-finalize.
  /// @throws std::logic_error if frame F is not anchored to the world.
  void SetFreeBodyPoseInAnchoredFrame(
      systems::Context<T>* context,
      const Frame<T>& frame_F, const Body<T>& body,
      const math::RigidTransform<T>& X_FB) const;
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
  /// @throws std::exception if Finalize() was not called on `this` model or if
  /// `body_B` does not belong to this model.
  const math::RigidTransform<T>& EvalBodyPoseInWorld(
      const systems::Context<T>& context,
      const Body<T>& body_B) const {
    return internal_tree().EvalBodyPoseInWorld(context, body_B);
  }

  /// Evaluate the spatial velocity `V_WB` of a body B in the world frame W.
  /// @param[in] context
  ///   The context storing the state of the model.
  /// @param[in] body_B
  ///   The body B for which the spatial velocity is requested.
  /// @returns V_WB
  ///   The spatial velocity of body frame B in the world frame W.
  /// @throws std::exception if Finalize() was not called on `this` model or if
  /// `body_B` does not belong to this model.
  const SpatialVelocity<T>& EvalBodySpatialVelocityInWorld(
      const systems::Context<T>& context,
      const Body<T>& body_B) const {
    return internal_tree().EvalBodySpatialVelocityInWorld(context, body_B);
  }

  /// Evaluates all point pairs of contact for a given state of the model stored
  /// in `context`.
  /// Each entry in the returned vector corresponds to a single point pair
  /// corresponding to two interpenetrating bodies A and B. The size of the
  /// returned vector corresponds to the total number of contact penetration
  /// pairs. If no geometry was registered, the output vector is empty.
  /// @see @ref mbp_geometry "Geometry" for geometry registration.
  /// @see PenetrationAsPointPair for further details on the returned data.
  /// @throws std::exception if called pre-finalize. See Finalize().
  const std::vector<geometry::PenetrationAsPointPair<T>>&
  EvalPointPairPenetrations(const systems::Context<T>& context) const {
    DRAKE_MBP_THROW_IF_NOT_FINALIZED();
    switch (contact_model_) {
      case ContactModel::kPointContactOnly:
        return this->get_cache_entry(cache_indexes_.point_pairs)
            .template Eval<std::vector<geometry::PenetrationAsPointPair<T>>>(
            context);
      case ContactModel::kHydroelasticWithFallback: {
        const auto& data =
            this->get_cache_entry(cache_indexes_.hydro_fallback)
                .template Eval<internal::HydroelasticFallbackCacheData<T>>(
                    context);
        return data.point_pairs;
      }
      default:
        throw std::logic_error(
            "Attempting to evaluate point pair contact for contact model that "
            "doesn't use it");
    }
  }

  /// Calculates the rigid transform (pose) `X_FG` relating frame F and frame G.
  /// @param[in] context
  ///    The state of the multibody system, which includes the system's
  ///    generalized positions q.  Note: `X_FG` is a function of q.
  /// @param[in] frame_F
  ///    The frame F designated in the rigid transform `X_FG`.
  /// @param[in] frame_G
  ///    The frame G designated in the rigid transform `X_FG`.
  /// @retval X_FG
  ///    The RigidTransform relating frame F and frame G.
  math::RigidTransform<T> CalcRelativeTransform(
      const systems::Context<T>& context,
      const Frame<T>& frame_F,
      const Frame<T>& frame_G) const {
    return internal_tree().CalcRelativeTransform(context, frame_F, frame_G);
  }

  /// Calculates the rotation matrix `R_FG` relating frame F and frame G.
  /// @param[in] context
  ///    The state of the multibody system, which includes the system's
  ///    generalized positions q.  Note: `R_FG` is a function of q.
  /// @param[in] frame_F
  ///    The frame F designated in the rigid transform `R_FG`.
  /// @param[in] frame_G
  ///    The frame G designated in the rigid transform `R_FG`.
  /// @retval R_FG
  ///    The RigidTransform relating frame F and frame G.
  math::RotationMatrix<T> CalcRelativeRotationMatrix(
      const systems::Context<T>& context,
      const Frame<T>& frame_F,
      const Frame<T>& frame_G) const {
    return internal_tree().CalcRelativeRotationMatrix(context,
                                                      frame_F, frame_G);
  }

  /// Given the positions `p_BQi` for a set of points `Qi` measured and
  /// expressed in a frame B, this method computes the positions `p_AQi(q)` of
  /// each point `Qi` in the set as measured and expressed in another frame A,
  /// as a function of the generalized positions q of the model.
  ///
  /// @param[in] context
  ///   The context containing the state of the model. It stores the
  ///   generalized positions q of the model.
  /// @param[in] frame_B
  ///   The frame B in which the positions `p_BQi` of a set of points `Qi` are
  ///   given.
  /// @param[in] p_BQi
  ///   The input positions of each point `Qi` in frame B. `p_BQi ∈ ℝ³ˣⁿᵖ` with
  ///   `np` the number of points in the set. Each column of `p_BQi` corresponds
  ///   to a vector in ℝ³ holding the position of one of the points in the set
  ///   as measured and expressed in frame B.
  /// @param[in] frame_A
  ///   The frame A in which it is desired to compute the positions `p_AQi` of
  ///   each point `Qi` in the set.
  /// @param[out] p_AQi
  ///   The output positions of each point `Qi` now computed as measured and
  ///   expressed in frame A. The output `p_AQi` **must** have the same size as
  ///   the input `p_BQi` or otherwise this method aborts. That is `p_AQi`
  ///   **must** be in `ℝ³ˣⁿᵖ`.
  ///
  /// @note Both `p_BQi` and `p_AQi` must have three rows. Otherwise this
  /// method will throw a std::runtime_error exception. This method also throws
  /// a std::runtime_error exception if `p_BQi` and `p_AQi` differ in the number
  /// of columns.
  void CalcPointsPositions(
      const systems::Context<T>& context,
      const Frame<T>& frame_B,
      const Eigen::Ref<const MatrixX<T>>& p_BQi,
      const Frame<T>& frame_A,
      EigenPtr<MatrixX<T>> p_AQi) const {
    return internal_tree().CalcPointsPositions(
        context, frame_B, p_BQi, frame_A, p_AQi);
  }

  /// This method computes the center of mass position p_WCcm of all bodies in
  /// `MultibodyPlant` measured and expressed in world frame W. The bodies are
  /// considered as a single composite body C, whose center of mass
  /// `composite_mass` is located at Ccm. The world_body() is ignored.
  ///
  /// @param[in] context
  ///   The context containing the state of the model. It stores the
  ///   generalized positions q of the model.
  /// @retval p_WCcm
  ///   The output position of center of mass in the world frame W.
  ///
  /// @throws std::runtime_error if `MultibodyPlant` has no body except
  ///   `world_body()`.
  /// @throws std::runtime_error unless `composite_mass > 0`.
  Vector3<T> CalcCenterOfMassPosition(
      const systems::Context<T>& context) const {
    return internal_tree().CalcCenterOfMassPosition(context);
  }

  /// This method computes the center of mass position p_WCcm of specified model
  /// instances measured and expressed in world frame W. The specified model
  /// instances are considered as a single composite body C, whose center of
  /// mass `composite_mass` is located at Ccm. The models are selected by a
  /// vector of model instances `model_instances`. This function does not
  /// distinguish between welded bodies, joint connected bodies and free
  /// bodies in the `model_instances`. The world_body() is ignored.
  ///
  /// @param[in] context
  ///   The context containing the state of the model. It stores the
  ///   generalized positions q of the model.
  /// @param[in] model_instances
  ///   The vector of selected model instances.
  /// @retval p_WCcm
  ///   The output position of center of mass in the world frame W.
  ///
  /// @throws std::runtime_error if `MultibodyPlant` has no model_instance
  ///   except `world_model_instance()`.
  /// @throws std::runtime_error unless `composite_mass > 0`.
  Vector3<T> CalcCenterOfMassPosition(
      const systems::Context<T>& context,
      const std::vector<ModelInstanceIndex>& model_instances) const {
    return internal_tree().CalcCenterOfMassPosition(context, model_instances);
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
      const systems::Context<T>& context,
      const VectorX<T>& known_vdot,
      std::vector<SpatialAcceleration<T>>* A_WB_array) const;

  /// Given the state of this model in `context` and a known vector
  /// of generalized accelerations `vdot`, this method computes the
  /// set of generalized forces `tau` that would need to be applied in order to
  /// attain the specified generalized accelerations.
  /// Mathematically, this method computes: <pre>
  ///   tau = M(q)v̇ + C(q, v)v - tau_app - ∑ J_WBᵀ(q) Fapp_Bo_W
  /// </pre>
  /// where `M(q)` is the model's mass matrix, `C(q, v)v` is the bias
  /// term containing Coriolis and gyroscopic effects and `tau_app` consists
  /// of a vector applied generalized forces. The last term is a summation over
  /// all bodies in the model where `Fapp_Bo_W` is an applied spatial force on
  /// body B at `Bo` which gets projected into the space of generalized forces
  /// with the transpose of `Jv_V_WB(q)` (where `Jv_V_WB` is B's spatial
  /// velocity Jacobian in W with respect to generalized velocities v).
  /// Note: B's spatial velocity in W can be written as `V_WB = Jv_V_WB * v`.
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
      const systems::Context<T>& context,
      const VectorX<T>& known_vdot,
      const MultibodyForces<T>& external_forces) const {
    return internal_tree().CalcInverseDynamics(
        context, known_vdot, external_forces);
  }

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
  void CalcForceElementsContribution(
      const systems::Context<T>& context, MultibodyForces<T>* forces) const;

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
    return internal_tree().CalcGravityGeneralizedForces(context);
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
  ///   A vector of of generalized velocities for this model.
  ///   This method aborts if v is not of size num_velocities().
  /// @param[out] qdot
  ///   A valid (non-null) pointer to a vector in `ℝⁿ` with n being the number
  ///   of generalized positions in this model,
  ///   given by `num_positions()`. This method aborts if `qdot` is nullptr
  ///   or if it is not of size num_positions().
  ///
  /// @see MapQDotToVelocity()
  /// @see Mobilizer::MapVelocityToQDot()
  void MapVelocityToQDot(
      const systems::Context<T>& context,
      const Eigen::Ref<const VectorX<T>>& v,
      EigenPtr<VectorX<T>> qdot) const {
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
  void MapQDotToVelocity(
      const systems::Context<T>& context,
      const Eigen::Ref<const VectorX<T>>& qdot,
      EigenPtr<VectorX<T>> v) const {
    internal_tree().MapQDotToVelocity(context, qdot, v);
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

  /// Performs the computation of the mass matrix `M(q)` of the model using
  /// inverse dynamics, where the generalized positions q are stored in
  /// `context`. See CalcInverseDynamics().
  ///
  /// Use CalcMassMatrix() for a faster implementation using the Composite Body
  /// Algorithm.
  ///
  /// @param[in] context
  ///   The context containing the state of the model.
  /// @param[out] M
  ///   A valid (non-null) pointer to a squared matrix in `ℛⁿˣⁿ` with n the
  ///   number of generalized velocities (num_velocities()) of the model.
  ///   This method aborts if H is nullptr or if it does not have the proper
  ///   size.
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
  void CalcMassMatrixViaInverseDynamics(
      const systems::Context<T>& context, EigenPtr<MatrixX<T>> M) const {
    internal_tree().CalcMassMatrixViaInverseDynamics(context, M);
  }

  /// Performs the computation of the mass matrix `M(q)` of the model, as a
  /// function of the generalized positions q stored in `context`.
  /// This method employs the Composite Body Algorithm, which is known to be the
  /// fastest O(n²) algorithm to compute the mass matrix of a multibody system.
  ///
  /// @param[in] context
  ///   The context containing the state of the model.
  /// @param[out] M
  ///   A valid (non-null) pointer to a squared matrix in `ℛⁿˣⁿ` with n the
  ///   number of generalized velocities (num_velocities()) of the model.
  ///   This method aborts if M is nullptr or if it does not have the proper
  ///   size.
  ///
  /// @warning This is an O(n²) algorithm. Avoid the explicit computation of the
  /// mass matrix whenever possible.
  void CalcMassMatrix(const systems::Context<T>& context,
                      EigenPtr<MatrixX<T>> M) const {
    internal_tree().CalcMassMatrix(context, M);
  }

  /// Computes the bias term `C(q, v)v` containing Coriolis, centripetal, and
  /// gyroscopic effects in the multibody equations of motion: <pre>
  ///   M(q) v̇ + C(q, v) v = tau_app + ∑ (Jv_V_WBᵀ(q) ⋅ Fapp_Bo_W)
  /// </pre>
  /// where `M(q)` is the multibody model's mass matrix and `tau_app` is a
  /// vector of generalized forces. The last term is a summation over all bodies
  /// of the dot-product of `Fapp_Bo_W` (applied spatial force on body B at Bo)
  /// with `Jv_V_WB(q)` (B's spatial Jacobian in world W with respect to
  /// generalized velocities v).
  /// Note: B's spatial velocity in W can be written `V_WB = Jv_V_WB * v`.
  ///
  /// @param[in] context
  ///   The context containing the state of the model. It stores the
  ///   generalized positions q and the generalized velocities v.
  /// @param[out] Cv
  ///   On output, `Cv` will contain the product `C(q, v)v`. It must be a valid
  ///   (non-null) pointer to a column vector in `ℛⁿ` with n the number of
  ///   generalized velocities (num_velocities()) of the model.
  ///   This method aborts if Cv is nullptr or if it does not have the
  ///   proper size.
  void CalcBiasTerm(
      const systems::Context<T>& context, EigenPtr<VectorX<T>> Cv) const {
    internal_tree().CalcBiasTerm(context, Cv);
  }

  /// For a point Fp that is fixed to a frame F, calculates Fp's translational
  /// acceleration "bias" term `abias_AFp = J̇s_v_AFp(q, s) * s` in frame A with
  /// respect to "speeds" 𝑠.
  /// <pre>
  ///   a_AFp = J𝑠_v_AFp(q)⋅ṡ + abias_AFp(q, v)
  /// </pre>
  /// a_AFp is point Fp's translational acceleration in frame A and 𝑠 is either
  /// q̇ ≜ [q̇₁ ... q̇ⱼ]ᵀ (time-derivatives of generalized positions) or
  /// v ≜ [v₁ ... vₖ]ᵀ (generalized velocities).
  /// Note: `abias_AFp = J̇s_v_AFp(q, s)⋅s`  is quadratic in 𝑠 ≜ [𝑠₁ ... 𝑠ₙ]ᵀ
  /// Note: This method is misnamed CalcBiasForJacobianTranslationalVelocity.
  /// Expect a name change to reflect its acceleration (not velocity) nature.
  ///
  /// This method computes `abias_AFp` for each point Fp in the `p_FP_list`.
  /// The `p_FP_list` is a list of position vectors from Fo (Frame F's origin)
  /// to each such point Fp, expressed in frame F.
  ///
  /// @see CalcJacobianTranslationalVelocity() to compute `J𝑠_v_AFp`, point Fp's
  /// translational velocity Jacobian in frame A with respect to s.
  ///
  /// @param[in] context The state of the multibody system, which includes the
  /// generalized positions q and generalized velocities v.
  /// @param[in] with_respect_to Enum equal to JacobianWrtVariable::kQDot or
  /// JacobianWrtVariable::kV, indicating whether the Jacobian `J𝑠_v_AFp` is
  /// partial derivatives with respect to 𝑠 = q̇ (time-derivatives of generalized
  /// positions) or with respect to 𝑠 = v (generalized velocities).
  /// @param[in] frame_F The frame on which point Fp is fixed/welded.
  /// @param[in] p_FP_list `3 x n` matrix of position vectors `p_FoFp_F` from
  /// Fo (frame F's origin) to each such point Fp, expressed in frame F.
  /// @param[in] frame_A The frame that measures `abias_AFp`.
  /// Currently, an exception is thrown if frame_A is not the World frame.
  /// @param[in] frame_E The frame in which `abias_AFp` is expressed on output.
  /// @returns abias_AFp_E matrix of translational acceleration bias terms
  /// in frame_A and expressed in frame_E for each of the `n` points associated
  /// with p_FP_list.  These bias terms are functions of the generalized
  /// positions q and the generalized velocities v and depend on whether
  /// `with_respect_to` is kQDot or kV.
  /// @throws std::exception if `p_FP_list` does not have 3 rows.
  /// @throws std::exception if `with_respect_to` is not JacobianWrtVariable::kV
  /// @throws std::exception if frame_A is not the world frame.
  VectorX<T> CalcBiasForJacobianTranslationalVelocity(
      const systems::Context<T>& context,
      JacobianWrtVariable with_respect_to,
      const Frame<T>& frame_F,
      const Eigen::Ref<const MatrixX<T>>& p_FP_list,
      const Frame<T>& frame_A,
      const Frame<T>& frame_E) const {
    // TODO(Mitiguy) Issue #12140: Rename to CalcBiasTranslationalAcceleration.
    // TODO(Mitiguy) Allow `with_respect_to` to be JacobianWrtVariable::kQDot
    // and/or allow frame_A to be a non-world frame.
    return internal_tree().CalcBiasForJacobianTranslationalVelocity(
        context, with_respect_to, frame_F, p_FP_list, frame_A, frame_E);
  }

  /// For a point Fp that is fixed to a frame F, calculates Fp's spatial
  /// acceleration "bias" term `Abias_AFp = J̇s_V_AFp * s` in frame A with
  /// respect to "speeds" 𝑠.
  /// <pre>
  ///   A_AFp = J𝑠_V_AFp(q)⋅ṡ + Abias_AFp(q, v)
  /// </pre>
  /// A_AFp is point Fp's spatial acceleration in frame A and 𝑠 is either
  /// q̇ ≜ [q̇₁ ... q̇ⱼ]ᵀ (time-derivatives of generalized positions) or
  /// v ≜ [v₁ ... vₖ]ᵀ (generalized velocities).
  /// Note: `Abias_AFp = J̇s_V_AFp(q, s)⋅s`  is quadratic in 𝑠 ≜ [𝑠₁ ... 𝑠ₙ]ᵀ
  /// Note: This method is misnamed CalcBiasForJacobianSpatialVelocity.
  /// Expect a name change to reflect its acceleration (not velocity) nature.
  ///
  /// @see CalcJacobianSpatialVelocity() to compute `J𝑠_V_AFp`, point Fp's
  /// spatial velocity Jacobian in frame A with respect to 𝑠.
  ///
  /// @param[in] context The state of the multibody system, which includes the
  /// generalized positions q and generalized velocities v.
  /// @param[in] with_respect_to Enum equal to JacobianWrtVariable::kQDot or
  /// JacobianWrtVariable::kV, indicating whether the Jacobian `J𝑠_v_AFp` is
  /// partial derivatives with respect to 𝑠 = q̇ (time-derivatives of generalized
  /// positions) or with respect to 𝑠 = v (generalized velocities).
  /// @param[in] frame_F The frame on which point Fp is fixed/welded.
  /// @param[in] p_FoFp_F position vector from Fo (frame F's origin) to
  /// point Fp, expressed in frame F.
  /// @param[in] frame_A The frame that measures `Abias_AFp`.
  /// Currently, an exception is thrown if frame_A is not the World frame.
  /// @param[in] frame_E The frame in which `Abias_AFp` is expressed on output.
  /// @returns Abias_AFp_E Fp's spatial acceleration bias in frame_A is returned
  /// in a `6 x 1` matrix whose first three elements are frame_F's angular
  /// acceleration bias in frame_A (expressed in frame_E) and whose last three
  /// elements are point Fp's translational acceleration bias in frame_A
  /// (expressed in frame_E).  These bias terms are functions of the generalized
  /// positions q and the generalized velocities v and depend on whether
  /// `with_respect_to` is kQDot or kV.  Note: Although the return quantity is a
  /// Vector6, it is actually a SpatialAcceleration (having units of that type).
  /// @throws std::exception if `with_respect_to` is not JacobianWrtVariable::kV
  /// @throws std::exception if frame_A is not the world frame.
  Vector6<T> CalcBiasForJacobianSpatialVelocity(
      const systems::Context<T>& context,
      JacobianWrtVariable with_respect_to,
      const Frame<T>& frame_F,
      const Eigen::Ref<const Vector3<T>>& p_FoFp_F,
      const Frame<T>& frame_A,
      const Frame<T>& frame_E) const {
    // TODO(Mitiguy) Issue #12140: Rename to CalcBiasSpatialAcceleration.
    // TODO(Mitiguy) Allow `with_respect_to` to be JacobianWrtVariable::kQDot
    // and/or allow frame_A to be a non-world frame.
    return internal_tree().CalcBiasForJacobianSpatialVelocity(
        context, with_respect_to, frame_F, p_FoFp_F, frame_A, frame_E);
  }

  /// For each point Bi of (fixed to) a frame B, calculates J𝑠_V_ABi, Bi's
  /// spatial velocity Jacobian in frame A with respect to "speeds" 𝑠.
  /// <pre>
  ///      J𝑠_V_ABi = [ ∂(V_ABi)/∂𝑠₁,  ...  ∂(V_ABi)/∂𝑠ₙ ]    (n is j or k)
  /// </pre>
  /// `V_ABi` is Bi's spatial velocity in frame A and "speeds" 𝑠 is either
  /// q̇ ≜ [q̇₁ ... q̇ⱼ]ᵀ (time-derivatives of j generalized positions) or
  /// v ≜ [v₁ ... vₖ]ᵀ (k generalized velocities).
  /// Note: `V_ABi = J𝑠_V_ABi ⋅ 𝑠`  which is linear in 𝑠 ≜ [𝑠₁ ... 𝑠ₙ]ᵀ.
  ///
  /// @param[in] context The state of the multibody system.
  /// @param[in] with_respect_to Enum equal to JacobianWrtVariable::kQDot or
  /// JacobianWrtVariable::kV, indicating whether the Jacobian `J𝑠_V_ABi` is
  /// partial derivatives with respect to 𝑠 = q̇ (time-derivatives of generalized
  /// positions) or with respect to 𝑠 = v (generalized velocities).
  /// @param[in] frame_B The frame on which point Bi is fixed (e.g., welded).
  /// @param[in] p_BoBi_B A position vector or list of p position vectors from
  /// Bo (frame_B's origin) to points Bi (regarded as fixed to B), where each
  /// position vector is expressed in frame_B.
  /// @param[in] frame_A The frame that measures `v_ABi` (Bi's velocity in A).
  /// Note: It is natural to wonder why there is no parameter p_AoAi_A (similar
  /// to the parameter p_BoBi_B for frame_B).  There is no need for p_AoAi_A
  /// because Bi's velocity in A is defined as the derivative in frame A of
  /// Bi's position vector from _any_ point fixed on A.
  /// @param[in] frame_E The frame in which `v_ABi` is expressed on input and
  /// the frame in which the Jacobian `J𝑠_V_ABi` is expressed on output.
  /// @param[out] J𝑠_V_ABi_E Point Bi's spatial velocity Jacobian in frame A
  /// with respect to speeds 𝑠 (which is either q̇ or v), expressed in frame E.
  /// `J𝑠_V_ABi_E` is a `6*p x n` matrix, where p is the number of points Bi and
  /// n is the number of elements in 𝑠.  The Jacobian is a function of only
  /// generalized positions q (which are pulled from the context).
  /// Note: If p = 1 (one point), a `6 x n` matrix is returned with the first
  /// three rows storing frame B's angular velocity Jacobian in A and rows 4-6
  /// storing point Bi's translational velocity Jacobian in A, i.e.,
  ///   ```
  ///     J𝑠_wAB_E = J𝑠_V_ABi_E.topRows<3>();
  ///     J𝑠_vAB1_E = J𝑠_V_ABi_E.bottomRows<3>();
  ///   ```
  /// If p = 2 (two points), a `12 x n` matrix is returned.  Rows 1-3 and 7-9
  /// store exactly identical information (B's angular velocity Jacobian in A).
  /// Rows 4-6 store point B1's translational velocity Jacobian which differs
  /// from rows 10-12 which store point B2's translational velocity Jacobian.
  /// If p is large and storage efficiency is a concern, calculate frame B's
  /// angular velocity Jacobian using CalcJacobianAngularVelocity() and then use
  /// CalcJacobianTranslationalVelocity().
  /// @throws std::exception if `J𝑠_V_ABi_E` is nullptr or not sized `3*p x n`.
  void CalcJacobianSpatialVelocity(const systems::Context<T>& context,
                                   JacobianWrtVariable with_respect_to,
                                   const Frame<T>& frame_B,
                                   const Eigen::Ref<const Vector3<T>>& p_BoBi_B,
                                   const Frame<T>& frame_A,
                                   const Frame<T>& frame_E,
                                   EigenPtr<MatrixX<T>> Jw_ABp_E) const {
    internal_tree().CalcJacobianSpatialVelocity(context, with_respect_to,
                                                frame_B, p_BoBi_B, frame_A,
                                                frame_E, Jw_ABp_E);
  }

  /// Calculates J𝑠_w_AB, a frame B's angular velocity Jacobian in a frame A
  /// with respect to "speeds" 𝑠.
  /// <pre>
  ///      J𝑠_w_AB = [ ∂(w_AB)/∂𝑠₁,  ...  ∂(w_AB)/∂𝑠ₙ ]    (n is j or k)
  /// </pre>
  /// `w_AB` is B's angular velocity in frame A and "speeds" 𝑠 is either
  /// q̇ ≜ [q̇₁ ... q̇ⱼ]ᵀ (time-derivatives of j generalized positions) or
  /// v ≜ [v₁ ... vₖ]ᵀ (k generalized velocities).
  /// Note: `w_AB = J𝑠_w_AB * 𝑠`  which is linear in 𝑠 ≜ [𝑠₁ ... 𝑠ₙ]ᵀ.
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
  /// @param[out] J𝑠_w_AB_E Frame B's angular velocity Jacobian in frame A with
  /// respect to speeds 𝑠 (which is either q̇ or v), expressed in frame E.
  /// The Jacobian is a function of only generalized positions q (which are
  /// pulled from the context).  The previous definition shows `J𝑠_w_AB_E` is
  /// a matrix of size `3 x n`, where n is the number of elements in 𝑠.
  /// @throws std::exception if `J𝑠_w_AB_E` is nullptr or not of size `3 x n`.
  void CalcJacobianAngularVelocity(const systems::Context<T>& context,
                                   const JacobianWrtVariable with_respect_to,
                                   const Frame<T>& frame_B,
                                   const Frame<T>& frame_A,
                                   const Frame<T>& frame_E,
                                   EigenPtr<Matrix3X<T>> Js_w_AB_E) const {
    return internal_tree().CalcJacobianAngularVelocity(
        context, with_respect_to, frame_B, frame_A, frame_E, Js_w_AB_E);
  }

  /// For each point Bi of (fixed to) a frame B, calculates J𝑠_v_ABi, Bi's
  /// translational velocity Jacobian in frame A with respect to "speeds" 𝑠.
  /// <pre>
  ///      J𝑠_v_ABi = [ ∂(v_ABi)/∂𝑠₁,  ...  ∂(v_ABi)/∂𝑠ₙ ]    (n is j or k)
  /// </pre>
  /// `v_ABi` is Bi's translational velocity in frame A and "speeds" 𝑠 is either
  /// q̇ ≜ [q̇₁ ... q̇ⱼ]ᵀ (time-derivatives of j generalized positions) or
  /// v ≜ [v₁ ... vₖ]ᵀ (k generalized velocities).
  /// Note: `v_ABi = J𝑠_v_ABi ⋅ 𝑠`  which is linear in 𝑠 ≜ [𝑠₁ ... 𝑠ₙ]ᵀ.
  ///
  /// @param[in] context The state of the multibody system.
  /// @param[in] with_respect_to Enum equal to JacobianWrtVariable::kQDot or
  /// JacobianWrtVariable::kV, indicating whether the Jacobian `J𝑠_v_ABi` is
  /// partial derivatives with respect to 𝑠 = q̇ (time-derivatives of generalized
  /// positions) or with respect to 𝑠 = v (generalized velocities).
  /// @param[in] frame_B The frame on which point Bi is fixed (e.g., welded).
  /// @param[in] p_BoBi_B A position vector or list of p position vectors from
  /// Bo (frame_B's origin) to points Bi (regarded as fixed to B), where each
  /// position vector is expressed in frame_B.
  /// @param[in] frame_A The frame that measures `v_ABi` (Bi's velocity in A).
  /// Note: It is natural to wonder why there is no parameter p_AoAi_A (similar
  /// to the parameter p_BoBi_B for frame_B).  There is no need for p_AoAi_A
  /// because Bi's velocity in A is defined as the derivative in frame A of
  /// Bi's position vector from _any_ point fixed on A.
  /// @param[in] frame_E The frame in which `v_ABi` is expressed on input and
  /// the frame in which the Jacobian `J𝑠_v_ABi` is expressed on output.
  /// @param[out] J𝑠_v_ABi_E Point Bi's velocity Jacobian in frame A with
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
  void CalcJacobianTranslationalVelocity(
      const systems::Context<T>& context, JacobianWrtVariable with_respect_to,
      const Frame<T>& frame_B, const Eigen::Ref<const Matrix3X<T>>& p_BoBi_B,
      const Frame<T>& frame_A, const Frame<T>& frame_E,
      EigenPtr<MatrixX<T>> Js_v_ABi_E) const {
    // TODO(amcastro-tri): provide the Jacobian-times-vector operation.  For
    // some applications it is all we need and it is more efficient to compute.
    internal_tree().CalcJacobianTranslationalVelocity(
        context, with_respect_to, frame_B, frame_B, p_BoBi_B, frame_A, frame_E,
        Js_v_ABi_E);
  }

  /// This method computes J𝑠_v_ACcm_E, point Ccm's translational velocity
  /// Jacobian in frame A with respect to "speeds" 𝑠, expressed in frame E,
  /// where point Ccm is the composite center of mass of the system of all
  /// bodies in the MultibodyPlant (except world_body()).
  ///
  /// @param[in] context The state of the multibody system.
  /// @param[in] with_respect_to Enum equal to JacobianWrtVariable::kQDot or
  /// JacobianWrtVariable::kV, indicating whether the Jacobian `J𝑠_v_ACcm_E` is
  /// partial derivatives with respect to 𝑠 = q̇ (time-derivatives of generalized
  /// positions) or with respect to 𝑠 = v (generalized velocities).
  /// @param[in] frame_A The frame in which the translational velocity
  /// v_ACcm and its Jacobian J𝑠_v_ACcm are measured.
  /// @param[in] frame_E The frame in which the Jacobian J𝑠_v_ACcm is
  /// expressed on output.
  /// @param[out] J𝑠_v_ACcm_E Point Ccm's translational velocity Jacobian in
  /// frame A with respect to speeds 𝑠 (𝑠 = q̇ or 𝑠 = v), expressed in frame E.
  /// J𝑠_v_ACcm_E is a 3 x n matrix, where n is the number of elements in 𝑠.
  /// The Jacobian is a function of only generalized positions q (which are
  /// pulled from the context).
  /// @throws std::runtime_error if CCm does not exist, which occurs if there
  /// are no massive bodies in MultibodyPlant (except world_body()).
  /// @throws std::runtime_error unless composite_mass > 0, where composite_mass
  /// is the total mass of all bodies except world_body() in MultibodyPlant.
  void CalcJacobianCenterOfMassTranslationalVelocity(
      const systems::Context<T>& context, JacobianWrtVariable with_respect_to,
      const Frame<T>& frame_A, const Frame<T>& frame_E,
      EigenPtr<Matrix3X<T>> Js_v_ACcm_E) const {
    // TODO(yangwill): Add an optional parameter to calculate this for a
    // subset of bodies instead of the full system
    internal_tree().CalcJacobianCenterOfMassTranslationalVelocity(
        context, with_respect_to, frame_A, frame_E, Js_v_ACcm_E);
  }

  /// Calculates abias_ACcm_E, point Ccm's translational "bias" acceleration
  /// term in frame A with respect to "speeds" 𝑠, expressed in frame E, where
  /// point Ccm is the composite center of mass of the system of all bodies
  /// (except world_body()) in the MultibodyPlant. abias_ACcm is the part of
  /// a_ACcm (Ccm's translational acceleration) that does not multiply ṡ, equal
  /// to abias_ACcm = J̇𝑠_v_ACcm * s. This allows a_ACcm to be written as
  /// a_ACcm = J̇𝑠_v_ACcm * s + abias_ACcm.
  ///
  /// @param[in] context The state of the multibody system.
  /// @param[in] with_respect_to Enum equal to JacobianWrtVariable::kQDot or
  /// JacobianWrtVariable::kV, indicating whether the Jacobian `abias_ACcm` is
  /// partial derivatives with respect to 𝑠 = q̇ (time-derivatives of generalized
  /// positions) or with respect to 𝑠 = v (generalized velocities).
  /// @param[in] frame_A The frame in which abias_ACcm is measured.
  /// @param[in] frame_E The frame in which abias_ACcm is expressed on output.
  /// @retval abias_ACcm_E Point Ccm's translational "bias" acceleration term
  /// in frame A with respect to "speeds" 𝑠, expressed in frame E.
  /// @throws std::runtime_error if Ccm does not exist, which occurs if there
  /// are no massive bodies in MultibodyPlant (except world_body()).
  /// @throws std::runtime_error unless composite_mass > 0, where composite_mass
  /// is the total mass of all bodies except world_body() in MultibodyPlant.
  /// @throws std::exception if frame_A is not the world frame.
  Vector3<T> CalcBiasCenterOfMassTranslationalAcceleration(
      const systems::Context<T>& context, JacobianWrtVariable with_respect_to,
      const Frame<T>& frame_A, const Frame<T>& frame_E) const {
    // TODO(yangwill): Add an optional parameter to calculate this for a
    // subset of bodies instead of the full system
    return internal_tree().CalcBiasCenterOfMassTranslationalAcceleration(
        context, with_respect_to, frame_A, frame_E);
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
  /// @throws std::logic_error if there are repeated indexes in
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
  /// `user_to_actuator_index_map`.
  /// The full vector of actuation values u is ordered by JointActuatorIndex.
  MatrixX<double> MakeActuatorSelectorMatrix(
      const std::vector<JointActuatorIndex>& user_to_actuator_index_map) const {
    return internal_tree().MakeActuatorSelectorMatrix(
        user_to_actuator_index_map);
  }

  /// This method creates an actuation matrix B mapping a vector of actuation
  /// values u into generalized forces `tau_u = B * u`, where B is a matrix of
  /// size `nv x nu` with `nu` equal to num_actuators() and `nv` equal to
  /// num_velocities().
  /// The vector u of actuation values is of size num_actuators(). For a given
  /// JointActuator, `u[JointActuator::index()]` stores the value for the
  /// external actuation corresponding to that actuator. `tau_u` on the other
  /// hand is indexed by generalized velocity indexes according to
  /// `Joint::velocity_start()`.
  /// @warning B is a permutation matrix. While making a permutation has
  /// `O(n)` complexity, making a full B matrix has `O(n²)` complexity. For most
  /// applications this cost can be neglected but it could become significant
  /// for very large systems.
  MatrixX<T> MakeActuationMatrix() const;

  /// Alternative signature to build an actuation selector matrix `Su` such
  /// that `u = Su⋅uₛ`, where u is the vector of actuation values for the full
  /// model (ordered by JointActuatorIndex) and uₛ is a vector of actuation
  /// values for the actuators acting on the joints listed by
  /// `user_to_joint_index_map`. It is assumed that all joints referenced by
  /// `user_to_joint_index_map` are actuated.
  /// See MakeActuatorSelectorMatrix(const std::vector<JointActuatorIndex>&) for
  /// details.
  /// @throws std::logic_error if any of the joints in
  /// `user_to_joint_index_map` does not have an actuator.
  MatrixX<double> MakeActuatorSelectorMatrix(
      const std::vector<JointIndex>& user_to_joint_index_map) const {
    return internal_tree().MakeActuatorSelectorMatrix(user_to_joint_index_map);
  }
  /// @} <!-- System matrix computations -->

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
  bool is_finalized() const { return internal_tree().topology_is_valid(); }

  /// Returns a constant reference to the *world* body.
  const RigidBody<T>& world_body() const {
    return internal_tree().world_body();
  }

  /// Returns a constant reference to the *world* frame.
  const BodyFrame<T>& world_frame() const {
    return internal_tree().world_frame();
  }

  /// Returns the number of bodies in the model, including the "world" body,
  /// which is always part of the model.
  /// @see AddRigidBody().
  int num_bodies() const {
    return internal_tree().num_bodies();
  }

  /// Returns a constant reference to the body with unique index `body_index`.
  /// @throws std::exception if `body_index` does not correspond to a body in
  /// this model.
  const Body<T>& get_body(BodyIndex body_index) const {
    return internal_tree().get_body(body_index);
  }

  /// Returns `true` if @p body is anchored (i.e. the kinematic path between
  /// @p body and the world only contains weld joints.)
  /// @throws std::exception if called pre-finalize.
  bool IsAnchored(const Body<T>& body) const {
    DRAKE_MBP_THROW_IF_NOT_FINALIZED();
    return internal_tree().get_topology().IsBodyAnchored(body.index());
  }

  /// @returns `true` if a body named `name` was added to the %MultibodyPlant.
  /// @see AddRigidBody().
  ///
  /// @throws std::logic_error if the body name occurs in multiple model
  /// instances.
  bool HasBodyNamed(const std::string& name) const {
    return internal_tree().HasBodyNamed(name);
  }

  /// @returns `true` if a body named `name` was added to the %MultibodyPlant
  /// in @p model_instance.
  /// @see AddRigidBody().
  ///
  /// @throws std::exception if @p model_instance is not valid for this model.
  bool HasBodyNamed(
      const std::string& name, ModelInstanceIndex model_instance) const {
    return internal_tree().HasBodyNamed(name, model_instance);
  }

  /// Returns a constant reference to a body that is identified
  /// by the string `name` in `this` %MultibodyPlant.
  /// @throws std::logic_error if there is no body with the requested name.
  /// @throws std::logic_error if the body name occurs in multiple model
  /// instances.
  /// @see HasBodyNamed() to query if there exists a body in `this`
  /// %MultibodyPlant with a given specified name.
  const Body<T>& GetBodyByName(const std::string& name) const {
    return internal_tree().GetBodyByName(name);
  }

  /// Returns a constant reference to the body that is uniquely identified
  /// by the string `name` and @p model_instance in `this` %MultibodyPlant.
  /// @throws std::logic_error if there is no body with the requested name.
  /// @see HasBodyNamed() to query if there exists a body in `this`
  /// %MultibodyPlant with a given specified name.
  const Body<T>& GetBodyByName(
      const std::string& name, ModelInstanceIndex model_instance) const {
    return internal_tree().GetBodyByName(name, model_instance);
  }

  /// Returns a list of body indices associated with `model_instance`.
  std::vector<BodyIndex> GetBodyIndices(ModelInstanceIndex model_instance)
  const {
    return internal_tree().GetBodyIndices(model_instance);
  }

  /// Returns a constant reference to a rigid body that is identified
  /// by the string `name` in `this` model.
  /// @throws std::logic_error if there is no body with the requested name.
  /// @throws std::logic_error if the body name occurs in multiple model
  /// instances.
  /// @throws std::logic_error if the requested body is not a RigidBody.
  /// @see HasBodyNamed() to query if there exists a body in `this` model with a
  /// given specified name.
  const RigidBody<T>& GetRigidBodyByName(const std::string& name) const {
    return internal_tree().GetRigidBodyByName(name);
  }

  /// Returns a constant reference to the rigid body that is uniquely identified
  /// by the string `name` in @p model_instance.
  /// @throws std::logic_error if there is no body with the requested name.
  /// @throws std::logic_error if the requested body is not a RigidBody.
  /// @throws std::runtime_error if @p model_instance is not valid for this
  ///         model.
  /// @see HasBodyNamed() to query if there exists a body in `this` model with a
  /// given specified name.
  const RigidBody<T>& GetRigidBodyByName(
      const std::string& name, ModelInstanceIndex model_instance) const {
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
  std::vector<const Body<T>*> GetBodiesWeldedTo(const Body<T>& body) const;

  /// Returns the number of joints in the model.
  /// @see AddJoint().
  int num_joints() const {
    return internal_tree().num_joints();
  }

  /// Returns a constant reference to the joint with unique index `joint_index`.
  /// @throws std::runtime_error when `joint_index` does not correspond to a
  /// joint in this model.
  const Joint<T>& get_joint(JointIndex joint_index) const {
    return internal_tree().get_joint(joint_index);
  }

  /// @returns `true` if a joint named `name` was added to this model.
  /// @see AddJoint().
  /// @throws std::logic_error if the joint name occurs in multiple model
  /// instances.
  bool HasJointNamed(const std::string& name) const {
    return internal_tree().HasJointNamed(name);
  }

  /// @returns `true` if a joint named `name` was added to @p model_instance.
  /// @see AddJoint().
  /// @throws std::exception if @p model_instance is not valid for this model.
  bool HasJointNamed(
      const std::string& name, ModelInstanceIndex model_instance) const {
    return internal_tree().HasJointNamed(name, model_instance);
  }

  /// Returns a mutable reference to the joint with unique index `joint_index`.
  /// @throws std::runtime_error when `joint_index` does not correspond to a
  /// joint in this model.
  Joint<T>& get_mutable_joint(JointIndex joint_index) {
    return this->mutable_tree().get_mutable_joint(joint_index);
  }

  /// Returns a list of joint indices associated with `model_instance`.
  std::vector<JointIndex> GetJointIndices(ModelInstanceIndex model_instance)
  const {
    return internal_tree().GetJointIndices(model_instance);
  }

  /// Returns a constant reference to a joint that is identified
  /// by the string `name` in `this` %MultibodyPlant.  If the optional
  /// template argument is supplied, then the returned value is downcast to
  /// the specified `JointType`.
  /// @tparam JointType The specific type of the Joint to be retrieved. It must
  /// be a subclass of Joint.
  /// @throws std::logic_error if the named joint is not of type `JointType` or
  /// if there is no Joint with that name.
  /// @throws std::exception if @p model_instance is not valid for this model.
  /// @see HasJointNamed() to query if there exists a joint in `this`
  /// %MultibodyPlant with a given specified name.
  template <template <typename> class JointType = Joint>
  const JointType<T>& GetJointByName(
      const std::string& name,
      std::optional<ModelInstanceIndex> model_instance = std::nullopt) const {
    return internal_tree().template GetJointByName<JointType>(
        name, model_instance);
  }

  /// A version of GetJointByName that returns a mutable reference.
  /// @see GetJointByName.
  template <template <typename> class JointType = Joint>
  JointType<T>& GetMutableJointByName(
      const std::string& name,
      std::optional<ModelInstanceIndex> model_instance = std::nullopt) {
    return this->mutable_tree().template GetMutableJointByName<JointType>(
        name, model_instance);
  }

  /// Returns the number of Frame objects in this model.
  /// Frames include body frames associated with each of the bodies,
  /// including the _world_ body. This means the minimum number of frames is
  /// one.
  int num_frames() const {
    return internal_tree().num_frames();
  }

  /// Returns a constant reference to the frame with unique index `frame_index`.
  /// @throws std::exception if `frame_index` does not correspond to a frame in
  /// this plant.
  const Frame<T>& get_frame(FrameIndex frame_index) const {
    return internal_tree().get_frame(frame_index);
  }

  /// @returns `true` if a frame named `name` was added to the model.
  /// @see AddFrame().
  /// @throws std::logic_error if the frame name occurs in multiple model
  /// instances.
  bool HasFrameNamed(const std::string& name) const {
    return internal_tree().HasFrameNamed(name);
  }

  /// @returns `true` if a frame named `name` was added to @p model_instance.
  /// @see AddFrame().
  /// @throws std::exception if @p model_instance is not valid for this model.
  bool HasFrameNamed(const std::string& name,
                     ModelInstanceIndex model_instance) const {
    return internal_tree().HasFrameNamed(name, model_instance);
  }

  /// Returns a constant reference to a frame that is identified by the
  /// string `name` in `this` model.
  /// @throws std::logic_error if there is no frame with the requested name.
  /// @throws std::logic_error if the frame name occurs in multiple model
  /// instances.
  /// @see HasFrameNamed() to query if there exists a frame in `this` model with
  /// a given specified name.
  const Frame<T>& GetFrameByName(const std::string& name) const {
    return internal_tree().GetFrameByName(name);
  }

  /// Returns a constant reference to the frame that is uniquely identified
  /// by the string `name` in @p model_instance.
  /// @throws std::logic_error if there is no frame with the requested name.
  /// @throws std::runtime_error if @p model_instance is not valid for this
  ///         model.
  /// @see HasFrameNamed() to query if there exists a frame in `this` model with
  /// a given specified name.
  const Frame<T>& GetFrameByName(
      const std::string& name, ModelInstanceIndex model_instance) const {
    return internal_tree().GetFrameByName(name, model_instance);
  }

  /// Returns the number of joint actuators in the model.
  /// @see AddJointActuator().
  int num_actuators() const {
    return internal_tree().num_actuators();
  }

  /// Returns the total number of actuated degrees of freedom.
  /// That is, the vector of actuation values u has this size.
  /// See AddJointActuator().
  int num_actuated_dofs() const { return internal_tree().num_actuated_dofs(); }

  /// Returns the total number of actuated degrees of freedom for a specific
  /// model instance.  That is, the vector of actuation values u has this size.
  /// See AddJointActuator().
  int num_actuated_dofs(ModelInstanceIndex model_instance) const {
    return internal_tree().num_actuated_dofs(model_instance);
  }

  /// Returns a constant reference to the joint actuator with unique index
  /// `actuator_index`.
  /// @throws std::exception if `actuator_index` does not correspond to a joint
  /// actuator in this tree.
  const JointActuator<T>& get_joint_actuator(
      JointActuatorIndex actuator_index) const {
    return internal_tree().get_joint_actuator(actuator_index);
  }

  /// @returns `true` if an actuator named `name` was added to this model.
  /// @see AddJointActuator().
  /// @throws std::logic_error if the actuator name occurs in multiple model
  /// instances.
  bool HasJointActuatorNamed(const std::string& name) const {
    return internal_tree().HasJointActuatorNamed(name);
  }

  /// @returns `true` if an actuator named `name` was added to
  /// @p model_instance.
  /// @see AddJointActuator().
  /// @throws std::exception if @p model_instance is not valid for this model.
  bool HasJointActuatorNamed(
      const std::string& name, ModelInstanceIndex model_instance) const {
    return internal_tree().HasJointActuatorNamed(name, model_instance);
  }

  /// Returns a constant reference to an actuator that is identified
  /// by the string `name` in `this` %MultibodyPlant.
  /// @throws std::logic_error if there is no actuator with the requested name.
  /// @throws std::logic_error if the actuator name occurs in multiple model
  /// instances.
  /// @see HasJointActuatorNamed() to query if there exists an actuator in
  /// `this` %MultibodyPlant with a given specified name.
  const JointActuator<T>& GetJointActuatorByName(
      const std::string& name) const {
    return internal_tree().GetJointActuatorByName(name);
  }

  /// Returns a constant reference to the actuator that is uniquely identified
  /// by the string `name` and @p model_instance in `this` %MultibodyPlant.
  /// @throws std::logic_error if there is no actuator with the requested name.
  /// @throws std::exception if @p model_instance is not valid for this model.
  /// @see HasJointActuatorNamed() to query if there exists an actuator in
  /// `this` %MultibodyPlant with a given specified name.
  const JointActuator<T>& GetJointActuatorByName(
      const std::string& name, ModelInstanceIndex model_instance) const {
    return internal_tree().GetJointActuatorByName(name, model_instance);
  }

  /// Returns the number of ForceElement objects.
  /// @see AddForceElement().
  int num_force_elements() const {
    return internal_tree().num_force_elements();
  }

  /// Returns a constant reference to the force element with unique index
  /// `force_element_index`.
  /// @throws std::runtime_error when `force_element_index` does not correspond
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
  /// @throws std::logic_error if the force element is not of type
  /// `ForceElementType` or if there is no ForceElement with that index.
  template <template <typename> class ForceElementType = ForceElement>
  const ForceElementType<T>& GetForceElement(
      ForceElementIndex force_element_index) const {
    return internal_tree().template GetForceElement<ForceElementType>(
        force_element_index);
  }

  /// An accessor to the current gravity field.
  const UniformGravityFieldElement<T>& gravity_field() const {
    return internal_tree().gravity_field();
  }

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
  /// @throws std::logic_error when `model_instance` does not correspond to a
  /// model in this model.
  const std::string& GetModelInstanceName(
      ModelInstanceIndex model_instance) const {
    return internal_tree().GetModelInstanceName(model_instance);
  }

  /// @returns `true` if a model instance named `name` was added to this model.
  /// @see AddModelInstance().
  bool HasModelInstanceNamed(const std::string& name) const {
    return internal_tree().HasModelInstanceNamed(name);
  }

  /// Returns the index to the model instance that is uniquely identified
  /// by the string `name` in `this` %MultibodyPlant.
  /// @throws std::logic_error if there is no instance with the requested name.
  /// @see HasModelInstanceNamed() to query if there exists an instance in
  /// `this` %MultibodyPlant with a given specified name.
  ModelInstanceIndex GetModelInstanceByName(const std::string& name) const {
    return internal_tree().GetModelInstanceByName(name);
  }

  /// Returns a Graphviz string describing the topology of this plant.
  /// To render the string, use the Graphviz tool, ``dot``.
  /// http://www.graphviz.org/
  ///
  /// Note: this method can be called either before or after `Finalize()`.
  std::string GetTopologyGraphvizString() const;

  /// Returns the size of the generalized position vector q for this model.
  int num_positions() const { return internal_tree().num_positions(); }

  /// Returns the size of the generalized position vector qᵢ for model
  /// instance i.
  int num_positions(ModelInstanceIndex model_instance) const {
    return internal_tree().num_positions(model_instance);
  }

  /// Returns the size of the generalized velocity vector v for this model.
  int num_velocities() const { return internal_tree().num_velocities(); }

  /// Returns the size of the generalized velocity vector vᵢ for model
  /// instance i.
  int num_velocities(ModelInstanceIndex model_instance) const {
    return internal_tree().num_velocities(model_instance);
  }

  // N.B. The state in the Context may at some point contain values such as
  // integrated power and other discrete states, hence the specific name.
  /// Returns the size of the multibody system state vector x = [q v]. This
  /// will be `num_positions()` plus `num_velocities()`.
  int num_multibody_states() const { return internal_tree().num_states(); }

  /// Returns the size of the multibody system state vector xᵢ = [qᵢ vᵢ] for
  /// model instance i. (Here qᵢ ⊆ q and vᵢ ⊆ v.)
  /// will be `num_positions(model_instance)` plus
  /// `num_velocities(model_instance)`.
  int num_multibody_states(ModelInstanceIndex model_instance) const {
    return internal_tree().num_states(model_instance);
  }

  /// Returns a vector of size `num_positions()` containing the lower position
  /// limits for every generalized position coordinate. These include joint and
  /// free body coordinates. Any unbounded or unspecified limits will be
  /// -infinity.
  /// @throws std::logic_error if called pre-finalize.
  VectorX<double> GetPositionLowerLimits() const {
    return internal_tree().GetPositionLowerLimits();
  }

  /// Upper limit analog of GetPositionsLowerLimits(), where any unbounded or
  /// unspecified limits will be +infinity.
  /// @see GetPositionLowerLimits() for more information.
  VectorX<double> GetPositionUpperLimits() const {
    return internal_tree().GetPositionUpperLimits();
  }

  /// Returns a vector of size `num_velocities()` containing the lower velocity
  /// limits for every generalized velocity coordinate. These include joint and
  /// free body coordinates. Any unbounded or unspecified limits will be
  /// -infinity.
  /// @throws std::logic_error if called pre-finalize.
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
  /// @throws std::logic_error if called pre-finalize.
  VectorX<double> GetAccelerationLowerLimits() const {
    return internal_tree().GetAccelerationLowerLimits();
  }

  /// Upper limit analog of GetAccelerationsLowerLimits(), where any unbounded
  /// or unspecified limits will be +infinity.
  /// @see GetAccelerationLowerLimits() for more information.
  VectorX<double> GetAccelerationUpperLimits() const {
    return internal_tree().GetAccelerationUpperLimits();
  }

  /// Returns the model used for contact. See documentation for ContactModel.
  ContactModel get_contact_model() const;

  /// Returns the friction coefficients provided during geometry registration
  /// for the given geometry `id`. We call these the "default" coefficients but
  /// note that we mean user-supplied per-geometry default, not something more
  /// global.
  /// @throws std::exception if `id` does not correspond to a geometry in `this`
  /// model registered for contact modeling.
  /// @see RegisterCollisionGeometry() for details on geometry registration.
  const CoulombFriction<double>& default_coulomb_friction(
      geometry::GeometryId id) const {
    // TODO(amcastro-tri): This API might change or disappear completely as GS
    // provides support for the specification of surface properties.
    DRAKE_DEMAND(is_collision_geometry(id));
    const int collision_index = geometry_id_to_collision_index_.at(id);
    return default_coulomb_friction_[collision_index];
  }

  /// Returns the number of geometries registered for visualization.
  /// This method can be called at any time during the lifetime of `this` plant,
  /// either pre- or post-finalize, see Finalize().
  /// Post-finalize calls will always return the same value.
  int num_visual_geometries() const {
    return static_cast<int>(geometry_id_to_visual_index_.size());
  }

  /// Returns the number of geometries registered for contact modeling.
  /// This method can be called at any time during the lifetime of `this` plant,
  /// either pre- or post-finalize, see Finalize().
  /// Post-finalize calls will always return the same value.
  int num_collision_geometries() const {
    return geometry_id_to_collision_index_.size();
  }

  /// Returns the unique id identifying `this` plant as a source for a
  /// SceneGraph.
  /// Returns `nullopt` if `this` plant did not register any geometry.
  /// This method can be called at any time during the lifetime of `this` plant
  /// to query if `this` plant has been registered with a SceneGraph, either
  /// pre- or post-finalize, see Finalize(). However, a geometry::SourceId is
  /// only assigned once at the first call of any of this plant's geometry
  /// registration methods, and it does not change after that.
  /// Post-finalize calls will always return the same value.
  std::optional<geometry::SourceId> get_source_id() const {
    return source_id_;
  }

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
  /// @} <!-- Introspection -->

  using internal::MultibodyTreeSystem<T>::is_discrete;
  using internal::MultibodyTreeSystem<T>::EvalPositionKinematics;
  using internal::MultibodyTreeSystem<T>::EvalVelocityKinematics;

 private:
  using internal::MultibodyTreeSystem<T>::internal_tree;

  // Allow different specializations to access each other's private data for
  // scalar conversion.
  template <typename U> friend class MultibodyPlant;

  // Friend class to facilitate testing.
  friend class MultibodyPlantTester;

  // This struct stores in one single place all indexes related to
  // MultibodyPlant specific cache entries. These are initialized at Finalize()
  // when the plant declares its cache entries.
  struct CacheIndexes {
    systems::CacheIndex aba_accelerations;
    systems::CacheIndex aba_force_cache;
    systems::CacheIndex contact_info_and_body_spatial_forces;
    systems::CacheIndex contact_jacobians;
    systems::CacheIndex contact_results;
    systems::CacheIndex contact_surfaces;
    systems::CacheIndex generalized_accelerations;
    systems::CacheIndex generalized_contact_forces_continuous;
    systems::CacheIndex hydro_fallback;
    systems::CacheIndex point_pairs;
    systems::CacheIndex spatial_contact_forces_continuous;
    systems::CacheIndex tamsi_solver_results;
  };

  // Constructor to bridge testing from MultibodyTree to MultibodyPlant.
  // WARNING: This may *not* result in a plant with a valid state. Use
  // sparingly to test forwarding methods when the overhead is high to
  // reproduce the testing (e.g. benchmarks).
  explicit MultibodyPlant(
      std::unique_ptr<internal::MultibodyTree<T>> tree_in,
      double time_step = 0);

  // Helper method for throwing an exception within public methods that should
  // not be called post-finalize. The invoking method should pass its name so
  // that the error message can include that detail.
  void ThrowIfFinalized(const char* source_method) const;

  // Helper method for throwing an exception within public methods that should
  // not be called pre-finalize. The invoking method should pass it's name so
  // that the error message can include that detail.
  void ThrowIfNotFinalized(const char* source_method) const;

  // Helper method that is used to finalize the plant's internals after
  // MultibodyTree::Finalize() was called.
  void FinalizePlantOnly();

  // MemberSceneGraph is an alias for SceneGraph<T>, except when T = Expression.
  struct SceneGraphStub;
  using MemberSceneGraph = typename std::conditional<
      std::is_same<T, symbolic::Expression>::value,
      SceneGraphStub, geometry::SceneGraph<T>>::type;

  // Returns the SceneGraph that pre-Finalize geometry operations should
  // interact with.  In most cases, that will be whatever the user has passed
  // into RegisterAsSourceForSceneGraph.  However, when T = Expression, the
  // result will be a stub type instead.  (We can get rid of the stub once
  // SceneGraph supports symbolic::Expression.)
  MemberSceneGraph& member_scene_graph();

  // Checks that the provided State is consistent with this plant.
  void CheckValidState(const systems::State<T>*) const;

  // Helper method to apply collision filters based on body-adjacency. By
  // default, we don't consider collisions between geometries affixed to
  // bodies connected by a joint.
  void FilterAdjacentBodies();

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
  // joints reduces to a simple function of the mass of the bodies adjancent to
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

  // This is a *temporary* method to eliminate visual geometries from collision
  // while we wait for geometry roles to be introduced.
  // TODO(SeanCurtis-TRI): Remove this when geometry roles are introduced.
  void ExcludeCollisionsWithVisualGeometry();

  // Helper method to declare state, cache entries, and ports after Finalize().
  void DeclareStateCacheAndPorts();

  // Declare the system-level cache entries specific to MultibodyPlant.
  void DeclareCacheEntries();

  // Helper method to assemble actuation input vector from the appropriate
  // ports.
  VectorX<T> AssembleActuationInput(
      const systems::Context<T>& context) const;

  // Computes all externally applied forces including:
  //  - Force elements.
  //  - Joint actuation.
  //  - Externally applied spatial forces.
  //  - Joint limits.
  void CalcAppliedForces(const drake::systems::Context<T>& context,
                         MultibodyForces<T>* forces) const;

  // Given the state x and inputs u in `context`, this method uses the `O(n)`
  // Articulated Body Algorithm (ABA) to compute accelerations.
  // N.B. Please refer to @ref internal_forward_dynamics for further details on
  // the algorithm and implementation.
  void CalcForwardDynamics(const systems::Context<T>& context,
                           internal::AccelerationKinematicsCache<T>* ac) const;

  // Eval version of the method CalcForwardDynamics().
  const internal::AccelerationKinematicsCache<T>& EvalForwardDynamics(
      const systems::Context<T>& context) const {
    return this->get_cache_entry(cache_indexes_.aba_accelerations)
        .template Eval<internal::AccelerationKinematicsCache<T>>(context);
  }

  // Performs an O(n) tip-to-base recursion to compute bias forces Z_B and
  // Zplus_B, among other quantities needed by ABA.
  // N.B. Please refer to @ref internal_forward_dynamics for further details on
  // the algorithm and implementation.
  void CalcArticulatedBodyForceCache(
      const systems::Context<T>& context,
      internal::ArticulatedBodyForceCache<T>* aba_force_cache) const;

  // Eval version of the method CalcArticulatedBodyForceCache().
  const internal::ArticulatedBodyForceCache<T>&
  EvalArticulatedBodyForceCache(const systems::Context<T>& context) const {
    return this->get_cache_entry(cache_indexes_.aba_force_cache)
        .template Eval<internal::ArticulatedBodyForceCache<T>>(context);
  }

  // Implements the system dynamics according to this class's documentation.
  void DoCalcTimeDerivatives(
      const systems::Context<T>& context,
      systems::ContinuousState<T>* derivatives) const override;

  // If the plant is modeled as a discrete system with periodic updates (see
  // is_discrete()), this method computes the periodic updates of the state
  // using a semi-explicit Euler strategy, that is:
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
  void DoCalcDiscreteVariableUpdates(
      const drake::systems::Context<T>& context0,
      const std::vector<const drake::systems::DiscreteUpdateEvent<T>*>& events,
      drake::systems::DiscreteValues<T>* updates) const override;

  // Helper method used within DoCalcDiscreteVariableUpdates() to update
  // generalized velocities from previous step value v0 to next step value v.
  // This helper uses num_substeps within a time interval of duration dt
  // to perform the update using a step size dt_substep = dt/num_substeps.
  // During the time span dt the problem data M, Jn, Jt and minus_tau, are
  // approximated to be constant, a first order approximation.
  TamsiSolverResult SolveUsingSubStepping(
      int num_substeps,
      const MatrixX<T>& M0, const MatrixX<T>& Jn, const MatrixX<T>& Jt,
      const VectorX<T>& minus_tau,
      const VectorX<T>& stiffness, const VectorX<T>& damping,
      const VectorX<T>& mu,
      const VectorX<T>& v0, const VectorX<T>& phi0) const;

  // This method uses the time stepping method described in
  // TamsiSolver to advance the model's state stored in
  // `context0` taking a time step of size time_step().
  // Contact forces and velocities are computed and stored in `results`. See
  // TamsiSolverResults for further details on the returned data.
  void CalcTamsiResults(
      const drake::systems::Context<T>& context0,
      internal::TamsiSolverResults<T>* results) const;

  // Eval version of the method CalcTamsiResults().
  const internal::TamsiSolverResults<T>& EvalTamsiResults(
      const systems::Context<T>& context) const {
    return this
        ->get_cache_entry(cache_indexes_.tamsi_solver_results)
        .template Eval<internal::TamsiSolverResults<T>>(context);
  }

  // Computes the vector of ContactSurfaces for hydroelastic contact.
  void CalcContactSurfaces(
      const drake::systems::Context<T>& context,
      std::vector<geometry::ContactSurface<T>>* contact_surfaces) const;

  // Eval version of the method CalcContactSurfaces().
  const std::vector<geometry::ContactSurface<T>>& EvalContactSurfaces(
      const systems::Context<T>& context) const {
    switch (contact_model_) {
      case ContactModel::kHydroelasticWithFallback: {
        const auto& data =
            this->get_cache_entry(cache_indexes_.hydro_fallback)
                .template Eval<internal::HydroelasticFallbackCacheData<T>>(
                    context);
        return data.contact_surfaces;
      }
      case ContactModel::kHydroelasticsOnly:
        return this->get_cache_entry(cache_indexes_.contact_surfaces)
            .template Eval<std::vector<geometry::ContactSurface<T>>>(context);
      default:
        throw std::logic_error(
            "Attempting to evaluate contact surface for contact model that "
            "doesn't use it");
    }
  }

  // Computes the hydroelastic fallback method -- all contacts are partitioned
  // between ContactSurfaces and point pair contacts.
  void CalcHydroelasticWithFallback(
      const drake::systems::Context<T>& context,
      internal::HydroelasticFallbackCacheData<T>* data) const;

  // Helper method to fill in the ContactResults given the current context when
  // the model is continuous.
  void CalcContactResultsContinuous(const systems::Context<T>& context,
                                    ContactResults<T>* contact_results) const;

  // Helper method for the continuous mode plant, to fill in the ContactResults
  // for the point pair model, given the current context. Called by
  // CalcContactResultsContinuous.
  void CalcContactResultsContinuousPointPair(
      const systems::Context<T>& context,
      ContactResults<T>* contact_results) const;

  // Helper method for the continuous mode plant, to fill in the ContactResults
  // for the hydroelastic model, given the current context. Called by
  // CalcContactResultsContinuous.
  void CalcContactResultsContinuousHydroelastic(
      const systems::Context<T>& context,
      ContactResults<T>* contact_results) const;

  // Helper method to fill in the ContactResults given the current context when
  // the model is discrete. If cached contact solver results are not up-to-date
  // with `context`, they'll be  recomputed, see EvalTamsiResults(). The solver
  // results are then used to compute contact results into `contacts`.
  void CalcContactResultsDiscrete(const systems::Context<T>& context,
                                  ContactResults<T>* contact_results) const;

  // Evaluate contact results.
  const ContactResults<T>& EvalContactResults(
      const systems::Context<T>& context) const {
    return this->get_cache_entry(cache_indexes_.contact_results)
        .template Eval<ContactResults<T>>(context);
  }

  // Given the state x and input u in `context`, this method computes the
  // generalized acceleration into vdot.
  void CalcGeneralizedAccelerations(const drake::systems::Context<T>& context,
                                    VectorX<T>* vdot) const;

  // Discrete system version of CalcGeneralizedAccelerations().
  void CalcGeneralizedAccelerationsDiscrete(
      const drake::systems::Context<T>& context, VectorX<T>* vdot) const;

  // Continuous system version of CalcGeneralizedAccelerations().
  void CalcGeneralizedAccelerationsContinuous(
      const drake::systems::Context<T>& context, VectorX<T>* vdot) const;

  // Eval() version of the method CalcGeneralizedAccelerations().
  const VectorX<T>& EvalGeneralizedAccelerations(
      const systems::Context<T>& context) const {
    return this->get_cache_entry(cache_indexes_.generalized_accelerations)
        .template Eval<VectorX<T>>(context);
  }

  // Calc method for the reaction forces output port.
  // A joint constraints the motion between a frame Jp on a "parent" P and a
  // frame Jc on a "child" frame C. This generates reaction forces on bodies P
  // and C in order to satisfy the kinematic constraint between Jp and Jc. This
  // method computes the spatial force F_CJc_Jc on body C at frame Jc and
  // expressed in frame Jc. See get_reaction_forces_output_port() for further
  // details.
  void CalcReactionForces(const systems::Context<T>& context,
                          std::vector<SpatialForce<T>>* F_CJc_Jc) const;

  void DoMapQDotToVelocity(
      const systems::Context<T>& context,
      const Eigen::Ref<const VectorX<T>>& qdot,
      systems::VectorBase<T>* generalized_velocity) const override;

  void DoMapVelocityToQDot(
      const systems::Context<T>& context,
      const Eigen::Ref<const VectorX<T>>& generalized_velocity,
      systems::VectorBase<T>* qdot) const override;

  void AddAppliedExternalSpatialForces(
      const systems::Context<T>& context, MultibodyForces<T>* forces) const;

  // Helper method to register geometry for a given body, either visual or
  // collision. The registration includes:
  // 1. Register a frame for this body if not already done so. The body gets
  //    associated with a FrameId.
  // 2. Register geometry for the corresponding FrameId. This associates a
  //    GeometryId with the body FrameId.
  // This assumes:
  // 1. Finalize() was not called on `this` plant.
  // 2. RegisterAsSourceForSceneGraph() was called on `this` plant.
  // 3. `scene_graph` points to the same SceneGraph instance previously
  //    passed to RegisterAsSourceForSceneGraph().
  geometry::GeometryId RegisterGeometry(
      const Body<T>& body, const math::RigidTransform<double>& X_BG,
      const geometry::Shape& shape,
      const std::string& name);

  // Registers a geometry frame for every body. If the body already has a
  // geometry frame, it is unchanged. This registration is part of finalization.
  // This requires RegisterAsSourceForSceneGraph() was called on `this` plant.
  void RegisterGeometryFramesForAllBodies();

  bool body_has_registered_frame(const Body<T>& body) const {
    return body_index_to_frame_id_.find(body.index()) !=
        body_index_to_frame_id_.end();
  }

  // Registers the given body with this plant's SceneGraph instance (if it has
  // one).
  void RegisterRigidBodyWithSceneGraph(const Body<T>& body);

  // Calc method for the multibody state vector output port. It only copies the
  // multibody state [q, v], ignoring any miscellaneous state z if present.
  void CopyMultibodyStateOut(
      const systems::Context<T>& context, systems::BasicVector<T>* state) const;

  // Calc method for the per-model-instance multibody state vector output port.
  // It only copies the per-model-instance multibody state [q, v], ignoring any
  // miscellaneous state z if present.
  void CopyMultibodyStateOut(
      ModelInstanceIndex model_instance,
      const systems::Context<T>& context, systems::BasicVector<T>* state) const;

  // Method to compute spatial contact forces for continuous plants.
  void CalcSpatialContactForcesContinuous(
      const drake::systems::Context<T>& context,
      std::vector<SpatialForce<T>>* F_BBo_W_array) const;

  // Eval() version of the method CalcSpatialContactForcesContinuous().
  const std::vector<SpatialForce<T>>& EvalSpatialContactForcesContinuous(
      const systems::Context<T>& context) const {
    return this->get_cache_entry(
        cache_indexes_.spatial_contact_forces_continuous).
            template Eval<std::vector<SpatialForce<T>>>(context);
  }

  // Method to compute generalized contact forces for continuous plants.
  void CalcGeneralizedContactForcesContinuous(
    const drake::systems::Context<T>& context, VectorX<T>* tau_contact) const;

  // Eval() version of the method CalcGeneralizedContactForcesContinuous().
  const VectorX<T>& EvalGeneralizedContactForcesContinuous(
      const systems::Context<T>& context) const {
    return this->get_cache_entry(
        cache_indexes_.generalized_contact_forces_continuous).
            template Eval<VectorX<T>>(context);
  }

  // Calc method to output per model instance vector of generalized contact
  // forces.
  void CopyGeneralizedContactForcesOut(
      const internal::TamsiSolverResults<T>&,
      ModelInstanceIndex, systems::BasicVector<T>* tau_vector) const;

  // Helper method to declare output ports used by this plant to communicate
  // with a SceneGraph.
  void DeclareSceneGraphPorts();

  void CalcFramePoseOutput(const systems::Context<T>& context,
                           geometry::FramePoseVector<T>* poses) const;

  void CopyContactResultsOutput(
      const systems::Context<T>& context,
      ContactResults<T>* contact_results) const;

  // Helper to evaluate if a GeometryId corresponds to a collision model.
  bool is_collision_geometry(geometry::GeometryId id) const {
    return geometry_id_to_collision_index_.count(id) > 0;
  }

  // Helper method to compute penetration point pairs for a given `context`.
  // Having this as a separate method allows us to control specializations for
  // different scalar types.
  std::vector<geometry::PenetrationAsPointPair<T>>
  CalcPointPairPenetrations(const systems::Context<T>& context) const;

  // This helper method combines the friction properties for each pair of
  // contact points in `point_pairs` according to
  // CalcContactFrictionFromSurfaceProperties().
  // The i-th entry in the returned std::vector corresponds to the combined
  // friction properties for the i-th point pair in `point_pairs`.
  std::vector<CoulombFriction<double>> CalcCombinedFrictionCoefficients(
      const std::vector<geometry::PenetrationAsPointPair<T>>&
      point_pairs) const;

  // (Advanced) Helper method to compute contact forces in the normal direction
  // using a penalty method.
  void CalcAndAddContactForcesByPenaltyMethod(
      const systems::Context<T>& context,
      std::vector<SpatialForce<T>>* F_BBo_W_array) const;

  // Helper method to compute contact forces using the hydroelastic model.
  // F_BBo_W_array is indexed by BodyNodeIndex and it gets overwritten on
  // output. F_BBo_W_array must be of size num_bodies() or an exception is
  // thrown.
  void CalcHydroelasticContactForces(
      const systems::Context<T>& context,
      internal::HydroelasticContactInfoAndBodySpatialForces<T>* F_BBo_W_array)
      const;

  // Eval version of CalcHydroelasticContactForces().
  const internal::HydroelasticContactInfoAndBodySpatialForces<T>&
  EvalHydroelasticContactForces(const systems::Context<T>& context) const {
    return this
        ->get_cache_entry(cache_indexes_.contact_info_and_body_spatial_forces)
        .template Eval<
            internal::HydroelasticContactInfoAndBodySpatialForces<T>>(context);
  }

  // Helper method to add the contribution of external actuation forces to the
  // set of multibody `forces`. External actuation is applied through the
  // plant's input ports.
  void AddJointActuationForces(
      const systems::Context<T>& context, MultibodyForces<T>* forces) const;

  // Helper method to apply penalty forces that enforce joint limits.
  // At each joint with joint limits this penalty method applies a force law of
  // the form:
  //   τ = min(-k(q - qᵤ) - cv, 0) if q > qᵤ
  //   τ = max(-k(q - qₗ) - cv, 0) if q < qₗ
  // is used to limit the position q to be within the lower/upper limits
  // (qₗ, qᵤ).
  // The penalty parameters k (stiffness) and c (damping) are estimated using
  // a harmonic oscillator model within SetUpJointLimitsParameters().
  void AddJointLimitsPenaltyForces(
      const systems::Context<T>& context, MultibodyForces<T>* forces) const;

  // Given a set of point pairs in `point_pairs_set`, this method computes the
  // normal velocities Jacobian Jn(q) and the tangential velocities Jacobian
  // Jt(q).
  //
  // The normal velocities Jacobian Jn(q) is defined such that:
  //   vn = Jn(q) v
  // where the i-th component of vn corresponds to the "separation velocity"
  // for the i-th point pair in the set. The i-th separation velocity is defined
  // positive for when the depth in the i-th point pair (
  // PenetrationAsPointPair::depth) is decreasing. Since for contact problems
  // the (positive) depth in PenetrationAsPointPair is defined so that it
  // corresponds to interpenetrating body geometries, a positive separation
  // velocity corresponds to bodies moving apart.
  //
  // The tangential velocities Jacobian Jt(q) is defined such that:
  //   vt = Jt(q) v
  // where v ∈ ℝⁿᵛ is the vector of generalized velocities, Jt(q) is a matrix
  // of size 2⋅nc×nv and vt is a vector of size 2⋅nc. vt is defined such that
  // its 2⋅i-th and (2⋅i+1)-th entries correspond to relative velocity of the
  // i-th point pair in these two orthogonal directions. That is:
  //   vt(2 * i)     = vx_AB_C = Cx ⋅ v_AB
  //   vt(2 * i + 1) = vy_AB_C = Cy ⋅ v_AB
  //
  // If the optional argument R_WC_set is non-null, on output the i-th entry of
  // R_WC_set will contain the orientation R_WC (with columns Cx, Cy, Cz) in the
  // world using the mean of the pair of witnesses for point_pairs_set[i] as the
  // contact point.
  void CalcNormalAndTangentContactJacobians(
      const systems::Context<T>& context,
      const std::vector<geometry::PenetrationAsPointPair<T>>& point_pairs_set,
      MatrixX<T>* Jn, MatrixX<T>* Jt,
      std::vector<math::RotationMatrix<T>>* R_WC_set = nullptr) const;

  // Evaluates the contact Jacobians for the given state of the plant stored in
  // `context`.
  // This method first evaluates the point pair penetrations in the system for
  // the given `context`, see EvalPointPairPenetrations(). For each penetration
  // pair involving bodies A and B, a contact frame C is defined by the
  // rotation matrix `R_WC = [Cx_W, Cy_W, Cz_W]` where `Cz_W = nhat_BA_W`
  // equals the normal vector pointing from body B into body A, expressed in
  // the world frame W. See PenetrationAsPointPair for further details on the
  // definition of each contact pair. Versors `Cx_W` and `Cy_W` constitute a
  // basis of the plane normal to `Cz_W` and are arbitrarily chosen. The
  // contact frame basis can be accessed in the results, see
  // ContactJacobians::R_WC_list. Further, for each contact pair evaluated,
  // this method computes the Jacobians `Jn` and `Jt`. With the vector of
  // generalized velocities v of size `nv` and `nc` the number of contact
  // pairs;
  //   - `Jn` is a matrix of size `nc x nv` such that `vn = Jn⋅v` is the
  //     separation speed for each contact point, defined to be positive when
  //     bodies are moving away at the contact.
  //   - `Jt` is a matrix of size `2⋅nc x nv` such that `vt = Jt⋅v`
  //     concatenates the tangential components of the relative velocity vector
  //     `v_AcBc` in the frame C of contact. That is, for the k-th contact
  //     pair, `vt.segment<2>(2 * ik)` stores the components of `v_AcBc` in the
  //     `Cx` and `Cy` directions.
  //
  // If no geometry was registered or if `nc = 0`, ContactJacobians holds empty
  // results.
  //
  // See ContactJacobians for specifics on the returned data storage.
  // This method throws std::exception if called pre-finalize. See Finalize().
  const internal::ContactJacobians<T>& EvalContactJacobians(
      const systems::Context<T>& context) const {
    DRAKE_MBP_THROW_IF_NOT_FINALIZED();
    return this->get_cache_entry(cache_indexes_.contact_jacobians)
        .template Eval<internal::ContactJacobians<T>>(context);
  }

  // Registers a joint in the graph.
  void RegisterJointInGraph(const Joint<T>& joint) {
    const std::string type_name = joint.type_name();
    if (!multibody_graph_.IsJointTypeRegistered(type_name)) {
      multibody_graph_.RegisterJointType(type_name);
    }
    // Note changes in the graph.
    multibody_graph_.AddJoint(joint.name(), joint.model_instance(), type_name,
                              joint.parent_body().index(),
                              joint.child_body().index());
  }

  // Geometry source identifier for this system to interact with geometry
  // system. It is made optional for plants that do not register geometry
  // (dynamics only).
  std::optional<geometry::SourceId> source_id_{std::nullopt};

  // Frame Id's for each body in the model:
  // Not all bodies need to be in this map.

  // Map provided at construction that tells how bodies (referenced by name),
  // map to frame ids.
  std::unordered_map<std::string, geometry::FrameId> body_name_to_frame_id_;

  // This struct contains the parameters to compute forces to enforce
  // no-interpenetration between bodies by a penalty method.
  struct ContactByPenaltyMethodParameters {
    // Penalty method coefficients used to compute contact forces.
    // TODO(amcastro-tri): consider having these per body. That would allow us
    // for instance to calibrate the stiffness at the fingers (stiffness related
    // to the weight of the objects being manipulated) of a walking robot (
    // stiffness related to the weight of the entire robot) with the same
    // penetration allowance.
    double stiffness{0};
    double damping{0};
    // An estimated time scale in which objects come to a relative stop during
    // contact.
    double time_scale{-1.0};
    // Acceleration of gravity in the model. Used to estimate penalty method
    // constants from a static equilibrium analysis.
    std::optional<double> gravity;
  };
  ContactByPenaltyMethodParameters penalty_method_contact_parameters_;

  // Stribeck model of friction.
  class StribeckModel {
   public:
    DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(StribeckModel)

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
    T ComputeFrictionCoefficient(
        const T& speed_BcAc,
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
    // A negative value indicates it was not properly initialized.
    double v_stiction_tolerance_{-1};
    // Note: this is the *inverse* of the v_stiction_tolerance_ parameter to
    // optimize for the division.
    // A negative value indicates it was not properly initialized.
    double inv_v_stiction_tolerance_{-1};
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

  // Maps a GeometryId with a visual index. This allows, for instance, to find
  // out visual properties for a given geometry.
  // TODO(amcastro-tri): verify insertions were correct once visual_index gets
  // used with the landing of visual properties in SceneGraph.
  std::unordered_map<geometry::GeometryId, int> geometry_id_to_visual_index_;

  // Per-body arrays of visual geometries indexed by BodyIndex.
  // That is, visual_geometries_[body_index] corresponds to the array of visual
  // geometries for body with index body_index.
  std::vector<std::vector<geometry::GeometryId>> visual_geometries_;

  // Per-body arrays of collision geometries indexed by BodyIndex.
  // That is, collision_geometries_[body_index] corresponds to the array of
  // collision geometries for body with index body_index.
  std::vector<std::vector<geometry::GeometryId>> collision_geometries_;

  // Maps a GeometryId with a collision index. This allows, for instance, to
  // find out collision properties (such as friction coefficient) for a given
  // geometry.
  std::unordered_map<geometry::GeometryId, int> geometry_id_to_collision_index_;

  // Friction coefficients ordered by collision index.
  // See geometry_id_to_collision_index_.
  std::vector<CoulombFriction<double>> default_coulomb_friction_;

  // The model used by the plant to compute contact forces.
  ContactModel contact_model_{ContactModel::kPointContactOnly};

  // Port handles for geometry:
  systems::InputPortIndex geometry_query_port_;
  systems::OutputPortIndex geometry_pose_port_;

  // For geometry registration with a GS, we save a pointer to the GS instance
  // on which this plants calls RegisterAsSourceForSceneGraph(). This will be
  // set to `nullptr` after finalization, to mirror constraints presented by
  // scalar conversion (where we cannot easily obtain a reference to the
  // scalar-converted scene graph).
  geometry::SceneGraph<T>* scene_graph_{nullptr};

  // Input/Output port indexes:

  // A vector containing actuation ports for each model instance indexed by
  // ModelInstanceIndex. Every model instance has a corresponding port even
  // if that instance has no actuators.
  std::vector<systems::InputPortIndex> instance_actuation_ports_;

  // If only one model instance has actuated dofs, remember it here.  If
  // multiple instances have actuated dofs, this index will not be valid.
  ModelInstanceIndex actuated_instance_;

  // A port for externally applied generalized forces u.
  systems::InputPortIndex applied_generalized_force_input_port_;

  // Port for externally applied spatial forces F.
  systems::InputPortIndex applied_spatial_force_input_port_;

  // A port presenting state x=[q v] for the whole system, and a vector of
  // ports presenting state subsets xᵢ=[qᵢ vᵢ] ⊆ x for each model instance i,
  // indexed by ModelInstanceIndex. Every model instance has a corresponding
  // port even if it has no states.
  systems::OutputPortIndex state_output_port_;
  std::vector<systems::OutputPortIndex> instance_state_output_ports_;

  // A port presenting generalized accelerations v̇ for the whole system, and
  // a vector of ports presenting acceleration subsets v̇ᵢ ⊆ v̇ for each model
  // instance i, indexed by ModelInstanceIndex. Every model instance has a
  // corresponding port even if it has no states.
  systems::OutputPortIndex generalized_acceleration_output_port_;
  std::vector<systems::OutputPortIndex>
      instance_generalized_acceleration_output_ports_;

  // Index for the output port of ContactResults.
  systems::OutputPortIndex contact_results_port_;

  // Joint reactions forces port index.
  systems::OutputPortIndex reaction_forces_port_;

  // A vector containing the index for the generalized contact forces port for
  // each model instance. This vector is indexed by ModelInstanceIndex. An
  // invalid value indicates that the model instance has no generalized
  // velocities and thus no generalized forces.
  std::vector<systems::OutputPortIndex>
      instance_generalized_contact_forces_output_ports_;

  // A graph representing the body/joint topology of the multibody plant (Not
  // to be confused with the spanning-tree model we will build for analysis.)
  internal::MultibodyGraph multibody_graph_;

  // If the plant is modeled as a discrete system with periodic updates,
  // time_step_ corresponds to the period of those updates. Otherwise, if the
  // plant is modeled as a continuous system, it is exactly zero.
  double time_step_{0};

  // The solver used when the plant is modeled as a discrete system.
  std::unique_ptr<TamsiSolver<T>> tamsi_solver_;

  hydroelastics::internal::HydroelasticEngine<T> hydroelastics_engine_;

  // All MultibodyPlant cache indexes are stored in cache_indexes_.
  CacheIndexes cache_indexes_;

  // Vector (with size num_bodies()) of default poses for each body. This is
  // only used if Body::is_floating() is true.
  std::vector<math::RigidTransform<double>> X_WB_default_list_;
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
/// @relates MultibodyPlant
template <typename T>
AddMultibodyPlantSceneGraphResult<T>
AddMultibodyPlantSceneGraph(
    systems::DiagramBuilder<T>* builder,
    double time_step,
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
/// @relates MultibodyPlant
template <typename T>
AddMultibodyPlantSceneGraphResult<T>
AddMultibodyPlantSceneGraph(
    systems::DiagramBuilder<T>* builder,
    std::unique_ptr<MultibodyPlant<T>> plant,
    std::unique_ptr<geometry::SceneGraph<T>> scene_graph = nullptr);

/// Adds a new continuous MultibodyPlant to `builder`.
template <typename T>
DRAKE_DEPRECATED("2020-05-01", "Use alternative overloads explicitly providing a continuous or discrete MultibodyPlant modality. To retain the prior behavior of using a continuous-time plant, pass time_step = 0.0.")  // NOLINT(whitespace/line_length)
AddMultibodyPlantSceneGraphResult<T>
AddMultibodyPlantSceneGraph(
    systems::DiagramBuilder<T>* builder);

/// Temporary result from `AddMultibodyPlantSceneGraph`. This cannot be
/// constructed outside of this method.
/// @warning Do NOT use this as a function argument or member variable. The
/// lifetime of this object should be as short as possible.
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
  // Provided to support C++17's structured binding.
  template <std::size_t N>
  decltype(auto) get() const {
    if constexpr (N == 0)
      return plant;
    else if constexpr (N == 1)
      return scene_graph;
  }
#endif

#ifndef DRAKE_DOXYGEN_CXX
  // Only the move constructor is enabled; copy/assign/move-assign are deleted.
  AddMultibodyPlantSceneGraphResult(
      AddMultibodyPlantSceneGraphResult&&) = default;
  AddMultibodyPlantSceneGraphResult(
      const AddMultibodyPlantSceneGraphResult&) = delete;
  void operator=(const AddMultibodyPlantSceneGraphResult&) = delete;
  void operator=(AddMultibodyPlantSceneGraphResult&&) = delete;
#endif

 private:
  // Deter external usage by hiding construction.
  friend AddMultibodyPlantSceneGraphResult AddMultibodyPlantSceneGraph<T>(
    systems::DiagramBuilder<T>*, std::unique_ptr<MultibodyPlant<T>>,
    std::unique_ptr<geometry::SceneGraph<T>>);

  AddMultibodyPlantSceneGraphResult(
      MultibodyPlant<T>* plant_in, geometry::SceneGraph<T>* scene_graph_in)
      : plant(*plant_in), scene_graph(*scene_graph_in),
        plant_ptr(plant_in), scene_graph_ptr(scene_graph_in) {}

  // Pointers to enable implicit casts for `std::tie()` assignments using
  // `T*&`.
  MultibodyPlant<T>* plant_ptr{};
  geometry::SceneGraph<T>* scene_graph_ptr{};
};

#ifndef DRAKE_DOXYGEN_CXX
// Forward-declare specializations, prior to DRAKE_DECLARE... below.
// See the .cc file for an explanation why we specialize these methods.
template <>
typename MultibodyPlant<symbolic::Expression>::SceneGraphStub&
MultibodyPlant<symbolic::Expression>::member_scene_graph();
template <>
std::vector<geometry::PenetrationAsPointPair<double>>
MultibodyPlant<double>::CalcPointPairPenetrations(
    const systems::Context<double>&) const;
template <>
std::vector<geometry::PenetrationAsPointPair<AutoDiffXd>>
MultibodyPlant<AutoDiffXd>::CalcPointPairPenetrations(
    const systems::Context<AutoDiffXd>&) const;
template <>
void MultibodyPlant<symbolic::Expression>::CalcHydroelasticContactForces(
    const systems::Context<symbolic::Expression>&,
    internal::HydroelasticContactInfoAndBodySpatialForces<
        symbolic::Expression>*) const;
template <>
void MultibodyPlant<symbolic::Expression>::
    CalcContactResultsContinuousHydroelastic(
        const systems::Context<symbolic::Expression>&,
        ContactResults<symbolic::Expression>*) const;
template <>
void MultibodyPlant<symbolic::Expression>::CalcContactSurfaces(
    const systems::Context<symbolic::Expression>&,
    std::vector<geometry::ContactSurface<symbolic::Expression>>*) const;
template <>
void MultibodyPlant<double>::CalcHydroelasticWithFallback(
    const systems::Context<double>&,
    internal::HydroelasticFallbackCacheData<double>*) const;
#endif

}  // namespace multibody
}  // namespace drake

#ifndef DRAKE_DOXYGEN_CXX
// Specializations provided to support C++17's structured binding for
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
    class drake::multibody::MultibodyPlant)
DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    struct drake::multibody::AddMultibodyPlantSceneGraphResult)
