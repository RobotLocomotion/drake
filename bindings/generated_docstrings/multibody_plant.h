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

// #include "drake/multibody/plant/calc_distance_and_time_derivative.h"
// #include "drake/multibody/plant/compliant_contact_manager.h"
// #include "drake/multibody/plant/constraint_specs.h"
// #include "drake/multibody/plant/contact_jacobians.h"
// #include "drake/multibody/plant/contact_results.h"
// #include "drake/multibody/plant/contact_results_to_lcm.h"
// #include "drake/multibody/plant/coulomb_friction.h"
// #include "drake/multibody/plant/deformable_contact_info.h"
// #include "drake/multibody/plant/deformable_driver.h"
// #include "drake/multibody/plant/deformable_model.h"
// #include "drake/multibody/plant/desired_state_input.h"
// #include "drake/multibody/plant/discrete_contact_data.h"
// #include "drake/multibody/plant/discrete_contact_pair.h"
// #include "drake/multibody/plant/discrete_step_memory.h"
// #include "drake/multibody/plant/discrete_update_manager.h"
// #include "drake/multibody/plant/distance_constraint_params.h"
// #include "drake/multibody/plant/dummy_physical_model.h"
// #include "drake/multibody/plant/externally_applied_spatial_force.h"
// #include "drake/multibody/plant/externally_applied_spatial_force_multiplexer.h"
// #include "drake/multibody/plant/geometry_contact_data.h"
// #include "drake/multibody/plant/hydroelastic_contact_forces_continuous_cache_data.h"
// #include "drake/multibody/plant/hydroelastic_contact_info.h"
// #include "drake/multibody/plant/internal_geometry_names.h"
// #include "drake/multibody/plant/make_discrete_update_manager.h"
// #include "drake/multibody/plant/multibody_plant.h"
// #include "drake/multibody/plant/multibody_plant_config.h"
// #include "drake/multibody/plant/multibody_plant_config_functions.h"
// #include "drake/multibody/plant/multibody_plant_discrete_update_manager_attorney.h"
// #include "drake/multibody/plant/multibody_plant_icf_attorney.h"
// #include "drake/multibody/plant/multibody_plant_model_attorney.h"
// #include "drake/multibody/plant/physical_model.h"
// #include "drake/multibody/plant/physical_model_collection.h"
// #include "drake/multibody/plant/point_pair_contact_info.h"
// #include "drake/multibody/plant/propeller.h"
// #include "drake/multibody/plant/sap_driver.h"
// #include "drake/multibody/plant/scalar_convertible_component.h"
// #include "drake/multibody/plant/slicing_and_indexing.h"
// #include "drake/multibody/plant/tamsi_driver.h"
// #include "drake/multibody/plant/tamsi_solver.h"
// #include "drake/multibody/plant/wing.h"

// Symbol: pydrake_doc_multibody_plant
constexpr struct /* pydrake_doc_multibody_plant */ {
  // Symbol: drake
  struct /* drake */ {
    // Symbol: drake::multibody
    struct /* multibody */ {
      // Symbol: drake::multibody::AddMultibodyPlant
      struct /* AddMultibodyPlant */ {
        // Source: drake/multibody/plant/multibody_plant_config_functions.h
        const char* doc_2args =
R"""(Adds a new MultibodyPlant and SceneGraph to the given ``builder``. The
plant's settings such as ``time_step`` are set using the given
``config``.)""";
        // Source: drake/multibody/plant/multibody_plant_config_functions.h
        const char* doc_3args =
R"""(Adds a new MultibodyPlant and SceneGraph to the given ``builder``. The
plant's settings such as ``time_step`` are set using the given
``plant_config``. The scene graph's settings are set using the given
``scene_graph_config``.)""";
      } AddMultibodyPlant;
      // Symbol: drake::multibody::AddMultibodyPlantSceneGraph
      struct /* AddMultibodyPlantSceneGraph */ {
        // Source: drake/multibody/plant/multibody_plant.h
        const char* doc_3args_systemsDiagramBuilder_double_stduniqueptr =
R"""(Makes a new MultibodyPlant with discrete update period ``time_step``
and adds it to a diagram builder together with the provided SceneGraph
instance, connecting the geometry ports.

Note:
    Usage examples in add_multibody_plant_scene_graph
    "AddMultibodyPlantSceneGraph".

Parameter ``builder``:
    Builder to add to.

Parameter ``time_step``:
    The discrete update period for the new MultibodyPlant to be added.
    Please refer to the documentation provided in
    MultibodyPlant::MultibodyPlant(double) for further details on the
    parameter ``time_step``.

Parameter ``scene_graph``:
    (optional) Constructed scene graph. If none is provided, one will
    be created and used.

Returns:
    Pair of the registered plant and scene graph.

Precondition:
    ``builder`` must be non-null.)""";
        // Source: drake/multibody/plant/multibody_plant.h
        const char* doc_3args_systemsDiagramBuilder_stduniqueptr_stduniqueptr =
R"""(Adds a MultibodyPlant and a SceneGraph instance to a diagram builder,
connecting the geometry ports.

Note:
    Usage examples in add_multibody_plant_scene_graph
    "AddMultibodyPlantSceneGraph".

Parameter ``builder``:
    Builder to add to.

Parameter ``plant``:
    Plant to be added to the builder.

Parameter ``scene_graph``:
    (optional) Constructed scene graph. If none is provided, one will
    be created and used.

Returns:
    Pair of the registered plant and scene graph.

Precondition:
    ``builder`` and ``plant`` must be non-null.)""";
      } AddMultibodyPlantSceneGraph;
      // Symbol: drake::multibody::AddMultibodyPlantSceneGraphResult
      struct /* AddMultibodyPlantSceneGraphResult */ {
        // Source: drake/multibody/plant/multibody_plant.h
        const char* doc =
R"""(Temporary result from ``AddMultibodyPlantSceneGraph``. This cannot be
constructed outside of this method.

Warning:
    Do NOT use this as a function argument or member variable. The
    lifetime of this object should be as short as possible.)""";
        // Symbol: drake::multibody::AddMultibodyPlantSceneGraphResult::AddMultibodyPlantSceneGraphResult<T>
        struct /* ctor */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc = R"""()""";
        } ctor;
        // Symbol: drake::multibody::AddMultibodyPlantSceneGraphResult::get
        struct /* get */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc = R"""()""";
        } get;
        // Symbol: drake::multibody::AddMultibodyPlantSceneGraphResult::operator MultibodyPlant<type-parameter-0-0> &
        struct /* operator_MultibodyPlant_ */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(For assignment to a plant reference (ignoring the scene graph).)""";
        } operator_MultibodyPlant_;
        // Symbol: drake::multibody::AddMultibodyPlantSceneGraphResult::operator tuple<MultibodyPlant<type-parameter-0-0> *&, SceneGraph<type-parameter-0-0> *&>
        struct /* operator_tuple */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(For assignment to a std::tie of pointers.)""";
        } operator_tuple;
        // Symbol: drake::multibody::AddMultibodyPlantSceneGraphResult::plant
        struct /* plant */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc = R"""()""";
        } plant;
        // Symbol: drake::multibody::AddMultibodyPlantSceneGraphResult::scene_graph
        struct /* scene_graph */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc = R"""()""";
        } scene_graph;
      } AddMultibodyPlantSceneGraphResult;
      // Symbol: drake::multibody::ApplyMultibodyPlantConfig
      struct /* ApplyMultibodyPlantConfig */ {
        // Source: drake/multibody/plant/multibody_plant_config_functions.h
        const char* doc =
R"""(Applies settings given in ``config`` to an existing ``plant``. The
``time_step`` is the one value in ``config`` that cannot be updated --
it can only be set in the MultibodyPlant constructor. Consider using
AddMultibodyPlant() or manually passing ``config.time_step`` when you
construct the MultibodyPlant.

This method must be called pre-Finalize.

Raises:
    RuntimeError if ``plant`` is finalized or if time_step is changed.)""";
      } ApplyMultibodyPlantConfig;
      // Symbol: drake::multibody::BaseBodyJointType
      struct /* BaseBodyJointType */ {
        // Source: drake/multibody/plant/multibody_plant.h
        const char* doc =
R"""(The kind of joint to be used to connect base bodies to world at
Finalize(). See mbp_working_with_free_bodies "Working with free
bodies" for definitions and discussion.

See also:
    SetBaseBodyJointType() for details.)""";
        // Symbol: drake::multibody::BaseBodyJointType::kQuaternionFloatingJoint
        struct /* kQuaternionFloatingJoint */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc = R"""(6 dofs, unrestricted orientation.)""";
        } kQuaternionFloatingJoint;
        // Symbol: drake::multibody::BaseBodyJointType::kRpyFloatingJoint
        struct /* kRpyFloatingJoint */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc = R"""(6 dofs using 3 angles; has singularity.)""";
        } kRpyFloatingJoint;
        // Symbol: drake::multibody::BaseBodyJointType::kWeldJoint
        struct /* kWeldJoint */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc = R"""(0 dofs, fixed to World.)""";
        } kWeldJoint;
      } BaseBodyJointType;
      // Symbol: drake::multibody::CalcContactFrictionFromSurfaceProperties
      struct /* CalcContactFrictionFromSurfaceProperties */ {
        // Source: drake/multibody/plant/coulomb_friction.h
        const char* doc =
R"""(Given the surface properties of two different surfaces, this method
computes the Coulomb's law coefficients of friction characterizing the
interaction by friction of the given surface pair. The surface
properties are specified by individual Coulomb's law coefficients of
friction. As outlined in the class's documentation for
CoulombFriction, friction coefficients characterize a surface pair and
not individual surfaces. However, we find it useful in practice to
associate the abstract **idea** of friction coefficients to a single
surface. Please refer to the documentation for CoulombFriction for
details on this topic.

More specifically, this method computes the contact coefficients for
the given surface pair as:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    μ = 2μₘμₙ/(μₘ + μₙ)

.. raw:: html

    </details>

where the operation above is performed separately on the static and
dynamic friction coefficients.

Parameter ``surface_properties1``:
    Surface properties for surface 1. Specified as an individual set
    of Coulomb's law coefficients of friction.

Parameter ``surface_properties2``:
    Surface properties for surface 2. Specified as an individual set
    of Coulomb's law coefficients of friction.

Returns:
    the combined friction coefficients for the interacting surfaces.)""";
      } CalcContactFrictionFromSurfaceProperties;
      // Symbol: drake::multibody::CalcDistanceAndTimeDerivative
      struct /* CalcDistanceAndTimeDerivative */ {
        // Source: drake/multibody/plant/calc_distance_and_time_derivative.h
        const char* doc =
R"""(Given a pair of geometries and the generalized position/velocity of
the plant, compute the signed distance between the pair of geometries
and the time derivative of the signed distance. This function is
similar to QueryObject::ComputeSignedDistancePairClosestPoints(), but
it also provides the time derivative of the signed distance.

Parameter ``plant``:
    The plant on which the geometries are attached. This plant must
    have been connected to a SceneGraph.

Parameter ``geometry_pair``:
    The pair of geometries whose distance and time derivative are
    computed.

Parameter ``context``:
    The context of the plant. This must store both q and v. This
    context must have been extracted from the diagram context which
    contains both MultibodyPlant and SceneGraph contexts.)""";
      } CalcDistanceAndTimeDerivative;
      // Symbol: drake::multibody::ConnectContactResultsToDrakeVisualizer
      struct /* ConnectContactResultsToDrakeVisualizer */ {
        // Source: drake/multibody/plant/contact_results_to_lcm.h
        const char* doc_5args = R"""(MultibodyPlant-connecting overload.)""";
        // Source: drake/multibody/plant/contact_results_to_lcm.h
        const char* doc_6args = R"""(OutputPort-connecting overload.)""";
      } ConnectContactResultsToDrakeVisualizer;
      // Symbol: drake::multibody::ContactModel
      struct /* ContactModel */ {
        // Source: drake/multibody/plant/multibody_plant.h
        const char* doc = R"""(Enumeration for contact model options.)""";
        // Symbol: drake::multibody::ContactModel::kHydroelastic
        struct /* kHydroelastic */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(Contact forces are computed using the Hydroelastic model. Contact
between unsupported geometries will cause a runtime exception.)""";
        } kHydroelastic;
        // Symbol: drake::multibody::ContactModel::kHydroelasticWithFallback
        struct /* kHydroelasticWithFallback */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(Contact forces are computed using the hydroelastic model, where
possible. For most other unsupported colliding pairs, the point model
from kPoint is used. See
geometry::QueryObject::ComputeContactSurfacesWithFallback for more
details.)""";
        } kHydroelasticWithFallback;
        // Symbol: drake::multibody::ContactModel::kHydroelasticsOnly
        struct /* kHydroelasticsOnly */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(Legacy alias. TODO(jwnimmer-tri) Deprecate this constant.)""";
        } kHydroelasticsOnly;
        // Symbol: drake::multibody::ContactModel::kPoint
        struct /* kPoint */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(Contact forces are computed using a point contact model, see
compliant_point_contact.)""";
        } kPoint;
        // Symbol: drake::multibody::ContactModel::kPointContactOnly
        struct /* kPointContactOnly */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(Legacy alias. TODO(jwnimmer-tri) Deprecate this constant.)""";
        } kPointContactOnly;
      } ContactModel;
      // Symbol: drake::multibody::ContactResults
      struct /* ContactResults */ {
        // Source: drake/multibody/plant/contact_results.h
        const char* doc =
R"""(A container class storing the contact results information for each
contact pair for a given state of the simulation.

This class is immutable, so can be efficiently copied and moved.)""";
        // Symbol: drake::multibody::ContactResults::ContactResults<T>
        struct /* ctor */ {
          // Source: drake/multibody/plant/contact_results.h
          const char* doc_0args = R"""(Constructs an empty ContactResults.)""";
          // Source: drake/multibody/plant/contact_results.h
          const char* doc_4args =
R"""(Constructs a ContactResults with the given contact infos.

(Advanced) The optional ``backing_store`` argument allows the caller
to keep alive type-erased shared_ptr data that is referenced by the
contact infos. This object will hold the ``backing_store`` until this
object is destroyed. Because backing_store is type-erased (a pointer
to void), it can keep alive any kind of necessary storage, e.g., a
cache entry whose declaration is not available to this header file,
and call sites can change how that storage ends up being organized
without any changes to this file.)""";
        } ctor;
        // Symbol: drake::multibody::ContactResults::SelectHydroelastic
        struct /* SelectHydroelastic */ {
          // Source: drake/multibody/plant/contact_results.h
          const char* doc =
R"""(Returns a selective copy of this object. Only HydroelasticContactInfo
instances satisfying the selection criterion are copied; all other
contacts (point_pair and deformable) are unconditionally copied.

Parameter ``selector``:
    Boolean predicate that returns true to select which
    HydroelasticContactInfo.

Note:
    It uses deep copy (unless the operation is trivially identifiable
    as being vacuous, e.g., when num_hydroelastic_contacts() == 0).)""";
        } SelectHydroelastic;
        // Symbol: drake::multibody::ContactResults::deformable_contact_info
        struct /* deformable_contact_info */ {
          // Source: drake/multibody/plant/contact_results.h
          const char* doc =
R"""(Retrieves the ith DeformableContactInfo instance. The input index i
must be in the range [0, ``num_deformable_contacts()``) or this method
throws.)""";
        } deformable_contact_info;
        // Symbol: drake::multibody::ContactResults::hydroelastic_contact_info
        struct /* hydroelastic_contact_info */ {
          // Source: drake/multibody/plant/contact_results.h
          const char* doc =
R"""(Retrieves the ith HydroelasticContactInfo instance. The input index i
must be in the range [0, ``num_hydroelastic_contacts()``) or this
method throws.)""";
        } hydroelastic_contact_info;
        // Symbol: drake::multibody::ContactResults::num_deformable_contacts
        struct /* num_deformable_contacts */ {
          // Source: drake/multibody/plant/contact_results.h
          const char* doc =
R"""(Returns the number of deformable contacts.)""";
        } num_deformable_contacts;
        // Symbol: drake::multibody::ContactResults::num_hydroelastic_contacts
        struct /* num_hydroelastic_contacts */ {
          // Source: drake/multibody/plant/contact_results.h
          const char* doc =
R"""(Returns the number of hydroelastic contacts.)""";
        } num_hydroelastic_contacts;
        // Symbol: drake::multibody::ContactResults::num_point_pair_contacts
        struct /* num_point_pair_contacts */ {
          // Source: drake/multibody/plant/contact_results.h
          const char* doc =
R"""(Returns the number of point pair contacts.)""";
        } num_point_pair_contacts;
        // Symbol: drake::multibody::ContactResults::plant
        struct /* plant */ {
          // Source: drake/multibody/plant/contact_results.h
          const char* doc =
R"""(Returns the plant that produced these contact results. In most cases
the result will be non-null, but default-constructed results might
have nulls.)""";
        } plant;
        // Symbol: drake::multibody::ContactResults::point_pair_contact_info
        struct /* point_pair_contact_info */ {
          // Source: drake/multibody/plant/contact_results.h
          const char* doc =
R"""(Retrieves the ith PointPairContactInfo instance. The input index i
must be in the range [0, ``num_point_pair_contacts()``) or this method
throws.)""";
        } point_pair_contact_info;
        // Symbol: drake::multibody::ContactResults::set_plant
        struct /* set_plant */ {
          // Source: drake/multibody/plant/contact_results.h
          const char* doc = R"""()""";
        } set_plant;
      } ContactResults;
      // Symbol: drake::multibody::ContactResultsToLcmSystem
      struct /* ContactResultsToLcmSystem */ {
        // Source: drake/multibody/plant/contact_results_to_lcm.h
        const char* doc =
R"""(A System that encodes ContactResults into a
lcmt_contact_results_for_viz message. It has a single input port with
type ContactResults<T> and a single output port with
lcmt_contact_results_for_viz.

Although this class can be instantiated on all default scalars, its
functionality will be limited for ``T`` = symbolic::Expression. If
there are any symbolic::Variable instances in the expression,
attempting to evaluate the output port will throw an exception. The
support is sufficient that a systems::Diagram with a
ContactResultsToLcmSystem can be scalar converted to
symbolic::Expression without error, but not necessarily evaluated.

**Constructing instances**

Generally, you shouldn't construct ContactResultsToLcmSystem instances
directly. We recommend using one of the overloaded
contact_result_vis_creation "ConnectContactResultsToDrakeVisualizer()"
functions to add contact visualization to your diagram.

**How contacts are described in visualization**

In the visualizer, each contact between two bodies is uniquely
characterized by two triples of names: (model instance name, body
name, geometry name). These triples help distinguish contacts which
might otherwise be ambiguous (e.g., contact with two bodies, both
called "box" but part of different model instances).

ContactResultsToLcmSystem gets the model instance and body names from
an instance of MultibodyPlant, but *geometry* names are not available
from the plant. By default, ContactResultsToLcmSystem will *generate*
a unique name based on a geometry's unique id (e.g., "Id(7)"). For
many applications (those cases where each body has only a single
collision geometry), this is perfectly acceptable. However, in cases
where a body has multiple collision geometries, those default names
may not be helpful when viewing the visualized results. Instead,
ContactResultsToLcmSystem can use the names associated with the id in
a geometry::SceneGraph instance. The only method for doing this is via
the contact_result_vis_creation
"ConnectContactResultsToDrakeVisualizer()" functions and requires the
diagram to be instantiated as double valued. If a diagram with a
different scalar type is required, it should subsequently be scalar
converted.

.. pydrake_system::

    name: ContactResultsToLcmSystem
    input_ports:
    - u0
    output_ports:
    - y0)""";
        // Symbol: drake::multibody::ContactResultsToLcmSystem::ContactResultsToLcmSystem<T>
        struct /* ctor */ {
          // Source: drake/multibody/plant/contact_results_to_lcm.h
          const char* doc =
R"""(Constructs an instance with *default* geometry names (e.g., "Id(7)").

Parameter ``plant``:
    The MultibodyPlant that the ContactResults are generated from.

Precondition:
    The ``plant`` parameter (or a fully equivalent plant) connects to
    ``this`` system's input port.

Precondition:
    The ``plant`` parameter is finalized.)""";
          // Source: drake/multibody/plant/contact_results_to_lcm.h
          const char* doc_copyconvert = R"""(Scalar-converting copy constructor.)""";
        } ctor;
        // Symbol: drake::multibody::ContactResultsToLcmSystem::get_contact_result_input_port
        struct /* get_contact_result_input_port */ {
          // Source: drake/multibody/plant/contact_results_to_lcm.h
          const char* doc = R"""()""";
        } get_contact_result_input_port;
        // Symbol: drake::multibody::ContactResultsToLcmSystem::get_lcm_message_output_port
        struct /* get_lcm_message_output_port */ {
          // Source: drake/multibody/plant/contact_results_to_lcm.h
          const char* doc = R"""()""";
        } get_lcm_message_output_port;
      } ContactResultsToLcmSystem;
      // Symbol: drake::multibody::CoulombFriction
      struct /* CoulombFriction */ {
        // Source: drake/multibody/plant/coulomb_friction.h
        const char* doc =
R"""(Parameters for Coulomb's Law of Friction, namely:

- Static friction coefficient, for a pair of surfaces at rest relative to
  each other.
- Dynamic (or kinematic) friction coefficient, for a pair of surfaces in
  relative motion.

These coefficients are an empirical property characterizing the
interaction by friction between a pair of contacting surfaces.
Friction coefficients depend upon the mechanical properties of the
surfaces' materials and on the roughness of the surfaces. They are
determined experimentally.

Even though the Coulomb's law coefficients of friction characterize a
pair of surfaces interacting by friction, we associate the abstract
**idea** of friction coefficients to a single surface by considering
the coefficients for contact between two identical surfaces. For this
case of two identical surfaces, the friction coefficients that
describe the surface pair are taken to equal those of one of the
identical surfaces. We extend this idea to the case of different
surfaces by defining a **combination law** that allow us to obtain the
Coulomb's law coefficients of friction characterizing the pair of
surfaces, given the individual friction coefficients of each surface.
We would like this **combination law** to satisfy:

- The friction coefficient of two identical surfaces is the friction
  coefficient of one of the surfaces.
- The combination law is commutative. That is, surface A combined with
  surface B gives the same results as surface B combined with surface A.
- For two surfaces M and N with very different friction coefficients, say
  ``μₘ ≪ μₙ``, the combined friction coefficient should be in the order of
  magnitude of the smallest friction coefficient (in the example μₘ). To
  understand this requirement, consider rubber (high friction coefficient)
  sliding on ice (low friction coefficient). We'd like the surface pair
  to be defined by a friction coefficient close to that of ice, since rubber
  will easily slide on ice.

These requirements are met by the following ad-hoc combination law:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    μ = 2μₘμₙ/(μₘ + μₙ)

.. raw:: html

    </details>

See CalcContactFrictionFromSurfaceProperties(), which implements this
law. More complex combination laws could also be a function of other
parameters such as the mechanical properties of the interacting
surfaces or even their roughnesses. For instance, if the rubber
surface above has metal studs (somehow making the surface "rougher"),
it will definitely have a better grip on an ice surface. Therefore
this new variable should be taken into account in the combination law.
Notice that in this example, this new combination law model for tires,
will have a different set of requirements from the ones stated above.)""";
        // Symbol: drake::multibody::CoulombFriction::CoulombFriction<T>
        struct /* ctor */ {
          // Source: drake/multibody/plant/coulomb_friction.h
          const char* doc_0args =
R"""(Default constructor for a frictionless surface, i.e. with zero static
and dynamic coefficients of friction.)""";
          // Source: drake/multibody/plant/coulomb_friction.h
          const char* doc_2args =
R"""(Specifies both the static and dynamic friction coefficients for a
given surface.

Raises:
    RuntimeError if any of the friction coefficients are negative or
    if ``dynamic_friction > static_friction`` (they can be equal.))""";
        } ctor;
        // Symbol: drake::multibody::CoulombFriction::dynamic_friction
        struct /* dynamic_friction */ {
          // Source: drake/multibody/plant/coulomb_friction.h
          const char* doc =
R"""(Returns the coefficient of dynamic friction.)""";
        } dynamic_friction;
        // Symbol: drake::multibody::CoulombFriction::static_friction
        struct /* static_friction */ {
          // Source: drake/multibody/plant/coulomb_friction.h
          const char* doc =
R"""(Returns the coefficient of static friction.)""";
        } static_friction;
      } CoulombFriction;
      // Symbol: drake::multibody::DeformableContactInfo
      struct /* DeformableContactInfo */ {
        // Source: drake/multibody/plant/deformable_contact_info.h
        const char* doc =
R"""(A class containing information regarding contact and contact response
between two geometries belonging to a pair of bodies with at least one
of them being a deformable body. This class provides:

- The shared contact mesh between the two geometries.
- The tractions acting at the contact points on the contact mesh.
- The slip speeds at the contact points on the contact mesh.
- The spatial force from the integrated tractions that is applied at the
centroid of the contact surface.

The two geometries are denoted as A and B respectively with geometry A
guaranteed to be belonging to a deformable body.

Warning:
    This feature is considered to be **experimental** and may change
    or be removed at any time, without any deprecation notice ahead of
    time.)""";
        // Symbol: drake::multibody::DeformableContactInfo::DeformableContactInfo<T>
        struct /* ctor */ {
          // Source: drake/multibody/plant/deformable_contact_info.h
          const char* doc =
R"""(Constructs a DeformableContactInfo.

Parameter ``id_A``:
    The geometry id of the deformable geometry A.

Parameter ``id_B``:
    The geometry id of geometry B.

Parameter ``contact_mesh_W``:
    The contact mesh between geometries A and B in the world frame.

Parameter ``F_Ac_W``:
    Spatial force acting on body A, at contact mesh centroid C, and
    expressed in the world frame.

Precondition:
    id_A corresponds to a deformable geometry.)""";
        } ctor;
        // Symbol: drake::multibody::DeformableContactInfo::F_Ac_W
        struct /* F_Ac_W */ {
          // Source: drake/multibody/plant/deformable_contact_info.h
          const char* doc =
R"""(Gets the spatial force applied on the deformable body associated with
geometry A, at the centroid point C of the contact surface mesh, and
expressed in the world frame W.)""";
        } F_Ac_W;
        // Symbol: drake::multibody::DeformableContactInfo::contact_mesh
        struct /* contact_mesh */ {
          // Source: drake/multibody/plant/deformable_contact_info.h
          const char* doc =
R"""(Returns a reference to the contact mesh expressed in the world frame.)""";
        } contact_mesh;
        // Symbol: drake::multibody::DeformableContactInfo::id_A
        struct /* id_A */ {
          // Source: drake/multibody/plant/deformable_contact_info.h
          const char* doc =
R"""(The geometry id of geometry A, guaranteed to belong to a deformable
body in contact.)""";
        } id_A;
        // Symbol: drake::multibody::DeformableContactInfo::id_B
        struct /* id_B */ {
          // Source: drake/multibody/plant/deformable_contact_info.h
          const char* doc = R"""(The geometry id of geometry B.)""";
        } id_B;
      } DeformableContactInfo;
      // Symbol: drake::multibody::DeformableModel
      struct /* DeformableModel */ {
        // Source: drake/multibody/plant/deformable_model.h
        const char* doc =
R"""(DeformableModel implements the interface in PhysicalModel and provides
the functionalities to specify deformable bodies. Unlike rigid bodies,
the shape of deformable bodies can change in a simulation. Each
deformable body is modeled as a volumetric mesh with persisting
topology, changing vertex positions, and an approximated signed
distance field. A finite element model is built for each registered
deformable body that is used to evaluate the dynamics of the body.

Warning:
    This feature is considered to be **experimental** and may change
    or be removed at any time, without any deprecation notice ahead of
    time.)""";
        // Symbol: drake::multibody::DeformableModel::AddExternalForce
        struct /* AddExternalForce */ {
          // Source: drake/multibody/plant/deformable_model.h
          const char* doc =
R"""(Registers an external force density field that applies external force
to all deformable bodies.

Raises:
    RuntimeError if ``this`` DeformableModel is not of scalar type
    double.

Raises:
    RuntimeError if ``this`` DeformableModel belongs to a continuous
    MultibodyPlant.

Raises:
    RuntimeError if Finalize() has been called on the multibody plant
    owning this deformable model.)""";
        } AddExternalForce;
        // Symbol: drake::multibody::DeformableModel::AddFixedConstraint
        struct /* AddFixedConstraint */ {
          // Source: drake/multibody/plant/deformable_model.h
          const char* doc =
R"""(Defines a fixed constraint between a deformable body A and a rigid
body B. Such a fixed constraint is modeled as distance holonomic
constraints:

p_PᵢQᵢ(q) = 0 for each constrained vertex Pᵢ

where Pᵢ is the i-th vertex of the deformable body under constraint
and Qᵢ is a point rigidly affixed to the rigid body B. To specify the
constraint, we put the reference mesh M of the deformable body A in
B's body frame with the given pose ``X_BA`` and prescribe a shape G
with pose ``X_BG`` in B's body frame. All vertices Pᵢ in M that are
inside (or on the surface of) G are subject to the fixed constraints
with Qᵢ being coincident with Pᵢ when M is in pose X_BA. p_PᵢQᵢ(q)
denotes the relative position of point Qᵢ with respect to point Pᵢ as
a function of the configuration of the model q. Imposing this
constraint forces Pᵢ and Qᵢ to be coincident for each vertex i of the
deformable body specified to be under constraint.

Parameter ``body_A_id``:
    The unique id of the deformable body under the fixed constraint.

Parameter ``body_B``:
    The rigid body under constraint.

Parameter ``X_BA``:
    The pose of deformable body A's reference mesh in B's body frame

Parameter ``shape_G``:
    The prescribed geometry shape, attached to rigid body B, used to
    determine which vertices of the deformable body A is under
    constraint.

Parameter ``X_BG``:
    The fixed pose of the geometry frame of the given ``shape`` in
    body B's frame.

Returns:
    the unique id of the newly added constraint.

Raises:
    RuntimeError if no deformable body with the given ``body_A_id``
    has been registered.

Raises:
    RuntimeError unless ``body_B`` is registered with the same
    multibody plant owning this deformable model.

Raises:
    RuntimeError if Finalize() has been called on the multibody plant
    owning this deformable model.

Raises:
    RuntimeError if ``this`` DeformableModel is not of scalar type
    double.

Raises:
    RuntimeError if no constraint is added (i.e. no vertex of the
    deformable body is inside the given ``shape`` with the given
    poses).)""";
        } AddFixedConstraint;
        // Symbol: drake::multibody::DeformableModel::DeformableModel<T>
        struct /* ctor */ {
          // Source: drake/multibody/plant/deformable_model.h
          const char* doc =
R"""((Internal only) Constructs a DeformableModel to be owned by the given
MultibodyPlant. This constructor is only intended to be called
internally by MultibodyPlant.

Precondition:
    plant != nullptr.

Precondition:
    Finalize() has not been called on ``plant``.)""";
        } ctor;
        // Symbol: drake::multibody::DeformableModel::Disable
        struct /* Disable */ {
          // Source: drake/multibody/plant/deformable_model.h
          const char* doc =
R"""(Disables the deformable body with the given ``id`` in the given
context. Disabling a deformable body sets its vertex velocities and
accelerations to zero and freezes its vertex positions. A disabled
deformable body is not subject to any constraint (e.g. frictional
contact constraint or fixed constraint); it does not move under the
influence of external forces (e.g. gravity); and it does not
necessarily satisfy the prescribed boundary condition (if any). On the
flip side, a disabled deformable body does not affect the dynamics of
other bodies, even if the collision between the disabled body's
geometry and other geometries is not filtered. Effectively, the
physics of the deformable body stop being computed. The deformable
body can be enabled by calling Enable(). Calling Disable() on a body
which is already disabled has no effect.

See also:
    Enable().

Raises:
    RuntimeError if the passed in context isn't compatible with the
    MultibodyPlant associated with this DeformableModel.

Raises:
    RuntimeError if a deformable body with the given id is not
    registered.

Raises:
    RuntimeError if context is null.)""";
        } Disable;
        // Symbol: drake::multibody::DeformableModel::Enable
        struct /* Enable */ {
          // Source: drake/multibody/plant/deformable_model.h
          const char* doc =
R"""(Enables the deformable body with the given ``id`` in the given
context. Calling Enable() on a body which is already enabled has no
effect.

See also:
    Disable().

Raises:
    RuntimeError if the passed in context isn't compatible with the
    MultibodyPlant associated with this DeformableModel.

Raises:
    RuntimeError if a deformable body with the given id is not
    registered.

Raises:
    RuntimeError if context is null.)""";
        } Enable;
        // Symbol: drake::multibody::DeformableModel::GetBody
        struct /* GetBody */ {
          // Source: drake/multibody/plant/deformable_model.h
          const char* doc_1args_id =
R"""(Returns the deformable body with the given ``id``.

Raises:
    RuntimeError if no deformable body with the given ``id`` has been
    registered in this model.)""";
          // Source: drake/multibody/plant/deformable_model.h
          const char* doc_1args_index =
R"""(Returns the deformable body with the given ``index``.

Raises:
    RuntimeError if no deformable body with the given ``index`` is
    registered in this model.)""";
        } GetBody;
        // Symbol: drake::multibody::DeformableModel::GetBodyByName
        struct /* GetBodyByName */ {
          // Source: drake/multibody/plant/deformable_model.h
          const char* doc_1args =
R"""(Returns the DeformableBody with the given name.

Raises:
    RuntimeError if there's no body with the given name or if more
    than one model instance contains a deformable body with the given
    name.)""";
          // Source: drake/multibody/plant/deformable_model.h
          const char* doc_2args =
R"""(Returns the DeformableBody with the given name from the given model
instance.

Raises:
    RuntimeError if there's no body with the given name that is
    registered with the given model instance.)""";
        } GetBodyByName;
        // Symbol: drake::multibody::DeformableModel::GetBodyId
        struct /* GetBodyId */ {
          // Source: drake/multibody/plant/deformable_model.h
          const char* doc_1args_index =
R"""(Returns the DeformableBodyId of the body with the given body index.

Raises:
    RuntimeError if no deformable body with the given index has been
    registered in this model.)""";
          // Source: drake/multibody/plant/deformable_model.h
          const char* doc_1args_geometry_id =
R"""(Returns the DeformableBodyId associated with the given
``geometry_id``.

Raises:
    RuntimeError if the given ``geometry_id`` does not correspond to a
    deformable body registered with this model.)""";
        } GetBodyId;
        // Symbol: drake::multibody::DeformableModel::GetBodyIds
        struct /* GetBodyIds */ {
          // Source: drake/multibody/plant/deformable_model.h
          const char* doc =
R"""(Returns the DeformableIds of the bodies that belong to the given model
instance. Returns the empty vector if no deformable bodies are
registered with the given model instance.)""";
        } GetBodyIds;
        // Symbol: drake::multibody::DeformableModel::GetBodyIndex
        struct /* GetBodyIndex */ {
          // Source: drake/multibody/plant/deformable_model.h
          const char* doc =
R"""(Returns the DeformableBodyIndex of the body with the given id.

Raises:
    RuntimeError if no body with the given ``id`` has been registered.)""";
        } GetBodyIndex;
        // Symbol: drake::multibody::DeformableModel::GetDiscreteStateIndex
        struct /* GetDiscreteStateIndex */ {
          // Source: drake/multibody/plant/deformable_model.h
          const char* doc =
R"""(Returns the discrete state index of the deformable body identified by
the given ``id``.

Raises:
    RuntimeError if MultibodyPlant::Finalize() has not been called
    yet. or if no deformable body with the given ``id`` has been
    registered in this model.)""";
        } GetDiscreteStateIndex;
        // Symbol: drake::multibody::DeformableModel::GetExternalForces
        struct /* GetExternalForces */ {
          // Source: drake/multibody/plant/deformable_model.h
          const char* doc =
R"""(Returns the force density fields acting on the deformable body with
the given ``id``.

Raises:
    RuntimeError if MultibodyPlant::Finalize() has not been called
    yet. or if no deformable body with the given ``id`` has been
    registered in this model.)""";
        } GetExternalForces;
        // Symbol: drake::multibody::DeformableModel::GetFemModel
        struct /* GetFemModel */ {
          // Source: drake/multibody/plant/deformable_model.h
          const char* doc =
R"""(Returns the FemModel for the body with ``id``.

Raises:
    exception if no deformable body with ``id`` is registered with
    ``this`` DeformableModel.)""";
        } GetFemModel;
        // Symbol: drake::multibody::DeformableModel::GetGeometryId
        struct /* GetGeometryId */ {
          // Source: drake/multibody/plant/deformable_model.h
          const char* doc =
R"""(Returns the GeometryId of the geometry associated with the body with
the given ``id``.

Raises:
    RuntimeError if no body with the given ``id`` has been registered.)""";
        } GetGeometryId;
        // Symbol: drake::multibody::DeformableModel::GetMutableBody
        struct /* GetMutableBody */ {
          // Source: drake/multibody/plant/deformable_model.h
          const char* doc =
R"""(Returns a mutable reference to the deformable body with the given
``id``.

Raises:
    RuntimeError if no deformable body with the given ``id`` has been
    registered in this model.)""";
        } GetMutableBody;
        // Symbol: drake::multibody::DeformableModel::GetPositions
        struct /* GetPositions */ {
          // Source: drake/multibody/plant/deformable_model.h
          const char* doc =
R"""(Returns the matrix of vertex positions for the deformable body with
the given ``id`` in the provided ``context``.

Parameter ``context``:
    The context associated with the MultibodyPlant that owns this
    DeformableModel.

Parameter ``id``:
    The identifier of the deformable body whose positions are being
    queried.

Returns ``q``:
    A 3×N matrix containing the positions of all vertices of the body.

Raises:
    RuntimeError if any of the following conditions are met: 1.
    ``context`` does not belong to the MultibodyPlant associated with
    this DeformableModel. 2. No body with the given ``id`` is
    registered. 3. ``Finalize()`` has not been called on the
    MultibodyPlant that owns this deformable model.)""";
        } GetPositions;
        // Symbol: drake::multibody::DeformableModel::GetPositionsAndVelocities
        struct /* GetPositionsAndVelocities */ {
          // Source: drake/multibody/plant/deformable_model.h
          const char* doc =
R"""(Returns the matrix of vertex positions and velocities for the
deformable body with the given ``id`` in the provided ``context``. The
first N columns are the positions and the next N columns are the
velocities.

Parameter ``context``:
    The context associated with the MultibodyPlant that owns this
    DeformableModel.

Parameter ``id``:
    The identifier of the deformable body whose state is being
    queried.

Returns:
    A 3x2N matrix containing the positions and velocities of all
    vertices of the body.

Raises:
    RuntimeError if any of the following conditions are met: 1.
    ``context`` does not belong to the MultibodyPlant associated with
    this DeformableModel. 2. No body with the given ``id`` is
    registered. 3. ``Finalize()`` has not been called on the
    MultibodyPlant that owns this deformable model.)""";
        } GetPositionsAndVelocities;
        // Symbol: drake::multibody::DeformableModel::GetReferencePositions
        struct /* GetReferencePositions */ {
          // Source: drake/multibody/plant/deformable_model.h
          const char* doc =
R"""(Returns the reference positions of the vertices of the deformable body
identified by the given ``id``. The reference positions are
represented as a VectorX with 3N values where N is the number of
vertices. The x-, y-, and z-positions (measured and expressed in the
world frame) of the j-th vertex are 3j, 3j + 1, and 3j + 2 in the
VectorX.

Raises:
    RuntimeError if no deformable body with the given ``id`` has been
    registered in this model.)""";
        } GetReferencePositions;
        // Symbol: drake::multibody::DeformableModel::GetVelocities
        struct /* GetVelocities */ {
          // Source: drake/multibody/plant/deformable_model.h
          const char* doc =
R"""(Returns the matrix of vertex velocities for the deformable body with
the given ``id`` in the provided ``context``.

Parameter ``context``:
    The context associated with the MultibodyPlant that owns this
    DeformableModel.

Parameter ``id``:
    The identifier of the deformable body whose velocities are being
    queried.

Returns ``v``:
    A 3×N matrix containing the velocities of all vertices of the
    body.

Raises:
    RuntimeError if any of the following conditions are met: 1.
    ``context`` does not belong to the MultibodyPlant associated with
    this DeformableModel. 2. No body with the given ``id`` is
    registered. 3. ``Finalize()`` has not been called on the
    MultibodyPlant that owns this deformable model.)""";
        } GetVelocities;
        // Symbol: drake::multibody::DeformableModel::HasBodyNamed
        struct /* HasBodyNamed */ {
          // Source: drake/multibody/plant/deformable_model.h
          const char* doc_1args =
R"""(Returns true if and only if a deformable body with the given ``name``
has been registered with this model.)""";
          // Source: drake/multibody/plant/deformable_model.h
          const char* doc_2args =
R"""(Returns true if and only if a deformable body with the given ``name``
has been registered with this model under the given
``model_instance``.)""";
        } HasBodyNamed;
        // Symbol: drake::multibody::DeformableModel::HasConstraint
        struct /* HasConstraint */ {
          // Source: drake/multibody/plant/deformable_model.h
          const char* doc =
R"""(Returns the true if the deformable body with the given ``id`` has
constraints associated with it.)""";
        } HasConstraint;
        // Symbol: drake::multibody::DeformableModel::RegisterDeformableBody
        struct /* RegisterDeformableBody */ {
          // Source: drake/multibody/plant/deformable_model.h
          const char* doc_4args =
R"""(Registers a deformable body in ``this`` DeformableModel with the given
GeometryInstance. The body is represented in the world frame and
simulated with FEM with linear elements and a first order quadrature
rule that integrates linear functions exactly. See FemModel for
details. Returns a unique identifier for the added geometry.

Parameter ``geometry_instance``:
    The geometry to be registered with the model.

Parameter ``config``:
    The physical properties of deformable body.

Parameter ``model_instance``:
    The model instance index which this body is part of.

Parameter ``resolution_hint``:
    The parameter that guides the level of mesh refinement of the
    deformable geometry. It has length units (in meters) and roughly
    corresponds to a typical edge length in the resulting mesh for a
    primitive shape.

Precondition:
    resolution_hint > 0.

Raises:
    RuntimeError if ``this`` DeformableModel is not of scalar type
    double.

Raises:
    RuntimeError if ``this`` DeformableModel belongs to a continuous
    MultibodyPlant.

Raises:
    RuntimeError if the model instance does not exist.

Raises:
    RuntimeError if a deformable body with the same name has already
    been registered to the model instance.

Raises:
    RuntimeError if Finalize() has been called on the multibody plant
    owning this deformable model.)""";
          // Source: drake/multibody/plant/deformable_model.h
          const char* doc_3args =
R"""(Registers a deformable body in ``this`` DeformableModel with the
default model instance.)""";
        } RegisterDeformableBody;
        // Symbol: drake::multibody::DeformableModel::SetDefaultState
        struct /* SetDefaultState */ {
          // Source: drake/multibody/plant/deformable_model.h
          const char* doc =
R"""((Internal use only) Sets the default state for the deformable model.
This should only be called by MultibodyPlant as a part of
MultibodyPlant::SetDefaultState().)""";
        } SetDefaultState;
        // Symbol: drake::multibody::DeformableModel::SetParallelism
        struct /* SetParallelism */ {
          // Source: drake/multibody/plant/deformable_model.h
          const char* doc =
R"""((Internal use only) Configures the parallelism that ``this``
DeformableModel uses when opportunities for parallel computation
arises.)""";
        } SetParallelism;
        // Symbol: drake::multibody::DeformableModel::SetPositions
        struct /* SetPositions */ {
          // Source: drake/multibody/plant/deformable_model.h
          const char* doc =
R"""(Sets the vertex positions of the deformable body with the given ``id``
in the provided ``context``.

Parameter ``out``:
    ] context The context associated with the MultibodyPlant that owns
    this DeformableModel.

Parameter ``id``:
    The identifier of the deformable body whose positions are being
    set.

Parameter ``q``:
    A 3×N matrix of vertex positions.

Raises:
    RuntimeError if any of the following conditions are met: 1.
    ``context`` is nullptr. 2. ``context`` does not belong to the
    MultibodyPlant associated with this DeformableModel. 3. No body
    with the given ``id`` is registered. 4. The number of columns of
    ``q`` does not match the number of vertices of the body. 5. ``q``
    contains non-finite values. 6. ``Finalize()`` has not been called
    on the MultibodyPlant that owns this deformable model.)""";
        } SetPositions;
        // Symbol: drake::multibody::DeformableModel::SetPositionsAndVelocities
        struct /* SetPositionsAndVelocities */ {
          // Source: drake/multibody/plant/deformable_model.h
          const char* doc =
R"""(Sets the vertex positions and velocities of the deformable body with
the given ``id`` in the provided ``context``.

Parameter ``out``:
    ] context The context associated with the MultibodyPlant that owns
    this DeformableModel.

Parameter ``id``:
    The identifier of the deformable body whose positions and
    velocities are being set.

Parameter ``q``:
    A 3×N matrix of vertex positions.

Parameter ``v``:
    A 3×N matrix of vertex velocities.

Raises:
    RuntimeError if any of the following conditions are met: 1.
    ``context`` is nullptr. 2. ``context`` does not belong to the
    MultibodyPlant associated with this DeformableModel. 3. No body
    with the given ``id`` is registered. 4. The number of columns of
    ``q`` or ``v`` does not match the number of vertices of the body.
    5. ``q`` or ``v`` contains non-finite values. 6. ``Finalize()``
    has not been called on the MultibodyPlant that owns this
    deformable model.)""";
        } SetPositionsAndVelocities;
        // Symbol: drake::multibody::DeformableModel::SetVelocities
        struct /* SetVelocities */ {
          // Source: drake/multibody/plant/deformable_model.h
          const char* doc =
R"""(Sets the vertex velocities of the deformable body with the given
``id`` in the provided ``context``.

Parameter ``out``:
    ] context The context associated with the MultibodyPlant that owns
    this DeformableModel.

Parameter ``id``:
    The identifier of the deformable body whose velocities are being
    set.

Parameter ``v``:
    A 3×N matrix of vertex velocities.

Raises:
    RuntimeError if any of the following conditions are met: 1.
    ``context`` is nullptr. 2. ``context`` does not belong to the
    MultibodyPlant associated with this DeformableModel. 3. No body
    with the given ``id`` is registered. 4. The number of columns of
    ``v`` does not match the number of vertices of the body. 5. ``v``
    contains non-finite values. 6. ``Finalize()`` has not been called
    on the MultibodyPlant that owns this deformable model.)""";
        } SetVelocities;
        // Symbol: drake::multibody::DeformableModel::SetWallBoundaryCondition
        struct /* SetWallBoundaryCondition */ {
          // Source: drake/multibody/plant/deformable_model.h
          const char* doc =
R"""(Sets wall boundary conditions for the body with the given ``id``. All
vertices of the mesh of the deformable body whose reference positions
are inside the prescribed open half space are put under zero
displacement boundary conditions. The open half space is defined by a
plane with outward normal n_W. A vertex V is considered to be subject
to the boundary condition if n̂ ⋅ p_QV < 0 where Q is a point on the
plane and n̂ is normalized n_W.

Parameter ``id``:
    The body to be put under boundary condition.

Parameter ``p_WQ``:
    The position of a point Q on the plane in the world frame.

Parameter ``n_W``:
    Outward normal to the half space expressed in the world frame.

Precondition:
    n_W.norm() > 1e-10.

Warning:
    Be aware of round-off errors in floating computations when placing
    a vertex very close to the plane defining the half space.

Raises:
    RuntimeError if no deformable body with the given ``id`` has been
    registered in this model.)""";
        } SetWallBoundaryCondition;
        // Symbol: drake::multibody::DeformableModel::configuration_output_port_index
        struct /* configuration_output_port_index */ {
          // Source: drake/multibody/plant/deformable_model.h
          const char* doc =
R"""((Internal use only) Returns the output port index of the vertex
positions port for all registered deformable bodies.

Raises:
    RuntimeError if called before ``DeclareSceneGraphPorts()`` is
    called.)""";
        } configuration_output_port_index;
        // Symbol: drake::multibody::DeformableModel::integrator
        struct /* integrator */ {
          // Source: drake/multibody/plant/deformable_model.h
          const char* doc =
R"""((Internal use only) Returns the time integrator used to for all
FemModels in this model.

Raises:
    RuntimeError if the integrator hasn't been set.)""";
        } integrator;
        // Symbol: drake::multibody::DeformableModel::is_cloneable_to_autodiff
        struct /* is_cloneable_to_autodiff */ {
          // Source: drake/multibody/plant/deformable_model.h
          const char* doc =
R"""(Returns true if and only if this DeformableModel is empty.)""";
        } is_cloneable_to_autodiff;
        // Symbol: drake::multibody::DeformableModel::is_cloneable_to_double
        struct /* is_cloneable_to_double */ {
          // Source: drake/multibody/plant/deformable_model.h
          const char* doc = R"""()""";
        } is_cloneable_to_double;
        // Symbol: drake::multibody::DeformableModel::is_cloneable_to_symbolic
        struct /* is_cloneable_to_symbolic */ {
          // Source: drake/multibody/plant/deformable_model.h
          const char* doc =
R"""(Returns true if and only if this DeformableModel is empty.)""";
        } is_cloneable_to_symbolic;
        // Symbol: drake::multibody::DeformableModel::is_empty
        struct /* is_empty */ {
          // Source: drake/multibody/plant/deformable_model.h
          const char* doc =
R"""(Returns true if there's no deformable body or external force
registered to ``this`` DeformableModel.)""";
        } is_empty;
        // Symbol: drake::multibody::DeformableModel::is_enabled
        struct /* is_enabled */ {
          // Source: drake/multibody/plant/deformable_model.h
          const char* doc =
R"""(Returns:
    true if and only if the deformable body with the given id is
    enabled.

Raises:
    RuntimeError if the passed in context isn't compatible with the
    MultibodyPlant associated with this DeformableModel.

Raises:
    RuntimeError if a deformable body with the given id is not
    registered.)""";
        } is_enabled;
        // Symbol: drake::multibody::DeformableModel::num_bodies
        struct /* num_bodies */ {
          // Source: drake/multibody/plant/deformable_model.h
          const char* doc =
R"""(Returns the number of deformable bodies registered with this
DeformableModel.)""";
        } num_bodies;
        // Symbol: drake::multibody::DeformableModel::parallelism
        struct /* parallelism */ {
          // Source: drake/multibody/plant/deformable_model.h
          const char* doc =
R"""((Internal use only) Returns the parallelism that ``this``
DeformableModel uses when opportunities for parallel computation
arises.)""";
        } parallelism;
      } DeformableModel;
      // Symbol: drake::multibody::DiscreteContactApproximation
      struct /* DiscreteContactApproximation */ {
        // Source: drake/multibody/plant/multibody_plant.h
        const char* doc_deprecated =
R"""(The type of the contact approximation used for a discrete
MultibodyPlant model.

kTamsi, kSimilar and kLagged are all approximations to the same
contact model -- Compliant contact with regularized friction, refer to
mbp_contact_modeling "Contact Modeling" for further details. The key
difference however, is that the kSimilar and kLagged approximations
are convex and therefore our contact solver has both theoretical and
practical convergence guarantees --- the solver will always succeed.
Conversely, being non-convex, kTamsi can fail to find a solution.

kSap is also a convex model of compliant contact with regularized
friction. There are a couple of key differences however: - Dissipation
is modeled using a linear Kelvin–Voigt model, parameterized by a
relaxation time constant. See accessing_contact_properties "contact
parameters". - Unlike kTamsi, kSimilar and kLagged where
regularization of friction is parameterized by a stiction tolerance
(see set_stiction_tolerance()), SAP determines regularization
automatically solely based on numerics. Users have no control on the
amount of regularization.

How to choose an approximation

The Hunt & Crossley model is based on physics, it is continuous and
has been experimentally validated. Therefore it is the preferred model
to capture the physics of contact.

Being approximations, kSap and kSimilar introduce a spurious effect of
"gliding" in sliding contact, see [Castro et al., 2023]. This artifact
is 𝒪(δt) but can be significant at large time steps and/or large slip
velocities. kLagged does not suffer from this, but introduces a "weak"
coupling of friction that can introduce non-negligible effects in the
dynamics during impacts or strong transients.

Summarizing, kLagged is the preferred approximation when strong
transients are not expected or don't need to be accurately resolved.
If strong transients do need to be accurately resolved (unusual for
robotics applications), kSimilar is the preferred approximation.

References
----------

- [Castro et al., 2019] Castro A., Qu A., Kuppuswamy N., Alspach A.,
  Sherman M, 2019. A Transition-Aware Method for the Simulation of
  Compliant Contact with Regularized Friction. Available online at
  https://arxiv.org/abs/1909.05700.
- [Castro et al., 2022] Castro A., Permenter F. and Han X., 2022. An
  Unconstrained Convex Formulation of Compliant Contact. Available online at
  https://arxiv.org/abs/2110.10107.
- [Castro et al., 2023] Castro A., Han X., and Masterjohn J., 2023. A Theory
  of Irrotational Contact Fields. Available online at
  https://arxiv.org/abs/2312.03908 (Deprecated.)

Deprecated:
    The TAMSI solver is deprecated for removal. This will be removed
    from Drake on or after 2026-09-01.)""";
        // Symbol: drake::multibody::DiscreteContactApproximation::kLagged
        struct /* kLagged */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(Approximation in which the normal force is lagged in Coulomb's law,
such that ‖γₜ‖ ≤ μ γₙ₀, [Castro et al., 2023].)""";
        } kLagged;
        // Symbol: drake::multibody::DiscreteContactApproximation::kSap
        struct /* kSap */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(SAP solver model approximation, see [Castro et al., 2022].)""";
        } kSap;
        // Symbol: drake::multibody::DiscreteContactApproximation::kSimilar
        struct /* kSimilar */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(Similarity approximation found in [Castro et al., 2023].)""";
        } kSimilar;
        // Symbol: drake::multibody::DiscreteContactApproximation::kTamsi
        struct /* kTamsi */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(TAMSI solver approximation, see [Castro et al., 2019].)""";
        } kTamsi;
      } DiscreteContactApproximation;
      // Symbol: drake::multibody::DiscreteContactSolver
      struct /* DiscreteContactSolver */ {
        // Source: drake/multibody/plant/multibody_plant.h
        const char* doc_deprecated =
R"""(The type of the contact solver used for a discrete MultibodyPlant
model.

Note: the SAP solver only fully supports scalar type ``double``. For
scalar type ``AutoDiffXd``, the SAP solver throws if any constraint
(including contact) is detected. As a consequence, one can only run
dynamic simulations without any constraints under the combination of
SAP and ``AutoDiffXd``. The SAP solver does not support symbolic
calculations.

References
----------

- [Castro et al., 2019] Castro, A.M, Qu, A., Kuppuswamy, N., Alspach, A.,
  Sherman, M.A., 2019. A Transition-Aware Method for the Simulation of
  Compliant Contact with Regularized Friction. Available online at
  https://arxiv.org/abs/1909.05700.
- [Castro et al., 2022] Castro A., Permenter F. and Han X., 2022. An
  Unconstrained Convex Formulation of Compliant Contact. Available online at
  https://arxiv.org/abs/2110.10107. (Deprecated.)

Deprecated:
    The TAMSI solver is deprecated for removal. This will be removed
    from Drake on or after 2026-09-01.)""";
        // Symbol: drake::multibody::DiscreteContactSolver::kSap
        struct /* kSap */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc = R"""(SAP solver, see [Castro et al., 2022].)""";
        } kSap;
        // Symbol: drake::multibody::DiscreteContactSolver::kTamsi
        struct /* kTamsi */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(TAMSI solver, see [Castro et al., 2019].)""";
        } kTamsi;
      } DiscreteContactSolver;
      // Symbol: drake::multibody::DistanceConstraintParams
      struct /* DistanceConstraintParams */ {
        // Source: drake/multibody/plant/distance_constraint_params.h
        const char* doc =
R"""(Parameters for a distance constraint. A distance constraint is modeled
as a holonomic constraint. Distance constraints can be "soft", and are
implemented as a spring force, f: f = -k⋅(d(q) - d₀) - c⋅ḋ(q), where
d₀ is a fixed length, k a stiffness parameter in N/m and c a damping
parameter in N⋅s/m. We use d(q) to denote the Euclidean distance
between two points P and Q, rigidly affixed to bodies A and B
respectively, as a function of the configuration of the model q. This
constraint reduces to d(q) = d₀ in the limit to infinite stiffness and
it behaves as a linear spring damper for finite values of stiffness
and damping.

Warning:
    A distance constraint is the wrong modeling choice if the distance
    needs to go through zero. To constrain two points to be coincident
    we need a 3-dof ball constraint, the 1-dof distance constraint is
    singular in this case. Therefore we require the distance parameter
    to be strictly positive.)""";
        // Symbol: drake::multibody::DistanceConstraintParams::DistanceConstraintParams
        struct /* ctor */ {
          // Source: drake/multibody/plant/distance_constraint_params.h
          const char* doc_0args =
R"""(Construction to an invalid set of parameters only meant to facilitate
use with STL containers.)""";
          // Source: drake/multibody/plant/distance_constraint_params.h
          const char* doc_7args =
R"""(Constructor for a set of valid parameters.

Parameter ``bodyA``:
    Index of body A in the MultibodyPlant.

Parameter ``bodyB``:
    Index of body B in the MultibodyPlant.

Parameter ``p_AP``:
    Position of a point P in A's body frame.

Parameter ``p_BQ``:
    Position of a point Q in Q's body frame.

Parameter ``distance``:
    The fixed length of this constraint.

Parameter ``stiffness``:
    If finite, this constraint models a massless elastic rod of length
    ``distance`` with the given ``stiffness`` and ``damping``.
    "Infinite" is a valid value if a hard constraint is intended. See
    the class documentation for details.

Parameter ``damping``:
    When ``stiffness`` is finite, this value parametrizes the damping
    model of the elastic massless rod. See the class documentation for
    details.

Warning:
    This class does not have a way to check whether body indices do
    correspond to valid indices in a MultibodyPlant. Use
    MultibodyPlant APIs to safely register distance constraints
    between valid bodies and to later retrieve their valid set of
    parameters.

Raises:
    RuntimeError if BodyIndex::is_valid() is false for either body.

Raises:
    RuntimeError if bodies A and B are the same body.

Raises:
    RuntimeError if ``distance`` is not strictly positive.

Raises:
    RuntimeError if ``stiffness`` is not strictly positive.

Raises:
    RuntimeError if ``damping`` is not positive or zero.)""";
        } ctor;
        // Symbol: drake::multibody::DistanceConstraintParams::bodyA
        struct /* bodyA */ {
          // Source: drake/multibody/plant/distance_constraint_params.h
          const char* doc = R"""(Index of body A.)""";
        } bodyA;
        // Symbol: drake::multibody::DistanceConstraintParams::bodyB
        struct /* bodyB */ {
          // Source: drake/multibody/plant/distance_constraint_params.h
          const char* doc = R"""(Index of body B.)""";
        } bodyB;
        // Symbol: drake::multibody::DistanceConstraintParams::damping
        struct /* damping */ {
          // Source: drake/multibody/plant/distance_constraint_params.h
          const char* doc =
R"""(Constraint damping. See class documentation.)""";
        } damping;
        // Symbol: drake::multibody::DistanceConstraintParams::distance
        struct /* distance */ {
          // Source: drake/multibody/plant/distance_constraint_params.h
          const char* doc = R"""(The fixed length of the constraint.)""";
        } distance;
        // Symbol: drake::multibody::DistanceConstraintParams::p_AP
        struct /* p_AP */ {
          // Source: drake/multibody/plant/distance_constraint_params.h
          const char* doc = R"""(Position of point P in body A's frame.)""";
        } p_AP;
        // Symbol: drake::multibody::DistanceConstraintParams::p_BQ
        struct /* p_BQ */ {
          // Source: drake/multibody/plant/distance_constraint_params.h
          const char* doc = R"""(Position of point Q in body B's frame.)""";
        } p_BQ;
        // Symbol: drake::multibody::DistanceConstraintParams::stiffness
        struct /* stiffness */ {
          // Source: drake/multibody/plant/distance_constraint_params.h
          const char* doc =
R"""(Constraint stiffness. See class documentation.)""";
        } stiffness;
      } DistanceConstraintParams;
      // Symbol: drake::multibody::ExternallyAppliedSpatialForce
      struct /* ExternallyAppliedSpatialForce */ {
        // Source: drake/multibody/plant/externally_applied_spatial_force.h
        const char* doc = R"""()""";
        // Symbol: drake::multibody::ExternallyAppliedSpatialForce::F_Bq_W
        struct /* F_Bq_W */ {
          // Source: drake/multibody/plant/externally_applied_spatial_force.h
          const char* doc =
R"""(A spatial force applied to Body B at point Bq, expressed in the world
frame.)""";
        } F_Bq_W;
        // Symbol: drake::multibody::ExternallyAppliedSpatialForce::body_index
        struct /* body_index */ {
          // Source: drake/multibody/plant/externally_applied_spatial_force.h
          const char* doc =
R"""(The index of the body that the force is to be applied to.)""";
        } body_index;
        // Symbol: drake::multibody::ExternallyAppliedSpatialForce::p_BoBq_B
        struct /* p_BoBq_B */ {
          // Source: drake/multibody/plant/externally_applied_spatial_force.h
          const char* doc =
R"""(A position vector from Body B's origin (Bo) to a point Bq (a point of
B), expressed in B's frame.)""";
        } p_BoBq_B;
      } ExternallyAppliedSpatialForce;
      // Symbol: drake::multibody::ExternallyAppliedSpatialForceMultiplexer
      struct /* ExternallyAppliedSpatialForceMultiplexer */ {
        // Source: drake/multibody/plant/externally_applied_spatial_force_multiplexer.h
        const char* doc =
R"""(Concatenates multiple std::vector<>'s of
ExternallyAppliedSpatialForce<T>.

.. pydrake_system::

    name: ExternallyAppliedSpatialForceMultiplexer
    input_ports:
    - u0
    - ...
    - u(N-1)
    output_ports:
    - y0)""";
        // Symbol: drake::multibody::ExternallyAppliedSpatialForceMultiplexer::ExternallyAppliedSpatialForceMultiplexer<T>
        struct /* ctor */ {
          // Source: drake/multibody/plant/externally_applied_spatial_force_multiplexer.h
          const char* doc =
R"""(Constructor.

Parameter ``num_inputs``:
    Number of input ports to be added.)""";
          // Source: drake/multibody/plant/externally_applied_spatial_force_multiplexer.h
          const char* doc_copyconvert =
R"""(Scalar-converting copy constructor. See system_scalar_conversion.)""";
        } ctor;
      } ExternallyAppliedSpatialForceMultiplexer;
      // Symbol: drake::multibody::HydroelasticContactInfo
      struct /* HydroelasticContactInfo */ {
        // Source: drake/multibody/plant/hydroelastic_contact_info.h
        const char* doc =
R"""(A class containing information regarding contact and contact response
between two geometries attached to a pair of bodies. This class
provides the output from the Hydroelastic contact model and includes:

- The shared contact surface between the two geometries, which includes
the virtual pressures acting at every point on the contact surface.
- The spatial force from the integrated tractions that is applied at the
centroid of the contact surface.

The two geometries, denoted M and N (and obtainable via
``contact_surface().id_M()`` and ``contact_surface().id_N()``) are
attached to bodies A and B, respectively.

When T = Expression, the class is specialized to not contain any
member data, because ContactSurface doesn't support Expression.)""";
        // Symbol: drake::multibody::HydroelasticContactInfo::F_Ac_W
        struct /* F_Ac_W */ {
          // Source: drake/multibody/plant/hydroelastic_contact_info.h
          const char* doc =
R"""(Gets the spatial force applied on body A, at the centroid point C of
the surface mesh M, and expressed in the world frame W. The position
``p_WC`` of the centroid point C in the world frame W can be obtained
with ``contact_surface().centroid()``.)""";
        } F_Ac_W;
        // Symbol: drake::multibody::HydroelasticContactInfo::HydroelasticContactInfo<T>
        struct /* ctor */ {
          // Source: drake/multibody/plant/hydroelastic_contact_info.h
          const char* doc_was_unable_to_choose_unambiguous_names =
R"""(Constructs this structure using the given contact surface and spatial
force.

The geometry::ContactSurface defines contact between two geometries M
and N (via contact_surface().id_M() and contact_surface().id_N(),
respectively). HydroelasticContactInfo further associates geometries M
and N with the bodies to which they are rigidly fixed, A and B,
respectively. It is the responsibility of the caller of this
constructor to ensure that the parameters satisfy the documented
invariants, see below. Similarly, the spatial force ``F_Ac_W`` must be
provided as indicated by the monogram notation in use, that is, it is
the spatial force on body A, at the contact surface's centroid C, and
expressed in the world frame.

Parameter ``contact_surface``:
    Contact surface between two geometries M and N, see
    geometry::ContactSurface::id_M() and
    geometry::ContactSurface::id_N(). This must point to immutable
    data (i.e., data that will never change), and must not be nullptr.

Parameter ``F_Ac_W``:
    Spatial force applied on body A, at contact surface centroid C,
    and expressed in the world frame W. The position ``p_WC`` of C in
    the world frame W can be obtained with
    ``ContactSurface::centroid()``.)""";
        } ctor;
        // Symbol: drake::multibody::HydroelasticContactInfo::contact_surface
        struct /* contact_surface */ {
          // Source: drake/multibody/plant/hydroelastic_contact_info.h
          const char* doc =
R"""(Returns a reference to the ContactSurface data structure. Note that
the mesh and gradient vector fields are expressed in the world frame.)""";
        } contact_surface;
      } HydroelasticContactInfo;
      // Symbol: drake::multibody::MultibodyPlant
      struct /* MultibodyPlant */ {
        // Source: drake/multibody/plant/multibody_plant.h
        const char* doc =
R"""(MultibodyPlant is a Drake system framework representation (see
systems::System) for the model of a physical system consisting of a
collection of interconnected bodies. See multibody for an overview of
concepts/notation.

.. pydrake_system::

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

The ports whose names begin with <em style="color:gray">
model_instance_name[i]</em> represent groups of ports, one for each of
the model_instances "model instances", with i ∈ {0, ..., N-1} for the
N model instances. If a model instance does not contain any data of
the indicated type the port will still be present but its value will
be a zero-length vector. (Model instances ``world_model_instance()``
and ``default_model_instance()`` always exist.)

The ports shown in <span style="color:green">green</span> are for
communication with Drake's geometry::SceneGraph "SceneGraph" system
for dealing with geometry.

MultibodyPlant provides a user-facing API for:

- mbp_input_and_output_ports "Ports":
Access input and output ports.
- mbp_construction "Construction":
Add bodies, joints, frames, force elements, and actuators.
- mbp_geometry "Geometry":
Register geometries to a provided SceneGraph instance.
- mbp_contact_modeling "Contact modeling":
Select and parameterize contact models.
- mbp_state_accessors_and_mutators "State access and modification":
Obtain and manipulate position and velocity state variables.
- mbp_parameters "Parameters"
Working with system parameters for various multibody elements.
- mbp_working_with_free_bodies "Free and floating base bodies":
Work conveniently with free (floating) bodies.
- mbp_kinematic_and_dynamic_computations "Kinematics and dynamics":
Perform systems::Context "Context"-dependent kinematic and dynamic
queries.
- mbp_system_matrix_computations "System matrices":
Explicitly form matrices that appear in the equations of motion.
- mbp_introspection "Introspection":
Perform introspection to find out what's in the MultibodyPlant.

**** Model Instances

A MultibodyPlant may contain multiple model instances. Each model
instance corresponds to a set of bodies and their connections
(joints). Model instances provide methods to get or set the state of
the set of bodies (e.g., through GetPositionsAndVelocities() and
SetPositionsAndVelocities()), connecting controllers (through
get_state_output_port() and get_actuation_input_port()), and
organizing duplicate models (read through a parser). In fact, many
MultibodyPlant methods are overloaded to allow operating on the entire
plant or just the subset corresponding to the model instance; for
example, one GetPositions() method obtains the generalized positions
for the entire plant while another GetPositions() method obtains the
generalized positions for model instance.

Model instances are frequently defined through SDFormat files (using
the ``model`` tag) and are automatically created when SDFormat files
are parsed (by Parser). There are two special
multibody::ModelInstanceIndex values. The world body is always
multibody::ModelInstanceIndex 0. multibody::ModelInstanceIndex 1 is
reserved for all elements with no explicit model instance and is
generally only relevant for elements created programmatically (and
only when a model instance is not explicitly specified). Note that
Parser creates model instances (resulting in a
multibody::ModelInstanceIndex ≥ 2) as needed.

See num_model_instances(), num_positions(), num_velocities(),
num_actuated_dofs(), AddModelInstance() GetPositionsAndVelocities(),
GetPositions(), GetVelocities(), SetPositionsAndVelocities(),
SetPositions(), SetVelocities(), GetPositionsFromArray(),
GetVelocitiesFromArray(), SetPositionsInArray(),
SetVelocitiesInArray(), SetActuationInArray(),
HasModelInstanceNamed(), GetModelInstanceName(),
get_state_output_port(), get_actuation_input_port().

**** System dynamics

The state of a multibody system ``x = [q; v]`` is given by its
generalized positions vector q, of size ``nq`` (see num_positions()),
and by its generalized velocities vector v, of size ``nv`` (see
num_velocities()).

A MultibodyPlant can be constructed to be either continuous or
discrete. The choice is indicated by the time_step passed to the
constructor -- a non-zero time_step indicates a discrete plant, while
a zero time_step indicates continuous. A systems::Simulator
"Simulator" will step a discrete plant using the indicated time_step,
but will allow a numerical integrator to choose how to advance time
for a continuous MultibodyPlant.

We'll discuss continuous plant dynamics in this section. Discrete
dynamics is more complicated and gets its own section below.

As a Drake systems::System "System", MultibodyPlant implements the
governing equations for a multibody dynamical system in the form ``ẋ
= f(t, x, u)`` with t being time and u external inputs such as
actuation forces. The governing equations for the dynamics of a
multibody system modeled with MultibodyPlant are [Featherstone 2008,
Jain 2010]:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    q̇ = N(q)v
    (1)    M(q)v̇ + C(q, v)v = τ

.. raw:: html

    </details>

where ``M(q)`` is the mass matrix of the multibody system (including
rigid body mass properties and reflected_inertia "reflected
inertias"), ``C(q, v)v`` contains Coriolis, centripetal, and
gyroscopic terms and ``N(q)`` is the kinematic coupling matrix
describing the relationship between q̇ (the time derivatives of the
generalized positions) and the generalized velocities v, [Seth 2010].
``N(q)`` is an ``nq x nv`` matrix. The vector ``τ ∈ ℝⁿᵛ`` on the right
hand side of Eq. (1) is the system's generalized forces. These
incorporate gravity, springs, externally applied body forces,
constraint forces, and contact forces.

**** Discrete system dynamics

We'll start with the basic difference equation interpretation of a
discrete plant and then explain some Drake-specific subtleties.

Note:
    We use "kinematics" here to refer to quantities that involve only
    position or velocity, and "dynamics" to refer to quantities that
    also involve forces.

By default, a discrete MultibodyPlant has these update dynamics:

x[0] = initial kinematics state variables x (={q, v}), s s[0] = empty
(no sample yet)

s[n+1] = g(t[n], x[n], u[n]) record sample x[n+1] = f(t[n], x[n],
u[n]) update kinematics yd[n+1] = gd(s) dynamic outputs use sampled
values yk[n+1] = gk(x) kinematic outputs use current x

Optionally, output port sampling can be disabled. In that case we
have:

x[n+1] = f(t[n], x[n], u[n]) update kinematics yd[n+1] = gd(g(t, x,
u)) dynamic outputs use current values yk[n+1] = gk(x) kinematic
outputs use current x

We're using ``yd`` and ``yk`` above to represent the calculated values
of dynamic and kinematic output ports, resp. Kinematic output ports
are those that depend only on position and velocity: ``state``,
`body_poses`, ``body_spatial_velocities``. Everything else depends on
forces so is a dynamic output port: ``body_spatial_accelerations``,
`generalized_acceleration`, ``net_actuation``, `reaction_forces`, and
``contact_results``.

Use the function SetUseSampledOutputPorts() to choose which dynamics
you prefer. The default behavior (output port sampling) is more
efficient for simulation, but use slightly-different kinematics for
the dynamic output port computations versus the kinematic output
ports. Disabling output port sampling provides "live" output port
results that are recalculated from the current state and inputs
whenever changes occur. It also eliminates the sampling state variable
(s above). Note that kinematic output ports (that is, those depending
only on position and velocity) are always "live" -- they are
calculated as needed from the current (updated) state.

The reason that the default mode is more efficient for simulation is
that the sample variable s records expensive-to-compute results (such
as hydroelastic contact forces) that are needed to advance the state
x. Those results are thus available for free at the start of step n.
If instead we wait until after the state is updated to n+1, we would
have to recalculate those expensive results at the new state in order
to report them. Thus sampling means the output ports show the results
that were calculated using kinematics values x[n], although the
Context has been updated to kinematics values x[n+1]. If that isn't
tolerable you should disable output port sampling. You can also force
an update to occur using ExecuteForcedEvents().

See output_port_sampling "Output port sampling" below for more
practical considerations.

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

**** Output port sampling

As described in mbp_discrete_dynamics "Discrete system dynamics"
above, the semantics of certain MultibodyPlant output ports depends on
whether the plant is configured to advance using continuous time
integration or discrete time steps (see is_discrete()). This section
explains the details, focusing on the practical aspects moreso than
the equations.

Output ports that only depend on the [q, v] kinematic state (such as
get_body_poses_output_port() or
get_body_spatial_velocities_output_port()) do *not* change semantics
for continuous vs discrete time. In all cases, the output value is a
function of the kinematic state in the context.

Output ports that incorporate dynamics (i.e., forces) *do* change
semantics based on the plant mode. Imagine that the
get_applied_spatial_force_input_port() provides a continuously
time-varying input force. The
get_body_spatial_accelerations_output_port() output is dependent on
that force. We could return a snapshot of the acceleration that was
used in the last time step, or we could recalculate the acceleration
to immediately reflect the changing forces. We call the former a
"sampled" port and the latter a "live" port.

For a continuous-time plant, there is no distinction -- the output
port is always live -- it immediately reflects the instantaneous input
value. It is a "direct feedthrough" output port (see
SystemBase::GetDirectFeedthroughs()).

For a discrete-time plant, the user can choose whether the output
should be sampled or live: Use the function SetUseSampledOutputPorts()
to change whether output ports are sampled or not, and
has_sampled_output_ports() to check the current setting. When sampling
is disabled, the only state in the context is the kinematic [q, v], so
dynamics output ports will always reflect the instantaneous answer
(i.e., direct feedthrough). When sampling is enabled (the default),
the plant state incorporates a snapshot of the most recent step's
kinematics and dynamics, and the output ports will reflect that
sampled state (i.e., not direct feedthrough). For a detailed
discussion, see mbp_discrete_dynamics "Discrete system dynamics".

For a discrete-time plant, the sampled outputs are generally *much*
faster to calculate than the feedthrough outputs when any inputs ports
are changing values faster than the discrete time step, e.g., during a
simulation. When input ports are fixed, or change at the time step
rate (e.g., during motion planning), sampled vs feedthrough will have
similar computational performance.

Direct plant API function calls (e.g.,
EvalBodySpatialAccelerationInWorld()) that depend on forces always use
the instantaneous (not sampled) accelerations.

Here are some practical tips that might help inform your particular
situation:

(1) If you need a minimal-state representation for motion planning,
mathematical optimization, or similar, then you can either use a
continuous-time plant or set the config option
``use_sampled_output_ports=false`` on a discrete-time plant.

(2) By default, setting the positions of a discrete-time plant in the
Context will not have any effect on the dynamics-related output ports,
e.g., the contact results will not change. If you need to see changes
to outputs without running the plant in a Simulator, then you can
either use a continuous-time plant, set the config option
``use_sampled_output_ports=false``, or use ExecuteForcedEvents() to
force a dynamics step and then the outputs (and positions) will
change.

**** Actuation

In a MultibodyPlant model an actuator can be added as a JointActuator,
see AddJointActuator(). The plant declares actuation input ports to
provide feedforward actuation, both for the MultibodyPlant as a whole
(see get_actuation_input_port()) and for each individual
model_instances "model instance" in the MultibodyPlant (see
get_actuation_input_port(ModelInstanceIndex)const
"get_actuation_input_port(ModelInstanceIndex)"). - Actuation inputs
and actuation effort limits are taken to be in joint coordinates (they
are not affected by the actuator gear ratio). - Any actuation input
ports not connected are assumed to be zero. - Actuation values from
the full MultibodyPlant model port (get_actuation_input_port()) and
from the per model-instance ports (
get_actuation_input_port(ModelInstanceIndex)const
"get_actuation_input_port(ModelInstanceIndex)") are summed up.

Note:
    A JointActuator's index into the vector data supplied to
    MultibodyPlant's actuation input port for all actuators
    (get_actuation_input_port()) is given by
    JointActuator::input_start(), NOT by its JointActuatorIndex. That
    is, the vector element data for a JointActuator at index
    JointActuatorIndex(i) in the full input port vector is found at
    index:
    MultibodyPlant::get_joint_actuator(JointActuatorIndex(i)).input_start().
    For the get_actuation_input_port(ModelInstanceIndex)const
    "get_actuation_input_port(ModelInstanceIndex)" specific to a model
    index, the vector data is ordered by monotonically increasing
    JointActuatorIndex for the actuators within that model instance:
    the 0ᵗʰ vector element corresponds to the lowest-numbered
    JointActuatorIndex of that instance, the 1ˢᵗ vector element
    corresponds to the second-lowest-numbered JointActuatorIndex of
    that instance, etc.

Note:
    The following snippet shows how per model instance actuation can
    be set:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

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

.. raw:: html

    </details>

Note:
    To inter-operate between the whole plant actuation vector and sets
    of per-model instance actuation vectors, see SetActuationInArray()
    to gather the model instance vectors into a whole plant vector and
    GetActuationFromArray() to scatter the whole plant vector into
    per-model instance vectors.

Warning:
    Effort limits (JointActuator::effort_limit()) are not enforced,
    unless PD controllers are defined. See pd_controllers "Using PD
    controlled actuators".

** Using PD controlled actuators

While PD controllers can be modeled externally and be connected to the
MultibodyPlant model via the get_actuation_input_port(), simulation
stability at discrete-time steps can be compromised for high
controller gains. For such cases, simulation stability and robustness
can be improved significantly by moving your PD controller into the
plant where the discrete solver can strongly couple controller and
model dynamics.

Note:
    PD controllers are ignored when a joint is locked (see
    Joint::Lock()).

Warning:
    Currently, this feature is only supported for discrete models
    (is_discrete() is true) using the SAP solver
    (get_discrete_contact_solver() returns
    DiscreteContactSolver::kSap.)

PD controlled joint actuators can be defined by setting PD gains for
each joint actuator, see JointActuator::set_controller_gains(). Unless
these gains are specified, joint actuators will not be PD controlled
and JointActuator::has_controller() will return ``False``.

For models with PD controllers, the actuation torque per actuator is
computed according to:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    ũ = -Kp⋅(q − qd) - Kd⋅(v − vd) + u_ff
    u = max(−e, min(e, ũ))

.. raw:: html

    </details>

where qd and vd are desired configuration and velocity (see
get_desired_state_input_port()) for the actuated joint (see
JointActuator::joint()), Kp and Kd are the proportional and derivative
gains of the actuator (see JointActuator::get_controller_gains()),
``u_ff`` is the feed-forward actuation specified with
get_actuation_input_port(), and ``e`` corresponds to effort limit (see
JointActuator::effort_limit()).

Notice that actuation through get_actuation_input_port() and PD
control are not mutually exclusive, and they can be used together.
This is better explained through examples: 1. **PD controlled
gripper**. In this case, only PD control is used to drive the opening
and closing of the fingers. The feed-forward term is assumed to be
zero and the actuation input port is not required to be connected. 2.
**Robot arm**. A typical configuration consists on applying gravity
compensation in the feed-forward term plus PD control to drive the
robot to a given desired state.

** Actuation input ports requirements

Actuation input ports and desired state input ports need not be
connected: - Unconnected actuation inputs default to zero, simplifying
diagram wiring for models relying solely on PD controllers. - PD
controllers are disarmed when their model instance's desired state
input port is disconnected. In this state, they have no effect on
dynamics, behaving as if no PD controller exists. This allows a
MultibodyPlant model to be used outside simulation (e.g., for
visualization).

Note that both ports are always created but will be zero-sized for
model instances without actuation.

** Net actuation

The total joint actuation applied via the actuation input port
(get_actuation_input_port()) and applied by the PD controllers is
reported by the net actuation port (get_net_actuation_output_port()).
That is, the net actuation port reports the total actuation applied by
a given actuator.

Note:
    PD controllers are ignored when a joint is locked (see
    Joint::Lock()), and thus they have no effect on the actuation
    output nor reaction forces.

**** Loading models from SDFormat files

Drake has the capability to load multibody models from SDFormat and
URDF files. Consider the example below which loads an acrobot model:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    MultibodyPlant<T> acrobot;
    SceneGraph<T> scene_graph;
    Parser parser(&acrobot, &scene_graph);
    const std::string url =
    "package://drake/multibody/benchmarks/acrobot/acrobot.sdf";
    parser.AddModelsFromUrl(url);

.. raw:: html

    </details>

As in the example above, for models including visual geometry,
collision geometry or both, the user must specify a SceneGraph for
geometry handling. You can find a full example of the LQR controlled
acrobot in examples/multibody/acrobot/run_lqr.cc.

AddModelFromFile() can be invoked multiple times on the same plant in
order to load multiple model instances. Other methods are available on
Parser such as AddModels() which allows creating model instances per
each ``<model>`` tag found in the file. Please refer to each of these
methods' documentation for further details.

**** Working with SceneGraph

** Adding a MultibodyPlant connected to a SceneGraph to your Diagram

Probably the simplest way to add and wire up a MultibodyPlant with a
SceneGraph in your Diagram is using AddMultibodyPlantSceneGraph().

Recommended usages:

Assign to a MultibodyPlant reference (ignoring the SceneGraph):


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    MultibodyPlant<double>& plant =
    AddMultibodyPlantSceneGraph(&builder, 0.0 /+ time_step +/);
    plant.DoFoo(...);

.. raw:: html

    </details>

This flavor is the simplest, when the SceneGraph is not explicitly
needed. (It can always be retrieved later via
GetSubsystemByName("scene_graph").)

Assign to auto, and use the named public fields:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    auto items = AddMultibodyPlantSceneGraph(&builder, 0.0 /+ time_step +/);
    items.plant.DoFoo(...);
    items.scene_graph.DoBar(...);

.. raw:: html

    </details>

or taking advantage of C++'s structured binding:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    auto [plant, scene_graph] = AddMultibodyPlantSceneGraph(&builder, 0.0);
    ...
    plant.DoFoo(...);
    scene_graph.DoBar(...);

.. raw:: html

    </details>

This is the easiest way to use both the plant and scene_graph.

Assign to already-declared pointer variables:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    MultibodyPlant<double>* plant{};
    SceneGraph<double>* scene_graph{};
    std::tie(plant, scene_graph) =
    AddMultibodyPlantSceneGraph(&builder, 0.0 /+ time_step +/);
    plant->DoFoo(...);
    scene_graph->DoBar(...);

.. raw:: html

    </details>

This flavor is most useful when the pointers are class member fields
(and so perhaps cannot be references).

** Registering geometry with a SceneGraph

MultibodyPlant users can register geometry with a SceneGraph for
essentially two purposes; a) visualization and, b) contact modeling.

Before any geometry registration takes place, a user **must** first
make a call to RegisterAsSourceForSceneGraph() in order to register
the MultibodyPlant as a client of a SceneGraph instance, point at
which the plant will have assigned a valid geometry::SourceId. At
Finalize(), MultibodyPlant will declare input/output ports as
appropriate to communicate with the SceneGraph instance on which
registrations took place. All geometry registration **must** be
performed pre-finalize.

Multibodyplant declares an input port for geometric queries, see
get_geometry_query_input_port(). If MultibodyPlant registers geometry
with a SceneGraph via calls to RegisterCollisionGeometry(), users may
use this port for geometric queries. The port must be connected to the
same SceneGraph used for registration. The preferred mechanism is to
use AddMultibodyPlantSceneGraph() as documented above.

In extraordinary circumstances, this can be done by hand and the setup
process will include:

1. Call to RegisterAsSourceForSceneGraph().
2. Calls to RegisterCollisionGeometry(), as many as needed.
3. Call to Finalize(), user is done specifying the model.
4. Connect geometry::SceneGraph::get_query_output_port() to
get_geometry_query_input_port().
5. Connect get_geometry_pose_output_port() to
geometry::SceneGraph::get_source_pose_port()

Refer to the documentation provided in each of the methods above for
further details.

** Accessing point contact parameters MultibodyPlant's point contact
model looks for model parameters stored as
geometry::ProximityProperties by geometry::SceneGraph. These
properties can be obtained before or after context creation through
geometry::SceneGraphInspector APIs as outlined below. MultibodyPlant
expects the following properties for point contact modeling:

|Group name|Property Name|Required|Property Type|Property Description|
|:--------:|:-----------:|:------:|:----------------:|:-------------------|
|material|coulomb_friction|yes¹|CoulombFriction<T>|Static and Dynamic
friction.| |material|point_contact_stiffness|no²|T| Compliant point
contact stiffness.| |material|hunt_crossley_dissipation |no²⁴|T|
Compliant contact dissipation.|
|material|relaxation_time|yes³⁴|T|Linear Kelvin–Voigt model
parameter.|

¹ Collision geometry is required to be registered with a
geometry::ProximityProperties object that contains the ("material",
"coulomb_friction") property. If the property is missing,
MultibodyPlant will throw an exception.

² If the property is missing, MultibodyPlant will use a heuristic
value as the default. Refer to the section point_contact_defaults
"Point Contact Default Parameters" for further details.

³ When using a linear Kelvin–Voigt model of dissipation (for instance
when selecting the SAP solver), collision geometry is required to be
registered with a geometry::ProximityProperties object that contains
the ("material", "relaxation_time") property. If the property is
missing, an exception will be thrown.

⁴ We allow to specify both hunt_crossley_dissipation and
relaxation_time for a given geometry. However only one of these will
get used, depending on the configuration of the MultibodyPlant. As an
example, if the SAP contact approximation is specified (see
set_discrete_contact_approximation()) only the relaxation_time is used
while hunt_crossley_dissipation is ignored. Conversely, if the TAMSI,
Similar or Lagged approximation is used (see
set_discrete_contact_approximation()) only hunt_crossley_dissipation
is used while relaxation_time is ignored. Currently, a continuous
MultibodyPlant model will always use the Hunt & Crossley model and
relaxation_time will be ignored.

Accessing and modifying contact properties requires interfacing with
geometry::SceneGraph's model inspector. Interfacing with a model
inspector obtained from geometry::SceneGraph will provide the default
registered values for a given parameter. These are the values that
will initially appear in a systems::Context created by
CreateDefaultContext(). Subsequently, true system parameters can be
accessed and changed through a systems::Context once available. For
both of the above cases, proximity properties are accessed through
geometry::SceneGraphInspector APIs.

Before context creation an inspector can be retrieved directly from
SceneGraph as:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    // For a SceneGraph<T> instance called scene_graph.
    const geometry::SceneGraphInspector<T>& inspector =
    scene_graph.model_inspector();

.. raw:: html

    </details>

After context creation, an inspector can be retrieved from the state
stored in the context:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    // For a MultibodyPlant<T> instance called mbp and a Context<T> called
    // context.
    const geometry::SceneGraphInspector<T>& inspector =
    mbp.EvalSceneGraphInspector(context);

.. raw:: html

    </details>

Once an inspector is available, proximity properties can be retrieved
as:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    // For a body with GeometryId called geometry_id
    const geometry::ProximityProperties* props =
    inspector.GetProximityProperties(geometry_id);
    const CoulombFriction<T>& geometry_friction =
    props->GetProperty<CoulombFriction<T>>("material",
    "coulomb_friction");

.. raw:: html

    </details>

**** Working with MultibodyElement parameters Several
MultibodyElements expose parameters, allowing the user flexible
modification of some aspects of the plant's model, post
systems::Context creation. For details, refer to the documentation for
the MultibodyElement whose parameters you are trying to modify/access
(e.g. RigidBody, FixedOffsetFrame, etc.)

As an example, here is how to access and modify rigid body mass
parameters:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

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

.. raw:: html

    </details>

Another example, working with automatic differentiation in order to
take derivatives with respect to one of the bodies' masses:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

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

.. raw:: html

    </details>

**** Adding modeling elements

Add multibody elements to a MultibodyPlant with methods like:

- Bodies: AddRigidBody()
- Joints: AddJoint()
- see mbp_construction "Construction" for more.

All modeling elements **must** be added before Finalize() is called.
See mbp_finalize_stage "Finalize stage" for a discussion.

**** Modeling contact

Please refer to drake_contacts "Contact Modeling in Drake" for details
on the available approximations, setup, and considerations for a
multibody simulation with frictional contact.

**** Energy and Power

MultibodyPlant implements the System energy and power methods, with
some limitations. - Kinetic energy: fully implemented. - Potential
energy and conservative power: currently include only gravity and
contributions from ForceElement objects; potential energy from
compliant contact and joint limits are not included. - Nonconservative
power: currently includes only contributions from ForceElement
objects; actuation and input port forces, joint damping, and
dissipation from joint limits, friction, and contact dissipation are
not included.

See Drake issue #12942 for more discussion.

**** Finalize() stage

Once the user is done adding modeling elements and registering
geometry, a call to Finalize() must be performed. This call will: -
Build the underlying tree structure of the multibody model, - declare
the plant's state, - declare the plant's input and output ports, -
declare collision filters to ignore collisions among rigid bodies: -
between rigid bodies connected by a joint, - within subgraphs of
welded rigid bodies.

Note that MultibodyPlant will *not* introduce any automatic collision
filters on deformable bodies. Collision filters for deformable bodies
can be explicitly applied via
ExcludeCollisionGeometriesWithCollisionFilterGroupPair() or during
parsing.

**** References

- [Featherstone 2008] Featherstone, R., 2008.
Rigid body dynamics algorithms. Springer.
- [Jain 2010] Jain, A., 2010.
Robot and multibody dynamics: analysis and algorithms.
Springer Science & Business Media.
- [Seth 2010] Seth, A., Sherman, M., Eastman, P. and Delp, S., 2010.
Minimal formulation of joint motion for biomechanisms.
Nonlinear dynamics, 62(1), pp.291-303.)""";
        // Symbol: drake::multibody::MultibodyPlant::AddBallConstraint
        struct /* AddBallConstraint */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(Defines a constraint such that point P affixed to body A is coincident
at all times with point Q affixed to body B, effectively modeling a
ball-and-socket joint.

Parameter ``body_A``:
    RigidBody to which point P is rigidly attached.

Parameter ``p_AP``:
    Position of point P in body A's frame.

Parameter ``body_B``:
    RigidBody to which point Q is rigidly attached.

Parameter ``p_BQ``:
    (optional) Position of point Q in body B's frame. If p_BQ is
    std::nullopt, then p_BQ will be computed so that the constraint is
    satisfied for the default configuration at Finalize() time;
    subsequent changes to the default configuration will not change
    the computed p_BQ.

Returns:
    the id of the newly added constraint.

Raises:
    RuntimeError if bodies A and B are the same body.

Raises:
    RuntimeError if the MultibodyPlant has already been finalized.

Raises:
    RuntimeError if ``this`` MultibodyPlant's underlying contact
    solver is not SAP. (i.e. get_discrete_contact_solver() !=
    DiscreteContactSolver::kSap))""";
        } AddBallConstraint;
        // Symbol: drake::multibody::MultibodyPlant::AddCouplerConstraint
        struct /* AddCouplerConstraint */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(Defines a holonomic constraint between two single-dof joints
``joint0`` and ``joint1`` with positions q₀ and q₁, respectively, such
that q₀ = ρ⋅q₁ + Δq, where ρ is the gear ratio and Δq is a fixed
offset. The gear ratio can have units if the units of q₀ and q₁ are
different. For instance, between a prismatic and a revolute joint the
gear ratio will specify the "pitch" of the resulting mechanism. As
defined, ``offset`` has units of ``q₀``.

Note:
    joint0 and/or joint1 can still be actuated, regardless of whether
    we have coupler constraint among them. That is, one or both of
    these joints can have external actuation applied to them.

Note:
    Generally, to couple (q0, q1, q2), the user would define a coupler
    between (q0, q1) and a second coupler between (q1, q2), or any
    combination therein.

Raises:
    if joint0 and joint1 are not both single-dof joints.

Raises:
    RuntimeError if the MultibodyPlant has already been finalized.

Raises:
    RuntimeError if ``this`` MultibodyPlant's underlying contact
    solver is not SAP. (i.e. get_discrete_contact_solver() !=
    DiscreteContactSolver::kSap))""";
        } AddCouplerConstraint;
        // Symbol: drake::multibody::MultibodyPlant::AddDistanceConstraint
        struct /* AddDistanceConstraint */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(Defines a distance constraint between a point P on a body A and a
point Q on a body B.

This constraint can be compliant, modeling a spring with free length
``distance`` and given ``stiffness`` and ``damping`` parameters
between points P and Q. For d = ‖p_PQ‖, then a compliant distance
constraint models a spring with force along p_PQ given by:

f = −stiffness ⋅ d − damping ⋅ ḋ

Parameter ``body_A``:
    RigidBody to which point P is rigidly attached.

Parameter ``p_AP``:
    Position of point P in body A's frame.

Parameter ``body_B``:
    RigidBody to which point Q is rigidly attached.

Parameter ``p_BQ``:
    Position of point Q in body B's frame.

Parameter ``distance``:
    Fixed length of the distance constraint, in meters. It must be
    strictly positive.

Parameter ``stiffness``:
    For modeling a spring with free length equal to ``distance``, the
    stiffness parameter in N/m. Optional, with its default value being
    infinite to model a rigid massless rod of length ``distance``
    connecting points A and B.

Parameter ``damping``:
    For modeling a spring with free length equal to ``distance``,
    damping parameter in N⋅s/m. Optional, with its default value being
    zero for a non-dissipative constraint.

Returns:
    the id of the newly added constraint.

Warning:
    Currently, it is the user's responsibility to initialize the
    model's context in a configuration compatible with the newly added
    constraint.

Warning:
    A distance constraint is the wrong modeling choice if the distance
    needs to go through zero. To constrain two points to be coincident
    we need a 3-dof ball constraint, the 1-dof distance constraint is
    singular in this case. Therefore we require the distance parameter
    to be strictly positive.

Note:
    When a new context is created, a DistanceConstraintParams is
    initialized to store the parameters passed to this function.
    Parameters in the context can be modified with calls to
    SetDistanceConstraintParams().

Raises:
    RuntimeError if bodies A and B are the same body.

Raises:
    RuntimeError if ``distance`` is not strictly positive.

Raises:
    RuntimeError if ``stiffness`` is not positive or zero.

Raises:
    RuntimeError if ``damping`` is not positive or zero.

Raises:
    RuntimeError if the MultibodyPlant has already been finalized.

Raises:
    RuntimeError if ``this`` MultibodyPlant's underlying contact
    solver is not SAP. (i.e. get_discrete_contact_solver() !=
    DiscreteContactSolver::kSap))""";
        } AddDistanceConstraint;
        // Symbol: drake::multibody::MultibodyPlant::AddDummyModel
        struct /* AddDummyModel */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc = R"""()""";
        } AddDummyModel;
        // Symbol: drake::multibody::MultibodyPlant::AddForceElement
        struct /* AddForceElement */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(Adds a new force element model of type ``ForceElementType`` to
``this`` MultibodyPlant. The arguments to this method ``args`` are
forwarded to ``ForceElementType`'s constructor.

Parameter ``args``:
    Zero or more parameters provided to the constructor of the new
    force element. It must be the case that
    `ForceElementType<T>(args)`` is a valid constructor.

Template parameter ``ForceElementType``:
    The type of the ForceElement to add. As there is always a
    UniformGravityFieldElement present (accessible through
    gravity_field()), an exception will be thrown if this function is
    called to add another UniformGravityFieldElement.

Returns:
    A constant reference to the new ForceElement just added, of type
    ``ForceElementType<T>`` specialized on the scalar type T of
    ``this`` MultibodyPlant. It will remain valid for the lifetime of
    ``this`` MultibodyPlant.

See also:
    The ForceElement class's documentation for further details on how
    a force element is defined.)""";
        } AddForceElement;
        // Symbol: drake::multibody::MultibodyPlant::AddFrame
        struct /* AddFrame */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(This method adds a Frame of type ``FrameType<T>``. For more
information, please see the corresponding constructor of
``FrameType``.

Template parameter ``FrameType``:
    Template which will be instantiated on ``T``.

Parameter ``frame``:
    Unique pointer frame instance.

Returns:
    A constant reference to the new Frame just added, which will
    remain valid for the lifetime of ``this`` MultibodyPlant.)""";
        } AddFrame;
        // Symbol: drake::multibody::MultibodyPlant::AddJoint
        struct /* AddJoint */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc_1args =
R"""(This method adds a Joint of type ``JointType`` between two bodies. For
more information, see the below overload of ``AddJoint<>``.)""";
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc_6args =
R"""(This method adds a Joint of type ``JointType`` between two bodies. The
two bodies connected by this Joint object are referred to as *parent*
and *child* bodies. The parent/child ordering defines the sign
conventions for the generalized coordinates and the coordinate
ordering for multi-DOF joints.

@image html drake/multibody/plant/images/BodyParentChildJointCM.png
width=50%

Note: The previous figure also shows Pcm which is body P's center of
mass and point Bcm which is body B's center of mass.

As explained in the Joint class's documentation, in Drake we define a
frame F attached to the parent body P with pose ``X_PF`` and a frame M
attached to the child body B with pose ``X_BM``. This method helps
creating a joint between two bodies with fixed poses ``X_PF`` and
``X_BM``. Refer to the Joint class's documentation for more details.

Parameter ``name``:
    A string that uniquely identifies the new joint to be added to
    ``this`` model. A RuntimeError is thrown if a joint named ``name``
    already is part of the model. See HasJointNamed(), Joint::name().

Parameter ``parent``:
    The parent body connected by the new joint.

Parameter ``X_PF``:
    The fixed pose of frame F attached to the parent body, measured in
    the frame P of that body. ``X_PF`` is an optional parameter; empty
    curly braces ``{}`` imply that frame F **is** the same body frame
    P. If instead your intention is to make a frame F with pose
    ``X_PF`` equal to the identity pose, provide
    ``RigidTransform<double>::Identity()`` as your input. When
    non-nullopt, adds a FixedOffsetFrame named ``{name}_parent``.

Parameter ``child``:
    The child body connected by the new joint.

Parameter ``X_BM``:
    The fixed pose of frame M attached to the child body, measured in
    the frame B of that body. ``X_BM`` is an optional parameter; empty
    curly braces ``{}`` imply that frame M **is** the same body frame
    B. If instead your intention is to make a frame M with pose
    ``X_BM`` equal to the identity pose, provide
    ``RigidTransform<double>::Identity()`` as your input. When
    non-nullopt, adds a FixedOffsetFrame named ``{name}_child``.

Parameter ``args``:
    Zero or more parameters provided to the constructor of the new
    joint. It must be the case that ``JointType<T>( const
    std::string&, const Frame<T>&, const Frame<T>&, args)`` is a valid
    constructor.

Template parameter ``JointType``:
    The type of the Joint to add.

Returns:
    A constant reference to the new joint just added, of type
    ``JointType<T>`` specialized on the scalar type T of ``this``
    MultibodyPlant. It will remain valid for the lifetime of ``this``
    MultibodyPlant.

Example of usage:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    MultibodyPlant<T> plant;
      // Code to define bodies serving as the joint's parent and child bodies.
      const RigidBody<double>& body_1 =
        plant.AddRigidBody("Body1", SpatialInertia<double>(...));
      const RigidBody<double>& body_2 =
        plant.AddRigidBody("Body2", SpatialInertia<double>(...));
      // RigidBody 1 serves as parent, RigidBody 2 serves as child.
      // Define the pose X_BM of a frame M rigidly attached to child body B.
      const RevoluteJoint<double>& elbow =
        plant.AddJoint<RevoluteJoint>(
          "Elbow",                /* joint name 
          body_1,                 /* parent body 
          {},                     /* frame F IS the parent body frame P 
          body_2,                 /* child body, the pendulum 
          X_BM,                   /* pose of frame M in the body frame B 
          Vector3d::UnitZ());     /* revolute axis in this case

.. raw:: html

    </details>

Raises:
    RuntimeError if ``this`` MultibodyPlant already contains a joint
    with the given ``name``. See HasJointNamed(), Joint::name().

Raises:
    RuntimeError if parent and child are the same body or if they are
    not both from ``this`` MultibodyPlant.

See also:
    The Joint class's documentation for further details on how a Joint
    is defined.)""";
        } AddJoint;
        // Symbol: drake::multibody::MultibodyPlant::AddJointActuator
        struct /* AddJointActuator */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(Creates and adds a JointActuator model for an actuator acting on a
given ``joint``. This method returns a constant reference to the
actuator just added, which will remain valid for the lifetime of
``this`` plant.

Parameter ``name``:
    A string that uniquely identifies the new actuator to be added to
    ``this`` model. A RuntimeError is thrown if an actuator with the
    same name already exists in the model. See
    HasJointActuatorNamed().

Parameter ``joint``:
    The Joint to be actuated by the new JointActuator.

Parameter ``effort_limit``:
    The maximum effort for the actuator. It must be strictly positive,
    otherwise an RuntimeError is thrown. If +∞, the actuator has no
    limit, which is the default. The effort limit has physical units
    in accordance to the joint type it actuates. For instance, it will
    have units of N⋅m (torque) for revolute joints while it will have
    units of N (force) for prismatic joints.

Note:
    The effort limit is unused by MultibodyPlant and is simply
    provided here for bookkeeping purposes. It will not, for instance,
    saturate external actuation inputs based on this value. If, for
    example, a user intends to saturate the force/torque that is
    applied to the MultibodyPlant via this actuator, the user-level
    code (e.g., a controller) should query this effort limit and
    impose the saturation there.

Returns:
    A constant reference to the new JointActuator just added, which
    will remain valid for the lifetime of ``this`` plant or until the
    JointActuator has been removed from the plant with
    RemoveJointActuator().

Raises:
    RuntimeError if ``joint.num_velocities() > 1`` since for now we
    only support actuators for single dof joints.

See also:
    RemoveJointActuator())""";
        } AddJointActuator;
        // Symbol: drake::multibody::MultibodyPlant::AddModelInstance
        struct /* AddModelInstance */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(Creates a new model instance. Returns the index for the model
instance.

Parameter ``name``:
    A string that uniquely identifies the new instance to be added to
    ``this`` model. An exception is thrown if an instance with the
    same name already exists in the model. See
    HasModelInstanceNamed().)""";
        } AddModelInstance;
        // Symbol: drake::multibody::MultibodyPlant::AddRigidBody
        struct /* AddRigidBody */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc_3args =
R"""(Creates a rigid body with the provided name and spatial inertia. This
method returns a constant reference to the body just added, which will
remain valid for the lifetime of ``this`` MultibodyPlant.

Example of usage:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    MultibodyPlant<T> plant;
      // ... Code to define spatial_inertia, a SpatialInertia<T> object ...
      ModelInstanceIndex model_instance = plant.AddModelInstance("instance");
      const RigidBody<T>& body =
        plant.AddRigidBody("BodyName", model_instance, spatial_inertia);

.. raw:: html

    </details>

Parameter ``name``:
    A string that identifies the new body to be added to ``this``
    model. A RuntimeError is thrown if a body named ``name`` already
    is part of ``model_instance``. See HasBodyNamed(),
    RigidBody::name().

Parameter ``model_instance``:
    A model instance index which this body is part of.

Parameter ``M_BBo_B``:
    The SpatialInertia of the new rigid body to be added to ``this``
    MultibodyPlant, computed about the body frame origin ``Bo`` and
    expressed in the body frame B. When not provided, defaults to
    zero.

Returns:
    A constant reference to the new RigidBody just added, which will
    remain valid for the lifetime of ``this`` MultibodyPlant.)""";
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc_2args =
R"""(Creates a rigid body with the provided name and spatial inertia. This
method returns a constant reference to the body just added, which will
remain valid for the lifetime of ``this`` MultibodyPlant. The body
will use the default model instance (model_instance "more on model
instances").

Example of usage:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    MultibodyPlant<T> plant;
      // ... Code to define spatial_inertia, a SpatialInertia<T> object ...
      const RigidBody<T>& body =
        plant.AddRigidBody("BodyName", spatial_inertia);

.. raw:: html

    </details>

Parameter ``name``:
    A string that identifies the new body to be added to ``this``
    model. A RuntimeError is thrown if a body named ``name`` already
    is part of the model in the default model instance. See
    HasBodyNamed(), RigidBody::name().

Parameter ``M_BBo_B``:
    The SpatialInertia of the new rigid body to be added to ``this``
    MultibodyPlant, computed about the body frame origin ``Bo`` and
    expressed in the body frame B. When not provided, defaults to
    zero.

Returns:
    A constant reference to the new RigidBody just added, which will
    remain valid for the lifetime of ``this`` MultibodyPlant.

Raises:
    RuntimeError if additional model instances have been created
    beyond the world and default instances.)""";
        } AddRigidBody;
        // Symbol: drake::multibody::MultibodyPlant::AddTendonConstraint
        struct /* AddTendonConstraint */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(Defines a set of unilateral constraints on the length of an abstract
tendon defined as:

l(q) = aᵀ⋅q + offset ∈ ℝ

where **q** is the configuration of the model, **a** is a vector of
coefficients, and **offset** a scalar offset. This constraint imposes:

lₗ ≤ l(q) ≤ lᵤ

where **lₗ** and **lᵤ** are lower and upper bounds, respectively. Both
limits are not strictly required. At most one of **lₗ** or **lᵤ** may
be infinite (−∞ for **lₗ** and ∞ for **lᵤ**), indicating no lower or
upper limit, respectively.

For finite ``stiffness`` and ``damping``, this constraint is modeled
by compliant spring-like forces:

fₗ = −stiffness⋅(l - lₗ) − damping⋅dl(q)/dt

fᵤ = −stiffness⋅(lᵤ - l) + damping⋅dl(q)/dt

that act to keep the length within bounds. If the user provided
stiffness is either omitted or set to ∞, this constraint is modeled as
close to rigid as possible by the underlying solver.

Note:
    The coefficients in a are expected to have units such that the
    abstract length l(q) has consistent units (either meters or
    radians) and it is up to the user to maintain consistency in these
    units. The (optionally user provided) ``stiffness`` and
    ``damping`` are expected to have consistent units such that their
    products have units of the corresponding generalized force. E.g.
    N/m for ``stiffness`` and N⋅s/m for ``damping`` when l has units
    of m, so that **fₗ** and **fᵤ** have units of N.

Note:
    Any joint involved in this constraint can still be actuated.

Note:
    See the MuJoCo model documentation for details the equivalent
    concept of a "fixed" tendon:
    https://mujoco.readthedocs.io/en/stable/XMLreference.html#tendon-fixed

Parameter ``joints``:
    Non-empty vector of single-dof joint indices where the
    configuration, qᵢ, of joints[i] corresponds to the entry a[i].

Parameter ``a``:
    Non-empty vector of coefficients where a[i] corresponds to the
    configuration, qᵢ, of joints[i].

Parameter ``offset``:
    (optional) Scalar length offset in either [m] or [rad]. If
    std::nullopt, it is set to 0.

Parameter ``lower_limit``:
    (optional) Lower bound on l in either [m] or [rad]. If
    std::nullopt, it is set to −∞.

Parameter ``upper_limit``:
    Upper bound on l in either [m] or [rad]. If std::nullopt, it is
    set to ∞.

Parameter ``stiffness``:
    (optional) Constraint stiffness in either [N/m] or [N⋅m/rad]. If
    std::nullopt, its default value is set to ∞ to model a rigid
    constraint.

Parameter ``damping``:
    (optional) Constraint damping in either [N⋅s/m] or [N⋅m⋅rad/s]. If
    std::nullopt, it is set to 0 to model a non-dissipative
    constraint.

Warning:
    Because of a restriction in the SAP solver, **at most** two
    kinematic trees can be represented by the joints in ``joints``.
    This violation is only detected after the simulation has been
    started, in which case the solver will throw an exception when
    trying to add the constraint.

Precondition:
    ``joints.size() > 0``

Precondition:
    ``joints`` contains no duplicates.

Precondition:
    ``a.size() == joints.size()``

Precondition:
    ``index ∈ joints`` is a valid (non-removed) index to a joint in
    this plant.

Precondition:
    ``get_joint(index).%num_velocities() == 1`` for each index in
    ``joints``.

Precondition:
    ``lower_limit < ∞`` (if not std::nullopt).

Precondition:
    ``upper_limit > -∞`` (if not std::nullopt).

Precondition:
    At least one of ``lower_limit`` and ``upper_limit`` are finite.

Precondition:
    ``lower_limit ≤ upper_limit`` (if not std::nullopt).

Precondition:
    ``stiffness > 0`` (if not std::nullopt).

Precondition:
    ``damping >= 0`` (if not std::nullopt).

Raises:
    RuntimeError if the MultibodyPlant has already been finalized.

Raises:
    RuntimeError if ``this`` MultibodyPlant's underlying contact
    solver is not SAP. (i.e. get_discrete_contact_solver() !=
    DiscreteContactSolver::kSap).)""";
        } AddTendonConstraint;
        // Symbol: drake::multibody::MultibodyPlant::AddWeldConstraint
        struct /* AddWeldConstraint */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(Defines a constraint such that frame P affixed to body A is coincident
at all times with frame Q affixed to body B, effectively modeling a
weld joint.

Parameter ``body_A``:
    RigidBody to which frame P is rigidly attached.

Parameter ``X_AP``:
    Pose of frame P in body A's frame.

Parameter ``body_B``:
    RigidBody to which frame Q is rigidly attached.

Parameter ``X_BQ``:
    Pose of frame Q in body B's frame.

Returns:
    the id of the newly added constraint.

Raises:
    RuntimeError if bodies A and B are the same body.

Raises:
    RuntimeError if the MultibodyPlant has already been finalized.

Raises:
    RuntimeError if ``this`` MultibodyPlant's underlying contact
    solver is not SAP. (i.e. get_discrete_contact_solver() !=
    DiscreteContactSolver::kSap))""";
        } AddWeldConstraint;
        // Symbol: drake::multibody::MultibodyPlant::CalcBiasCenterOfMassTranslationalAcceleration
        struct /* CalcBiasCenterOfMassTranslationalAcceleration */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc_4args =
R"""(For the system S of all bodies other than the world body, calculates
a𝑠Bias_AScm_E, Scm's translational acceleration bias in frame A with
respect to "speeds" 𝑠, expressed in frame E, where Scm is the center
of mass of S and speeds 𝑠 is either q̇ or v.

Parameter ``context``:
    Contains the state of the multibody system.

Parameter ``with_respect_to``:
    Enum equal to JacobianWrtVariable::kQDot or
    JacobianWrtVariable::kV, indicating whether the accceleration bias
    is with respect to 𝑠 = q̇ or 𝑠 = v. Currently, an exception is
    thrown if with_respect_to is JacobianWrtVariable::kQDot.

Parameter ``frame_A``:
    The frame in which a𝑠Bias_AScm is measured.

Parameter ``frame_E``:
    The frame in which a𝑠Bias_AScm is expressed on output.

Returns:
    a𝑠Bias_AScm_E Point Scm's translational acceleration bias in frame
    A with respect to speeds 𝑠 (𝑠 = q̇ or 𝑠 = v), expressed in frame
    E.

Raises:
    RuntimeError if ``this`` has no body except world_body().

Raises:
    RuntimeError if mₛ ≤ 0, where mₛ is the mass of system S.

Raises:
    RuntimeError if with_respect_to is JacobianWrtVariable::kQDot.

See also:
    CalcJacobianCenterOfMassTranslationalVelocity() to compute
    J𝑠_v_Scm, point Scm's translational velocity Jacobian in frame A
    with respect to 𝑠.

Note:
    The world_body() is ignored. asBias_AScm_E = ∑ (mᵢ aᵢ) / mₛ, where
    mₛ = ∑ mᵢ is the mass of system S, mᵢ is the mass of the iᵗʰ body,
    and aᵢ is the translational bias acceleration of Bᵢcm in frame A
    expressed in frame E for speeds 𝑠 (Bᵢcm is the center of mass of
    the iᵗʰ body).

Note:
    See bias_acceleration_functions "Bias acceleration functions" for
    theory and details.)""";
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc_5args =
R"""(For the system S containing the selected model instances, calculates
a𝑠Bias_AScm_E, Scm's translational acceleration bias in frame A with
respect to "speeds" 𝑠, expressed in frame E, where Scm is the center
of mass of S and speeds 𝑠 is either q̇ or v.

Parameter ``context``:
    Contains the state of the multibody system.

Parameter ``model_instances``:
    Vector of selected model instances. If a model instance is
    repeated in the vector (unusual), it is only counted once.

Parameter ``with_respect_to``:
    Enum equal to JacobianWrtVariable::kQDot or
    JacobianWrtVariable::kV, indicating whether the accceleration bias
    is with respect to 𝑠 = q̇ or 𝑠 = v. Currently, an exception is
    thrown if with_respect_to is JacobianWrtVariable::kQDot.

Parameter ``frame_A``:
    The frame in which a𝑠Bias_AScm is measured.

Parameter ``frame_E``:
    The frame in which a𝑠Bias_AScm is expressed on output.

Returns:
    a𝑠Bias_AScm_E Point Scm's translational acceleration bias in frame
    A with respect to speeds 𝑠 (𝑠 = q̇ or 𝑠 = v), expressed in frame
    E.

Raises:
    RuntimeError if ``this`` has no body except world_body().

Raises:
    RuntimeError if mₛ ≤ 0, where mₛ is the mass of system S.

Raises:
    RuntimeError if with_respect_to is JacobianWrtVariable::kQDot.

See also:
    CalcJacobianCenterOfMassTranslationalVelocity() to compute
    J𝑠_v_Scm, point Scm's translational velocity Jacobian in frame A
    with respect to 𝑠.

Note:
    The world_body() is ignored. asBias_AScm_E = ∑ (mᵢ aᵢ) / mₛ, where
    mₛ = ∑ mᵢ is the mass of system S, mᵢ is the mass of the iᵗʰ body,
    and aᵢ is the translational bias acceleration of Bᵢcm in frame A
    expressed in frame E for speeds 𝑠 (Bᵢcm is the center of mass of
    the iᵗʰ body).

Note:
    See bias_acceleration_functions "Bias acceleration functions" for
    theory and details.)""";
        } CalcBiasCenterOfMassTranslationalAcceleration;
        // Symbol: drake::multibody::MultibodyPlant::CalcBiasSpatialAcceleration
        struct /* CalcBiasSpatialAcceleration */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(For one point Bp affixed/welded to a frame B, calculates A𝑠Bias_ABp,
Bp's spatial acceleration bias in frame A with respect to "speeds" 𝑠,
expressed in frame E, where speeds 𝑠 is either q̇ or v.

Parameter ``context``:
    Contains the state of the multibody system.

Parameter ``with_respect_to``:
    Enum equal to JacobianWrtVariable::kQDot or
    JacobianWrtVariable::kV, indicating whether the spatial
    accceleration bias is with respect to 𝑠 = q̇ or 𝑠 = v. Currently,
    an exception is thrown if with_respect_to is
    JacobianWrtVariable::kQDot.

Parameter ``frame_B``:
    The frame on which point Bp is affixed/welded.

Parameter ``p_BoBp_B``:
    Position vector from Bo (frame_B's origin) to point Bp (regarded
    as affixed/welded to B), expressed in frame_B.

Parameter ``frame_A``:
    The frame in which A𝑠Bias_ABp is measured.

Parameter ``frame_E``:
    The frame in which A𝑠Bias_ABp is expressed on output.

Returns:
    A𝑠Bias_ABp_E Point Bp's spatial acceleration bias in frame A with
    respect to speeds 𝑠 (𝑠 = q̇ or 𝑠 = v), expressed in frame E.

See also:
    CalcJacobianSpatialVelocity() to compute J𝑠_V_ABp, point Bp's
    spatial velocity Jacobian in frame A with respect to 𝑠.

Raises:
    RuntimeError if with_respect_to is JacobianWrtVariable::kQDot.

Note:
    Use CalcBiasTranslationalAcceleration() to efficiently calculate
    bias translational accelerations for a list of points (each fixed
    to frame B). This function returns only one bias spatial
    acceleration, which contains both frame B's bias angular
    acceleration and point Bp's bias translational acceleration.

Note:
    See bias_acceleration_functions "Bias acceleration functions" for
    theory and details.)""";
        } CalcBiasSpatialAcceleration;
        // Symbol: drake::multibody::MultibodyPlant::CalcBiasTerm
        struct /* CalcBiasTerm */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(Computes the bias term ``C(q, v) v`` containing Coriolis, centripetal,
and gyroscopic effects in the multibody equations of motion:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    M(q) v̇ + C(q, v) v = tau_app + ∑ (Jv_V_WBᵀ(q) ⋅ Fapp_Bo_W)

.. raw:: html

    </details>

where ``M(q)`` is the multibody model's mass matrix (including rigid
body mass properties and reflected_inertia "reflected inertias") and
``tau_app`` is a vector of applied generalized forces. The last term
is a summation over all bodies of the dot-product of ``Fapp_Bo_W``
(applied spatial force on body B at Bo) with ``Jv_V_WB(q)`` (B's
spatial Jacobian in world W with respect to generalized velocities v).
Note: B's spatial velocity in W can be written ``V_WB = Jv_V_WB * v``.

Parameter ``context``:
    Contains the state of the multibody system, including the
    generalized positions q and the generalized velocities v.

Parameter ``Cv``:
    On output, ``Cv`` will contain the product ``C(q, v)v``. It must
    be a valid (non-null) pointer to a column vector in ``ℛⁿ`` with n
    the number of generalized velocities (num_velocities()) of the
    model. This method aborts if Cv is nullptr or if it does not have
    the proper size.)""";
        } CalcBiasTerm;
        // Symbol: drake::multibody::MultibodyPlant::CalcBiasTranslationalAcceleration
        struct /* CalcBiasTranslationalAcceleration */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(For each point Bi affixed/welded to a frame B, calculates a𝑠Bias_ABi,
Bi's translational acceleration bias in frame A with respect to
"speeds" 𝑠, expressed in frame E, where speeds 𝑠 is either q̇ or v.

Parameter ``context``:
    Contains the state of the multibody system.

Parameter ``with_respect_to``:
    Enum equal to JacobianWrtVariable::kQDot or
    JacobianWrtVariable::kV, indicating whether the translational
    acceleration bias is with respect to 𝑠 = q̇ or 𝑠 = v. Currently,
    an exception is thrown if with_respect_to is
    JacobianWrtVariable::kQDot.

Parameter ``frame_B``:
    The frame on which points Bi are affixed/welded.

Parameter ``p_BoBi_B``:
    A position vector or list of p position vectors from Bo (frame_B's
    origin) to points Bi (regarded as affixed to B), where each
    position vector is expressed in frame_B. Each column in the ``3 x
    p`` matrix p_BoBi_B corresponds to a position vector.

Parameter ``frame_A``:
    The frame in which a𝑠Bias_ABi is measured.

Parameter ``frame_E``:
    The frame in which a𝑠Bias_ABi is expressed on output.

Returns:
    a𝑠Bias_ABi_E Point Bi's translational acceleration bias in frame A
    with respect to speeds 𝑠 (𝑠 = q̇ or 𝑠 = v), expressed in frame E.
    a𝑠Bias_ABi_E is a ``3 x p`` matrix, where p is the number of
    points Bi.

See also:
    CalcJacobianTranslationalVelocity() to compute J𝑠_v_ABi, point
    Bi's translational velocity Jacobian in frame A with respect to 𝑠.

Precondition:
    p_BoBi_B must have 3 rows.

Raises:
    RuntimeError if with_respect_to is JacobianWrtVariable::kQDot.

Note:
    See bias_acceleration_functions "Bias acceleration functions" for
    theory and details.)""";
        } CalcBiasTranslationalAcceleration;
        // Symbol: drake::multibody::MultibodyPlant::CalcCenterOfMassPositionInWorld
        struct /* CalcCenterOfMassPositionInWorld */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc_1args =
R"""(Calculates the position vector from the world origin Wo to the center
of mass of all bodies in this MultibodyPlant, expressed in the world
frame W.

Parameter ``context``:
    Contains the state of the model.

Returns ``p_WoScm_W``:
    position vector from Wo to Scm expressed in world frame W, where
    Scm is the center of mass of the system S stored by ``this``
    plant.

Raises:
    RuntimeError if ``this`` has no body except world_body().

Raises:
    RuntimeError if mₛ ≤ 0 (where mₛ is the mass of system S).

Note:
    The world_body() is ignored. p_WoScm_W = ∑ (mᵢ pᵢ) / mₛ, where mₛ
    = ∑ mᵢ, mᵢ is the mass of the iᵗʰ body, and pᵢ is Bᵢcm's position
    from Wo expressed in frame W (Bᵢcm is the center of mass of the
    iᵗʰ body).)""";
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc_2args =
R"""(Calculates the position vector from the world origin Wo to the center
of mass of all non-world bodies contained in model_instances,
expressed in the world frame W.

Parameter ``context``:
    Contains the state of the model.

Parameter ``model_instances``:
    Vector of selected model instances. If a model instance is
    repeated in the vector (unusual), it is only counted once.

Returns ``p_WoScm_W``:
    position vector from world origin Wo to Scm expressed in the world
    frame W, where Scm is the center of mass of the system S of
    non-world bodies contained in model_instances.

Raises:
    RuntimeError if model_instances is empty or only has world body.

Raises:
    RuntimeError if mₛ ≤ 0 (where mₛ is the mass of system S).

Note:
    The world_body() is ignored. p_WoScm_W = ∑ (mᵢ pᵢ) / mₛ, where mₛ
    = ∑ mᵢ, mᵢ is the mass of the iᵗʰ body contained in
    model_instances, and pᵢ is Bᵢcm's position vector from Wo
    expressed in frame W (Bᵢcm is the center of mass of the iᵗʰ body).)""";
        } CalcCenterOfMassPositionInWorld;
        // Symbol: drake::multibody::MultibodyPlant::CalcCenterOfMassTranslationalAccelerationInWorld
        struct /* CalcCenterOfMassTranslationalAccelerationInWorld */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc_1args =
R"""(For the system S contained in this MultibodyPlant, calculates Scm's
translational acceleration in the world frame W expressed in W, where
Scm is the center of mass of S.

Parameter ``context``:
    The context contains the state of the model.

Returns ``a_WScm_W``:
    Scm's translational acceleration in the world frame W expressed in
    the world frame W.

Raises:
    RuntimeError if ``this`` has no body except world_body().

Raises:
    RuntimeError if mₛ ≤ 0, where mₛ is the mass of system S.

Note:
    The world_body() is ignored. a_WScm_W = ∑ (mᵢ aᵢ) / mₛ, where mₛ =
    ∑ mᵢ is the mass of system S, mᵢ is the mass of the iᵗʰ body, and
    aᵢ is the translational acceleration of Bᵢcm in world W expressed
    in W (Bᵢcm is the center of mass of the iᵗʰ body).

Note:
    When cached values are out of sync with the state stored in
    context, this method performs an expensive forward dynamics
    computation, whereas once evaluated, successive calls to this
    method are inexpensive.)""";
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc_2args =
R"""(For the system S containing the selected model instances, calculates
Scm's translational acceleration in the world frame W expressed in W,
where Scm is the center of mass of S.

Parameter ``context``:
    The context contains the state of the model.

Parameter ``model_instances``:
    Vector of selected model instances. If a model instance is
    repeated in the vector (unusual), it is only counted once.

Returns ``a_WScm_W``:
    Scm's translational acceleration in the world frame W expressed in
    the world frame W.

Raises:
    RuntimeError if model_instances is empty or only has world body.

Raises:
    RuntimeError if mₛ ≤ 0, where mₛ is the mass of system S.

Note:
    The world_body() is ignored. a_WScm_W = ∑ (mᵢ aᵢ) / mₛ, where mₛ =
    ∑ mᵢ is the mass of system S, mᵢ is the mass of the iᵗʰ body in
    model_instances, and aᵢ is the translational acceleration of Bᵢcm
    in world W expressed in W (Bᵢcm is the center of mass of the iᵗʰ
    body).

Note:
    When cached values are out of sync with the state stored in
    context, this method performs an expensive forward dynamics
    computation, whereas once evaluated, successive calls to this
    method are inexpensive.)""";
        } CalcCenterOfMassTranslationalAccelerationInWorld;
        // Symbol: drake::multibody::MultibodyPlant::CalcCenterOfMassTranslationalVelocityInWorld
        struct /* CalcCenterOfMassTranslationalVelocityInWorld */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc_1args =
R"""(Calculates system center of mass translational velocity in world frame
W.

Parameter ``context``:
    The context contains the state of the model.

Returns ``v_WScm_W``:
    Scm's translational velocity in frame W, expressed in W, where Scm
    is the center of mass of the system S stored by ``this`` plant.

Raises:
    RuntimeError if ``this`` has no body except world_body().

Raises:
    RuntimeError if mₛ ≤ 0 (where mₛ is the mass of system S).

Note:
    The world_body() is ignored. v_WScm_W = ∑ (mᵢ vᵢ) / mₛ, where mₛ =
    ∑ mᵢ, mᵢ is the mass of the iᵗʰ body, and vᵢ is Bᵢcm's velocity in
    world W (Bᵢcm is the center of mass of the iᵗʰ body).)""";
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc_2args =
R"""(Calculates system center of mass translational velocity in world frame
W.

Parameter ``context``:
    The context contains the state of the model.

Parameter ``model_instances``:
    Vector of selected model instances. If a model instance is
    repeated in the vector (unusual), it is only counted once.

Returns ``v_WScm_W``:
    Scm's translational velocity in frame W, expressed in W, where Scm
    is the center of mass of the system S of non-world bodies
    contained in model_instances.

Raises:
    RuntimeError if model_instances is empty or only has world body.

Raises:
    RuntimeError if mₛ ≤ 0 (where mₛ is the mass of system S).

Note:
    The world_body() is ignored. v_WScm_W = ∑ (mᵢ vᵢ) / mₛ, where mₛ =
    ∑ mᵢ, mᵢ is the mass of the iᵗʰ body contained in model_instances,
    and vᵢ is Bᵢcm's velocity in world W expressed in frame W (Bᵢcm is
    the center of mass of the iᵗʰ body).)""";
        } CalcCenterOfMassTranslationalVelocityInWorld;
        // Symbol: drake::multibody::MultibodyPlant::CalcForceElementsContribution
        struct /* CalcForceElementsContribution */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(Computes the combined force contribution of ForceElement objects in
the model. A ForceElement can apply forces as a spatial force per body
or as generalized forces, depending on the ForceElement model.
ForceElement contributions are a function of the state and time only.
The output from this method can immediately be used as input to
CalcInverseDynamics() to include the effect of applied forces by force
elements.

Parameter ``context``:
    The context containing the state of this model.

Parameter ``forces``:
    A pointer to a valid, non nullptr, multibody forces object. On
    output ``forces`` will store the forces exerted by all the
    ForceElement objects in the model.

Raises:
    RuntimeError if ``forces`` is null or not compatible with this
    model, per MultibodyForces::CheckInvariants().)""";
        } CalcForceElementsContribution;
        // Symbol: drake::multibody::MultibodyPlant::CalcFullSystemJacobian
        struct /* CalcFullSystemJacobian */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""((Internal use only) Evaluates the block system Jacobian, then uses it
to fill in an equivalent full matrix of size 6n x m where n is the
number of mobilized bodies and m the number of generalized velocities
(mobilities). Each mobilized body generates a 6 x m strip of the
Jacobian (6 rows) and those are ordered by MobodIndex. Note that World
is the 0th mobilized body so to keep the numbering consistent the
first 6 rows of the Jacobian correspond to the World Mobod (and are
thus all zero).

This is most useful for testing; it is more efficient to use
EvalBlockSystemJacobian() and to work with the individual blocks.

See also:
    EvalBlockSystemJacobian(), CalcJacobianSpatialVelocity())""";
        } CalcFullSystemJacobian;
        // Symbol: drake::multibody::MultibodyPlant::CalcGeneralizedForces
        struct /* CalcGeneralizedForces */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(Computes the generalized forces result of a set of MultibodyForces
applied to this model.

MultibodyForces stores applied forces as both generalized forces τ and
spatial forces F on each body, refer to documentation in
MultibodyForces for details. Users of MultibodyForces will use
MultibodyForces::mutable_generalized_forces() to mutate the stored
generalized forces directly and will use
RigidBody::AddInForceInWorld() to append spatial forces.

For a given set of forces stored as MultibodyForces, this method will
compute the total generalized forces on this model. More precisely, if
J_WBo is the Jacobian (with respect to velocities) for this model,
including all bodies, then this method computes:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    τᵣₑₛᵤₗₜ = τ + J_WBo⋅F

.. raw:: html

    </details>

Parameter ``context``:
    Context that stores the state of the model.

Parameter ``forces``:
    Set of multibody forces, including both generalized forces and
    per-body spatial forces.

Parameter ``generalized_forces``:
    The total generalized forces on the model that would result from
    applying ``forces``. In other words, ``forces`` can be replaced by
    the equivalent ``generalized_forces``. On output,
    ``generalized_forces`` is resized to num_velocities().

Raises:
    RuntimeError if ``forces`` is null or not compatible with this
    model.

Raises:
    RuntimeError if ``generalized_forces`` is not a valid non-null
    pointer.)""";
        } CalcGeneralizedForces;
        // Symbol: drake::multibody::MultibodyPlant::CalcGravityGeneralizedForces
        struct /* CalcGravityGeneralizedForces */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(Computes the generalized forces ``tau_g(q)`` due to gravity as a
function of the generalized positions ``q`` stored in the input
``context``. The vector of generalized forces due to gravity
``tau_g(q)`` is defined such that it appears on the right hand side of
the equations of motion together with any other generalized forces,
like so:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    Mv̇ + C(q, v)v = tau_g(q) + tau_app

.. raw:: html

    </details>

where ``tau_app`` includes any other generalized forces applied on the
system.

Parameter ``context``:
    The context storing the state of the model.

Returns:
    tau_g A vector containing the generalized forces due to gravity.
    The generalized forces are consistent with the vector of
    generalized velocities ``v`` for ``this`` so that the inner
    product ``v⋅tau_g`` corresponds to the power applied by the
    gravity forces on the mechanical system. That is, ``v⋅tau_g > 0``
    corresponds to potential energy going into the system, as either
    mechanical kinetic energy, some other potential energy, or heat,
    and therefore to a decrease of the gravitational potential energy.)""";
        } CalcGravityGeneralizedForces;
        // Symbol: drake::multibody::MultibodyPlant::CalcInverseDynamics
        struct /* CalcInverseDynamics */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(Given the state of this model in ``context`` and a known vector of
generalized accelerations ``vdot``, this method computes the set of
generalized forces ``tau`` that would need to be applied in order to
attain the specified generalized accelerations. Mathematically, this
method computes:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    tau = M(q)v̇ + C(q, v)v - tau_app - ∑ J_WBᵀ(q) Fapp_Bo_W

.. raw:: html

    </details>

where ``M(q)`` is the model's mass matrix (including rigid body mass
properties and reflected_inertia "reflected inertias"), ``C(q, v)v``
is the bias term for Coriolis and gyroscopic effects and ``tau_app``
consists of a vector applied generalized forces. The last term is a
summation over all bodies in the model where ``Fapp_Bo_W`` is an
applied spatial force on body B at ``Bo`` which gets projected into
the space of generalized forces with the transpose of ``Jv_V_WB(q)``
(where ``Jv_V_WB`` is B's spatial velocity Jacobian in W with respect
to generalized velocities v). Note: B's spatial velocity in W can be
written as ``V_WB = Jv_V_WB * v``.

This method does not compute explicit expressions for the mass matrix
nor for the bias term, which would be of at least ``O(n²)``
complexity, but it implements an ``O(n)`` Newton-Euler recursive
algorithm, where n is the number of bodies in the model. The explicit
formation of the mass matrix ``M(q)`` would require the calculation of
``O(n²)`` entries while explicitly forming the product ``C(q, v) * v``
could require up to ``O(n³)`` operations (see [Featherstone 1987,
§4]), depending on the implementation. The recursive Newton-Euler
algorithm is the most efficient currently known general method for
solving inverse dynamics [Featherstone 2008].

Parameter ``context``:
    The context containing the state of the model.

Parameter ``known_vdot``:
    A vector with the known generalized accelerations ``vdot`` for the
    full model. Use the provided Joint APIs in order to access entries
    into this array.

Parameter ``external_forces``:
    A set of forces to be applied to the system either as body spatial
    forces ``Fapp_Bo_W`` or generalized forces ``tau_app``, see
    MultibodyForces for details.

Returns:
    the vector of generalized forces that would need to be applied to
    the mechanical system in order to achieve the desired acceleration
    given by ``known_vdot``.)""";
        } CalcInverseDynamics;
        // Symbol: drake::multibody::MultibodyPlant::CalcJacobianAngularVelocity
        struct /* CalcJacobianAngularVelocity */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(Calculates J𝑠_w_AB, a frame B's angular velocity Jacobian in a frame A
with respect to "speeds" 𝑠.


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    J𝑠_w_AB ≜ [ ∂(w_AB)/∂𝑠₁,  ...  ∂(w_AB)/∂𝑠ₙ ]    (n is j or k)
         w_AB = J𝑠_w_AB ⋅ 𝑠          w_AB is linear in 𝑠 ≜ [𝑠₁ ... 𝑠ₙ]ᵀ

.. raw:: html

    </details>

``w_AB`` is B's angular velocity in frame A and "speeds" 𝑠 is either
q̇ ≜ [q̇₁ ... q̇ⱼ]ᵀ (time-derivatives of j generalized positions) or v
≜ [v₁ ... vₖ]ᵀ (k generalized velocities).

Parameter ``context``:
    The state of the multibody system.

Parameter ``with_respect_to``:
    Enum equal to JacobianWrtVariable::kQDot or
    JacobianWrtVariable::kV, indicating whether the Jacobian
    ``J𝑠_w_AB`` is partial derivatives with respect to 𝑠 = q̇
    (time-derivatives of generalized positions) or with respect to 𝑠 =
    v (generalized velocities).

Parameter ``frame_B``:
    The frame B in ``w_AB`` (B's angular velocity in A).

Parameter ``frame_A``:
    The frame A in ``w_AB`` (B's angular velocity in A).

Parameter ``frame_E``:
    The frame in which ``w_AB`` is expressed on input and the frame in
    which the Jacobian ``J𝑠_w_AB`` is expressed on output.

Parameter ``Js_w_AB_E``:
    Frame B's angular velocity Jacobian in frame A with respect to
    speeds 𝑠 (which is either q̇ or v), expressed in frame E. The
    Jacobian is a function of only generalized positions q (which are
    pulled from the context). The previous definition shows
    ``J𝑠_w_AB_E`` is a matrix of size ``3 x n``, where n is the number
    of elements in 𝑠.

See also:
    See Jacobian_functions "Jacobian functions" for related functions.

Raises:
    RuntimeError if ``J𝑠_w_AB_E`` is nullptr or not of size ``3 x n``.)""";
        } CalcJacobianAngularVelocity;
        // Symbol: drake::multibody::MultibodyPlant::CalcJacobianCenterOfMassTranslationalVelocity
        struct /* CalcJacobianCenterOfMassTranslationalVelocity */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc_5args =
R"""(Calculates J𝑠_v_AScm_E, point Scm's translational velocity Jacobian in
frame A with respect to "speeds" 𝑠, expressed in frame E, where point
Scm is the center of mass of the system S of all non-world bodies
contained in ``this`` MultibodyPlant.

Parameter ``context``:
    contains the state of the model.

Parameter ``with_respect_to``:
    Enum equal to JacobianWrtVariable::kQDot or
    JacobianWrtVariable::kV, indicating whether the Jacobian
    ``J𝑠_v_AScm_E`` is partial derivatives with respect to 𝑠 = q̇
    (time-derivatives of generalized positions) or with respect to 𝑠 =
    v (generalized velocities).

Parameter ``frame_A``:
    The frame in which the translational velocity v_AScm and its
    Jacobian J𝑠_v_AScm are measured.

Parameter ``frame_E``:
    The frame in which the Jacobian J𝑠_v_AScm is expressed on output.

Parameter ``Js_v_AScm_E``:
    Point Scm's translational velocity Jacobian in frame A with
    respect to speeds 𝑠 (𝑠 = q̇ or 𝑠 = v), expressed in frame E.
    J𝑠_v_AScm_E is a 3 x n matrix, where n is the number of elements
    in 𝑠. The Jacobian is a function of only generalized positions q
    (which are pulled from the context).

Raises:
    RuntimeError if Scm does not exist, which occurs if there are no
    massive bodies in MultibodyPlant (except world_body()).

Raises:
    RuntimeError if mₛ ≤ 0 (where mₛ is the mass of all non-world
    bodies contained in ``this`` MultibodyPlant).

See also:
    See Jacobian_functions "Jacobian functions" for related functions.)""";
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc_6args =
R"""(Calculates J𝑠_v_AScm_E, point Scm's translational velocity Jacobian in
frame A with respect to "speeds" 𝑠, expressed in frame E, where point
Scm is the center of mass of the system S of all non-world bodies
contained in model_instances.

Parameter ``context``:
    contains the state of the model.

Parameter ``model_instances``:
    Vector of selected model instances. If a model instance is
    repeated in the vector (unusual), it is only counted once.

Parameter ``with_respect_to``:
    Enum equal to JacobianWrtVariable::kQDot or
    JacobianWrtVariable::kV, indicating whether the Jacobian
    ``J𝑠_v_AScm_E`` is partial derivatives with respect to 𝑠 = q̇
    (time-derivatives of generalized positions) or with respect to 𝑠 =
    v (generalized velocities).

Parameter ``frame_A``:
    The frame in which the translational velocity v_AScm and its
    Jacobian J𝑠_v_AScm are measured.

Parameter ``frame_E``:
    The frame in which the Jacobian J𝑠_v_AScm is expressed on output.

Parameter ``Js_v_AScm_E``:
    Point Scm's translational velocity Jacobian in frame A with
    respect to speeds 𝑠 (𝑠 = q̇ or 𝑠 = v), expressed in frame E.
    J𝑠_v_AScm_E is a 3 x n matrix, where n is the number of elements
    in 𝑠. The Jacobian is a function of only generalized positions q
    (which are pulled from the context).

Raises:
    RuntimeError if mₛ ≤ 0 (where mₛ is the mass of all non-world
    bodies contained in model_instances).

Raises:
    RuntimeError if model_instances is empty or only has world body.

Note:
    The world_body() is ignored. J𝑠_v_AScm_ = ∑ (mᵢ Jᵢ) / mₛ, where mₛ
    = ∑ mᵢ, mᵢ is the mass of the iᵗʰ body contained in
    model_instances, and Jᵢ is Bᵢcm's translational velocity Jacobian
    in frame A, expressed in frame E (Bᵢcm is the center of mass of
    the iᵗʰ body).

See also:
    See Jacobian_functions "Jacobian functions" for related functions.)""";
        } CalcJacobianCenterOfMassTranslationalVelocity;
        // Symbol: drake::multibody::MultibodyPlant::CalcJacobianPositionVector
        struct /* CalcJacobianPositionVector */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(For each point Bi affixed/welded to a frame B, calculates Jq_p_AoBi,
Bi's position vector Jacobian in frame A with respect to the
generalized positions q ≜ [q₁ ... qₙ]ᵀ as


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    Jq_p_AoBi ≜ [ ᴬ∂(p_AoBi)/∂q₁,  ...  ᴬ∂(p_AoBi)/∂qₙ ]

.. raw:: html

    </details>

where p_AoBi is Bi's position vector from point Ao (frame A's origin)
and ᴬ∂(p_AoBi)/∂qᵣ denotes the partial derivative in frame A of p_AoBi
with respect to the generalized position qᵣ, where qᵣ is one of q₁ ...
qₙ.

Parameter ``context``:
    The state of the multibody system.

Parameter ``frame_B``:
    The frame on which point Bi is affixed/welded.

Parameter ``p_BoBi_B``:
    A position vector or list of k position vectors from Bo (frame_B's
    origin) to points Bi (Bi is regarded as affixed to B), where each
    position vector is expressed in frame_B.

Parameter ``frame_A``:
    The frame in which partial derivatives are calculated and the
    frame in which point Ao is affixed.

Parameter ``frame_E``:
    The frame in which the Jacobian Jq_p_AoBi is expressed on output.

Parameter ``Jq_p_AoBi_E``:
    Point Bi's position vector Jacobian in frame A with generalized
    positions q, expressed in frame E. Jq_p_AoBi_E is a ``3*k x n``
    matrix, where k is the number of points Bi and n is the number of
    elements in q. The Jacobian is a function of only generalized
    positions q (which are pulled from the context).

Raises:
    RuntimeError if Jq_p_AoBi_E is nullptr or not sized ``3*k x n``.

Note:
    Jq̇_v_ABi = Jq_p_AoBi. In other words, point Bi's velocity
    Jacobian in frame A with respect to q̇ is equal to point Bi's
    position vector Jacobian in frame A with respect to q.


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    [∂(v_ABi)/∂q̇₁, ... ∂(v_ABi)/∂q̇ₙ] = [ᴬ∂(p_AoBi)/∂q₁, ... ᴬ∂(p_AoBi)/∂qₙ]

.. raw:: html

    </details>

See also:
    CalcJacobianTranslationalVelocity() for details on Jq̇_v_ABi.
    Note: Jq_p_AaBi = Jq_p_AoBi, where point Aa is *any* point
    fixed/welded to frame A, i.e., this calculation's result is the
    same if point Ao is replaced with any point fixed on frame A.

See also:
    See Jacobian_functions "Jacobian functions" for related functions.)""";
        } CalcJacobianPositionVector;
        // Symbol: drake::multibody::MultibodyPlant::CalcJacobianSpatialVelocity
        struct /* CalcJacobianSpatialVelocity */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(For one point Bp fixed/welded to a frame B, calculates J𝑠_V_ABp, Bp's
spatial velocity Jacobian in frame A with respect to "speeds" 𝑠.


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    J𝑠_V_ABp ≜ [ ∂(V_ABp)/∂𝑠₁,  ...  ∂(V_ABp)/∂𝑠ₙ ]    (n is j or k)
         V_ABp = J𝑠_V_ABp ⋅ 𝑠          V_ABp is linear in 𝑠 ≜ [𝑠₁ ... 𝑠ₙ]ᵀ

.. raw:: html

    </details>

``V_ABp`` is Bp's spatial velocity in frame A and "speeds" 𝑠 is either
q̇ ≜ [q̇₁ ... q̇ⱼ]ᵀ (time-derivatives of j generalized positions) or v
≜ [v₁ ... vₖ]ᵀ (k generalized velocities).

Parameter ``context``:
    The state of the multibody system.

Parameter ``with_respect_to``:
    Enum equal to JacobianWrtVariable::kQDot or
    JacobianWrtVariable::kV, indicating whether the Jacobian
    ``J𝑠_V_ABp`` is partial derivatives with respect to 𝑠 = q̇
    (time-derivatives of generalized positions) or with respect to 𝑠 =
    v (generalized velocities).

Parameter ``frame_B``:
    The frame on which point Bp is fixed/welded.

Parameter ``p_BoBp_B``:
    A position vector from Bo (frame_B's origin) to point Bp (regarded
    as fixed/welded to B), expressed in frame_B.

Parameter ``frame_A``:
    The frame that measures ``v_ABp`` (Bp's velocity in A). Note: It
    is natural to wonder why there is no parameter p_AoAp_A (similar
    to the parameter p_BoBp_B for frame_B). There is no need for
    p_AoAp_A because Bp's velocity in A is defined as the derivative
    in frame A of Bp's position vector from *any* point fixed to A.

Parameter ``frame_E``:
    The frame in which ``v_ABp`` is expressed on input and the frame
    in which the Jacobian ``J𝑠_V_ABp`` is expressed on output.

Parameter ``Js_V_ABp_E``:
    Point Bp's spatial velocity Jacobian in frame A with respect to
    speeds 𝑠 (which is either q̇ or v), expressed in frame E.
    ``J𝑠_V_ABp_E`` is a ``6 x n`` matrix, where n is the number of
    elements in 𝑠. The Jacobian is a function of only generalized
    positions q (which are pulled from the context).

Note:
    The returned ``6 x n`` matrix stores frame B's angular velocity
    Jacobian in A in rows 1-3 and stores point Bp's translational
    velocity Jacobian in A in rows 4-6, i.e.,


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    J𝑠_w_AB_E = J𝑠_V_ABp_E.topRows<3>();
        J𝑠_v_ABp_E = J𝑠_V_ABp_E.bottomRows<3>();

.. raw:: html

    </details>

Note:
    Consider CalcJacobianTranslationalVelocity() for multiple points
    fixed to frame B and consider CalcJacobianAngularVelocity() to
    calculate frame B's angular velocity Jacobian.

See also:
    See Jacobian_functions "Jacobian functions" for related functions.

Raises:
    RuntimeError if ``J𝑠_V_ABp_E`` is nullptr or not sized ``6 x n``.)""";
        } CalcJacobianSpatialVelocity;
        // Symbol: drake::multibody::MultibodyPlant::CalcJacobianTranslationalVelocity
        struct /* CalcJacobianTranslationalVelocity */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(For each point Bi affixed/welded to a frame B, calculates J𝑠_v_ABi,
Bi's translational velocity Jacobian in frame A with respect to
"speeds" 𝑠.


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    J𝑠_v_ABi ≜ [ ∂(v_ABi)/∂𝑠₁,  ...  ∂(v_ABi)/∂𝑠ₙ ]    (n is j or k)
         v_ABi = J𝑠_v_ABi ⋅ 𝑠          v_ABi is linear in 𝑠 ≜ [𝑠₁ ... 𝑠ₙ]ᵀ

.. raw:: html

    </details>

``v_ABi`` is Bi's translational velocity in frame A and "speeds" 𝑠 is
either q̇ ≜ [q̇₁ ... q̇ⱼ]ᵀ (time-derivatives of j generalized
positions) or v ≜ [v₁ ... vₖ]ᵀ (k generalized velocities).

Parameter ``context``:
    The state of the multibody system.

Parameter ``with_respect_to``:
    Enum equal to JacobianWrtVariable::kQDot or
    JacobianWrtVariable::kV, indicating whether the Jacobian
    ``J𝑠_v_ABi`` is partial derivatives with respect to 𝑠 = q̇
    (time-derivatives of generalized positions) or with respect to 𝑠 =
    v (generalized velocities).

Parameter ``frame_B``:
    The frame on which point Bi is affixed/welded.

Parameter ``p_BoBi_B``:
    A position vector or list of p position vectors from Bo (frame_B's
    origin) to points Bi (regarded as affixed to B), where each
    position vector is expressed in frame_B.

Parameter ``frame_A``:
    The frame that measures ``v_ABi`` (Bi's velocity in A). Note: It
    is natural to wonder why there is no parameter p_AoAi_A (similar
    to the parameter p_BoBi_B for frame_B). There is no need for
    p_AoAi_A because Bi's velocity in A is defined as the derivative
    in frame A of Bi's position vector from *any* point affixed to A.

Parameter ``frame_E``:
    The frame in which ``v_ABi`` is expressed on input and the frame
    in which the Jacobian ``J𝑠_v_ABi`` is expressed on output.

Parameter ``Js_v_ABi_E``:
    Point Bi's velocity Jacobian in frame A with respect to speeds 𝑠
    (which is either q̇ or v), expressed in frame E. ``J𝑠_v_ABi_E`` is
    a ``3*p x n`` matrix, where p is the number of points Bi and n is
    the number of elements in 𝑠. The Jacobian is a function of only
    generalized positions q (which are pulled from the context).

Raises:
    RuntimeError if ``J𝑠_v_ABi_E`` is nullptr or not sized ``3*p x
    n``.

Note:
    When 𝑠 = q̇, ``Jq̇_v_ABi = Jq_p_AoBi``. In other words, point Bi's
    velocity Jacobian in frame A with respect to q̇ is equal to point
    Bi's position Jacobian from Ao (A's origin) in frame A with
    respect to q.


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    [∂(v_ABi)/∂q̇₁,  ...  ∂(v_ABi)/∂q̇ⱼ] = [∂(p_AoBi)/∂q₁,  ...  ∂(p_AoBi)/∂qⱼ]

.. raw:: html

    </details>

Note: Each partial derivative of p_AoBi is taken in frame A.

See also:
    CalcJacobianPositionVector() for details on Jq_p_AoBi.

See also:
    See Jacobian_functions "Jacobian functions" for related functions.)""";
        } CalcJacobianTranslationalVelocity;
        // Symbol: drake::multibody::MultibodyPlant::CalcMassMatrix
        struct /* CalcMassMatrix */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(Efficiently computes the mass matrix ``M(q)`` of the model. The
generalized positions q are taken from the given ``context``. M
includes the mass properties of rigid bodies and reflected_inertia
"reflected inertias" as provided with JointActuator specifications.

This method employs the Composite Body Algorithm, which we believe to
be the fastest O(n²) algorithm to compute the mass matrix of a
multibody system.

Parameter ``context``:
    The Context containing the state of the model from which
    generalized coordinates q are extracted.

Parameter ``M``:
    A pointer to a square matrix in ``ℛⁿˣⁿ`` with n the number of
    generalized velocities (num_velocities()) of the model. Although
    symmetric, the matrix is filled in completely on return.

Precondition:
    M is non-null and has the right size.

Warning:
    This is an O(n²) algorithm. Avoid the explicit computation of the
    mass matrix whenever possible.

See also:
    CalcMassMatrixViaInverseDynamics() (slower))""";
        } CalcMassMatrix;
        // Symbol: drake::multibody::MultibodyPlant::CalcMassMatrixViaInverseDynamics
        struct /* CalcMassMatrixViaInverseDynamics */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(Computes the mass matrix ``M(q)`` of the model using a slow method
(inverse dynamics). The generalized positions q are taken from the
given ``context``. M includes the mass properties of rigid bodies and
reflected_inertia "reflected inertias" as provided with JointActuator
specifications.

Use CalcMassMatrix() for a faster implementation using the Composite
Body Algorithm.

Parameter ``context``:
    The Context containing the state of the model from which
    generalized coordinates q are extracted.

Parameter ``M``:
    A pointer to a square matrix in ``ℛⁿˣⁿ`` with n the number of
    generalized velocities (num_velocities()) of the model. Although
    symmetric, the matrix is filled in completely on return.

Precondition:
    M is non-null and has the right size.

The algorithm used to build ``M(q)`` consists in computing one column
of ``M(q)`` at a time using inverse dynamics. The result from inverse
dynamics, with no applied forces, is the vector of generalized forces:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    tau = M(q)v̇ + C(q, v)v

.. raw:: html

    </details>

where q and v are the generalized positions and velocities,
respectively. When ``v = 0`` the Coriolis and gyroscopic forces term
``C(q, v)v`` is zero. Therefore the ``i-th`` column of ``M(q)`` can be
obtained performing inverse dynamics with an acceleration vector ``v̇
= eᵢ``, with ``eᵢ`` the standard (or natural) basis of ``ℛⁿ`` with n
the number of generalized velocities. We write this as:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    M.ᵢ(q) = M(q) * e_i

.. raw:: html

    </details>

where ``M.ᵢ(q)`` (notice the dot for the rows index) denotes the
``i-th`` column in M(q).

Warning:
    This is an O(n²) algorithm. Avoid the explicit computation of the
    mass matrix whenever possible.

See also:
    CalcMassMatrix(), CalcInverseDynamics())""";
        } CalcMassMatrixViaInverseDynamics;
        // Symbol: drake::multibody::MultibodyPlant::CalcPointsPositions
        struct /* CalcPointsPositions */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(Given the positions ``p_BQi`` for a set of points ``Qi`` measured and
expressed in a frame B, this method computes the positions
``p_AQi(q)`` of each point ``Qi`` in the set as measured and expressed
in another frame A, as a function of the generalized positions q of
the model.

Example of usage: Given two points Q0 and Q1 that are fixed to a frame
B, the code below calculates their positions from the world frame
origin, expressed in the world frame W.


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    constexpr int num_position_vectors = 2;
     MatrixX<double> p_BQi(3, num_position_vectors);
     p_BQi.col(0) = Vector3<double>(1.1, 2.2, 3.3);
     p_BQi.col(1) = Vector3<double>(-9.8, 7.6, -5.43);
     MatrixX<double> p_WQi(3, num_position_vectors);
     const Frame<double>& frame_W = plant.world_frame();
     plant.CalcPointsPositions(*context_, frame_B, p_BQi, frame_W, &p_WQi);

.. raw:: html

    </details>

Parameter ``context``:
    The context containing the state of the model. It stores the
    generalized positions q of the model.

Parameter ``frame_B``:
    The frame B in which the positions ``p_BQi`` of a set of points
    ``Qi`` are given.

Parameter ``p_BQi``:
    The input positions of each point ``Qi`` in frame B. ``p_BQi ∈
    ℝ³ˣⁿ`` with ``n`` the number of points in the set. Each column of
    ``p_BQi`` corresponds to a vector in ℝ³ holding the position of
    one of the points in the set as measured and expressed in frame B.
    Each column of p_BQi is a position vector associated with one
    point Qi, and the number of columns in p_BQi is the number n of
    points.

Parameter ``frame_A``:
    The frame A in which it is desired to compute the positions
    ``p_AQi`` of each point ``Qi`` in the set.

Parameter ``p_AQi``:
    The output positions of each point ``Qi`` now computed as measured
    and expressed in frame A. The output ``p_AQi`` **must** have the
    same size as the input ``p_BQi`` or otherwise this method aborts.
    That is ``p_AQi`` **must** be in ``ℝ³ˣⁿ``. Each column of p_AQi is
    a position vector associated with one point Qi, and the number of
    columns in p_BQi is the number n of points.

Note:
    Both ``p_BQi`` and ``p_AQi`` must have three rows. Otherwise this
    method will throw a RuntimeError. This method also throws a
    RuntimeError if ``p_BQi`` and ``p_AQi`` differ in the number of
    columns.)""";
        } CalcPointsPositions;
        // Symbol: drake::multibody::MultibodyPlant::CalcPointsVelocities
        struct /* CalcPointsVelocities */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(For a set of n points Qi (i = 0, ... n-1) that are regarded as fixed
on a frame B, calculates the velocities v_AQi_E of Qi measured in a
frame A and expressed in a frame E.

Example of usage: Given two points Q0 and Q1 that are fixed to a frame
B, the code below calculates their velocities measured and expressed
in the world frame W.


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    constexpr int num_position_vectors = 2;
     MatrixX<double> p_BQi(3, num_position_vectors);
     p_BQi.col(0) = Vector3<double>(1.1, 2.2, 3.3);
     p_BQi.col(1) = Vector3<double>(-9.8, 7.6, -5.43);
     MatrixX<double> v_WQi_W(3, num_position_vectors);
     const Frame<double>& frame_W = plant.world_frame();
     plant.CalcPointsVelocities(*context_, frame_B, p_BQi, frame_W, frame_W,
                                &v_WQi_W);

.. raw:: html

    </details>

Parameter ``context``:
    Contains the state of the multibody system, including the
    generalized positions q and the generalized velocities v.

Parameter ``frame_B``:
    The frame B in which each point Qi is fixed and whose frame origin
    Bo is the starting point for position vectors in p_BQi. frame_B is
    also the expressed-in-frame for position vectors p_BQi.

Parameter ``p_BQi``:
    Position vectors from Bo (frame B's origin) to each point Qi (i =
    0, ... n-1), expressed in frame B. Each column of p_BQi is a
    position vector associated with one point Qi, and the number of
    columns in p_BQi is the number n of points.

Parameter ``frame_A``:
    The frame in which the velocities are to be measured.

Parameter ``frame_E``:
    The frame in which the velocities are to be expressed.

Parameter ``v_AQi_E``:
    The velocities of each point Qi (i = 0, ... n-1) measured in frame
    A and expressed in frame E. Each column of v_AQi_E is a
    translational velocity vector associated with one point Qi, and
    the number of columns in v_AQi_E is the number n of points.

Raises:
    RuntimeError if p_BQi and v_AQi_E do not have three rows (are not
    3 element vectors) or do not have the same number (n > 0) of
    columns.)""";
        } CalcPointsVelocities;
        // Symbol: drake::multibody::MultibodyPlant::CalcRelativeRotationMatrix
        struct /* CalcRelativeRotationMatrix */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(Calculates the rotation matrix ``R_AB`` relating frame A and frame B.

Parameter ``context``:
    The state of the multibody system, which includes the system's
    generalized positions q. Note: ``R_AB`` is a function of q.

Parameter ``frame_A``:
    The frame A designated in the rigid transform ``R_AB``.

Parameter ``frame_B``:
    The frame G designated in the rigid transform ``R_AB``.

Returns ``R_AB``:
    The RigidTransform relating frame A and frame B.)""";
        } CalcRelativeRotationMatrix;
        // Symbol: drake::multibody::MultibodyPlant::CalcRelativeTransform
        struct /* CalcRelativeTransform */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(Calculates the rigid transform (pose) ``X_AB`` relating frame A and
frame B.

Parameter ``context``:
    The state of the multibody system, which includes the system's
    generalized positions q. Note: ``X_AB`` is a function of q.

Parameter ``frame_A``:
    The frame A designated in the rigid transform ``X_AB``.

Parameter ``frame_B``:
    The frame G designated in the rigid transform ``X_AB``.

Returns ``X_AB``:
    The RigidTransform relating frame A and frame B.)""";
        } CalcRelativeTransform;
        // Symbol: drake::multibody::MultibodyPlant::CalcSpatialAccelerationsFromVdot
        struct /* CalcSpatialAccelerationsFromVdot */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(Given the state of this model in ``context`` and a known vector of
generalized accelerations ``known_vdot``, this method computes the
spatial acceleration ``A_WB`` for each body as measured and expressed
in the world frame W.

Parameter ``context``:
    The context containing the state of this model.

Parameter ``known_vdot``:
    A vector with the generalized accelerations for the full model.

Parameter ``A_WB_array``:
    A pointer to a valid, non nullptr, vector of spatial accelerations
    containing the spatial acceleration ``A_WB`` for each body. It
    must be of size equal to the number of bodies in the model. On
    output, entries will be ordered by BodyIndex.

Raises:
    RuntimeError if A_WB_array is not of size ``num_bodies()``.)""";
        } CalcSpatialAccelerationsFromVdot;
        // Symbol: drake::multibody::MultibodyPlant::CalcSpatialInertia
        struct /* CalcSpatialInertia */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(Returns M_SFo_F, the spatial inertia of a set S of bodies about point
Fo (the origin of a frame F), expressed in frame F. You may regard
M_SFo_F as measuring spatial inertia as if the set S of bodies were
welded into a single composite body at the configuration specified in
the ``context``.

Parameter ``context``:
    Contains the configuration of the set S of bodies.

Parameter ``frame_F``:
    specifies the about-point Fo (frame_F's origin) and the
    expressed-in frame for the returned spatial inertia.

Parameter ``body_indexes``:
    Array of selected bodies. This method does not distinguish between
    welded bodies, joint-connected bodies, etc.

Raises:
    RuntimeError if body_indexes contains an invalid BodyIndex or if
    there is a repeated BodyIndex.

Note:
    The mass and inertia of the world_body() does not contribute to
    the the returned spatial inertia.)""";
        } CalcSpatialInertia;
        // Symbol: drake::multibody::MultibodyPlant::CalcSpatialMomentumInWorldAboutPoint
        struct /* CalcSpatialMomentumInWorldAboutPoint */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc_2args =
R"""(This method returns the spatial momentum of ``this`` MultibodyPlant in
the world frame W, about a designated point P, expressed in the world
frame W.

Parameter ``context``:
    Contains the state of the model.

Parameter ``p_WoP_W``:
    Position from Wo (origin of the world frame W) to an arbitrary
    point P, expressed in the world frame W.

Returns ``L_WSP_W``:
    , spatial momentum of the system S represented by ``this`` plant,
    measured in the world frame W, about point P, expressed in W.

Note:
    To calculate the spatial momentum of this system S in W about Scm
    (the system's center of mass), use something like:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    MultibodyPlant<T> plant;
      // ... code to load a model ....
      const Vector3<T> p_WoScm_W =
        plant.CalcCenterOfMassPositionInWorld(context);
      const SpatialMomentum<T> L_WScm_W =
        plant.CalcSpatialMomentumInWorldAboutPoint(context, p_WoScm_W);

.. raw:: html

    </details>)""";
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc_3args =
R"""(This method returns the spatial momentum of a set of model instances
in the world frame W, about a designated point P, expressed in frame
W.

Parameter ``context``:
    Contains the state of the model.

Parameter ``model_instances``:
    Set of selected model instances.

Parameter ``p_WoP_W``:
    Position from Wo (origin of the world frame W) to an arbitrary
    point P, expressed in the world frame W.

Returns ``L_WSP_W``:
    , spatial momentum of the system S represented by the
    model_instances, measured in world frame W, about point P,
    expressed in W.

Note:
    To calculate the spatial momentum of this system S in W about Scm
    (the system's center of mass), use something like:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    MultibodyPlant<T> plant;
      // ... code to create a set of selected model instances, e.g., ...
      const ModelInstanceIndex gripper_model_instance =
        plant.GetModelInstanceByName("gripper");
      const ModelInstanceIndex robot_model_instance =
        plant.GetBodyByName("end_effector").model_instance();
      const std::vector<ModelInstanceIndex> model_instances{
        gripper_model_instance, robot_model_instance};
      const Vector3<T> p_WoScm_W =
        plant.CalcCenterOfMassPositionInWorld(context, model_instances);
      SpatialMomentum<T> L_WScm_W =
        plant.CalcSpatialMomentumInWorldAboutPoint(context, model_instances,
                                                   p_WoScm_W);

.. raw:: html

    </details>

Raises:
    RuntimeError if model_instances contains an invalid
    ModelInstanceIndex.)""";
        } CalcSpatialMomentumInWorldAboutPoint;
        // Symbol: drake::multibody::MultibodyPlant::CalcTotalMass
        struct /* CalcTotalMass */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc_1args =
R"""(Calculates the total mass of all bodies in this MultibodyPlant.

Parameter ``context``:
    Contains the state of the model.

Returns ``The``:
    total mass of all bodies or 0 if there are none.

Note:
    The mass of the world_body() does not contribute to the total
    mass.)""";
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc_2args =
R"""(Calculates the total mass of all bodies contained in model_instances.

Parameter ``context``:
    Contains the state of the model.

Parameter ``model_instances``:
    Vector of selected model instances. This method does not
    distinguish between welded, joint connected, or floating bodies.

Returns ``The``:
    total mass of all bodies belonging to a model instance in
    model_instances or 0 if model_instances is empty.

Note:
    The mass of the world_body() does not contribute to the total mass
    and each body only contributes to the total mass once, even if the
    body has repeated occurrence (instance) in model_instances.)""";
        } CalcTotalMass;
        // Symbol: drake::multibody::MultibodyPlant::CollectRegisteredGeometries
        struct /* CollectRegisteredGeometries */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(For each of the provided ``bodies``, collects up all geometries that
have been registered to that body. Intended to be used in conjunction
with CollisionFilterDeclaration and CollisionFilterManager::Apply() to
filter collisions between the geometries registered to the bodies.

For example:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    // Don't report on collisions between geometries affixed to `body1`,
    // `body2`, or `body3`.
    std::vector<const RigidBody<T>*> bodies{&body1, &body2, &body3};
    geometry::GeometrySet set = plant.CollectRegisteredGeometries(bodies);
    scene_graph.collision_filter_manager().Apply(
        CollisionFilterDeclaration().ExcludeWithin(set));

.. raw:: html

    </details>

Note:
    There is a *very* specific order of operations:

1. Bodies and geometries must be added to the MultibodyPlant.
2. Create GeometrySet instances from bodies (via this method).
3. Invoke SceneGraph::ExcludeCollisions*() to filter collisions.
4. Allocate context.

Changing the order will cause exceptions to be thrown.

Raises:
    RuntimeError if ``this`` MultibodyPlant was not registered with a
    SceneGraph.)""";
        } CollectRegisteredGeometries;
        // Symbol: drake::multibody::MultibodyPlant::EvalBlockSystemJacobian
        struct /* EvalBlockSystemJacobian */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""((Internal use only) Returns the System Jacobian Jv_V_WB(q) in block
form. Each block is dense and corresponds to one Tree of the as-built
internal::SpanningForest. The blocks follow the Tree ordering defined
by the SpanningForest, so are in TreeIndex order. The block for Treeᵢ
is a MatrixX of size 6nᵢ x mᵢ, where nᵢ is the number of mobilized
bodies in Treeᵢ and mᵢ is the total number of mobilizer velocity
degrees of freedom (mobilities) in the Tree. Every Tree has an entry
even if it has no mobilities (in that case mᵢ=0). World is not part of
any Tree so there is no block corresponding to World here.

To be precise: the iᵗʰ block Jvi_V_WB ≡ ∂Vi_WB/∂vᵢ where Vi_WB is the
stacked spatial velocities for each mobilized body in Treeᵢ (in order
of MobodIndex), and vᵢ is the vector of generalized velocities
associated with those mobilized bodies, in the same order. Thus
Jvi_V_WB⋅v for some set of mᵢ generalized velocities v, returns the
spatial velocities for each body in Treeᵢ that would result from
velocities v.

Note that locking and unlocking mobilizers does not affect the
Jacobian; the Jacobian reflects what would happen if a velocity
variable changed regardless of whether it can currently do so.

See also:
    CalcJacobianSpatialVelocity(), CalcFullSystemJacobian())""";
        } EvalBlockSystemJacobian;
        // Symbol: drake::multibody::MultibodyPlant::EvalBodyPoseInWorld
        struct /* EvalBodyPoseInWorld */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(Evaluate the pose ``X_WB`` of a body B in the world frame W.

Parameter ``context``:
    The context storing the state of the model.

Parameter ``body_B``:
    The body B for which the pose is requested.

Returns ``X_WB``:
    The pose of body frame B in the world frame W.

Raises:
    RuntimeError if Finalize() was not called on ``this`` model or if
    ``body_B`` does not belong to this model.)""";
        } EvalBodyPoseInWorld;
        // Symbol: drake::multibody::MultibodyPlant::EvalBodySpatialAccelerationInWorld
        struct /* EvalBodySpatialAccelerationInWorld */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(Evaluates A_WB, body B's spatial acceleration in the world frame W.

Parameter ``context``:
    The context storing the state of the model.

Parameter ``body_B``:
    The body for which spatial acceleration is requested.

Returns ``A_WB_W``:
    RigidBody B's spatial acceleration in the world frame W, expressed
    in W (for point Bo, the body's origin).

Raises:
    RuntimeError if Finalize() was not called on ``this`` model or if
    ``body_B`` does not belong to this model.

Note:
    When cached values are out of sync with the state stored in
    context, this method performs an expensive forward dynamics
    computation, whereas once evaluated, successive calls to this
    method are inexpensive.)""";
        } EvalBodySpatialAccelerationInWorld;
        // Symbol: drake::multibody::MultibodyPlant::EvalBodySpatialVelocityInWorld
        struct /* EvalBodySpatialVelocityInWorld */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(Evaluates V_WB, body B's spatial velocity in the world frame W.

Parameter ``context``:
    The context storing the state of the model.

Parameter ``body_B``:
    The body B for which the spatial velocity is requested.

Returns ``V_WB_W``:
    RigidBody B's spatial velocity in the world frame W, expressed in
    W (for point Bo, the body's origin).

Raises:
    RuntimeError if Finalize() was not called on ``this`` model or if
    ``body_B`` does not belong to this model.)""";
        } EvalBodySpatialVelocityInWorld;
        // Symbol: drake::multibody::MultibodyPlant::EvalSceneGraphInspector
        struct /* EvalSceneGraphInspector */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(Returns the inspector from the ``context`` for the SceneGraph
associated with this plant, via this plant's "geometry_query" input
port. (In the future, the inspector might come from a different
context source that is more efficient than the "geometry_query" input
port.))""";
        } EvalSceneGraphInspector;
        // Symbol: drake::multibody::MultibodyPlant::ExcludeCollisionGeometriesWithCollisionFilterGroupPair
        struct /* ExcludeCollisionGeometriesWithCollisionFilterGroupPair */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(Excludes the collision geometries between two given collision filter
groups.

Precondition:
    RegisterAsSourceForSceneGraph() has been called.

Precondition:
    Finalize() has *not* been called.)""";
        } ExcludeCollisionGeometriesWithCollisionFilterGroupPair;
        // Symbol: drake::multibody::MultibodyPlant::Finalize
        struct /* Finalize */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(This method must be called after all elements in the model (joints,
bodies, force elements, constraints, etc.) are added and before any
computations are performed. It essentially compiles all the necessary
"topological information", i.e. how bodies, joints and, any other
elements connect with each other, and performs all the required
pre-processing to enable computations at a later stage.

If the finalize stage is successful, the topology of this
MultibodyPlant is valid, meaning that the topology is up-to-date after
this call. No more multibody elements can be added after a call to
Finalize().

At Finalize(), state and input/output ports for ``this`` plant are
declared.

For a full account of the effects of Finalize(), see
mbp_finalize_stage "Finalize() stage".

See also:
    is_finalized(), mbp_finalize_stage "Finalize() stage".

Raises:
    RuntimeError if the MultibodyPlant has already been finalized.)""";
        } Finalize;
        // Symbol: drake::multibody::MultibodyPlant::FindSubgraphsOfWeldedBodies
        struct /* FindSubgraphsOfWeldedBodies */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc = R"""()""";
        } FindSubgraphsOfWeldedBodies;
        // Symbol: drake::multibody::MultibodyPlant::GetAccelerationLowerLimits
        struct /* GetAccelerationLowerLimits */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(Returns a vector of size ``num_velocities()`` containing the lower
acceleration limits for every generalized velocity coordinate. These
include joint and free body coordinates. Any unbounded or unspecified
limits will be -infinity.

Raises:
    RuntimeError if called pre-finalize.)""";
        } GetAccelerationLowerLimits;
        // Symbol: drake::multibody::MultibodyPlant::GetAccelerationUpperLimits
        struct /* GetAccelerationUpperLimits */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(Upper limit analog of GetAccelerationsLowerLimits(), where any
unbounded or unspecified limits will be +infinity.

See also:
    GetAccelerationLowerLimits() for more information.)""";
        } GetAccelerationUpperLimits;
        // Symbol: drake::multibody::MultibodyPlant::GetActuatedJointIndices
        struct /* GetActuatedJointIndices */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(Returns a list of actuated joint indices associated with
``model_instance``.

Raises:
    RuntimeError if called pre-finalize.)""";
        } GetActuatedJointIndices;
        // Symbol: drake::multibody::MultibodyPlant::GetActuationFromArray
        struct /* GetActuationFromArray */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(Returns a vector of actuation values for ``model_instance`` from a
vector ``u`` of actuation values for the entire plant model. Refer to
mbp_actuation "Actuation" for further details.

Parameter ``u``:
    Actuation values for the entire model. The actuation value in
    ``u`` for a particular actuator must be found at offset
    JointActuator::input_start().

Returns:
    Actuation values for ``model_instance``, ordered by monotonically
    increasing JointActuatorIndex.

Raises:
    RuntimeError if ``u`` is not of size
    MultibodyPlant::num_actuated_dofs().)""";
        } GetActuationFromArray;
        // Symbol: drake::multibody::MultibodyPlant::GetActuatorNames
        struct /* GetActuatorNames */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc_1args =
R"""(Returns a list of string names corresponding to each element of the
actuation vector. These strings take the form
``{model_instance_name}_{joint_actuator_name}``, but the prefix may
optionally be withheld using ``add_model_instance_prefix``.

The returned names are guaranteed to be unique if
``add_model_instance_prefix`` is ``True`` (the default).

Raises:
    RuntimeError if the plant is not finalized.)""";
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc_2args =
R"""(Returns a list of string names corresponding to each element of the
actuation vector. These strings take the form
``{model_instance_name}_{joint_actuator_name}``, but the prefix may
optionally be withheld using ``add_model_instance_prefix``.

The returned names are guaranteed to be unique.

Raises:
    RuntimeError if the plant is not finalized or if the
    ``model_instance`` is invalid.)""";
        } GetActuatorNames;
        // Symbol: drake::multibody::MultibodyPlant::GetBaseBodyJointType
        struct /* GetBaseBodyJointType */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(Returns the currently-set choice for base body joint type, either for
the global setting or for a specific model instance if provided. If a
model instance is provided for which no explicit choice has been made,
the global setting is returned. Any model instance index is acceptable
here; if not recognized then the global setting is returned. This can
be called any time -- pre-finalize it returns the joint type that will
be used by Finalize(); post-finalize it returns the joint type that
*was* used if there were any base bodies in need of a joint.

See also:
    SetBaseBodyJointType())""";
        } GetBaseBodyJointType;
        // Symbol: drake::multibody::MultibodyPlant::GetBodiesKinematicallyAffectedBy
        struct /* GetBodiesKinematicallyAffectedBy */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(Returns all bodies whose kinematics are transitively affected by the
given vector of Joints. This is a *kinematic* relationship rather than
a dynamic one. It is is inherently a query on the topology of the
plant's modeled tree. Constraints are likewise not considered.

The affected bodies are returned in increasing order of body indices.
A body is included in the output if that body's spatial velocity is
affected by the generalized velocities v of one of the indicated
joints.

As such, there are some notable implications:

1. If a body has an inboard free (6 dof) joint, it will be
*kinematically* affected by joints further inboard, even though there
might not be any dynamic influence on that body. 2. If the set of
joints have no velocities (i.e., they are all weld (0 dof) joints),
then, by definition, no bodies will be affected.

This function can be only be called post-finalize, see Finalize().

Raises:
    RuntimeError if any of the given joint has an invalid index,
    doesn't correspond to a mobilizer, or is welded.)""";
        } GetBodiesKinematicallyAffectedBy;
        // Symbol: drake::multibody::MultibodyPlant::GetBodiesWeldedTo
        struct /* GetBodiesWeldedTo */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(Returns all bodies that are transitively welded, or rigidly affixed,
to ``body``, per these two definitions:

1. A body is always considered welded to itself.
2. Two unique bodies are considered welded together exclusively by the
presence of a weld joint, not by other constructs that prevent mobility
(e.g. constraints).

This method can be called at any time during the lifetime of ``this``
plant, either pre- or post-finalize, see Finalize().

Meant to be used with ``CollectRegisteredGeometries``.

The following example demonstrates filtering collisions between all
bodies rigidly affixed to a door (which could be moving) and all
bodies rigidly affixed to the world:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    GeometrySet g_world = plant.CollectRegisteredGeometries(
        plant.GetBodiesWeldedTo(plant.world_body()));
    GeometrySet g_door = plant.CollectRegisteredGeometries(
        plant.GetBodiesWeldedTo(plant.GetBodyByName("door")));
    scene_graph.ExcludeCollisionsBetweeen(g_world, g_door);

.. raw:: html

    </details>

Note:
    Usages akin to this example may introduce redundant collision
    filtering; this will not have a functional impact, but may have a
    minor performance impact.

Returns:
    all bodies rigidly fixed to ``body``. This does not return the
    bodies in any prescribed order.

Raises:
    RuntimeError if ``body`` is not part of this plant.)""";
        } GetBodiesWeldedTo;
        // Symbol: drake::multibody::MultibodyPlant::GetBodyByName
        struct /* GetBodyByName */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc_1args =
R"""(Returns a constant reference to a body that is identified by the
string ``name`` in ``this`` MultibodyPlant.

Raises:
    RuntimeError if there is no body with the requested name.

Raises:
    RuntimeError if the body name occurs in multiple model instances.

See also:
    HasBodyNamed() to query if there exists a body in ``this``
    MultibodyPlant with a given specified name.)""";
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc_2args =
R"""(Returns a constant reference to the body that is uniquely identified
by the string ``name`` and ``model_instance`` in ``this``
MultibodyPlant.

Raises:
    RuntimeError if there is no body with the requested name.

See also:
    HasBodyNamed() to query if there exists a body in ``this``
    MultibodyPlant with a given specified name.)""";
        } GetBodyByName;
        // Symbol: drake::multibody::MultibodyPlant::GetBodyFrameIdIfExists
        struct /* GetBodyFrameIdIfExists */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(If the body with ``body_index`` belongs to the called plant, it
returns the geometry::FrameId associated with it. Otherwise, it
returns nullopt.)""";
        } GetBodyFrameIdIfExists;
        // Symbol: drake::multibody::MultibodyPlant::GetBodyFrameIdOrThrow
        struct /* GetBodyFrameIdOrThrow */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(If the body with ``body_index`` belongs to the called plant, it
returns the geometry::FrameId associated with it. Otherwise this
method throws an exception.

Raises:
    RuntimeError if the called plant does not have the body indicated
    by ``body_index``.)""";
        } GetBodyFrameIdOrThrow;
        // Symbol: drake::multibody::MultibodyPlant::GetBodyFromFrameId
        struct /* GetBodyFromFrameId */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(Given a geometry frame identifier, returns a pointer to the body
associated with that id (nullptr if there is no such body).)""";
        } GetBodyFromFrameId;
        // Symbol: drake::multibody::MultibodyPlant::GetBodyIndices
        struct /* GetBodyIndices */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(Returns a list of body indices associated with ``model_instance``.)""";
        } GetBodyIndices;
        // Symbol: drake::multibody::MultibodyPlant::GetCollisionGeometriesForBody
        struct /* GetCollisionGeometriesForBody */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(Returns an array of GeometryId's identifying the different contact
geometries for ``body`` previously registered with a SceneGraph.

Note:
    This method can be called at any time during the lifetime of
    ``this`` plant, either pre- or post-finalize, see Finalize().
    Post-finalize calls will always return the same value.

See also:
    RegisterCollisionGeometry(), Finalize())""";
        } GetCollisionGeometriesForBody;
        // Symbol: drake::multibody::MultibodyPlant::GetConstraintActiveStatus
        struct /* GetConstraintActiveStatus */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(Returns the active status of the constraint given by ``id`` in
``context``.

Raises:
    RuntimeError if the MultibodyPlant has not been finalized.

Raises:
    RuntimeError if ``id`` does not belong to any multibody constraint
    in ``context``.)""";
        } GetConstraintActiveStatus;
        // Symbol: drake::multibody::MultibodyPlant::GetConstraintIds
        struct /* GetConstraintIds */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(Returns a list of all constraint identifiers. The returned vector
becomes invalid after any calls to Add*Constraint() or
RemoveConstraint().)""";
        } GetConstraintIds;
        // Symbol: drake::multibody::MultibodyPlant::GetDefaultContactSurfaceRepresentation
        struct /* GetDefaultContactSurfaceRepresentation */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(Return the default value for contact representation, given the desired
time step. Discrete systems default to use polygons; continuous
systems default to use triangles.)""";
        } GetDefaultContactSurfaceRepresentation;
        // Symbol: drake::multibody::MultibodyPlant::GetDefaultDistanceConstraintParams
        struct /* GetDefaultDistanceConstraintParams */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(Returns all default distance constraint parameters, as registered via
AddDistanceConstraint(). See GetDistanceConstraintParams() and
SetDistanceConstraintParams() for working with parameters stored in a
context.)""";
        } GetDefaultDistanceConstraintParams;
        // Symbol: drake::multibody::MultibodyPlant::GetDefaultFloatingBaseBodyPose
        struct /* GetDefaultFloatingBaseBodyPose */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(Gets the provisional default pose of ``body`` as set by
SetDefaultFloatingBaseBodyPose(). If no pose was specified for
``body``, returns the identity pose. This may be called pre- or
post-Finalize().

Warning:
    This value is only meaningful for bodies that turn out to be
    floating base bodies after Finalize(). If called on any other
    body, the result simply echoes whatever provisional pose was set
    in SetDefaultFloatingBaseBodyPose() but has no other effect. Use
    the Joint API to get the default pose for any body that has an
    explicitly-defined joint to its parent body.

Note:
    Post-Finalize(), a floating base body's default pose may be set
    either by SetDefaultFloatingBaseBodyPose() or by setting the
    default pose directly through the Joint API applied to the
    automatically-added floating joint.
    GetDefaultFloatingBaseBodyPose() will return the most-recent value
    set by either method.

See mbp_working_with_free_bodies "above for details".

Parameter ``body``:
    RigidBody whose default pose will be retrieved.

Returns ``X_WB``:
    The default pose of the floating base body B in World. Not
    meaningful if ``body`` is not a floating base body.)""";
        } GetDefaultFloatingBaseBodyPose;
        // Symbol: drake::multibody::MultibodyPlant::GetDefaultFreeBodyPose
        struct /* GetDefaultFreeBodyPose */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc_deprecated =
R"""((Deprecated.)

Deprecated:
    Use GetDefaultFloatingBaseBodyPose() instead. This will be removed
    from Drake on or after 2026-06-01.)""";
        } GetDefaultFreeBodyPose;
        // Symbol: drake::multibody::MultibodyPlant::GetDefaultPositions
        struct /* GetDefaultPositions */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc_0args =
R"""(Gets the default positions for the plant, which can be changed via
SetDefaultPositions().

Raises:
    RuntimeError if the plant is not finalized.)""";
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc_1args =
R"""(Gets the default positions for the plant for a given model instance,
which can be changed via SetDefaultPositions().

Raises:
    RuntimeError if the plant is not finalized, or if the
    model_instance is invalid,)""";
        } GetDefaultPositions;
        // Symbol: drake::multibody::MultibodyPlant::GetDistanceConstraintParams
        struct /* GetDistanceConstraintParams */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc_1args =
R"""(Returns all distance constraint parameters currently stored in
``context``.)""";
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc_2args =
R"""(Returns a constant reference to the parameters for the distance
constraint that corresponds to identifier ``id``.

Raises:
    if ``id`` is not a valid identifier for a distance constraint.)""";
        } GetDistanceConstraintParams;
        // Symbol: drake::multibody::MultibodyPlant::GetEffortLowerLimits
        struct /* GetEffortLowerLimits */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(Returns a vector of size ``num_actuated_dofs()`` containing the lower
effort limits for every actuator. Any unbounded or unspecified limits
will be -∞. The returned vector is indexed by JointActuatorIndex, see
JointActuator::index().

See also:
    GetEffortUpperLimits()

Raises:
    RuntimeError if called pre-finalize.)""";
        } GetEffortLowerLimits;
        // Symbol: drake::multibody::MultibodyPlant::GetEffortUpperLimits
        struct /* GetEffortUpperLimits */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(Returns a vector of size ``num_actuated_dofs()`` containing the upper
effort limits for every actuator. Any unbounded or unspecified limits
will be +∞. The returned vector is indexed by JointActuatorIndex, see
JointActuator::index().

See also:
    GetEffortLowerLimits()

Raises:
    RuntimeError if called pre-finalize.)""";
        } GetEffortUpperLimits;
        // Symbol: drake::multibody::MultibodyPlant::GetFloatingBaseBodies
        struct /* GetFloatingBaseBodies */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(Returns the set of body indices corresponding to the floating base
bodies in the model, in no particular order. See
mbp_working_with_free_bodies "above for details".

Raises:
    RuntimeError if called pre-finalize, see Finalize().)""";
        } GetFloatingBaseBodies;
        // Symbol: drake::multibody::MultibodyPlant::GetForceElement
        struct /* GetForceElement */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(Returns a constant reference to a force element identified by its
unique index in ``this`` MultibodyPlant. If the optional template
argument is supplied, then the returned value is downcast to the
specified ``ForceElementType``.

Template parameter ``ForceElementType``:
    The specific type of the ForceElement to be retrieved. It must be
    a subclass of ForceElement.

Raises:
    RuntimeError if the force element is not of type
    ``ForceElementType`` or if there is no ForceElement with that
    index.)""";
        } GetForceElement;
        // Symbol: drake::multibody::MultibodyPlant::GetFrameByName
        struct /* GetFrameByName */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc_1args =
R"""(Returns a constant reference to a frame that is identified by the
string ``name`` in ``this`` model.

Raises:
    RuntimeError if there is no frame with the requested name.

Raises:
    RuntimeError if the frame name occurs in multiple model instances.

See also:
    HasFrameNamed() to query if there exists a frame in ``this`` model
    with a given specified name.)""";
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc_2args =
R"""(Returns a constant reference to the frame that is uniquely identified
by the string ``name`` in ``model_instance``.

Raises:
    RuntimeError if there is no frame with the requested name.

Raises:
    RuntimeError if ``model_instance`` is not valid for this model.

See also:
    HasFrameNamed() to query if there exists a frame in ``this`` model
    with a given specified name.)""";
        } GetFrameByName;
        // Symbol: drake::multibody::MultibodyPlant::GetFrameIndices
        struct /* GetFrameIndices */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(Returns a list of frame indices associated with ``model_instance``.)""";
        } GetFrameIndices;
        // Symbol: drake::multibody::MultibodyPlant::GetFreeBodyPose
        struct /* GetFreeBodyPose */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(For any free body's 6-dof joint, gets the pose X_JpJc of the child
frame Jc in its parent frame Jp.

Note:
    Unless ``body`` is a floating base body, the parent frame Jp is
    not necessarily the World frame W, and the child frame Jc is not
    necessarily the body frame B.

See mbp_working_with_free_bodies "above for details".

Returns ``X_JpJc``:
    The current pose of child frame Jc in its parent frame Jp. Returns
    X_WB if ``body`` B is a floating base body.

Raises:
    RuntimeError if called pre-finalize.

Raises:
    RuntimeError if ``body`` is not a free body.)""";
        } GetFreeBodyPose;
        // Symbol: drake::multibody::MultibodyPlant::GetJointActuatorByName
        struct /* GetJointActuatorByName */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc_1args =
R"""(Returns a constant reference to an actuator that is identified by the
string ``name`` in ``this`` MultibodyPlant.

Raises:
    RuntimeError if there is no actuator with the requested name.

Raises:
    RuntimeError if the actuator name occurs in multiple model
    instances.

See also:
    HasJointActuatorNamed() to query if there exists an actuator in
    ``this`` MultibodyPlant with a given specified name.)""";
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc_2args =
R"""(Returns a constant reference to the actuator that is uniquely
identified by the string ``name`` and ``model_instance`` in ``this``
MultibodyPlant.

Raises:
    RuntimeError if there is no actuator with the requested name.

Raises:
    RuntimeError if ``model_instance`` is not valid for this model.

See also:
    HasJointActuatorNamed() to query if there exists an actuator in
    ``this`` MultibodyPlant with a given specified name.)""";
        } GetJointActuatorByName;
        // Symbol: drake::multibody::MultibodyPlant::GetJointActuatorIndices
        struct /* GetJointActuatorIndices */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc_0args =
R"""(Returns a list of all joint actuator indices. The vector is ordered by
monotonically increasing JointActuatorIndex, but the indices will in
general not be consecutive due to actuators that were removed.)""";
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc_1args =
R"""(Returns a list of joint actuator indices associated with
``model_instance``. The vector is ordered by monotonically increasing
JointActuatorIndex.

Raises:
    RuntimeError if called pre-finalize.)""";
        } GetJointActuatorIndices;
        // Symbol: drake::multibody::MultibodyPlant::GetJointByName
        struct /* GetJointByName */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(Returns a constant reference to a joint that is identified by the
string ``name`` in ``this`` MultibodyPlant. If the optional template
argument is supplied, then the returned value is downcast to the
specified ``JointType``.

Template parameter ``JointType``:
    The specific type of the Joint to be retrieved. It must be a
    subclass of Joint.

Raises:
    RuntimeError if the named joint is not of type ``JointType`` or if
    there is no Joint with that name.

Raises:
    RuntimeError if ``model_instance`` is not valid for this model.

See also:
    HasJointNamed() to query if there exists a joint in ``this``
    MultibodyPlant with a given specified name.)""";
        } GetJointByName;
        // Symbol: drake::multibody::MultibodyPlant::GetJointIndices
        struct /* GetJointIndices */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc_0args =
R"""(Returns a list of all joint indices. The vector is ordered by
monotonically increasing JointIndex, but the indices will in general
not be consecutive due to joints that were removed.)""";
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc_1args =
R"""(Returns a list of joint indices associated with ``model_instance``.)""";
        } GetJointIndices;
        // Symbol: drake::multibody::MultibodyPlant::GetModelInstanceByName
        struct /* GetModelInstanceByName */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(Returns the index to the model instance that is uniquely identified by
the string ``name`` in ``this`` MultibodyPlant.

Raises:
    RuntimeError if there is no instance with the requested name.

See also:
    HasModelInstanceNamed() to query if there exists an instance in
    ``this`` MultibodyPlant with a given specified name.)""";
        } GetModelInstanceByName;
        // Symbol: drake::multibody::MultibodyPlant::GetModelInstanceName
        struct /* GetModelInstanceName */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(Returns the name of a ``model_instance``.

Raises:
    RuntimeError when ``model_instance`` does not correspond to a
    model in this model.)""";
        } GetModelInstanceName;
        // Symbol: drake::multibody::MultibodyPlant::GetMutableJointByName
        struct /* GetMutableJointByName */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(A version of GetJointByName that returns a mutable reference.

See also:
    GetJointByName.)""";
        } GetMutableJointByName;
        // Symbol: drake::multibody::MultibodyPlant::GetMutableSceneGraphPreFinalize
        struct /* GetMutableSceneGraphPreFinalize */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""((Internal use only) Returns a mutable pointer to the SceneGraph that
this plant is registered as a source for. This method can only be used
pre-Finalize.

Raises:
    RuntimeError if is_finalized() == true ||
    geometry_source_is_registered() == false)""";
        } GetMutableSceneGraphPreFinalize;
        // Symbol: drake::multibody::MultibodyPlant::GetPositionLowerLimits
        struct /* GetPositionLowerLimits */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(Returns a vector of size ``num_positions()`` containing the lower
position limits for every generalized position coordinate. These
include joint and free body coordinates. Any unbounded or unspecified
limits will be -infinity.

Raises:
    RuntimeError if called pre-finalize.)""";
        } GetPositionLowerLimits;
        // Symbol: drake::multibody::MultibodyPlant::GetPositionNames
        struct /* GetPositionNames */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc_2args =
R"""(Returns a list of string names corresponding to each element of the
position vector. These strings take the form
``{model_instance_name}_{joint_name}_{joint_position_suffix}``, but
the prefix and suffix may optionally be withheld using
``add_model_instance_prefix`` and ``always_add_suffix``.

Parameter ``always_add_suffix``:
    (optional). If true, then the suffix is always added. If false,
    then the suffix is only added for joints that have more than one
    position (in this case, not adding would lead to ambiguity).

The returned names are guaranteed to be unique if
``add_model_instance_prefix`` is ``True`` (the default).

Raises:
    RuntimeError if the plant is not finalized.)""";
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc_3args =
R"""(Returns a list of string names corresponding to each element of the
position vector. These strings take the form
``{model_instance_name}_{joint_name}_{joint_position_suffix}``, but
the prefix and suffix may optionally be withheld using
``add_model_instance_prefix`` and ``always_add_suffix``.

Parameter ``always_add_suffix``:
    (optional). If true, then the suffix is always added. If false,
    then the suffix is only added for joints that have more than one
    position (in this case, not adding would lead to ambiguity).

The returned names are guaranteed to be unique.

Raises:
    RuntimeError if the plant is not finalized or if the
    ``model_instance`` is invalid.)""";
        } GetPositionNames;
        // Symbol: drake::multibody::MultibodyPlant::GetPositionUpperLimits
        struct /* GetPositionUpperLimits */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(Upper limit analog of GetPositionLowerLimits(), where any unbounded or
unspecified limits will be +infinity.

See also:
    GetPositionLowerLimits() for more information.)""";
        } GetPositionUpperLimits;
        // Symbol: drake::multibody::MultibodyPlant::GetPositions
        struct /* GetPositions */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc_1args =
R"""(Returns a const vector reference to the vector of generalized
positions q in a given Context.

Note:
    This method returns a reference to existing data, exhibits
    constant i.e., O(1) time complexity, and runs very quickly.

Raises:
    RuntimeError if ``context`` does not correspond to the Context for
    a multibody model.)""";
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc_2args =
R"""(Returns a vector containing the generalized positions q of a specified
model instance in a given Context.

Note:
    Returns a dense vector of dimension
    ``num_positions(model_instance)`` associated with
    ``model_instance`` by copying from ``context``.

Raises:
    RuntimeError if ``context`` does not correspond to the Context for
    a multibody model or ``model_instance`` is invalid.)""";
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc_3args =
R"""((Advanced) Populates output vector q_out with the generalized
positions q of a specified model instance in a given Context.

Note:
    q_out is a dense vector of dimension
    ``num_positions(model_instance)`` associated with
    ``model_instance`` and is populated by copying from ``context``.

Note:
    This function is guaranteed to allocate no heap.

Raises:
    RuntimeError if ``context`` does not correspond to the Context for
    a multibody model or ``model_instance`` is invalid.)""";
        } GetPositions;
        // Symbol: drake::multibody::MultibodyPlant::GetPositionsAndVelocities
        struct /* GetPositionsAndVelocities */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc_1args =
R"""(Returns a const vector reference ``[q; v]`` to the generalized
positions q and generalized velocities v in a given Context.

Note:
    This method returns a reference to existing data, exhibits
    constant i.e., O(1) time complexity, and runs very quickly.

Raises:
    RuntimeError if ``context`` does not correspond to the Context for
    a multibody model.)""";
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc_2args =
R"""(Returns a vector ``[q; v]`` containing the generalized positions q and
generalized velocities v of a specified model instance in a given
Context.

Note:
    Returns a dense vector of dimension
    ``num_positions(model_instance) + num_velocities(model_instance)``
    associated with ``model_instance`` by copying from ``context``.

Raises:
    RuntimeError if ``context`` does not correspond to the Context for
    a multibody model or ``model_instance`` is invalid.)""";
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc_3args =
R"""((Advanced) Populates output vector qv_out representing the generalized
positions q and generalized velocities v of a specified model instance
in a given Context.

Note:
    qv_out is a dense vector of dimensions
    ``num_positions(model_instance) + num_velocities(model_instance)``
    associated with ``model_instance`` and is populated by copying
    from ``context``.

Note:
    This function is guaranteed to allocate no heap.

Raises:
    RuntimeError if ``context`` does not correspond to the Context for
    a multibody model or ``model_instance`` is invalid.

Raises:
    RuntimeError if qv_out does not have size
    ``num_positions(model_instance) + num_velocities(model_instance)``)""";
        } GetPositionsAndVelocities;
        // Symbol: drake::multibody::MultibodyPlant::GetPositionsFromArray
        struct /* GetPositionsFromArray */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc_2args =
R"""(Returns a vector of generalized positions for ``model_instance`` from
a vector ``q_array`` of generalized positions for the entire model
model. This method throws an exception if ``q`` is not of size
MultibodyPlant::num_positions().)""";
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc_3args =
R"""((Advanced) Populates output vector q_out and with the generalized
positions for ``model_instance`` from a vector ``q`` of generalized
positions for the entire model. This method throws an exception if
``q`` is not of size MultibodyPlant::num_positions().)""";
        } GetPositionsFromArray;
        // Symbol: drake::multibody::MultibodyPlant::GetRigidBodyByName
        struct /* GetRigidBodyByName */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc_1args =
R"""(Returns a constant reference to a rigid body that is identified by the
string ``name`` in ``this`` model.

Raises:
    RuntimeError if there is no body with the requested name.

Raises:
    RuntimeError if the body name occurs in multiple model instances.

Raises:
    RuntimeError if the requested body is not a RigidBody.

See also:
    HasBodyNamed() to query if there exists a body in ``this`` model
    with a given specified name.)""";
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc_2args =
R"""(Returns a constant reference to the rigid body that is uniquely
identified by the string ``name`` in ``model_instance``.

Raises:
    RuntimeError if there is no body with the requested name.

Raises:
    RuntimeError if the requested body is not a RigidBody.

Raises:
    RuntimeError if ``model_instance`` is not valid for this model.

See also:
    HasBodyNamed() to query if there exists a body in ``this`` model
    with a given specified name.)""";
        } GetRigidBodyByName;
        // Symbol: drake::multibody::MultibodyPlant::GetStateNames
        struct /* GetStateNames */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc_1args =
R"""(Returns a list of string names corresponding to each element of the
multibody state vector. These strings take the form
``{model_instance_name}_{joint_name}_{joint_position_suffix |
joint_velocity_suffix}``, but the prefix may optionally be withheld
using ``add_model_instance_prefix``.

The returned names are guaranteed to be unique if
``add_model_instance_prefix`` is ``True`` (the default).

Raises:
    RuntimeError if the plant is not finalized.)""";
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc_2args =
R"""(Returns a list of string names corresponding to each element of the
multibody state vector. These strings take the form
``{model_instance_name}_{joint_name}_{joint_position_suffix |
joint_velocity_suffix}``, but the prefix may optionally be withheld
using ``add_model_instance_prefix``.

The returned names are guaranteed to be unique.

Raises:
    RuntimeError if the plant is not finalized or if the
    ``model_instance`` is invalid.)""";
        } GetStateNames;
        // Symbol: drake::multibody::MultibodyPlant::GetTopologyGraphvizString
        struct /* GetTopologyGraphvizString */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(Returns a Graphviz string describing the topology of this plant. To
render the string, use the Graphviz tool, ``dot``.
http://www.graphviz.org/

Note: this method can be called either before or after ``Finalize()``.)""";
        } GetTopologyGraphvizString;
        // Symbol: drake::multibody::MultibodyPlant::GetUniqueFloatingBaseBodyOrThrow
        struct /* GetUniqueFloatingBaseBodyOrThrow */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(If there is a single base body in the model given by
``model_instance``, and that body is a floating base body, returns
that floating base body. Otherwise, throws an exception. Use
HasUniqueFloatingBaseBody() to check first.

See mbp_working_with_free_bodies "above for details".

Raises:
    RuntimeError if called pre-finalize.

Raises:
    RuntimeError if ``model_instance`` is not valid.

Raises:
    RuntimeError if !HasUniqueFloatingBaseBody(model_instance).

See also:
    HasUniqueFloatingBaseBody(), GetFloatingBaseBodies())""";
        } GetUniqueFloatingBaseBodyOrThrow;
        // Symbol: drake::multibody::MultibodyPlant::GetUniqueFreeBaseBodyOrThrow
        struct /* GetUniqueFreeBaseBodyOrThrow */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc_deprecated =
R"""((Deprecated.)

Deprecated:
    Use GetUniqueFloatingBaseBodyOrThrow() instead. This will be
    removed from Drake on or after 2026-06-01.)""";
        } GetUniqueFreeBaseBodyOrThrow;
        // Symbol: drake::multibody::MultibodyPlant::GetVelocities
        struct /* GetVelocities */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc_1args =
R"""(Returns a const vector reference to the generalized velocities v in a
given Context.

Note:
    This method returns a reference to existing data, exhibits
    constant i.e., O(1) time complexity, and runs very quickly.

Raises:
    RuntimeError if ``context`` does not correspond to the Context for
    a multibody model.)""";
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc_2args =
R"""(Returns a vector containing the generalized velocities v of a
specified model instance in a given Context.

Note:
    returns a dense vector of dimension
    ``num_velocities(model_instance)`` associated with
    ``model_instance`` by copying from ``context``.

Raises:
    RuntimeError if ``context`` does not correspond to the Context for
    a multibody model or ``model_instance`` is invalid.)""";
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc_3args =
R"""((Advanced) Populates output vector v_out with the generalized
velocities v of a specified model instance in a given Context.

Note:
    v_out is a dense vector of dimension
    ``num_velocities(model_instance)`` associated with
    ``model_instance`` and is populated by copying from ``context``.

Note:
    This function is guaranteed to allocate no heap.

Raises:
    RuntimeError if ``context`` does not correspond to the Context for
    a multibody model or ``model_instance`` is invalid.)""";
        } GetVelocities;
        // Symbol: drake::multibody::MultibodyPlant::GetVelocitiesFromArray
        struct /* GetVelocitiesFromArray */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc_2args =
R"""(Returns a vector of generalized velocities for ``model_instance`` from
a vector ``v`` of generalized velocities for the entire MultibodyPlant
model. This method throws an exception if the input array is not of
size MultibodyPlant::num_velocities().)""";
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc_3args =
R"""((Advanced) Populates output vector v_out with the generalized
velocities for ``model_instance`` from a vector ``v`` of generalized
velocities for the entire model. This method throws an exception if
``v`` is not of size MultibodyPlant::num_velocities().)""";
        } GetVelocitiesFromArray;
        // Symbol: drake::multibody::MultibodyPlant::GetVelocityLowerLimits
        struct /* GetVelocityLowerLimits */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(Returns a vector of size ``num_velocities()`` containing the lower
velocity limits for every generalized velocity coordinate. These
include joint and free body coordinates. Any unbounded or unspecified
limits will be -infinity.

Raises:
    RuntimeError if called pre-finalize.)""";
        } GetVelocityLowerLimits;
        // Symbol: drake::multibody::MultibodyPlant::GetVelocityNames
        struct /* GetVelocityNames */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc_2args =
R"""(Returns a list of string names corresponding to each element of the
velocity vector. These strings take the form
``{model_instance_name}_{joint_name}_{joint_velocity_suffix}``, but
the prefix and suffix may optionally be withheld using
``add_model_instance_prefix`` and ``always_add_suffix``.

Parameter ``always_add_suffix``:
    (optional). If true, then the suffix is always added. If false,
    then the suffix is only added for joints that have more than one
    position (in this case, not adding would lead to ambiguity).

The returned names are guaranteed to be unique if
``add_model_instance_prefix`` is ``True`` (the default).

Raises:
    RuntimeError if the plant is not finalized.)""";
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc_3args =
R"""(Returns a list of string names corresponding to each element of the
velocity vector. These strings take the form
``{model_instance_name}_{joint_name}_{joint_velocity_suffix}``, but
the prefix and suffix may optionally be withheld using
``add_model_instance_prefix`` and ``always_add_suffix``.

Parameter ``always_add_suffix``:
    (optional). If true, then the suffix is always added. If false,
    then the suffix is only added for joints that have more than one
    position (in this case, not adding would lead to ambiguity).

The returned names are guaranteed to be unique.

Raises:
    RuntimeError if the plant is not finalized or if the
    ``model_instance`` is invalid.)""";
        } GetVelocityNames;
        // Symbol: drake::multibody::MultibodyPlant::GetVelocityUpperLimits
        struct /* GetVelocityUpperLimits */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(Upper limit analog of GetVelocitysLowerLimits(), where any unbounded
or unspecified limits will be +infinity.

See also:
    GetVelocityLowerLimits() for more information.)""";
        } GetVelocityUpperLimits;
        // Symbol: drake::multibody::MultibodyPlant::GetVisualGeometriesForBody
        struct /* GetVisualGeometriesForBody */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(Returns an array of GeometryId's identifying the different visual
geometries for ``body`` previously registered with a SceneGraph.

Note:
    This method can be called at any time during the lifetime of
    ``this`` plant, either pre- or post-finalize, see Finalize().
    Post-finalize calls will always return the same value.

See also:
    RegisterVisualGeometry(), Finalize())""";
        } GetVisualGeometriesForBody;
        // Symbol: drake::multibody::MultibodyPlant::HasBodyNamed
        struct /* HasBodyNamed */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc_1args =
R"""(Returns:
    ``True`` if a body named ``name`` was added to the MultibodyPlant.

See also:
    AddRigidBody().

Raises:
    RuntimeError if the body name occurs in multiple model instances.)""";
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc_2args =
R"""(Returns:
    ``True`` if a body named ``name`` was added to the MultibodyPlant
    in ``model_instance``.

See also:
    AddRigidBody().

Raises:
    RuntimeError if ``model_instance`` is not valid for this model.)""";
        } HasBodyNamed;
        // Symbol: drake::multibody::MultibodyPlant::HasFrameNamed
        struct /* HasFrameNamed */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc_1args =
R"""(Returns:
    ``True`` if a frame named ``name`` was added to the model.

See also:
    AddFrame().

Raises:
    RuntimeError if the frame name occurs in multiple model instances.)""";
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc_2args =
R"""(Returns:
    ``True`` if a frame named ``name`` was added to
    ``model_instance``.

See also:
    AddFrame().

Raises:
    RuntimeError if ``model_instance`` is not valid for this model.)""";
        } HasFrameNamed;
        // Symbol: drake::multibody::MultibodyPlant::HasJointActuatorNamed
        struct /* HasJointActuatorNamed */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc_1args =
R"""(Returns:
    ``True`` if an actuator named ``name`` was added to this model.

See also:
    AddJointActuator().

Raises:
    RuntimeError if the actuator name occurs in multiple model
    instances.)""";
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc_2args =
R"""(Returns:
    ``True`` if an actuator named ``name`` was added to
    ``model_instance``.

See also:
    AddJointActuator().

Raises:
    RuntimeError if ``model_instance`` is not valid for this model.)""";
        } HasJointActuatorNamed;
        // Symbol: drake::multibody::MultibodyPlant::HasJointNamed
        struct /* HasJointNamed */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc_1args =
R"""(Returns:
    ``True`` if a joint named ``name`` was added to this model.

See also:
    AddJoint().

Raises:
    RuntimeError if the joint name occurs in multiple model instances.)""";
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc_2args =
R"""(Returns:
    ``True`` if a joint named ``name`` was added to
    ``model_instance``.

See also:
    AddJoint().

Raises:
    RuntimeError if ``model_instance`` is not valid for this model.)""";
        } HasJointNamed;
        // Symbol: drake::multibody::MultibodyPlant::HasModelInstanceNamed
        struct /* HasModelInstanceNamed */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(Returns:
    ``True`` if a model instance named ``name`` was added to this
    model.

See also:
    AddModelInstance().)""";
        } HasModelInstanceNamed;
        // Symbol: drake::multibody::MultibodyPlant::HasUniqueFloatingBaseBody
        struct /* HasUniqueFloatingBaseBody */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(Returns true if there is a single base body in the model given by
``model_instance``, and that body is a floating base body.

See mbp_working_with_free_bodies "above for details".

Raises:
    RuntimeError if called pre-finalize.

Raises:
    RuntimeError if ``model_instance`` is not valid.

See also:
    GetUniqueFloatingBaseBodyOrThrow())""";
        } HasUniqueFloatingBaseBody;
        // Symbol: drake::multibody::MultibodyPlant::HasUniqueFreeBaseBody
        struct /* HasUniqueFreeBaseBody */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc_deprecated =
R"""((Deprecated.)

Deprecated:
    Use HasUniqueFloatingBaseBody() instead. This will be removed from
    Drake on or after 2026-06-01.)""";
        } HasUniqueFreeBaseBody;
        // Symbol: drake::multibody::MultibodyPlant::IsAnchored
        struct /* IsAnchored */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(Returns ``True`` if ``body`` is anchored (i.e. the kinematic path
between ``body`` and the world only contains weld joints.)

Raises:
    RuntimeError if called pre-finalize.)""";
        } IsAnchored;
        // Symbol: drake::multibody::MultibodyPlant::IsVelocityEqualToQDot
        struct /* IsVelocityEqualToQDot */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(Returns true iff the generalized velocity v is exactly the time
derivative q̇ of the generalized coordinates q. In this case
MapQDotToVelocity() and MapVelocityToQDot() implement the identity
map. This method is, in the worst case, O(n), where n is the number of
joints.)""";
        } IsVelocityEqualToQDot;
        // Symbol: drake::multibody::MultibodyPlant::MakeActuationMatrix
        struct /* MakeActuationMatrix */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(This method creates an actuation matrix B mapping a vector of
actuation values u into generalized forces ``tau_u = B * u``, where B
is a matrix of size ``nv x nu`` with ``nu`` equal to
num_actuated_dofs() and ``nv`` equal to num_velocities(). The vector u
of actuation values is of size num_actuated_dofs(). For a given
JointActuator, ``u[JointActuator::input_start()]`` stores the value
for the external actuation corresponding to that actuator. ``tau_u``
on the other hand is indexed by generalized velocity indices according
to ``Joint::velocity_start()``.

Warning:
    B is a permutation matrix. While making a permutation has ``O(n)``
    complexity, making a full B matrix has ``O(n²)`` complexity. For
    most applications this cost can be neglected but it could become
    significant for very large systems.)""";
        } MakeActuationMatrix;
        // Symbol: drake::multibody::MultibodyPlant::MakeActuationMatrixPseudoinverse
        struct /* MakeActuationMatrixPseudoinverse */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(Creates the pseudoinverse of the actuation matrix B directly (without
requiring an explicit inverse calculation). See MakeActuationMatrix().

Notably, when B is full row rank (the system is fully actuated), then
the pseudoinverse is a true inverse.)""";
        } MakeActuationMatrixPseudoinverse;
        // Symbol: drake::multibody::MultibodyPlant::MakeActuatorSelectorMatrix
        struct /* MakeActuatorSelectorMatrix */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc_1args_user_to_actuator_index_map =
R"""(This method allows user to map a vector ``uₛ`` containing the
actuation for a set of selected actuators into the vector u containing
the actuation values for ``this`` full model. The mapping, or
selection, is returned in the form of a selector matrix Su such that
``u = Su⋅uₛ``. The size nₛ of uₛ is always smaller or equal than the
size of the full vector of actuation values u. That is, a user might
be interested in only a given subset of actuators in the model.

This selection matrix is particularly useful when adding PID control
on a portion of the state, see systems::controllers::PidController.

A user specifies the preferred order in uₛ via
``user_to_actuator_index_map``. The actuation values in uₛ are a
concatenation of the values for each actuator in the order they appear
in ``user_to_actuator_index_map``. The actuation value in the full
vector of actuation values ``u`` for a particular actuator can be
found at offset JointActuator::input_start().)""";
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc_1args_user_to_joint_index_map =
R"""(Alternative signature to build an actuation selector matrix ``Su``
such that ``u = Su⋅uₛ``, where u is the vector of actuation values for
the full model (see get_actuation_input_port()) and uₛ is a vector of
actuation values for the actuators acting on the joints listed by
``user_to_joint_index_map``. It is assumed that all joints referenced
by ``user_to_joint_index_map`` are actuated. See
MakeActuatorSelectorMatrix(const std::vector<JointActuatorIndex>&) for
details.

Raises:
    RuntimeError if any of the joints in ``user_to_joint_index_map``
    does not have an actuator.)""";
        } MakeActuatorSelectorMatrix;
        // Symbol: drake::multibody::MultibodyPlant::MakeQDotToVelocityMap
        struct /* MakeQDotToVelocityMap */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(Returns the matrix ``N⁺(q)``, which maps ``v = N⁺(q)⋅q̇``, as
described in MapQDotToVelocity(). Prefer calling MapQDotToVelocity()
directly; this entry point is provided to support callers that require
the explicit linear form (once q is given) of the relationship. This
method is, in the worst case, O(n), where n is the number of joints.

Parameter ``context``:
    The context containing the state of the model.

See also:
    MapVelocityToQDot())""";
        } MakeQDotToVelocityMap;
        // Symbol: drake::multibody::MultibodyPlant::MakeStateSelectorMatrix
        struct /* MakeStateSelectorMatrix */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(This method allows users to map the state of ``this`` model, x, into a
vector of selected state xₛ with a given preferred ordering. The
mapping, or selection, is returned in the form of a selector matrix Sx
such that ``xₛ = Sx⋅x``. The size nₛ of xₛ is always smaller or equal
than the size of the full state x. That is, a user might be interested
in only a given portion of the full state x.

This selection matrix is particularly useful when adding PID control
on a portion of the state, see systems::controllers::PidController.

A user specifies the preferred order in xₛ via
``user_to_joint_index_map``. The selected state is built such that
selected positions are followed by selected velocities, as in ``xₛ =
[qₛ, vₛ]``. The positions in qₛ are a concatenation of the positions
for each joint in the order they appear in
``user_to_joint_index_map``. That is, the positions for
``user_to_joint_index_map[0]`` are first, followed by the positions
for ``user_to_joint_index_map[1]``, etc. Similarly for the selected
velocities vₛ.

Raises:
    RuntimeError if there are repeated indices in
    ``user_to_joint_index_map``.)""";
        } MakeStateSelectorMatrix;
        // Symbol: drake::multibody::MultibodyPlant::MakeVelocityToQDotMap
        struct /* MakeVelocityToQDotMap */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(Returns the matrix ``N(q)``, which maps ``q̇ = N(q)⋅v``, as described
in MapVelocityToQDot(). Prefer calling MapVelocityToQDot() directly;
this entry point is provided to support callers that require the
explicit linear form (once q is given) of the relationship. Do not
take the (pseudo-)inverse of ``N(q)``; call MakeQDotToVelocityMap
instead. This method is, in the worst case, O(n), where n is the
number of joints.

Parameter ``context``:
    The context containing the state of the model.

See also:
    MapVelocityToQDot())""";
        } MakeVelocityToQDotMap;
        // Symbol: drake::multibody::MultibodyPlant::MapQDotToVelocity
        struct /* MapQDotToVelocity */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(Transforms the time derivative ``qdot`` of the generalized positions
vector ``q`` (stored in ``context``) to generalized velocities ``v``.
`v` and ``q̇`` are related linearly by ``q̇ = N(q)⋅v``. Although
``N(q)`` is not necessarily square, its left pseudo-inverse ``N⁺(q)``
can be used to invert that relationship without residual error,
provided that ``qdot`` is in the range space of ``N(q)`` (that is, if
it *could* have been produced as ``q̇ = N(q)⋅v`` for some ``v``).
Using the configuration ``q`` stored in the given ``context`` this
method calculates ``v = N⁺(q)⋅q̇``.

Parameter ``context``:
    The context containing the state of the model.

Parameter ``qdot``:
    A vector containing the time derivatives of the generalized
    positions. This method aborts if ``qdot`` is not of size
    num_positions().

Parameter ``v``:
    A valid (non-null) pointer to a vector in ``ℛⁿ`` with n the number
    of generalized velocities. This method aborts if v is nullptr or
    if it is not of size num_velocities().

See also:
    MapVelocityToQDot()

See also:
    Mobilizer::MapQDotToVelocity())""";
        } MapQDotToVelocity;
        // Symbol: drake::multibody::MultibodyPlant::MapVelocityToQDot
        struct /* MapVelocityToQDot */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(Transforms generalized velocities v to time derivatives ``qdot`` of
the generalized positions vector ``q`` (stored in ``context``). `v`
and ``qdot`` are related linearly by ``q̇ = N(q)⋅v``. Using the
configuration ``q`` stored in the given ``context`` this method
calculates ``q̇ = N(q)⋅v``.

Parameter ``context``:
    The context containing the state of the model.

Parameter ``v``:
    A vector of generalized velocities for this model. This method
    aborts if v is not of size num_velocities().

Parameter ``qdot``:
    A valid (non-null) pointer to a vector in ``ℝⁿ`` with n being the
    number of generalized positions in this model, given by
    ``num_positions()``. This method aborts if ``qdot`` is nullptr or
    if it is not of size num_positions().

See also:
    MapQDotToVelocity()

See also:
    Mobilizer::MapVelocityToQDot())""";
        } MapVelocityToQDot;
        // Symbol: drake::multibody::MultibodyPlant::MultibodyPlant<T>
        struct /* ctor */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(This constructor creates a plant with a single "world" body.
Therefore, right after creation, num_bodies() returns one.

MultibodyPlant offers two different modalities to model mechanical
systems in time. These are: 1. As a discrete system with periodic
updates, ``time_step`` is strictly greater than zero. 2. As a
continuous system, ``time_step`` equals exactly zero.

Currently the discrete model is preferred for simulation given its
robustness and speed in problems with frictional contact. However this
might change as we work towards developing better strategies to model
contact. See multibody_simulation for further details.

Warning:
    Users should be aware of current limitations in either modeling
    modality. While the discrete model is often the preferred option
    for problems with frictional contact given its robustness and
    speed, it might become unstable when using large feedback gains,
    high damping or large external forcing. MultibodyPlant will throw
    an exception whenever the discrete solver is detected to fail.
    Conversely, the continuous modality has the potential to leverage
    the robustness and accuracy control provide by Drake's
    integrators. However thus far this has proved difficult in
    practice and especially due to poor performance.

Parameter ``time_step``:
    Indicates whether ``this`` plant is modeled as a continuous system
    (``time_step = 0``) or as a discrete system with periodic updates
    of period ``time_step > 0``. See multibody_simulation for further
    details.

Warning:
    Currently the continuous modality with ``time_step = 0`` does not
    support joint limits for simulation, these are ignored.
    MultibodyPlant prints a warning to console if joint limits are
    provided. If your simulation requires joint limits currently you
    must use a discrete MultibodyPlant model.

Raises:
    RuntimeError if ``time_step`` is negative.)""";
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc_copyconvert =
R"""(Scalar-converting copy constructor. See system_scalar_conversion.)""";
        } ctor;
        // Symbol: drake::multibody::MultibodyPlant::NumBodiesWithName
        struct /* NumBodiesWithName */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(Returns:
    The total number of bodies (across all model instances) with the
    given name.)""";
        } NumBodiesWithName;
        // Symbol: drake::multibody::MultibodyPlant::RegisterAsSourceForSceneGraph
        struct /* RegisterAsSourceForSceneGraph */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(Registers ``this`` plant to serve as a source for an instance of
SceneGraph. This registration allows MultibodyPlant to register
geometry with ``scene_graph`` for visualization and/or collision
queries. The string returned by ``this->get_name()`` is passed to
SceneGraph's RegisterSource, so it is highly recommended that you give
the plant a recognizable name before calling this. Successive
registration calls with SceneGraph **must** be performed on the same
instance to which the pointer argument ``scene_graph`` points to.
Failure to do so will result in runtime exceptions.

Parameter ``scene_graph``:
    A valid non nullptr to the SceneGraph instance for which ``this``
    plant will sever as a source, see SceneGraph documentation for
    further details.

Returns:
    the SourceId of ``this`` plant in ``scene_graph``. It can also
    later on be retrieved with get_source_id().

Raises:
    RuntimeError if called post-finalize.

Raises:
    RuntimeError if ``scene_graph`` is the nullptr.

Raises:
    RuntimeError if called more than once.)""";
        } RegisterAsSourceForSceneGraph;
        // Symbol: drake::multibody::MultibodyPlant::RegisterCollisionGeometry
        struct /* RegisterCollisionGeometry */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc_5args_body_X_BG_shape_name_properties =
R"""(Registers geometry in a SceneGraph with a given geometry::Shape to be
used for the contact modeling of a given ``body``. More than one
geometry can be registered with a body, in which case the body's
contact geometry is the union of all geometries registered to that
body.

Parameter ``body``:
    The body for which geometry is being registered.

Parameter ``X_BG``:
    The fixed pose of the geometry frame G in the body frame B.

Parameter ``shape``:
    The geometry::Shape used for collision and contact. E.g.:
    geometry::Sphere, geometry::Cylinder, etc.

Parameter ``properties``:
    The proximity properties associated with the collision geometry.

Raises:
    RuntimeError if called post-finalize.)""";
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc_5args_body_X_BG_shape_name_coulomb_friction =
R"""(Overload which specifies a single property: coulomb_friction.)""";
        } RegisterCollisionGeometry;
        // Symbol: drake::multibody::MultibodyPlant::RegisterVisualGeometry
        struct /* RegisterVisualGeometry */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc_5args_body_X_BG_shape_name_properties =
R"""(Registers geometry in a SceneGraph with a given geometry::Shape to be
used for visualization of a given ``body``. The perception properties
are a copy of the given ``properties``. If the resulting perception
properties do not include ("label", "id"), then it is added as
documented above.

See mbp_geometry "the overview" for more details.

Parameter ``body``:
    The body for which geometry is being registered.

Parameter ``X_BG``:
    The fixed pose of the geometry frame G in the body frame B.

Parameter ``shape``:
    The geometry::Shape used for visualization. E.g.:
    geometry::Sphere, geometry::Cylinder, etc.

Parameter ``name``:
    The name for the geometry. It must satisfy the requirements
    defined in drake::geometry::GeometryInstance.

Parameter ``properties``:
    The illustration properties for this geometry.

Raises:
    RuntimeError if called post-finalize.

Returns:
    the id for the registered geometry.)""";
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc_2args_body_geometry_instance =
R"""(Registers the given ``geometry_instance`` in a SceneGraph to be used
for visualization of a given ``body``.

The roles that ``geometry_instance`` gets assigned
(illustration/perception) in SceneGraph depend solely on the
properties that have *already* been assigned to ``geometry_instance``.
If *any* visual roles have been assigned, those will be the only roles
used. If *no* visual roles have been assigned, then both roles will be
assigned using the default set of property values.

If the registered geometry has the perception role, it will have the
("label", "id") property. Possibly assigned as documented above.

See mbp_geometry "the overview" for more details.

Parameter ``body``:
    The body for which geometry is being registered.

Parameter ``geometry_instance``:
    The geometry to associate with the visual appearance of ``body``.

Raises:
    RuntimeError if ``geometry_instance`` is null.

Raises:
    RuntimeError if called post-finalize.

Returns:
    the id for the registered geometry.)""";
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc_5args_body_X_BG_shape_name_diffuse_color =
R"""(Overload for visual geometry registration. The following properties
are set: - ("phong", "diffuse") = ``diffuse_color`` in both sets of
properties. - ("label", "id") in perception properties as documented
above.

See mbp_geometry "the overview" for more details.)""";
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc_4args_body_X_BG_shape_name =
R"""(Overload for visual geometry registration. The ("label", "id")
property is set in the perception properties (as documented above).

See mbp_geometry "the overview" for more details.)""";
        } RegisterVisualGeometry;
        // Symbol: drake::multibody::MultibodyPlant::RemoveAllJointActuatorEffortLimits
        struct /* RemoveAllJointActuatorEffortLimits */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(Removes the effort limits on all joint actuators. (In other words,
sets all effort limits to +∞.) This is a convenient way to obtain a
plant without any built-in effort limits, in case models loaded by the
Parser have unwanted limits.)""";
        } RemoveAllJointActuatorEffortLimits;
        // Symbol: drake::multibody::MultibodyPlant::RemoveConstraint
        struct /* RemoveConstraint */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(Removes the constraint ``id`` from the plant. Note that this will
*not* remove constraints registered directly with DeformableModel.

Raises:
    RuntimeError if the MultibodyPlant has already been finalized.

Raises:
    RuntimeError if ``id`` does not identify any multibody constraint
    in this plant.)""";
        } RemoveConstraint;
        // Symbol: drake::multibody::MultibodyPlant::RemoveJoint
        struct /* RemoveJoint */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(Removes and deletes ``joint`` from this MultibodyPlant. Any existing
references to ``joint`` will become invalid, and future calls to
``get_joint(joint_index)`` will throw an exception. Other elements of
the plant may depend on ``joint`` at the time of removal and should be
removed first. For example, a JointActuator that depends on ``joint``
should be removed with RemoveJointActuator(). Currently, we do not
provide joint dependency tracking for force elements or constraints,
so this function will throw an exception if there are *any* user-added
force elements or constraints in the plant.

Raises:
    RuntimeError if the plant is already finalized.

Raises:
    RuntimeError if the plant contains a non-zero number of user-added
    force elements or user-added constraints.

Raises:
    RuntimeError if ``joint`` has a dependent JointActuator.

See also:
    AddJoint()

Note:
    It is important to note that the JointIndex assigned to a joint is
    immutable. New joint indices are assigned in increasing order,
    even if a joint with a lower index has been removed. This has the
    consequence that when a joint is removed from the plant, the
    sequence ``[0, num_joints())`` is not necessarily the correct set
    of un-removed joint indices in the plant. Thus, it is important
    *NOT* to loop over joint indices sequentially from ``0`` to
    ``num_joints() - 1``. Instead users should use the provided
    GetJointIndices() and GetJointIndices(ModelIndex) functions:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    for (JointIndex index : plant.GetJointIndices()) {
      const Joint<double>& joint = plant.get_joint(index);
      ...
     }

.. raw:: html

    </details>)""";
        } RemoveJoint;
        // Symbol: drake::multibody::MultibodyPlant::RemoveJointActuator
        struct /* RemoveJointActuator */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(Removes and deletes ``actuator`` from this MultibodyPlant. Any
existing references to ``actuator`` will become invalid, and future
calls to ``get_joint_actuator(actuator_index)`` will throw an
exception.

Raises:
    RuntimeError if the plant is already finalized.

See also:
    AddJointActuator())""";
        } RemoveJointActuator;
        // Symbol: drake::multibody::MultibodyPlant::RenameModelInstance
        struct /* RenameModelInstance */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(Renames an existing model instance.

Parameter ``model_instance``:
    The instance to rename.

Parameter ``name``:
    A string that uniquely identifies the instance within ``this``
    model.

Raises:
    RuntimeError if called after Finalize().

Raises:
    RuntimeError if ``model_instance`` is not a valid index.

Raises:
    RuntimeError if HasModelInstanceNamed(``name``) is true.)""";
        } RenameModelInstance;
        // Symbol: drake::multibody::MultibodyPlant::SetActuationInArray
        struct /* SetActuationInArray */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(Given actuation values ``u_instance`` for the actuators in
``model_instance``, this function updates the actuation vector u for
the entire plant model to which this actuator belongs to. Refer to
mbp_actuation "Actuation" for further details.

Parameter ``u_instance``:
    Actuation values for the model instance. Values are ordered by
    monotonically increasing JointActuatorIndex within the model
    instance.

Parameter ``u``:
    Actuation values for the entire plant model. The actuation value
    in ``u`` for a particular actuator must be found at offset
    JointActuator::input_start(). Only values corresponding to
    ``model_instance`` are changed.

Raises:
    RuntimeError if the size of ``u_instance`` is not equal to the
    number of actuation inputs for the joints of ``model_instance``.)""";
        } SetActuationInArray;
        // Symbol: drake::multibody::MultibodyPlant::SetBaseBodyJointType
        struct /* SetBaseBodyJointType */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(Sets the type of joint to be used by Finalize() to connect any
otherwise unconnected bodies to World. Bodies connected by a joint to
World are called *base bodies* and are determined during Finalize()
when we build a forest structure to model the multibody system for
efficient computation. See mbp_working_with_free_bodies "Working with
free bodies" for a longer discussion.

This can be set globally or for a particular model instance. Global
options are used for any model elements that belong to model instances
for which no options have been set explicitly. The default is to use a
quaternion floating joint.

| BaseBodyJointType:: | Notes | | ------------------------ |
-------------------------------------- | | kQuaternionFloatingJoint |
6 dofs, unrestricted orientation | | kRpyFloatingJoint † | 6 dofs,
uses 3 angles for orientation | | kWeldJoint | 0 dofs, welded to World
("anchored") |

† The 3-angle orientation representation used by RpyFloatingJoint can
be easier to work with than a quaternion (especially for optimization)
but has a singular orientation which must be avoided (pitch angle near
90°).

Note:
    Reminder: if you aren't satisfied with the particular selection of
    joints here, you can always add an explicit joint to World with
    AddJoint().

Parameter ``joint_type``:
    The joint type to be used for base bodies.

Parameter ``model_instance``:
    (optional) the index of the model instance to which ``joint_type``
    is to be applied.

Raises:
    RuntimeError if called after Finalize().)""";
        } SetBaseBodyJointType;
        // Symbol: drake::multibody::MultibodyPlant::SetConstraintActiveStatus
        struct /* SetConstraintActiveStatus */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(Sets the active status of the constraint given by ``id`` in
``context``.

Raises:
    RuntimeError if the MultibodyPlant has not been finalized.

Raises:
    RuntimeError if ``context`` == nullptr

Raises:
    RuntimeError if ``id`` does not belong to any multibody constraint
    in ``context``.)""";
        } SetConstraintActiveStatus;
        // Symbol: drake::multibody::MultibodyPlant::SetDefaultFloatingBaseBodyPose
        struct /* SetDefaultFloatingBaseBodyPose */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(Provisionally records a default World pose for ``body``, to be used in
case ``body`` turns out to be a floating base body after Finalize().

This may be called pre- or post-Finalize(). Pre-Finalize() this is the
only way to set the default pose of a floating base body.
Post-Finalize(), a floating base body's default pose may be set either
by this function or by setting the default pose directly through the
Joint API applied to the automatically-added floating joint. The most
recent value set by either method will be used to initialize the
floating joint's coordinates in subsequently-created Contexts.

Warning:
    If this is called on a ``body`` that does *not* turn out to be a
    floating base body after Finalize(), it will have no effect other
    than to be echoed back in GetDefaultFloatingBaseBodyPose(); in
    particular it will not affect the initial state in a
    subsequently-created Context. Use the Joint API to set the default
    pose for any body that has an explicitly-defined joint to its
    parent body.

See mbp_working_with_free_bodies "above for details".

Parameter ``body``:
    RigidBody whose default pose will be set if it turns out to be a
    floating base body.

Parameter ``X_WB``:
    Default pose of the floating base body in the World frame.)""";
        } SetDefaultFloatingBaseBodyPose;
        // Symbol: drake::multibody::MultibodyPlant::SetDefaultFreeBodyPose
        struct /* SetDefaultFreeBodyPose */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc_deprecated =
R"""((Deprecated.)

Deprecated:
    Use SetDefaultFloatingBaseBodyPose() instead. This will be removed
    from Drake on or after 2026-06-01.)""";
        } SetDefaultFreeBodyPose;
        // Symbol: drake::multibody::MultibodyPlant::SetDefaultPositions
        struct /* SetDefaultPositions */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc_1args =
R"""(Sets the default positions for the plant. Calls to
CreateDefaultContext or SetDefaultContext/SetDefaultState will return
a Context populated with these position values. They have no other
effects on the dynamics of the system.

Raises:
    RuntimeError if the plant is not finalized, if q is not of size
    num_positions(), or ``q`` contains non-finite values.)""";
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc_2args =
R"""(Sets the default positions for the model instance. Calls to
CreateDefaultContext or SetDefaultContext/SetDefaultState will return
a Context populated with these position values. They have no other
effects on the dynamics of the system.

Raises:
    RuntimeError if the plant is not finalized, if the model_instance
    is invalid, if the length of ``q_instance`` is not equal to
    ``num_positions(model_instance)``, or if ``q_instance`` contains
    non-finite values.)""";
        } SetDefaultPositions;
        // Symbol: drake::multibody::MultibodyPlant::SetDefaultState
        struct /* SetDefaultState */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(Sets ``state`` according to defaults set by the user for joints (e.g.
RevoluteJoint::set_default_angle()) and free bodies
(SetDefaultFreeBodyPose()). If the user does not specify defaults, the
state corresponds to zero generalized positions and velocities.

Raises:
    RuntimeError if called pre-finalize. See Finalize().)""";
        } SetDefaultState;
        // Symbol: drake::multibody::MultibodyPlant::SetDiscreteUpdateManager
        struct /* SetDiscreteUpdateManager */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(For use only by advanced developers wanting to try out their custom
time stepping strategies, including contact resolution.

Warning:
    This feature is considered to be **experimental** and may change
    or be removed at any time, without any deprecation notice ahead of
    time.

With this method MultibodyPlant takes ownership of ``manager``.

Note:
    Setting a contact manager bypasses the mechanism to set a
    different contact solver with SetContactSolver(). Use only one of
    these two experimental mechanisms but never both.

Parameter ``manager``:
    After this call the new manager is used to advance discrete
    states.

Precondition:
    this MultibodyPlant is discrete.

Precondition:
    manager != nullptr.

Raises:
    RuntimeError if called pre-finalize. See Finalize().)""";
        } SetDiscreteUpdateManager;
        // Symbol: drake::multibody::MultibodyPlant::SetDistanceConstraintParams
        struct /* SetDistanceConstraintParams */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(Stores in ``context`` the parameters ``params`` for the distance
constraint with identifier ``id``.

Parameter ``out``:
    ] context The plant's context. On output it stores ``params`` for
    the requested distance constraint.

Parameter ``id``:
    Unique identifier of the constraint.

Parameter ``params``:
    The new set of parameters to be stored in ``context``.

Raises:
    if ``id`` is not a valid identifier for a distance constraint.

Raises:
    if params.bodyA() or params.bodyB() do not correspond to rigid
    bodies in ``this`` MultibodyPlant.)""";
        } SetDistanceConstraintParams;
        // Symbol: drake::multibody::MultibodyPlant::SetFloatingBaseBodyPoseInAnchoredFrame
        struct /* SetFloatingBaseBodyPoseInAnchoredFrame */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(Updates ``context`` to store the World-frame pose of floating base
body B, given its pose ``X_FB`` in an arbitrary anchored frame F.

Frame F must be *anchored*, meaning that it is either on a body which
is directly welded to a frame on the World body, or more generally,
that it is on a body for which there is a kinematic path between that
body and the world body that only includes weld joints.

Warning:
    The World-frame pose is calculated here and stored in ``context``.
    Moving F subsequently will not change the stored pose unless you
    call this method again.

See mbp_working_with_free_bodies "above for details".

Raises:
    RuntimeError if called pre-finalize.

Raises:
    RuntimeError if frame F is not anchored to the world.

Raises:
    RuntimeError if ``body`` is not a floating base body.)""";
        } SetFloatingBaseBodyPoseInAnchoredFrame;
        // Symbol: drake::multibody::MultibodyPlant::SetFloatingBaseBodyPoseInWorldFrame
        struct /* SetFloatingBaseBodyPoseInWorldFrame */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(Updates ``context`` to store the pose ``X_WB`` of a given floating
base body B's body frame in the World frame W.

See mbp_working_with_free_bodies "above for details".

Raises:
    RuntimeError if called pre-finalize.

Raises:
    RuntimeError if ``body`` is not a floating base body.)""";
        } SetFloatingBaseBodyPoseInWorldFrame;
        // Symbol: drake::multibody::MultibodyPlant::SetFreeBodyPose
        struct /* SetFreeBodyPose */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc_3args =
R"""(For any free body's 6-dof joint, sets ``context`` to store the pose
X_JpJc of child frame Jc in its parent frame Jp.

Note:
    Unless ``body`` is a floating base body, the parent frame Jp is
    not necessarily the World frame W, and the child frame Jc is not
    necessarily the body frame B. For a floating base body B, this
    method sets X_WB, the pose of body B in World.

See mbp_working_with_free_bodies "above for details".

Raises:
    RuntimeError if called pre-finalize.

Raises:
    RuntimeError if ``body`` is not a free body.)""";
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc_4args =
R"""((Advanced) Variant of SetFreeBodyPose() that writes to a given
``state`` rather than directly to the Context.

Precondition:
    ``state`` comes from this MultibodyPlant.)""";
        } SetFreeBodyPose;
        // Symbol: drake::multibody::MultibodyPlant::SetFreeBodyPoseInAnchoredFrame
        struct /* SetFreeBodyPoseInAnchoredFrame */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc_deprecated =
R"""((Deprecated.)

Deprecated:
    Use SetFloatingBaseBodyPoseInAnchoredFrame() instead. This will be
    removed from Drake on or after 2026-06-01.)""";
        } SetFreeBodyPoseInAnchoredFrame;
        // Symbol: drake::multibody::MultibodyPlant::SetFreeBodyPoseInWorldFrame
        struct /* SetFreeBodyPoseInWorldFrame */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc_deprecated =
R"""((Deprecated.)

Deprecated:
    Use SetFloatingBaseBodyPoseInWorldFrame() instead. This will be
    removed from Drake on or after 2026-06-01.)""";
        } SetFreeBodyPoseInWorldFrame;
        // Symbol: drake::multibody::MultibodyPlant::SetFreeBodyRandomAnglesDistribution
        struct /* SetFreeBodyRandomAnglesDistribution */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(For any free body's 6-dof joint, sets the distribution used by
SetRandomState() to populate the orientation of its child frame Jc
with respect to its parent frame Jp, expressed with roll-pitch-yaw
angles. Requires that the free body is modeled using an
RpyFloatingJoint.

Note:
    Unless ``body`` is a floating base body, the parent frame Jp is
    not necessarily the World frame W, and the child frame Jc is not
    necessarily the body frame B. For a floating base body B, this
    method sets the distribution of R_WB, the orientation of body B's
    frame in World (as roll-pitch-yaw angles).

Note:
    This distribution is not uniform over the sphere reachable by the
    three angles. For a uniform alternative, switch the joint to
    QuaternionFloatingJoint and use
    SetFreeBodyRandomRotationDistributionToUniform().

See mbp_working_with_free_bodies "above for details".

Raises:
    RuntimeError if called pre-finalize.

Raises:
    RuntimeError if ``body`` is not a free body in the model.

Raises:
    RuntimeError if the free body is not modeled with an
    RpyFloatingJoint.

See also:
    SetFreeBodyRandomRotationDistribution() for a free body that is
    modeled using a QuaternionFloatingJoint.

See also:
    SetBaseBodyJointType() for control over the type of automatically-
    added joints.)""";
        } SetFreeBodyRandomAnglesDistribution;
        // Symbol: drake::multibody::MultibodyPlant::SetFreeBodyRandomRotationDistribution
        struct /* SetFreeBodyRandomRotationDistribution */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(For any free body's 6-dof joint, sets the distribution used by
SetRandomState() to populate the orientation of its child frame Jc
with respect to its parent frame Jp, expressed as a quaternion.
Requires that the free body is modeled using a
QuaternionFloatingJoint.

Note:
    Unless ``body`` is a floating base body, the parent frame Jp is
    not necessarily the World frame W, and the child frame Jc is not
    necessarily the body frame B. For a floating base body B, this
    method sets the distribution of R_WB, the orientation of body B's
    frame in World (as a quaternion).

Note:
    This distribution is not necessarily uniform over the sphere
    reachable by the quaternion; that depends on the quaternion
    expression provided in ``rotation``. See
    SetFreeBodyRandomRotationDistributionToUniform() for a uniform
    alternative.

See mbp_working_with_free_bodies "above for details".

Raises:
    RuntimeError if called pre-finalize.

Raises:
    RuntimeError if ``body`` is not a free body.

Raises:
    RuntimeError if the free body is not modeled with a
    QuaternionFloatingJoint.

See also:
    SetFreeBodyRandomAnglesDistribution() for a free body that is
    modeled using an RpyFloatingJoint.

See also:
    SetBaseBodyJointType() for control over the type of automatically-
    added joints.)""";
        } SetFreeBodyRandomRotationDistribution;
        // Symbol: drake::multibody::MultibodyPlant::SetFreeBodyRandomRotationDistributionToUniform
        struct /* SetFreeBodyRandomRotationDistributionToUniform */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(For any free body's 6-dof joint, sets the distribution used by
SetRandomState() to populate the orientation of its child frame Jc
with respect to its parent frame Jp using uniformly random rotations
(expressed as a quaternion). Requires that the free body is modeled
using a QuaternionFloatingJoint.

Note:
    Unless ``body`` is a floating base body, the parent frame Jp is
    not necessarily the World frame W, and the child frame Jc is not
    necessarily the body frame B. For a floating base body B, this
    method sets the distribution of R_WB, the orientation of body B's
    frame in World (as a quaternion).

See mbp_working_with_free_bodies "above for details".

Raises:
    RuntimeError if called pre-finalize.

Raises:
    RuntimeError if ``body`` is not a free body in the model.

Raises:
    RuntimeError if the free body is not modeled with a
    QuaternionFloatingJoint.

See also:
    SetFreeBodyRandomAnglesDistribution() for a free body that is
    modeled using an RpyFloatingJoint.

See also:
    SetBaseBodyJointType() for control over the type of automatically-
    added joints.)""";
        } SetFreeBodyRandomRotationDistributionToUniform;
        // Symbol: drake::multibody::MultibodyPlant::SetFreeBodyRandomTranslationDistribution
        struct /* SetFreeBodyRandomTranslationDistribution */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(For any free body's 6-dof joint, sets the distribution used by
SetRandomState() to populate the x-y-z ``translation`` of its child
frame Jc with respect to its parent frame Jp.

Note:
    Unless ``body`` is a floating base body, the parent frame Jp is
    not necessarily the World frame W, and the child frame Jc is not
    necessarily the body frame B. For a floating base body B, this
    method sets the distribution of p_WBo, the position of body B's
    frame origin Bo in World.

See mbp_working_with_free_bodies "above for details".

Raises:
    RuntimeError if called pre-finalize.

Raises:
    RuntimeError if ``body`` is not a free body.)""";
        } SetFreeBodyRandomTranslationDistribution;
        // Symbol: drake::multibody::MultibodyPlant::SetFreeBodySpatialVelocity
        struct /* SetFreeBodySpatialVelocity */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc_3args =
R"""(For any free body's 6-dof joint, sets ``context`` to store the spatial
velocity V_JpJc of child frame Jc in its parent frame Jp.

Note:
    Unless ``body`` is a floating base body, the parent frame Jp is
    not necessarily the World frame W, and the child frame Jc is not
    necessarily the body frame B. For a floating base body B, this
    method sets V_WB, the spatial velocity of body B in World.

See mbp_working_with_free_bodies "above for details".

Raises:
    RuntimeError if called pre-finalize.

Raises:
    RuntimeError if ``body`` is not a free body.)""";
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc_4args =
R"""((Advanced) Variant of SetFreeBodySpatialVelocity() that writes to a
given ``state`` rather than directly to the Context.

Precondition:
    ``state`` comes from this MultibodyPlant.)""";
        } SetFreeBodySpatialVelocity;
        // Symbol: drake::multibody::MultibodyPlant::SetPositions
        struct /* SetPositions */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc_2args =
R"""(Sets the generalized positions q in a given Context from a given
vector. Prefer this method over GetMutablePositions().

Raises:
    RuntimeError if ``context`` is nullptr, if ``context`` does not
    correspond to the Context for a multibody model, if the length of
    ``q`` is not equal to ``num_positions()``, or if ``q`` contains
    non-finite values.)""";
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc_3args =
R"""(Sets the generalized positions q for a particular model instance in a
given Context from a given vector.

Raises:
    RuntimeError if the ``context`` is nullptr, if ``context`` does
    not correspond to the Context for a multibody model, if the model
    instance index is invalid, if the length of ``q_instance`` is not
    equal to ``num_positions(model_instance)``, or if ``q_instance``
    contains non-finite values.)""";
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc_4args =
R"""((Advanced) Sets the generalized positions q for a particular model
instance in a given State from a given vector.

Note:
    No cache invalidation occurs.

Raises:
    RuntimeError if the ``context`` is nullptr, if ``context`` does
    not correspond to the Context for a multibody model, if the model
    instance index is invalid, if the length of ``q_instance`` is not
    equal to ``num_positions(model_instance)``, or if ``q_instance``
    contains non-finite values.

Precondition:
    ``state`` comes from this MultibodyPlant.)""";
        } SetPositions;
        // Symbol: drake::multibody::MultibodyPlant::SetPositionsAndVelocities
        struct /* SetPositionsAndVelocities */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc_2args =
R"""(Sets generalized positions q and generalized velocities v in a given
Context from a given vector [q; v]. Prefer this method over
GetMutablePositionsAndVelocities().

Raises:
    RuntimeError if ``context`` is nullptr, if ``context`` does not
    correspond to the context for a multibody model, if the length of
    ``q_v`` is not equal to ``num_positions() + num_velocities()``, or
    if ``q_v`` contains non-finite values.)""";
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc_3args =
R"""(Sets generalized positions q and generalized velocities v from a given
vector [q; v] for a specified model instance in a given Context.

Raises:
    RuntimeError if ``context`` is nullptr, if ``context`` does not
    correspond to the Context for a multibody model, if the model
    instance index is invalid, if the length of ``q_v`` is not equal
    to ``num_positions(model_instance) +
    num_velocities(model_instance)``, or if ``q_v`` contains
    non-finite values.)""";
        } SetPositionsAndVelocities;
        // Symbol: drake::multibody::MultibodyPlant::SetPositionsInArray
        struct /* SetPositionsInArray */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(Sets the vector of generalized positions for ``model_instance`` in
``q`` using ``q_instance``, leaving all other elements in the array
untouched. This method throws an exception if ``q`` is not of size
MultibodyPlant::num_positions() or ``q_instance`` is not of size
``MultibodyPlant::num_positions(model_instance)``.)""";
        } SetPositionsInArray;
        // Symbol: drake::multibody::MultibodyPlant::SetRandomState
        struct /* SetRandomState */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(Assigns random values to all elements of the state, by drawing samples
independently for each joint/free body (coming soon: and then solving
a mathematical program to "project" these samples onto the registered
system constraints). If a random distribution is not specified for a
joint/free body, the default state is used.

See also:
    stochastic_systems)""";
        } SetRandomState;
        // Symbol: drake::multibody::MultibodyPlant::SetUseSampledOutputPorts
        struct /* SetUseSampledOutputPorts */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""((Advanced) For a discrete-time plant, configures whether the output
ports are sampled (the default) or live (opt-in). See
output_port_sampling "Output port sampling" for details.

Raises:
    RuntimeError if the plant is already finalized.

Raises:
    RuntimeError if ``use_sampled_output_ports`` is ``True`` but
    ``this`` MultibodyPlant is not a discrete model (is_discrete() ==
    false).)""";
        } SetUseSampledOutputPorts;
        // Symbol: drake::multibody::MultibodyPlant::SetVelocities
        struct /* SetVelocities */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc_2args =
R"""(Sets the generalized velocities v in a given Context from a given
vector. Prefer this method over GetMutableVelocities().

Raises:
    RuntimeError if the ``context`` is nullptr, if the context does
    not correspond to the context for a multibody model, if the length
    of ``v`` is not equal to ``num_velocities()``, or if ``v``
    contains non-finite values.)""";
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc_3args =
R"""(Sets the generalized velocities v for a particular model instance in a
given Context from a given vector.

Raises:
    RuntimeError if the ``context`` is nullptr, if ``context`` does
    not correspond to the Context for a multibody model, if the model
    instance index is invalid, if the length of ``v_instance`` is not
    equal to ``num_velocities(model_instance)``, or if ``v_instance``
    contains non-finite values.)""";
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc_4args =
R"""((Advanced) Sets the generalized velocities v for a particular model
instance in a given State from a given vector.

Note:
    No cache invalidation occurs.

Raises:
    RuntimeError if the ``context`` is nullptr, if ``context`` does
    not correspond to the Context for a multibody model, if the model
    instance index is invalid, if the length of ``v_instance`` is not
    equal to ``num_velocities(model_instance)``, or if ``v_instance``
    contains non-finite values.

Precondition:
    ``state`` comes from this MultibodyPlant.)""";
        } SetVelocities;
        // Symbol: drake::multibody::MultibodyPlant::SetVelocitiesInArray
        struct /* SetVelocitiesInArray */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(Sets the vector of generalized velocities for ``model_instance`` in
``v`` using ``v_instance``, leaving all other elements in the array
untouched. This method throws an exception if ``v`` is not of size
MultibodyPlant::num_velocities(), ``v_instance`` is not of size
``MultibodyPlant::num_positions(model_instance)``, or ``v_instance``
contains non-finite values.)""";
        } SetVelocitiesInArray;
        // Symbol: drake::multibody::MultibodyPlant::WeldFrames
        struct /* WeldFrames */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(Welds ``frame_on_parent_F`` and ``frame_on_child_M`` with relative
pose ``X_FM``. That is, the pose of frame M in frame F is fixed, with
value ``X_FM``. If ``X_FM`` is omitted, the identity transform will be
used. The call to this method creates and adds a new WeldJoint to the
model. The new WeldJoint is named as: frame_on_parent_F.name() +
"_welds_to_" + frame_on_child_M.name().

Returns:
    a constant reference to the WeldJoint welding frames F and M.

Raises:
    RuntimeError if the weld produces a duplicate joint name.)""";
        } WeldFrames;
        // Symbol: drake::multibody::MultibodyPlant::deformable_model
        struct /* deformable_model */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(Returns the DeformableModel owned by this plant.

Warning:
    This feature is considered to be **experimental** and may change
    or be removed at any time, without any deprecation notice ahead of
    time.)""";
        } deformable_model;
        // Symbol: drake::multibody::MultibodyPlant::geometry_source_is_registered
        struct /* geometry_source_is_registered */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(Returns ``True`` if ``this`` MultibodyPlant was registered with a
SceneGraph. This method can be called at any time during the lifetime
of ``this`` plant to query if ``this`` plant has been registered with
a SceneGraph, either pre- or post-finalize, see Finalize().)""";
        } geometry_source_is_registered;
        // Symbol: drake::multibody::MultibodyPlant::get_actuation_input_port
        struct /* get_actuation_input_port */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc_0args =
R"""(Returns a constant reference to the input port for external actuation
for all actuated dofs. This input port is a vector valued port and can
be set with JointActuator::set_actuation_vector(). The actuation value
for a particular actuator can be found at offset
JointActuator::input_start() in this vector. Refer to mbp_actuation
"Actuation" for further details.

Precondition:
    Finalize() was already called on ``this`` plant.

Raises:
    RuntimeError if called before Finalize().)""";
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc_1args =
R"""(Returns a constant reference to the input port for external actuation
for a specific model instance. This is a vector valued port with
entries ordered by monotonically increasing JointActuatorIndex within
``model_instance``. Refer to mbp_actuation "Actuation" for further
details.

Every model instance in ``this`` plant model has an actuation input
port, even if zero sized (for model instance with no actuators).

See also:
    GetJointActuatorIndices(), GetActuatedJointIndices().

Precondition:
    Finalize() was already called on ``this`` plant.

Raises:
    RuntimeError if called before Finalize().

Raises:
    RuntimeError if the model instance does not exist.)""";
        } get_actuation_input_port;
        // Symbol: drake::multibody::MultibodyPlant::get_adjacent_bodies_collision_filters
        struct /* get_adjacent_bodies_collision_filters */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(Returns whether to apply collision filters to topologically adjacent
bodies at Finalize() time.)""";
        } get_adjacent_bodies_collision_filters;
        // Symbol: drake::multibody::MultibodyPlant::get_applied_generalized_force_input_port
        struct /* get_applied_generalized_force_input_port */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(Returns a constant reference to the vector-valued input port for
applied generalized forces, and the vector will be added directly into
``tau`` (see mbp_equations_of_motion "System dynamics"). This vector
is ordered using the same convention as the plant velocities: you can
set the generalized forces that will be applied to model instance i
using, e.g., ``SetVelocitiesInArray(i, model_forces, &force_array)``.

Raises:
    RuntimeError if called before Finalize().)""";
        } get_applied_generalized_force_input_port;
        // Symbol: drake::multibody::MultibodyPlant::get_applied_spatial_force_input_port
        struct /* get_applied_spatial_force_input_port */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(Returns a constant reference to the input port for applying spatial
forces to bodies in the plant. The data type for the port is an
std::vector of ExternallyAppliedSpatialForce; any number of spatial
forces can be applied to any number of bodies in the plant.)""";
        } get_applied_spatial_force_input_port;
        // Symbol: drake::multibody::MultibodyPlant::get_ball_constraint_specs
        struct /* get_ball_constraint_specs */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc_1args =
R"""((Internal use only) Returns the ball constraint specification
corresponding to ``id``

Raises:
    if ``id`` is not a valid identifier for a ball constraint.)""";
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc_0args =
R"""((Internal use only) Returns a reference to all of the ball constraints
in this plant as a map from MultibodyConstraintId to
BallConstraintSpec.)""";
        } get_ball_constraint_specs;
        // Symbol: drake::multibody::MultibodyPlant::get_body
        struct /* get_body */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(Returns a constant reference to the body with unique index
``body_index``.

Raises:
    RuntimeError if ``body_index`` does not correspond to a body in
    this model.)""";
        } get_body;
        // Symbol: drake::multibody::MultibodyPlant::get_body_poses_output_port
        struct /* get_body_poses_output_port */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(Returns the output port of all body poses in the world frame. You can
obtain the pose ``X_WB`` of a body B in the world frame W with:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    const auto& X_WB_all = plant.get_body_poses_output_port().
          .Eval<std::vector<math::RigidTransform<double>>>(plant_context);
      const BodyIndex arm_body_index = plant.GetBodyByName("arm").index();
      const math::RigidTransform<double>& X_WArm = X_WB_all[arm_body_index];

.. raw:: html

    </details>

As shown in the example above, the resulting ``std::vector`` of body
poses is indexed by BodyIndex, and it has size num_bodies(). BodyIndex
"zero" (0) always corresponds to the world body, with pose equal to
the identity at all times.

Raises:
    RuntimeError if called pre-finalize.)""";
        } get_body_poses_output_port;
        // Symbol: drake::multibody::MultibodyPlant::get_body_spatial_accelerations_output_port
        struct /* get_body_spatial_accelerations_output_port */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(Returns the output port of all body spatial accelerations in the world
frame. You can obtain the spatial acceleration ``A_WB`` of a body B
(for point Bo, the body's origin) in the world frame W with:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    const auto& A_WB_all =
      plant.get_body_spatial_accelerations_output_port().
          .Eval<std::vector<SpatialAcceleration<double>>>(plant_context);
      const BodyIndex arm_body_index = plant.GetBodyByName("arm").index();
      const SpatialVelocity<double>& A_WArm = A_WB_all[arm_body_index];

.. raw:: html

    </details>

As shown in the example above, the resulting ``std::vector`` of body
spatial accelerations is indexed by BodyIndex, and it has size
num_bodies(). BodyIndex "zero" (0) always corresponds to the world
body, with zero spatial acceleration at all times.

In a discrete-time plant, the use_sampled_output_ports setting affects
the output of this port. See output_port_sampling "Output port
sampling" for details. When sampling is enabled and the plant has not
yet taken a step, the output value will be all zeros.

Raises:
    RuntimeError if called pre-finalize.)""";
        } get_body_spatial_accelerations_output_port;
        // Symbol: drake::multibody::MultibodyPlant::get_body_spatial_velocities_output_port
        struct /* get_body_spatial_velocities_output_port */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(Returns the output port of all body spatial velocities in the world
frame. You can obtain the spatial velocity ``V_WB`` of a body B in the
world frame W with:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    const auto& V_WB_all = plant.get_body_spatial_velocities_output_port().
          .Eval<std::vector<SpatialVelocity<double>>>(plant_context);
      const BodyIndex arm_body_index = plant.GetBodyByName("arm").index();
      const SpatialVelocity<double>& V_WArm = V_WB_all[arm_body_index];

.. raw:: html

    </details>

As shown in the example above, the resulting ``std::vector`` of body
spatial velocities is indexed by BodyIndex, and it has size
num_bodies(). BodyIndex "zero" (0) always corresponds to the world
body, with zero spatial velocity at all times.

Raises:
    RuntimeError if called pre-finalize.)""";
        } get_body_spatial_velocities_output_port;
        // Symbol: drake::multibody::MultibodyPlant::get_contact_model
        struct /* get_contact_model */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(Returns the model used for contact. See documentation for
ContactModel.)""";
        } get_contact_model;
        // Symbol: drake::multibody::MultibodyPlant::get_contact_penalty_method_time_scale
        struct /* get_contact_penalty_method_time_scale */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(Returns a time-scale estimate ``tc`` based on the requested
penetration allowance δ set with set_penetration_allowance(). For the
compliant contact model to enforce non-penetration, this time scale
relates to the time it takes the relative normal velocity between two
bodies to go to zero. This time scale ``tc`` is a global estimate of
the dynamics introduced by the compliant contact model and goes to
zero in the limit to ideal rigid contact. Since numerical integration
methods for continuum systems must be able to resolve a system's
dynamics, the time step used by an integrator must in general be much
smaller than the time scale ``tc``. How much smaller will depend on
the details of the problem and the convergence characteristics of the
integrator and should be tuned appropriately. Another factor to take
into account for setting up the simulation's time step is the speed of
the objects in your simulation. If ``vn`` represents a reference
velocity scale for the normal relative velocity between bodies, the
new time scale ``tn = δ / vn`` represents the time it would take for
the distance between two bodies approaching with relative normal
velocity ``vn`` to decrease by the penetration_allowance δ. In this
case a user should choose a time step for simulation that can resolve
the smallest of the two time scales ``tc`` and ``tn``.)""";
        } get_contact_penalty_method_time_scale;
        // Symbol: drake::multibody::MultibodyPlant::get_contact_results_output_port
        struct /* get_contact_results_output_port */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(Returns a constant reference to the port that outputs ContactResults.

In a discrete-time plant, the use_sampled_output_ports setting affects
the output of this port. See output_port_sampling "Output port
sampling" for details. When sampling is enabled and the plant has not
yet taken a step, the output value will be empty (no contacts).

Raises:
    RuntimeError if called pre-finalize, see Finalize().)""";
        } get_contact_results_output_port;
        // Symbol: drake::multibody::MultibodyPlant::get_contact_surface_representation
        struct /* get_contact_surface_representation */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(Gets the current representation of contact surfaces used by ``this``
MultibodyPlant.)""";
        } get_contact_surface_representation;
        // Symbol: drake::multibody::MultibodyPlant::get_coupler_constraint_specs
        struct /* get_coupler_constraint_specs */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc_1args =
R"""((Internal use only) Returns the coupler constraint specification
corresponding to ``id``

Raises:
    if ``id`` is not a valid identifier for a coupler constraint.)""";
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc_0args =
R"""((Internal use only) Returns a reference to the all of the coupler
constraints in this plant as a map from MultibodyConstraintId to
CouplerConstraintSpec.)""";
        } get_coupler_constraint_specs;
        // Symbol: drake::multibody::MultibodyPlant::get_deformable_body_configuration_output_port
        struct /* get_deformable_body_configuration_output_port */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(Returns the output port for vertex positions (configurations),
measured and expressed in the World frame, of the deformable bodies in
``this`` plant as a GeometryConfigurationVector.)""";
        } get_deformable_body_configuration_output_port;
        // Symbol: drake::multibody::MultibodyPlant::get_desired_state_input_port
        struct /* get_desired_state_input_port */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(For models with PD controlled joint actuators, returns the port to
provide the desired state for the given ``model_instance``. Refer to
mbp_actuation "Actuation" for further details.

For consistency with get_actuation_input_port(), each model instance
in ``this`` plant model has a desired states input port, even if zero
sized (for model instance with no actuators.)

Note:
    This port always has size 2 * num_actuators(model_instance), where
    we assume 1-DOF actuated joints. This port must provide one
    desired position and one desired velocity per joint actuator,
    packed as xd = [qd, vd], with positions and velocities in order of
    increasing JointActuatorIndex. Only desired states corresponding
    to PD-controlled actuators on non-locked joints
    (Joint::is_locked()) are used, the rest are ignored. That is PD
    control on just a subset of actuators is allowed.

Note:
    The desired state input port for a given model instance is not
    required to be connected. If disconnected, the controllers for
    such model instance will be *disarmed*. Refer to
    pd_controllers_and_ports for further details.

As an example of this structure, consider the following code to fix
desired states input values:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    MultibodyPlant<double> plant;
    // ... Load/parse plant model ...
    plant.Finalize();
    auto context = plant.CreateDefaultContext();
    const int num_u = plant.num_actuators(model_instance);
    const VectorXd model_xd(2 * num_u);
    auto model_qd = model_xd.head(num_u);
    auto model_vd = model_xd.tail(num_u);
    
    int a = 0;
    // Specify qd and vd in increasing order of JointActuatorIndex, as
    // returned by GetJointActuatorIndices().
    for (const JointActuatorIndex actuator_index :
        plant.GetJointActuatorIndices(model_instance)) {
      qd[a] = .... desired q value for actuator_index
      vd[a] = .... desired v value for actuator_index
      ++a;
    }
    // As an example, fix values in the context.
    plant.get_desired_state_input_port(model_instance).FixValue(
        &plant_context, model_xd);

.. raw:: html

    </details>)""";
        } get_desired_state_input_port;
        // Symbol: drake::multibody::MultibodyPlant::get_discrete_contact_approximation
        struct /* get_discrete_contact_approximation */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(Returns:
    the discrete contact solver approximation.)""";
        } get_discrete_contact_approximation;
        // Symbol: drake::multibody::MultibodyPlant::get_discrete_contact_solver
        struct /* get_discrete_contact_solver */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(Returns the contact solver type used for discrete MultibodyPlant
models.)""";
        } get_discrete_contact_solver;
        // Symbol: drake::multibody::MultibodyPlant::get_force_element
        struct /* get_force_element */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(Returns a constant reference to the force element with unique index
``force_element_index``.

Raises:
    RuntimeError when ``force_element_index`` does not correspond to a
    force element in this model.)""";
        } get_force_element;
        // Symbol: drake::multibody::MultibodyPlant::get_frame
        struct /* get_frame */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(Returns a constant reference to the frame with unique index
``frame_index``.

Raises:
    RuntimeError if ``frame_index`` does not correspond to a frame in
    this plant.)""";
        } get_frame;
        // Symbol: drake::multibody::MultibodyPlant::get_generalized_acceleration_output_port
        struct /* get_generalized_acceleration_output_port */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc_0args =
R"""(Returns a constant reference to the output port for generalized
accelerations v̇ of the model.

In a discrete-time plant, the use_sampled_output_ports setting affects
the output of this port. See output_port_sampling "Output port
sampling" for details. When sampling is enabled and the plant has not
yet taken a step, the output value will be all zeros.

Precondition:
    Finalize() was already called on ``this`` plant.

Raises:
    RuntimeError if called before Finalize().)""";
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc_1args =
R"""(Returns a constant reference to the output port for the generalized
accelerations v̇ᵢ ⊆ v̇ for model instance i.

In a discrete-time plant, the use_sampled_output_ports setting affects
the output of this port. See output_port_sampling "Output port
sampling" for details. When sampling is enabled and the plant has not
yet taken a step, the output value will be all zeros.

Precondition:
    Finalize() was already called on ``this`` plant.

Raises:
    RuntimeError if called before Finalize().

Raises:
    RuntimeError if the model instance does not exist.)""";
        } get_generalized_acceleration_output_port;
        // Symbol: drake::multibody::MultibodyPlant::get_generalized_contact_forces_output_port
        struct /* get_generalized_contact_forces_output_port */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(Returns a constant reference to the output port of generalized contact
forces for a specific model instance.

In a discrete-time plant, the use_sampled_output_ports setting affects
the output of this port. See output_port_sampling "Output port
sampling" for details. When sampling is enabled and the plant has not
yet taken a step, the output value will be all zeros.

Precondition:
    Finalize() was already called on ``this`` plant.

Raises:
    RuntimeError if called before Finalize().

Raises:
    RuntimeError if the model instance does not exist.)""";
        } get_generalized_contact_forces_output_port;
        // Symbol: drake::multibody::MultibodyPlant::get_geometry_pose_output_port
        struct /* get_geometry_pose_output_port */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(Returns the output port of frames' poses to communicate with a
SceneGraph.)""";
        } get_geometry_pose_output_port;
        // Symbol: drake::multibody::MultibodyPlant::get_geometry_query_input_port
        struct /* get_geometry_query_input_port */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(Returns a constant reference to the input port used to perform
geometric queries on a SceneGraph. See
SceneGraph::get_query_output_port(). Refer to section mbp_geometry
"Geometry" of this class's documentation for further details on
collision geometry registration and connection with a SceneGraph.)""";
        } get_geometry_query_input_port;
        // Symbol: drake::multibody::MultibodyPlant::get_joint
        struct /* get_joint */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(Returns a constant reference to the joint with unique index
``joint_index``.

Raises:
    RuntimeError when ``joint_index`` does not correspond to a joint
    in this model.)""";
        } get_joint;
        // Symbol: drake::multibody::MultibodyPlant::get_joint_actuator
        struct /* get_joint_actuator */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(Returns a constant reference to the joint actuator with unique index
``actuator_index``.

Raises:
    RuntimeError if ``actuator_index`` does not correspond to a joint
    actuator in this tree.)""";
        } get_joint_actuator;
        // Symbol: drake::multibody::MultibodyPlant::get_mutable_joint
        struct /* get_mutable_joint */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(Returns a mutable reference to the joint with unique index
``joint_index``.

Raises:
    RuntimeError when ``joint_index`` does not correspond to a joint
    in this model.)""";
        } get_mutable_joint;
        // Symbol: drake::multibody::MultibodyPlant::get_mutable_joint_actuator
        struct /* get_mutable_joint_actuator */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(Returns a mutable reference to the joint actuator with unique index
``actuator_index``.

Raises:
    RuntimeError if ``actuator_index`` does not correspond to a joint
    actuator in this tree.)""";
        } get_mutable_joint_actuator;
        // Symbol: drake::multibody::MultibodyPlant::get_net_actuation_output_port
        struct /* get_net_actuation_output_port */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc_0args =
R"""(Returns a constant reference to the output port that reports actuation
values applied through joint actuators. This output port is a vector
valued port. The actuation value for a particular actuator can be
found at offset JointActuator::input_start() in this vector. Models
that include PD controllers will include their contribution in this
port, refer to mbp_actuation "Actuation" for further details.

In a discrete-time plant, the use_sampled_output_ports setting affects
the output of this port. See output_port_sampling "Output port
sampling" for details. When sampling is enabled and the plant has not
yet taken a step, the output value will be all zeros.

Note:
    PD controllers are not considered for actuators on locked joints,
    see Joint::Lock(). Therefore they do not contribute to this port
    if the joint is locked.

Precondition:
    Finalize() was already called on ``this`` plant.

Raises:
    RuntimeError if called before Finalize().)""";
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc_1args =
R"""(Returns a constant reference to the output port that reports actuation
values applied through joint actuators, for a specific model instance.
Models that include PD controllers will include their contribution in
this port, refer to mbp_actuation "Actuation" for further details.
This is a vector valued port with entries ordered by monotonically
increasing JointActuatorIndex within ``model_instance``.

Every model instance in ``this`` plant model has a net actuation
output port, even if zero sized (for model instance with no
actuators).

In a discrete-time plant, the use_sampled_output_ports setting affects
the output of this port. See output_port_sampling "Output port
sampling" for details. When sampling is enabled and the plant has not
yet taken a step, the output value will be all zeros.

Note:
    PD controllers are not considered for actuators on locked joints,
    see Joint::Lock(). Therefore they do not contribute to this port
    if the joint is locked.

Precondition:
    Finalize() was already called on ``this`` plant.

Raises:
    RuntimeError if called before Finalize().)""";
        } get_net_actuation_output_port;
        // Symbol: drake::multibody::MultibodyPlant::get_reaction_forces_output_port
        struct /* get_reaction_forces_output_port */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(Returns the port for joint reaction forces. A Joint models the
kinematical relationship which characterizes the possible relative
motion between two bodies. In Drake, a joint connects a frame ``Jp``
on *parent* body P with a frame ``Jc`` on a *child* body C. This usage
of the terms *parent* and *child* is just a convention and implies
nothing about the inboard-outboard relationship between the bodies.
Since a Joint imposes a kinematical relationship which characterizes
the possible relative motion between frames Jp and Jc, reaction forces
on each body are established. That is, we could cut the model at the
joint and replace it with equivalent forces equal to these reaction
forces in order to attain the same motions of the mechanical system.

This output port allows to evaluate the reaction force ``F_CJc_Jc`` on
the *child* body C, at ``Jc``, and expressed in Jc for all joints in
the model. This port evaluates to a vector of type
std::vector<SpatialForce<T>> and size num_joints() indexed by
JointIndex, see Joint::index(). Each entry corresponds to the spatial
force ``F_CJc_Jc`` applied on the joint's child body C
(Joint::child_body()), at the joint's child frame ``Jc``
(Joint::frame_on_child()) and expressed in frame ``Jc``.

In a discrete-time plant, the use_sampled_output_ports setting affects
the output of this port. See output_port_sampling "Output port
sampling" for details. When sampling is enabled and the plant has not
yet taken a step, the output value will be all zeros.

Note:
    PD controllers are not considered for actuators on locked joints,
    see Joint::Lock(). Therefore they do not contribute to this port
    if the joint is locked.

Raises:
    RuntimeError if called pre-finalize.)""";
        } get_reaction_forces_output_port;
        // Symbol: drake::multibody::MultibodyPlant::get_sap_near_rigid_threshold
        struct /* get_sap_near_rigid_threshold */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(Returns:
    the SAP near rigid regime threshold.

See also:
    See set_sap_near_rigid_threshold().)""";
        } get_sap_near_rigid_threshold;
        // Symbol: drake::multibody::MultibodyPlant::get_source_id
        struct /* get_source_id */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(Returns the unique id identifying ``this`` plant as a source for a
SceneGraph. Returns ``nullopt`` if ``this`` plant did not register any
geometry. This method can be called at any time during the lifetime of
``this`` plant to query if ``this`` plant has been registered with a
SceneGraph, either pre- or post-finalize, see Finalize(). However, a
geometry::SourceId is only assigned once at the first call of any of
this plant's geometry registration methods, and it does not change
after that. Post-finalize calls will always return the same value.)""";
        } get_source_id;
        // Symbol: drake::multibody::MultibodyPlant::get_state_output_port
        struct /* get_state_output_port */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc_0args =
R"""(Returns a constant reference to the output port for the multibody
state x = [q, v] of the model.

Precondition:
    Finalize() was already called on ``this`` plant.

Raises:
    RuntimeError if called before Finalize().)""";
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc_1args =
R"""(Returns a constant reference to the output port for the state xᵢ = [qᵢ
vᵢ] of model instance i. (Here qᵢ ⊆ q and vᵢ ⊆ v.)

Precondition:
    Finalize() was already called on ``this`` plant.

Raises:
    RuntimeError if called before Finalize().

Raises:
    RuntimeError if the model instance does not exist.)""";
        } get_state_output_port;
        // Symbol: drake::multibody::MultibodyPlant::get_tendon_constraint_specs
        struct /* get_tendon_constraint_specs */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc_1args =
R"""((Internal use only) Returns the tendon constraint specification
corresponding to ``id``

Raises:
    if ``id`` is not a valid identifier for a tendon constraint.)""";
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc_0args =
R"""((Internal use only) Returns a reference to the all of the tendon
constraints in this plant as a map from MultibodyConstraintId to
TendonConstraintSpec.)""";
        } get_tendon_constraint_specs;
        // Symbol: drake::multibody::MultibodyPlant::get_weld_constraint_specs
        struct /* get_weld_constraint_specs */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc_1args =
R"""((Internal use only) Returns the weld constraint specification
corresponding to ``id``

Raises:
    if ``id`` is not a valid identifier for a weld constraint.)""";
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc_0args =
R"""((Internal use only) Returns a reference to the all of the weld
constraints in this plant as a map from MultibodyConstraintId to
WeldConstraintSpec.)""";
        } get_weld_constraint_specs;
        // Symbol: drake::multibody::MultibodyPlant::graph
        struct /* graph */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""((Internal use only) Provides access to the internal::LinkJointGraph.
You can use graph().forest() to access the as-built
internal::SpanningForest if you've already called Finalize().)""";
        } graph;
        // Symbol: drake::multibody::MultibodyPlant::gravity_field
        struct /* gravity_field */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(An accessor to the current gravity field.)""";
        } gravity_field;
        // Symbol: drake::multibody::MultibodyPlant::has_body
        struct /* has_body */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(Returns ``True`` if plant has a rigid body with unique index
``body_index``.)""";
        } has_body;
        // Symbol: drake::multibody::MultibodyPlant::has_joint
        struct /* has_joint */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(Returns true if plant has a joint with unique index ``joint_index``.
The value could be false if the joint was removed using RemoveJoint().)""";
        } has_joint;
        // Symbol: drake::multibody::MultibodyPlant::has_joint_actuator
        struct /* has_joint_actuator */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(Returns true if plant has a joint actuator with unique index
``actuator_index``. The value could be false if the actuator was
removed using RemoveJointActuator().)""";
        } has_joint_actuator;
        // Symbol: drake::multibody::MultibodyPlant::has_sampled_output_ports
        struct /* has_sampled_output_ports */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""((Advanced) If ``this`` plant is continuous (i.e., is_discrete() is
``False``), returns false. If ``this`` plant is discrete, returns
whether or not the output ports are sampled (change only at a time
step boundary) or live (instantaneously reflect changes to the input
ports). See output_port_sampling "Output port sampling" for details.)""";
        } has_sampled_output_ports;
        // Symbol: drake::multibody::MultibodyPlant::is_finalized
        struct /* is_finalized */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(Returns ``True`` if this MultibodyPlant was finalized with a call to
Finalize().

See also:
    Finalize().)""";
        } is_finalized;
        // Symbol: drake::multibody::MultibodyPlant::is_gravity_enabled
        struct /* is_gravity_enabled */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(Returns:
    ``True`` iff gravity is enabled for ``model_instance``.

See also:
    set_gravity_enabled().

Raises:
    RuntimeError if the model instance is invalid.)""";
        } is_gravity_enabled;
        // Symbol: drake::multibody::MultibodyPlant::mutable_deformable_model
        struct /* mutable_deformable_model */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(Returns a mutable reference to the DeformableModel owned by this
plant.

Raises:
    RuntimeError if the plant is finalized.

Warning:
    This feature is considered to be **experimental** and may change
    or be removed at any time, without any deprecation notice ahead of
    time.)""";
        } mutable_deformable_model;
        // Symbol: drake::multibody::MultibodyPlant::mutable_gravity_field
        struct /* mutable_gravity_field */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(A mutable accessor to the current gravity field.)""";
        } mutable_gravity_field;
        // Symbol: drake::multibody::MultibodyPlant::num_actuated_dofs
        struct /* num_actuated_dofs */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc_0args =
R"""(Returns the total number of actuated degrees of freedom. That is, the
vector of actuation values u has this size. See AddJointActuator().)""";
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc_1args =
R"""(Returns the total number of actuated degrees of freedom for a specific
model instance. That is, the vector of actuation values u has this
size. See AddJointActuator().

Raises:
    RuntimeError if called pre-finalize.)""";
        } num_actuated_dofs;
        // Symbol: drake::multibody::MultibodyPlant::num_actuators
        struct /* num_actuators */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc_0args =
R"""(Returns the number of joint actuators in the model.

See also:
    AddJointActuator().)""";
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc_1args =
R"""(Returns the number of actuators for a specific model instance.

Raises:
    RuntimeError if called pre-finalize.)""";
        } num_actuators;
        // Symbol: drake::multibody::MultibodyPlant::num_ball_constraints
        struct /* num_ball_constraints */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(Returns the total number of ball constraints specified by the user.)""";
        } num_ball_constraints;
        // Symbol: drake::multibody::MultibodyPlant::num_bodies
        struct /* num_bodies */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(Returns the number of RigidBody elements in the model, including the
"world" RigidBody, which is always part of the model.

See also:
    AddRigidBody().)""";
        } num_bodies;
        // Symbol: drake::multibody::MultibodyPlant::num_collision_geometries
        struct /* num_collision_geometries */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(Returns the number of geometries registered for contact modeling. This
method can be called at any time during the lifetime of ``this``
plant, either pre- or post-finalize, see Finalize(). Post-finalize
calls will always return the same value.)""";
        } num_collision_geometries;
        // Symbol: drake::multibody::MultibodyPlant::num_constraints
        struct /* num_constraints */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(Returns the total number of constraints specified by the user.)""";
        } num_constraints;
        // Symbol: drake::multibody::MultibodyPlant::num_coupler_constraints
        struct /* num_coupler_constraints */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(Returns the total number of coupler constraints specified by the user.)""";
        } num_coupler_constraints;
        // Symbol: drake::multibody::MultibodyPlant::num_distance_constraints
        struct /* num_distance_constraints */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(Returns the total number of distance constraints specified by the
user.)""";
        } num_distance_constraints;
        // Symbol: drake::multibody::MultibodyPlant::num_force_elements
        struct /* num_force_elements */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(Returns the number of ForceElement objects.

See also:
    AddForceElement().)""";
        } num_force_elements;
        // Symbol: drake::multibody::MultibodyPlant::num_frames
        struct /* num_frames */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(Returns the number of Frame objects in this model. Frames include body
frames associated with each of the bodies, including the *world* body.
This means the minimum number of frames is one.)""";
        } num_frames;
        // Symbol: drake::multibody::MultibodyPlant::num_joints
        struct /* num_joints */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(Returns the number of joints in the model.

See also:
    AddJoint().)""";
        } num_joints;
        // Symbol: drake::multibody::MultibodyPlant::num_model_instances
        struct /* num_model_instances */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(Returns the number of model instances in the model.

See also:
    AddModelInstance().)""";
        } num_model_instances;
        // Symbol: drake::multibody::MultibodyPlant::num_multibody_states
        struct /* num_multibody_states */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc_0args =
R"""(Returns the size of the multibody system state vector x = [q v]. This
will be ``num_positions()`` plus ``num_velocities()``.

Raises:
    RuntimeError if called pre-finalize.)""";
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc_1args =
R"""(Returns the size of the multibody system state vector xᵢ = [qᵢ vᵢ] for
model instance i. (Here qᵢ ⊆ q and vᵢ ⊆ v.) will be
``num_positions(model_instance)`` plus
``num_velocities(model_instance)``.

Raises:
    RuntimeError if called pre-finalize.)""";
        } num_multibody_states;
        // Symbol: drake::multibody::MultibodyPlant::num_positions
        struct /* num_positions */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc_0args =
R"""(Returns the size of the generalized position vector q for this model.

Raises:
    RuntimeError if called pre-finalize.)""";
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc_1args =
R"""(Returns the size of the generalized position vector qᵢ for model
instance i.

Raises:
    RuntimeError if called pre-finalize.)""";
        } num_positions;
        // Symbol: drake::multibody::MultibodyPlant::num_tendon_constraints
        struct /* num_tendon_constraints */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(Returns the total number of tendon constraints specified by the user.)""";
        } num_tendon_constraints;
        // Symbol: drake::multibody::MultibodyPlant::num_velocities
        struct /* num_velocities */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc_0args =
R"""(Returns the size of the generalized velocity vector v for this model.

Raises:
    RuntimeError if called pre-finalize.)""";
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc_1args =
R"""(Returns the size of the generalized velocity vector vᵢ for model
instance i.

Raises:
    RuntimeError if called pre-finalize.)""";
        } num_velocities;
        // Symbol: drake::multibody::MultibodyPlant::num_visual_geometries
        struct /* num_visual_geometries */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(Returns the number of geometries registered for visualization. This
method can be called at any time during the lifetime of ``this``
plant, either pre- or post-finalize, see Finalize(). Post-finalize
calls will always return the same value.)""";
        } num_visual_geometries;
        // Symbol: drake::multibody::MultibodyPlant::num_weld_constraints
        struct /* num_weld_constraints */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(Returns the total number of weld constraints specified by the user.)""";
        } num_weld_constraints;
        // Symbol: drake::multibody::MultibodyPlant::physical_models
        struct /* physical_models */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc = R"""()""";
        } physical_models;
        // Symbol: drake::multibody::MultibodyPlant::set_adjacent_bodies_collision_filters
        struct /* set_adjacent_bodies_collision_filters */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(Sets whether to apply collision filters to topologically adjacent
bodies at Finalize() time. Filters are applied when there exists a
joint between bodies, except in the case of 6-dof joints or joints in
which the parent body is ``world``.

Raises:
    RuntimeError iff called post-finalize.)""";
        } set_adjacent_bodies_collision_filters;
        // Symbol: drake::multibody::MultibodyPlant::set_contact_model
        struct /* set_contact_model */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(Sets the contact model to be used by ``this`` MultibodyPlant, see
ContactModel for available options. The default contact model is
ContactModel::kHydroelasticWithFallback.

Raises:
    RuntimeError iff called post-finalize.)""";
        } set_contact_model;
        // Symbol: drake::multibody::MultibodyPlant::set_contact_surface_representation
        struct /* set_contact_surface_representation */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(Sets the representation of contact surfaces to be used by ``this``
MultibodyPlant. See geometry::HydroelasticContactRepresentation for
available options. See GetDefaultContactSurfaceRepresentation() for
explanation of default values.

Raises:
    RuntimeError if called post-finalize.)""";
        } set_contact_surface_representation;
        // Symbol: drake::multibody::MultibodyPlant::set_discrete_contact_approximation
        struct /* set_discrete_contact_approximation */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(Sets the discrete contact model approximation.

Note:
    Calling this method also sets the contact solver type (see
    get_discrete_contact_solver()) according to: -
    DiscreteContactApproximation::kTamsi sets the solver to
    DiscreteContactSolver::kTamsi. -
    DiscreteContactApproximation::kSap,
    DiscreteContactApproximation::kSimilar and
    DiscreteContactApproximation::kLagged set the solver to
    DiscreteContactSolver::kSap.

Raises:
    iff ``this`` plant is continuous (i.e. is_discrete() is
    ``False``.)

Raises:
    RuntimeError iff called post-finalize.)""";
        } set_discrete_contact_approximation;
        // Symbol: drake::multibody::MultibodyPlant::set_gravity_enabled
        struct /* set_gravity_enabled */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(Sets is_gravity_enabled() for ``model_instance`` to ``is_enabled``.
The effect of ``is_enabled = false`` is effectively equivalent to
disabling (or making zero) gravity for all bodies in the specified
model instance. By default is_gravity_enabled() equals ``True`` for
all model instances.

Raises:
    RuntimeError if called post-finalize.

Raises:
    RuntimeError if the model instance is invalid.)""";
        } set_gravity_enabled;
        // Symbol: drake::multibody::MultibodyPlant::set_penetration_allowance
        struct /* set_penetration_allowance */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(Sets a penetration allowance used to estimate the point contact
stiffness and Hunt & Crossley dissipation parameters. Refer to the
section point_contact_defaults "Point Contact Default Parameters" for
further details.

Warning:
    This will be deprecated. Prefer using defaults specified in
    geometry::DefaultProximityProperties.

Warning:
    Values provided in geometry::DefaultProximityProperties have
    precedence. If values estimated based on penetration allowance are
    desired, set defaults in geometry::DefaultProximityProperties to
    std::nullopt.

Raises:
    RuntimeError if penetration_allowance is not positive.)""";
        } set_penetration_allowance;
        // Symbol: drake::multibody::MultibodyPlant::set_sap_near_rigid_threshold
        struct /* set_sap_near_rigid_threshold */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(Non-negative dimensionless number typically in the range [0.0, 1.0],
though larger values are allowed even if uncommon. This parameter
controls the "near rigid" regime of the SAP solver, β in section V.B
of [Castro et al., 2021]. It essentially controls a threshold value
for the maximum amount of stiffness SAP can handle robustly. Beyond
this value, stiffness saturates as explained in [Castro et al., 2021].
A value of 1.0 is a conservative choice to avoid ill-conditioning that
might lead to softer than expected contact. If this is your case,
consider turning off this approximation by setting this parameter to
zero. For difficult cases where ill-conditioning is a problem, a small
but non-zero number can be used, e.g. 1.0e-3.

Raises:
    RuntimeError if near_rigid_threshold is negative.

Raises:
    RuntimeError if called post-finalize.)""";
        } set_sap_near_rigid_threshold;
        // Symbol: drake::multibody::MultibodyPlant::set_stiction_tolerance
        struct /* set_stiction_tolerance */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(**** Stribeck model of friction

Currently MultibodyPlant uses the Stribeck approximation to model dry
friction. The Stribeck model of friction is an approximation to
Coulomb's law of friction that allows using continuous time
integration without the need to specify complementarity constraints.
While this results in a simpler model immediately tractable with
standard numerical methods for integration of ODE's, it often leads to
stiff dynamics that require an explicit integrator to take very small
time steps. It is therefore recommended to use error controlled
integrators when using this model or the discrete time stepping (see
multibody_simulation). See stribeck_approximation for a detailed
discussion of the Stribeck model.

Sets the stiction tolerance ``v_stiction`` for the Stribeck model,
where ``v_stiction`` must be specified in m/s (meters per second.)
``v_stiction`` defaults to a value of 1 millimeter per second. In
selecting a value for ``v_stiction``, you must ask yourself the
question, "When two objects are ostensibly in stiction, how much slip
am I willing to allow?" There are two opposing design issues in
picking a value for vₛ. On the one hand, small values of vₛ make the
problem numerically stiff during stiction, potentially increasing the
integration cost. On the other hand, it should be picked to be
appropriate for the scale of the problem. For example, a car
simulation could allow a "large" value for vₛ of 1 cm/s (1×10⁻² m/s),
but reasonable stiction for grasping a 10 cm box might require
limiting residual slip to 1×10⁻³ m/s or less. Ultimately, picking the
largest viable value will allow your simulation to run faster and more
robustly. Note that ``v_stiction`` is the slip velocity that we'd have
when we are at edge of the friction cone. For cases when the friction
force is well within the friction cone the slip velocity will always
be smaller than this value. See also stribeck_approximation.

Raises:
    RuntimeError if ``v_stiction`` is non-positive.)""";
        } set_stiction_tolerance;
        // Symbol: drake::multibody::MultibodyPlant::stiction_tolerance
        struct /* stiction_tolerance */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(Returns:
    the stiction tolerance parameter, in m/s.

See also:
    set_stiction_tolerance.)""";
        } stiction_tolerance;
        // Symbol: drake::multibody::MultibodyPlant::time_step
        struct /* time_step */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(The time step (or period) used to model ``this`` plant as a discrete
system with periodic updates. Returns 0 (zero) if the plant is modeled
as a continuous system. This property of the plant is specified at
construction and therefore this query can be performed either pre- or
post-finalize, see Finalize().

See also:
    MultibodyPlant::MultibodyPlant(double))""";
        } time_step;
        // Symbol: drake::multibody::MultibodyPlant::world_body
        struct /* world_body */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(Returns a constant reference to the *world* body.)""";
        } world_body;
        // Symbol: drake::multibody::MultibodyPlant::world_frame
        struct /* world_frame */ {
          // Source: drake/multibody/plant/multibody_plant.h
          const char* doc =
R"""(Returns a constant reference to the *world* frame.)""";
        } world_frame;
      } MultibodyPlant;
      // Symbol: drake::multibody::MultibodyPlantConfig
      struct /* MultibodyPlantConfig */ {
        // Source: drake/multibody/plant/multibody_plant_config.h
        const char* doc =
R"""(The set of configurable properties on a MultibodyPlant.

The field names and defaults here match MultibodyPlant's defaults
exactly, with the exception of time_step.)""";
        // Symbol: drake::multibody::MultibodyPlantConfig::Serialize
        struct /* Serialize */ {
          // Source: drake/multibody/plant/multibody_plant_config.h
          const char* doc =
R"""(Passes this object to an Archive. Refer to yaml_serialization "YAML
Serialization" for background.)""";
        } Serialize;
        // Symbol: drake::multibody::MultibodyPlantConfig::adjacent_bodies_collision_filters
        struct /* adjacent_bodies_collision_filters */ {
          // Source: drake/multibody/plant/multibody_plant_config.h
          const char* doc =
R"""(Configures the
MultibodyPlant::set_adjacent_bodies_collision_filters().)""";
        } adjacent_bodies_collision_filters;
        // Symbol: drake::multibody::MultibodyPlantConfig::contact_model
        struct /* contact_model */ {
          // Source: drake/multibody/plant/multibody_plant_config.h
          const char* doc =
R"""(Configures the MultibodyPlant::set_contact_model(). Refer to
drake::multibody::ContactModel for details. Valid strings are: -
"point" - "hydroelastic" - "hydroelastic_with_fallback")""";
        } contact_model;
        // Symbol: drake::multibody::MultibodyPlantConfig::contact_surface_representation
        struct /* contact_surface_representation */ {
          // Source: drake/multibody/plant/multibody_plant_config.h
          const char* doc =
R"""(Configures the MultibodyPlant::set_contact_surface_representation().
Refer to drake::geometry::HydroelasticContactRepresentation for
details. Valid strings are: - "triangle" - "polygon"

The default value used here is consistent with the default time_step
chosen above; keep this consistent with
MultibodyPlant::GetDefaultContactSurfaceRepresentation().)""";
        } contact_surface_representation;
        // Symbol: drake::multibody::MultibodyPlantConfig::discrete_contact_approximation
        struct /* discrete_contact_approximation */ {
          // Source: drake/multibody/plant/multibody_plant_config.h
          const char* doc =
R"""(Configures the MultibodyPlant::set_discrete_contact_approximation().
Refer to drake::multibody::DiscreteContactApproximation for details.
Valid strings are: - "tamsi" - "sap" - "similar" - "lagged"

Refer to MultibodyPlant::set_discrete_contact_approximation() and the
references therein for further details.)""";
        } discrete_contact_approximation;
        // Symbol: drake::multibody::MultibodyPlantConfig::penetration_allowance
        struct /* penetration_allowance */ {
          // Source: drake/multibody/plant/multibody_plant_config.h
          const char* doc =
R"""(Configures the MultibodyPlant::set_penetration_allowance().)""";
        } penetration_allowance;
        // Symbol: drake::multibody::MultibodyPlantConfig::sap_near_rigid_threshold
        struct /* sap_near_rigid_threshold */ {
          // Source: drake/multibody/plant/multibody_plant_config.h
          const char* doc =
R"""(Non-negative dimensionless number typically in the range [0.0, 1.0],
though larger values are allowed even if uncommon. This parameter
controls the "near rigid" regime of the SAP solver, β in section V.B
of [Castro et al., 2021]. It essentially controls a threshold value
for the maximum amount of stiffness SAP can handle robustly. Beyond
this value, stiffness saturates as explained in [Castro et al., 2021].
A value of 1.0 is a conservative choice to avoid numerical
ill-conditioning. However, this might introduce artificial softening
of the contact constraints. If this is your case try: 1. Set this
parameter to zero. 2. For difficult problems (hundreds of contacts for
instance), you might need to use a low value if the solver fails to
converge. For instance, set values in the range (1e-3, 1e-2).)""";
        } sap_near_rigid_threshold;
        // Symbol: drake::multibody::MultibodyPlantConfig::stiction_tolerance
        struct /* stiction_tolerance */ {
          // Source: drake/multibody/plant/multibody_plant_config.h
          const char* doc =
R"""(Configures the MultibodyPlant::set_stiction_tolerance().)""";
        } stiction_tolerance;
        // Symbol: drake::multibody::MultibodyPlantConfig::time_step
        struct /* time_step */ {
          // Source: drake/multibody/plant/multibody_plant_config.h
          const char* doc =
R"""(Configures the MultibodyPlant::MultibodyPlant() constructor time_step.

There is no default value for this within MultibodyPlant itself, so
here we choose a nominal value (a discrete system, with a 1ms periodic
update) as a reasonably conservative estimate that works in many
cases.)""";
        } time_step;
        // Symbol: drake::multibody::MultibodyPlantConfig::use_sampled_output_ports
        struct /* use_sampled_output_ports */ {
          // Source: drake/multibody/plant/multibody_plant_config.h
          const char* doc =
R"""(Configures the MultibodyPlant::SetUseSampledOutputPorts(). Ignored
when the time_step is zero.)""";
        } use_sampled_output_ports;
        auto Serialize__fields() const {
          return std::array{
            std::make_pair("adjacent_bodies_collision_filters", adjacent_bodies_collision_filters.doc),
            std::make_pair("contact_model", contact_model.doc),
            std::make_pair("contact_surface_representation", contact_surface_representation.doc),
            std::make_pair("discrete_contact_approximation", discrete_contact_approximation.doc),
            std::make_pair("penetration_allowance", penetration_allowance.doc),
            std::make_pair("sap_near_rigid_threshold", sap_near_rigid_threshold.doc),
            std::make_pair("stiction_tolerance", stiction_tolerance.doc),
            std::make_pair("time_step", time_step.doc),
            std::make_pair("use_sampled_output_ports", use_sampled_output_ports.doc),
          };
        }
      } MultibodyPlantConfig;
      // Symbol: drake::multibody::PhysicalModel
      struct /* PhysicalModel */ {
        // Source: drake/multibody/plant/physical_model.h
        const char* doc =
R"""((Internal) PhysicalModel provides the functionalities to extend the
type of physical model of MultibodyPlant. Developers can derive from
this PhysicalModel to incorporate additional model elements coupled
with the rigid body dynamics. For instance, simulation of deformable
objects requires additional state and ports to interact with externals
systems such as visualization.

Similar to the routine of adding multiple model elements in
MultibodyPlant, users should add all the model elements they wish to
add to a PhysicalModel before the owning MultibodyPlant calls
``Finalize()``. When ``Finalize()`` is invoked, MultibodyPlant will
allocate the system level context resources for each PhysicalModel it
owns. After the system resources are allocated, model mutation in the
PhysicalModels owned by MultibodyPlant is not allowed.

This class is for internal use only. Use derived concrete models (e.g.
DeformableModel) instead.)""";
        // Symbol: drake::multibody::PhysicalModel::CloneToAutoDiffXd
        struct /* CloneToAutoDiffXd */ {
          // Source: drake/multibody/plant/physical_model.h
          const char* doc = R"""()""";
        } CloneToAutoDiffXd;
        // Symbol: drake::multibody::PhysicalModel::CloneToDouble
        struct /* CloneToDouble */ {
          // Source: drake/multibody/plant/physical_model.h
          const char* doc = R"""()""";
        } CloneToDouble;
        // Symbol: drake::multibody::PhysicalModel::CloneToScalar
        struct /* CloneToScalar */ {
          // Source: drake/multibody/plant/physical_model.h
          const char* doc =
R"""((Internal only) Creates a clone of ``this`` concrete PhysicalModel
object with the scalar type ``ScalarType`` to be owned by the given
``plant``. The clone should be a deep copy of the original
PhysicalModel with the exception of members overwritten in
``DeclareSystemResources()``. This method is meant to be called by the
scalar-converting copy constructor of MultibodyPlant only and thus is
only called from a finalized MultibodyPlant. $Raises:

RuntimeError if plant is nullptr.

Raises:
    RuntimeError if the owning plant of ``this`` PhysicalModel is not
    finalized.

Parameter ``plant``:
    pointer to the MultibodyPlant owning the clone. This needs to be a
    mutable pointer because the constructor of the clone requires a
    mutable pointer to the owning plant.

Note:
    ``DeclareSystemResources()`` is not called on the clone and needs
    to be called from the plant owning the clone.)""";
        } CloneToScalar;
        // Symbol: drake::multibody::PhysicalModel::CloneToSymbolic
        struct /* CloneToSymbolic */ {
          // Source: drake/multibody/plant/physical_model.h
          const char* doc = R"""()""";
        } CloneToSymbolic;
        // Symbol: drake::multibody::PhysicalModel::DeclareAbstractOutputPort
        struct /* DeclareAbstractOutputPort */ {
          // Source: drake/multibody/plant/physical_model.h
          const char* doc = R"""()""";
        } DeclareAbstractOutputPort;
        // Symbol: drake::multibody::PhysicalModel::DeclareAbstractParameter
        struct /* DeclareAbstractParameter */ {
          // Source: drake/multibody/plant/physical_model.h
          const char* doc = R"""()""";
        } DeclareAbstractParameter;
        // Symbol: drake::multibody::PhysicalModel::DeclareDiscreteState
        struct /* DeclareDiscreteState */ {
          // Source: drake/multibody/plant/physical_model.h
          const char* doc = R"""()""";
        } DeclareDiscreteState;
        // Symbol: drake::multibody::PhysicalModel::DeclareSceneGraphPorts
        struct /* DeclareSceneGraphPorts */ {
          // Source: drake/multibody/plant/physical_model.h
          const char* doc =
R"""((Internal only) Declares zero or more output ports in the owning
MultibodyPlant to communicate with a SceneGraph.

Raises:
    RuntimeError if called after call to DeclareSystemResources().

Raises:
    RuntimeError if called more than once when at least one output
    port is created.)""";
        } DeclareSceneGraphPorts;
        // Symbol: drake::multibody::PhysicalModel::DeclareSystemResources
        struct /* DeclareSystemResources */ {
          // Source: drake/multibody/plant/physical_model.h
          const char* doc =
R"""((Internal only) MultibodyPlant calls this from within Finalize() to
declare additional system resources. This method is only meant to be
called by MultibodyPlant. The pointer to the owning plant is nulled
after call to this function.)""";
        } DeclareSystemResources;
        // Symbol: drake::multibody::PhysicalModel::DeclareVectorOutputPort
        struct /* DeclareVectorOutputPort */ {
          // Source: drake/multibody/plant/physical_model.h
          const char* doc = R"""()""";
        } DeclareVectorOutputPort;
        // Symbol: drake::multibody::PhysicalModel::DoDeclareSceneGraphPorts
        struct /* DoDeclareSceneGraphPorts */ {
          // Source: drake/multibody/plant/physical_model.h
          const char* doc = R"""()""";
        } DoDeclareSceneGraphPorts;
        // Symbol: drake::multibody::PhysicalModel::DoDeclareSystemResources
        struct /* DoDeclareSystemResources */ {
          // Source: drake/multibody/plant/physical_model.h
          const char* doc = R"""()""";
        } DoDeclareSystemResources;
        // Symbol: drake::multibody::PhysicalModel::DoToPhysicalModelPointerVariant
        struct /* DoToPhysicalModelPointerVariant */ {
          // Source: drake/multibody/plant/physical_model.h
          const char* doc = R"""()""";
        } DoToPhysicalModelPointerVariant;
        // Symbol: drake::multibody::PhysicalModel::PhysicalModel<T>
        struct /* ctor */ {
          // Source: drake/multibody/plant/physical_model.h
          const char* doc =
R"""(Constructs a PhysicalModel owned by the given ``owning_plant``. The
lifespan of the ``owning_plant`` must outlast ``this`` PhysicalModel.
This PhysicalModel declares System resources from the ``owning_plant``
in the call to ``DeclareSystemResources()`` through the call to
``MultibodyPlant::Finalize()``.

Precondition:
    owning_plant != nullptr.)""";
        } ctor;
        // Symbol: drake::multibody::PhysicalModel::ThrowIfSystemResourcesDeclared
        struct /* ThrowIfSystemResourcesDeclared */ {
          // Source: drake/multibody/plant/physical_model.h
          const char* doc = R"""()""";
        } ThrowIfSystemResourcesDeclared;
        // Symbol: drake::multibody::PhysicalModel::ThrowIfSystemResourcesNotDeclared
        struct /* ThrowIfSystemResourcesNotDeclared */ {
          // Source: drake/multibody/plant/physical_model.h
          const char* doc = R"""()""";
        } ThrowIfSystemResourcesNotDeclared;
        // Symbol: drake::multibody::PhysicalModel::ToPhysicalModelPointerVariant
        struct /* ToPhysicalModelPointerVariant */ {
          // Source: drake/multibody/plant/physical_model.h
          const char* doc =
R"""(Returns (a const pointer to) the specific model variant of ``this``
PhysicalModel. Note that the variant contains a pointer to the
concrete model and therefore should not persist longer than the
lifespan of this model.)""";
        } ToPhysicalModelPointerVariant;
        // Symbol: drake::multibody::PhysicalModel::internal_tree
        struct /* internal_tree */ {
          // Source: drake/multibody/plant/physical_model.h
          const char* doc = R"""()""";
        } internal_tree;
        // Symbol: drake::multibody::PhysicalModel::is_cloneable_to_autodiff
        struct /* is_cloneable_to_autodiff */ {
          // Source: drake/multibody/plant/physical_model.h
          const char* doc =
R"""(Defaults to false. Derived classes that support making a clone that
uses AutoDiffXd as a scalar type must override this to return true.)""";
        } is_cloneable_to_autodiff;
        // Symbol: drake::multibody::PhysicalModel::is_cloneable_to_double
        struct /* is_cloneable_to_double */ {
          // Source: drake/multibody/plant/physical_model.h
          const char* doc =
R"""(Defaults to false. Derived classes that support making a clone that
uses double as a scalar type must override this to return true.)""";
        } is_cloneable_to_double;
        // Symbol: drake::multibody::PhysicalModel::is_cloneable_to_symbolic
        struct /* is_cloneable_to_symbolic */ {
          // Source: drake/multibody/plant/physical_model.h
          const char* doc =
R"""(Defaults to false. Derived classes that support making a clone that
uses symbolic::Expression as a scalar type must override this to
return true.)""";
        } is_cloneable_to_symbolic;
        // Symbol: drake::multibody::PhysicalModel::mutable_plant
        struct /* mutable_plant */ {
          // Source: drake/multibody/plant/physical_model.h
          const char* doc = R"""()""";
        } mutable_plant;
        // Symbol: drake::multibody::PhysicalModel::mutable_scene_graph
        struct /* mutable_scene_graph */ {
          // Source: drake/multibody/plant/physical_model.h
          const char* doc = R"""()""";
        } mutable_scene_graph;
        // Symbol: drake::multibody::PhysicalModel::plant
        struct /* plant */ {
          // Source: drake/multibody/plant/physical_model.h
          const char* doc = R"""()""";
        } plant;
      } PhysicalModel;
      // Symbol: drake::multibody::PointPairContactInfo
      struct /* PointPairContactInfo */ {
        // Source: drake/multibody/plant/point_pair_contact_info.h
        const char* doc =
R"""(A class containing information regarding contact response between two
bodies including:

- The pair of bodies that are contacting, referenced by their BodyIndex.
- A resultant contact force.
- A contact point.
- Separation speed.
- Slip speed.)""";
        // Symbol: drake::multibody::PointPairContactInfo::PointPairContactInfo<T>
        struct /* ctor */ {
          // Source: drake/multibody/plant/point_pair_contact_info.h
          const char* doc =
R"""(Constructs the contact information for a given pair of two colliding
bodies.

Parameter ``bodyA_index``:
    Index that references body A in ``this`` contact pair.

Parameter ``bodyB_index``:
    Index that references body B in ``this`` contact pair.

Parameter ``f_Bc_W``:
    Force on body B applied at contact point C, expressed in the world
    frame W.

Parameter ``p_WC``:
    Position of the contact point C in the world frame W.

Parameter ``separation_speed``:
    Separation speed along the normal direction between body A and
    body B, in meters per second. A positive value indicates bodies
    are moving apart. A negative value indicates bodies are moving
    towards each other.

Parameter ``slip_speed``:
    Slip speed, that is, the magnitude of the relative tangential
    velocity at the contact point in meters per second. A non-negative
    value always.

Parameter ``point_pair``:
    Additional point pair information for ``this`` contact info. Refer
    to the documentation for PenetrationAsPointPair for further
    details. It is expected that the body indexed by ``bodyA_index``
    is the same body that contains the geometry indexed by
    ``point_pair.id_A``. Likewise for the body indexed by
    ``bodyB_index`` and the body contining geometry with id
    ``point_pair.id_B``.

Precondition:
    The two body indexes must reference bodies from the same
    MultibodyPlant. Contact values should likewise be generated by the
    same MultibodyPlant.)""";
        } ctor;
        // Symbol: drake::multibody::PointPairContactInfo::bodyA_index
        struct /* bodyA_index */ {
          // Source: drake/multibody/plant/point_pair_contact_info.h
          const char* doc =
R"""(Returns the index of body A in the contact pair.)""";
        } bodyA_index;
        // Symbol: drake::multibody::PointPairContactInfo::bodyB_index
        struct /* bodyB_index */ {
          // Source: drake/multibody/plant/point_pair_contact_info.h
          const char* doc =
R"""(Returns the index of body B in the contact pair.)""";
        } bodyB_index;
        // Symbol: drake::multibody::PointPairContactInfo::contact_force
        struct /* contact_force */ {
          // Source: drake/multibody/plant/point_pair_contact_info.h
          const char* doc =
R"""(Returns the contact force ``f_Bc_W`` on B at contact point C expressed
in the world frame W.)""";
        } contact_force;
        // Symbol: drake::multibody::PointPairContactInfo::contact_point
        struct /* contact_point */ {
          // Source: drake/multibody/plant/point_pair_contact_info.h
          const char* doc =
R"""(Returns the position ``p_WC`` of the contact point C in the world
frame W.)""";
        } contact_point;
        // Symbol: drake::multibody::PointPairContactInfo::point_pair
        struct /* point_pair */ {
          // Source: drake/multibody/plant/point_pair_contact_info.h
          const char* doc =
R"""(Returns additional information for the geometric contact query for
``this`` pair as a PenetrationAsPointPair. The body containing
``point_pair().id_A`` is the same body indexed by ``bodyA_index()``.
Likewise, the body containing ``point_pair().id_B`` is the same body
indexed by ``bodyB_index()``.)""";
        } point_pair;
        // Symbol: drake::multibody::PointPairContactInfo::separation_speed
        struct /* separation_speed */ {
          // Source: drake/multibody/plant/point_pair_contact_info.h
          const char* doc =
R"""(Returns the separation speed between body A and B along the normal
direction (see PenetrationAsPointPair::nhat_BA_W) at the contact
point. It is defined positive for bodies moving apart in the normal
direction.)""";
        } separation_speed;
        // Symbol: drake::multibody::PointPairContactInfo::slip_speed
        struct /* slip_speed */ {
          // Source: drake/multibody/plant/point_pair_contact_info.h
          const char* doc =
R"""(Returns the slip speed between body A and B at contact point C.)""";
        } slip_speed;
      } PointPairContactInfo;
      // Symbol: drake::multibody::Propeller
      struct /* Propeller */ {
        // Source: drake/multibody/plant/propeller.h
        const char* doc =
R"""(A System that connects to the MultibodyPlant in order to model the
effects of one or more controlled propellers acting on a Body.

.. pydrake_system::

    name: Propeller
    input_ports:
    - command
    - body_poses
    output_ports:
    - spatial_forces

- The command input is a BasicVector<T> with one element per propeller.
- It is expected that the body_poses input should be connected to the
MultibodyPlant::get_body_poses_output_port() "MultibodyPlant body_poses
output port".
- The output is of type std::vector<ExternallyAppliedSpatialForce<T>>; it is
expected that this output will be connected to the MultibodyPlant::get_applied_spatial_force_input_port()
"externally_applied_spatial_force input port" of the MultibodyPlant.
- This system does not have any state.

The resulting iᵗʰ spatial force will have a force component in the
z-axis of the iᵗʰ propeller frame with magnitude ``thrust_ratio *
command`` Newtons, and a moment around the z-axis with magnitude
``moment_ratio * command`` Newton-meters. (Including these moments
tends to be important -- a quadrotor does not have a stabilizable
linearization around a hovering fixed point in 3D without them).

Note:
    Set PropellerInfo::moment_ratio to zero if you want a simple
    thruster which applies only a force (no moment) in the Propeller
    coordinates.)""";
        // Symbol: drake::multibody::Propeller::Propeller<T>
        struct /* ctor */ {
          // Source: drake/multibody/plant/propeller.h
          const char* doc_4args =
R"""(Constructs a system describing a single propeller.

See also:
    PropellerInfo for details on the arguments.)""";
          // Source: drake/multibody/plant/propeller.h
          const char* doc_1args =
R"""(Constructs a system describing multiple propellers.

See also:
    PropellerInfo.)""";
          // Source: drake/multibody/plant/propeller.h
          const char* doc_copyconvert =
R"""(Scalar-converting copy constructor. See system_scalar_conversion.)""";
        } ctor;
        // Symbol: drake::multibody::Propeller::get_body_poses_input_port
        struct /* get_body_poses_input_port */ {
          // Source: drake/multibody/plant/propeller.h
          const char* doc =
R"""(Returns a reference to the body_poses input port. It is anticipated
that this port will be connected the body_poses output port of a
MultibodyPlant.)""";
        } get_body_poses_input_port;
        // Symbol: drake::multibody::Propeller::get_command_input_port
        struct /* get_command_input_port */ {
          // Source: drake/multibody/plant/propeller.h
          const char* doc =
R"""(Returns a reference to the vector-valued input port for the propeller
commands. It has size ``num_propellers()``.)""";
        } get_command_input_port;
        // Symbol: drake::multibody::Propeller::get_spatial_forces_output_port
        struct /* get_spatial_forces_output_port */ {
          // Source: drake/multibody/plant/propeller.h
          const char* doc =
R"""(Returns a reference to the spatial_forces output port. It is
anticipated that this port will be connected to the
MultibodyPlant::get_applied_spatial_force_input_port()
"applied_spatial_force" input port of a MultibodyPlant.)""";
        } get_spatial_forces_output_port;
        // Symbol: drake::multibody::Propeller::num_propellers
        struct /* num_propellers */ {
          // Source: drake/multibody/plant/propeller.h
          const char* doc =
R"""(Returns the number of propellers modeled by this system.)""";
        } num_propellers;
      } Propeller;
      // Symbol: drake::multibody::PropellerInfo
      struct /* PropellerInfo */ {
        // Source: drake/multibody/plant/propeller.h
        const char* doc =
R"""(Parameters that describe the kinematic frame and force-production
properties of a single propeller.)""";
        // Symbol: drake::multibody::PropellerInfo::PropellerInfo
        struct /* ctor */ {
          // Source: drake/multibody/plant/propeller.h
          const char* doc = R"""()""";
        } ctor;
        // Symbol: drake::multibody::PropellerInfo::X_BP
        struct /* X_BP */ {
          // Source: drake/multibody/plant/propeller.h
          const char* doc =
R"""(Pose of the propeller frame P measured in the body frame B.
$*Default:* is the identity matrix.)""";
        } X_BP;
        // Symbol: drake::multibody::PropellerInfo::body_index
        struct /* body_index */ {
          // Source: drake/multibody/plant/propeller.h
          const char* doc =
R"""(The BodyIndex of a RigidBody in the MultibodyPlant to which the
propeller is attached. The spatial forces will be applied to this
body.)""";
        } body_index;
        // Symbol: drake::multibody::PropellerInfo::moment_ratio
        struct /* moment_ratio */ {
          // Source: drake/multibody/plant/propeller.h
          const char* doc =
R"""(The moment about the z axis (in frame P) of the spatial force will
have magnitude ``moment_ratio*command`` in Newton-meters. The default
is 0, which makes the propeller a simple Cartesian force generator.)""";
        } moment_ratio;
        // Symbol: drake::multibody::PropellerInfo::thrust_ratio
        struct /* thrust_ratio */ {
          // Source: drake/multibody/plant/propeller.h
          const char* doc =
R"""(The z component (in frame P) of the spatial force will have magnitude
``thrust_ratio*command`` in Newtons. The default is 1 (command in
Newtons), but this can also be used to scale an actuator command to
the resulting force.)""";
        } thrust_ratio;
      } PropellerInfo;
      // Symbol: drake::multibody::SignedDistanceWithTimeDerivative
      struct /* SignedDistanceWithTimeDerivative */ {
        // Source: drake/multibody/plant/calc_distance_and_time_derivative.h
        const char* doc =
R"""(The struct containing the signed distance and its time derivative
between a pair of geometries.)""";
        // Symbol: drake::multibody::SignedDistanceWithTimeDerivative::distance
        struct /* distance */ {
          // Source: drake/multibody/plant/calc_distance_and_time_derivative.h
          const char* doc =
R"""(The signed distance between the pair of geometry.)""";
        } distance;
        // Symbol: drake::multibody::SignedDistanceWithTimeDerivative::distance_time_derivative
        struct /* distance_time_derivative */ {
          // Source: drake/multibody/plant/calc_distance_and_time_derivative.h
          const char* doc =
R"""(The time derivative of the signed distance.)""";
        } distance_time_derivative;
      } SignedDistanceWithTimeDerivative;
      // Symbol: drake::multibody::TamsiSolver
      struct /* TamsiSolver */ {
        // Source: drake/multibody/plant/tamsi_solver.h
        const char* doc =
R"""(TamsiSolver uses the Transition-Aware Modified Semi-Implicit (TAMSI)
method, castro_etal_2019 "[Castro et al., 2019]", to solve the
equations below for mechanical systems in contact with regularized
friction:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    q̇ = N(q) v
    (1)  M(q) v̇ = τ + Jₙᵀ(q) fₙ(q, v) + Jₜᵀ(q) fₜ(q, v)

.. raw:: html

    </details>

where ``v ∈ ℝⁿᵛ`` is the vector of generalized velocities, ``M(q) ∈
ℝⁿᵛˣⁿᵛ`` is the mass matrix, ``Jₙ(q) ∈ ℝⁿᶜˣⁿᵛ`` is the Jacobian of
normal separation velocities, ``Jₜ(q) ∈ ℝ²ⁿᶜˣⁿᵛ`` is the Jacobian of
tangent velocities, ``fₙ ∈ ℝⁿᶜ`` is the vector of normal contact
forces, ``fₜ ∈ ℝ²ⁿᶜ`` is the vector of tangent friction forces and τ ∈
ℝⁿᵛ is a vector of generalized forces containing all other applied
forces (e.g., Coriolis, gyroscopic terms, actuator forces, etc.) but
contact forces. This solver assumes a compliant law for the normal
forces ``fₙ(q, v)`` and therefore the functional dependence of ``fₙ(q,
v)`` with q and v is stated explicitly.

Since TamsiSolver uses regularized friction, we explicitly emphasize
the functional dependence of ``fₜ(q, v)`` with the generalized
velocities. The functional dependence of ``fₜ(q, v)`` with the
generalized positions stems from its direct dependence with the normal
forces ``fₙ(q, v)``.

TamsiSolver implements two different schemes. A "one-way coupling
scheme" which solves for the friction forces given the normal forces
are known. That is, normal forces affect the computation of the
friction forces however, the normal forces are kept constant during
the solution procedure.

A "two-way coupling scheme" treats both the normal and friction forces
implicitly in the generalized velocities resulting in a numerical
strategy in which normal forces affect friction forces and,
conversely, friction forces couple back to the computation of the
normal forces.

The two-way coupled scheme provides a more stable and therefore robust
solution to the problem stated in Eq. (1) with just a small increase
of the computational cost compared to the one-way coupled scheme. The
one-way coupled scheme is however very useful for testing and analysis
of the solver.

One-Way Coupling Scheme
-----------------------

Equation (1) is discretized in time using a variation of the first
order semi-implicit Euler scheme from step s to step s+1 with time
step ``δt`` as:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    qˢ⁺¹ = qˢ + δt N(qˢ) vˢ⁺¹
    (2)  M(qˢ) vˢ⁺¹ =
    M(qˢ) vˢ + δt [τˢ + Jₙᵀ(qˢ) fₙ(qˢ,vˢ) + Jₜᵀ(qˢ) fₜ(qˢ,vˢ⁺¹)]

.. raw:: html

    </details>

(We are using s for step counter rather than n to avoid
Unicode-induced confusion with the "normal direction" subscript n.)

Please see details in the one_way_coupling_derivation "Derivation of
the one-way coupling scheme" section. The equation for the generalized
velocities in Eq. (2) is rewritten as:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    (3)  M vˢ⁺¹ = p* + δt [Jₙᵀ fₙ + Jₜᵀ fₜ(vˢ⁺¹)]

.. raw:: html

    </details>

where ``p* = M vˢ + δt τˢ`` is the generalized momentum that the
system would have in the absence of contact forces and, for
simplicity, we have only kept the functional dependencies in
generalized velocities. Notice that TamsiSolver uses a precomputed
value of the normal forces. These normal forces could be available for
instance if using a compliant contact approach, for which normal
forces are a function of the state.

Two-Way Coupling Scheme
-----------------------

Equation (1) is discretized in time using a variation on the
semi-implicit Euler scheme with time step ``δt`` as:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    qˢ⁺¹ = qˢ + δt N(qˢ) vˢ⁺¹
    (4)  M(qˢ) vˢ⁺¹ = M(qˢ) vˢ +
    δt [τˢ + Jₙᵀ(qˢ) fₙˢ⁺¹ + Jₜᵀ(qˢ) fₜ(fₙˢ⁺¹,vₜˢ⁺¹)]

.. raw:: html

    </details>

This implicit scheme variation evaluates Jacobian matrices Jₙ and Jₜ
as well as the kinematic mapping N(q) at the previous time step. In
Eq. (4) we have fₙˢ⁺¹ = fₙ(xˢ⁺¹, vₙˢ⁺¹) with xˢ⁺¹ = x(qˢ⁺¹), the
signed *penetration* distance (defined positive when bodies overlap)
between contact point pairs and the *separation* velocities vₙˢ⁺¹ =
Jₙ(qˢ) vˢ⁺¹ (= −ẋˢ⁺¹). Also the functional dependence of fₜ with fₙ
and vₜ is highlighted in Eq. (4). More precisely, the two-way coupling
scheme uses a normal force law for each contact pair of the form:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    (5)  fₙ(x, vₙ) = k(vₙ)₊ x₊
    (6)      k(vₙ) = k (1 − d vₙ)₊

.. raw:: html

    </details>

where ``x₊`` is the positive part of x (x₊ = x if x ≥ 0 and x₊ = 0
otherwise) and ``k`` and d are the stiffness and dissipation
coefficients for a given contact point, respectively.

The two-way coupling scheme uses a first order approximation to the
signed distance functions vector:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    (7)  xˢ⁺¹ ≈ xˢ − δt vₙˢ⁺¹ =  xˢ − δt Jₙ(qˢ) vˢ⁺¹

.. raw:: html

    </details>

where the minus sign is needed given that ẋ = dx/dt = −vₙ. This
approximation is used in Eq. (5) to obtain a numerical scheme that
implicitly couples normal forces through their functional dependence
on the signed penetration distance. Notice that, according to Eq. (5),
normal forces at each contact point are decoupled from each other.
However their values are coupled given the choice of a common
variable, the generalized velocity v.

Equation (7) is used into Eq. (5) to obtain an expression of the
normal force in terms of the generalized velocity vˢ⁺¹ at the next
time step:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    (8) fₙ(xˢ⁺¹, vₙˢ⁺¹) = k (1 − d vₙˢ⁺¹)₊ xˢ⁺¹₊
    = k (1 − d Jₙ(qˢ) vˢ⁺¹)₊ (xˢ − δt Jₙ(qˢ) vˢ⁺¹)₊
    = fₙ(vˢ⁺¹)

.. raw:: html

    </details>

Similarly, the friction forces fₜ can be written in terms of the next
time step generalized velocities using vₜˢ⁺¹ = Jₜ(qˢ) vˢ⁺¹ and using
Eq. (8) to substitute an expression for the normal force in terms of
vˢ⁺¹. This allows to re-write the tangential forces as a function of
the generalized velocities as:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    (9)  fₜ(fₙˢ⁺¹, vₜˢ⁺¹) = fₜ(fₙ(x(vˢ⁺¹), vₙ(vˢ⁺¹)), vₜ((vˢ⁺¹)))
    = fₜ(vˢ⁺¹)

.. raw:: html

    </details>

where we write x(vˢ⁺¹) = xˢ − δt Jₙ(qˢ) vˢ⁺¹. Finally, Eqs. (8) and
(9) are used into Eq. (4) to obtain an expression in vˢ⁺¹:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    (10)  M(qˢ) vˢ⁺¹ = p* + δt [Jₙᵀ(qˢ) fₙ(vˢ⁺¹) + Jₜᵀ(qˢ) fₜ(vˢ⁺¹)]

.. raw:: html

    </details>

with p* = ``p* = M vˢ + δt τˢ`` the generalized momentum that the
system would have in the absence of contact forces.

TamsiSolver uses a Newton-Raphson strategy to solve Eq. (10) in the
generalized velocities, limiting the iteration update with the scheme
described in iteration_limiter.

Limits in the Iteration Updates
-------------------------------

TamsiSolver solves for the generalized velocity at the next time step
``vˢ⁺¹`` with either a one-way or two-way coupled scheme as described
in the previous sections. The solver uses a Newton-Raphson iteration
to compute an update ``Δvᵏ`` at the k-th Newton-Raphson iteration.
Once ``Δvᵏ`` is computed, the solver limits the change in the
tangential velocities ``Δvₜᵏ = Jₜᵀ Δvᵏ`` using the approach described
in uchida_etal_2015 "[Uchida et al., 2015]". This approach limits the
maximum angle change θ between two successive iterations in the
tangential velocity. Details of our implementation are provided in
castro_etal_2019 "[Castro et al., 2019]".

Derivation of the one-way coupling scheme
-----------------------------------------

In this section we provide a detailed derivation of the first order
time stepping approach in Eq. (2). We start from the continuous Eq.
(1):


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    (1)  M(q) v̇ = τ + Jₙᵀ(q) fₙ(q,v) + Jₜᵀ(q) fₜ(q,v)

.. raw:: html

    </details>

we can discretize Eq. (1) in time using a first order semi-implicit
Euler scheme in velocities:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    (11)  M(qˢ) vˢ⁺¹ = M(qˢ) vˢ +
    δt [τˢ⁺¹ + Jₙᵀ(qˢ) fₙ(qˢ,vˢ⁺¹) + Jₜᵀ(qˢ) fₜ(vˢ⁺¹)] + O₁(δt²)

.. raw:: html

    </details>

where the equality holds strictly since we included the leading terms
in ``O(δt²)``. We use ``τˢ⁺¹ = τ(tˢ, qˢ, vˢ⁺¹)`` for brevity in Eq.
(11). When moving from the continuous Eq. (1) to the discrete version
Eq. (11), we lost the nice property that our compliant normal forces
are decoupled from the friction forces (both depend on the same
unknown vˢ⁺¹ in Eq (11)). The reason is that Eq. (11) includes an
integration over a small interval of size δt. To solve the discrete
system in Eq. (11), we'd like to decouple the normal forces from the
tangential forces again, which will require a new (though still valid)
approximation. To do so we will expand in Taylor series the term
``fₙ(qˢ, vˢ⁺¹)``:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    (12)  fₙ(qˢ, vˢ⁺¹) = fₙ(qˢ,vˢ) + ∇ᵥfₙ(qˢ,vˢ) (vˢ⁺¹ − vˢ) + O₂(‖vˢ⁺¹ − vˢ‖²)

.. raw:: html

    </details>

The difference between ``vˢ`` and ``vˢ⁺¹`` can be written as:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    (13)  vˢ⁺¹ − vˢ = δtv̇ˢ + δtO₃(δt²) = O₄(δt)

.. raw:: html

    </details>

Substituting ``vˢ⁺¹ − vˢ`` from Eq. (13) into Eq. (12) we arrive to:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    (14)  fₙ(qˢ, vˢ⁺¹) = fₙ(qˢ,vˢ) + ∇ᵥfₙ(qˢ,vˢ) O₄(δt) + O₅(δt²)
    = fₙ(qˢ,vˢ) + O₆(δt)

.. raw:: html

    </details>

where ``O₅(δt²) = O₂(‖vˢ⁺¹ − vˢ‖²) = O₂(‖O₄(δt)‖²)``. A similar
argument for τˢ⁺¹ shows it also differs in O(δt) from τˢ = τ(tˢ, qˢ,
vˢ). We can now use Eq. (14) into Eq. (11) to arrive to:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    (15)  M(qˢ) vˢ⁺¹ = M(qˢ) vˢ +
    δt [τˢ + Jₙᵀ(qˢ) (fₙ(qˢ,vˢ) + O₆(δt)) + Jₜᵀ(qˢ) fₜ(vˢ⁺¹)] +
    O₁(δt²)

.. raw:: html

    </details>

which we can rewrite as:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    (16)  M(qˢ) vˢ⁺¹ = M(qˢ) vˢ +
    δt [τˢ + Jₙᵀ(qˢ) fₙ(qˢ, vˢ) + Jₜᵀ(qˢ) fₜ(vˢ⁺¹)] + O₇(δt²)

.. raw:: html

    </details>

with ``O₇(δt²) = δt Jₙᵀ(qˢ) O₆(δt) + O₁(δt²)``. That is, Eq. (16)
introduces the same order of approximation as in the semi-implicit
method in Eq. (11). Up to this point we have made no approximations
but we instead propagated the ``O(⋅)`` leading terms. Therefore the
equalities in the equations above are exact. To obtain an approximate
time stepping scheme, we drop ``O₇(δt²)`` (we neglect it) in Eq. (16)
to arrive to a first order scheme:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    (17)  M(qˢ) vˢ⁺¹ = M(qˢ) vˢ +
    δt [τˢ + Jₙᵀ(qˢ) fₙ(qˢ,vˢ) + Jₜᵀ(qˢ) fₜ(vˢ⁺¹)]

.. raw:: html

    </details>

Therefore, with the scheme in Eq. (17) we are able to decouple the
computation of (compliant) normal forces from that of friction forces.
A very important feature of this scheme however, is the explicit
nature (in the velocities v) of the term associated with the normal
forces (usually including dissipation in the normal direction), which
will become unstable for a sufficiently large time step. However, for
most applications in practice, the stability of the scheme is mostly
determined by the explicit update of normal forces with positions,
that is, Eq. (17) is explicit in positions through the normal forces
``fₙ(qˢ, vˢ)``. For many common applications, the explicit dependence
of ``τˢ(tˢ, qˢ, vˢ)`` on the previous time step velocities ``vˢ``
determines the overall stability of the scheme, since this term can
include velocity dependent contributions such as control forces and
dampers. Notice that Eq. (12) introduces an expansion of ``fₙ`` with
an order of approximation consistent with the first order scheme as
needed. Therefore, it propagates into a ``O(δt²)`` term exactly as
needed in Eq. (16).

References
----------

- [Castro et al., 2019] Castro, A.M, Qu, A.,
Kuppuswamy, N., Alspach, A., Sherman, M.A., 2019. A Transition-Aware Method
for the Simulation of Compliant Contact with Regularized Friction.
arXiv:1909.05700 [cs.RO].
- Uchida, T.K., Sherman, M.A. and Delp, S.L., 2015.
Making a meaningful impact: modelling simultaneous frictional collisions
in spatial multibody systems. Proc. R. Soc. A, 471(2177), p.20140859.

Authors:
    Alejandro Castro (2018) Original author.

Authors:
    Michael Sherman, Evan Drumwright (2018) Original PR #8925
    reviewers.

Authors:
    Drake team (see https://drake.mit.edu/credits).)""";
        // Symbol: drake::multibody::TamsiSolver::Clone
        struct /* Clone */ {
          // Source: drake/multibody/plant/tamsi_solver.h
          const char* doc =
R"""(Returns:
    a deep copy of this, with the problem data invalidated, i.e., one
    of the Set*ProblemData() methods must be called on the clone
    before it can be used to solve.)""";
        } Clone;
        // Symbol: drake::multibody::TamsiSolver::ResizeIfNeeded
        struct /* ResizeIfNeeded */ {
          // Source: drake/multibody/plant/tamsi_solver.h
          const char* doc =
R"""(Change the working size of the solver to use ``nv`` generalized
velocities. This can be used to either shrink or grow the workspaces.

Raises:
    RuntimeError if nv is non-positive.)""";
        } ResizeIfNeeded;
        // Symbol: drake::multibody::TamsiSolver::SetOneWayCoupledProblemData
        struct /* SetOneWayCoupledProblemData */ {
          // Source: drake/multibody/plant/tamsi_solver.h
          const char* doc =
R"""(Sets data for the problem to be solved as outlined by Eq. (3) in this
class's documentation:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    (3)  M v = p* + δt Jₙᵀ fₙ +  δt Jₜᵀ fₜ(v)

.. raw:: html

    </details>

Refer to this class's documentation for further details on the
structure of the problem and the solution strategy. In the documented
parameters below, ``nv`` is the number of generalized velocities and
``nc`` is the number of contact points.

Parameter ``M``:
    The mass matrix of the system, of size ``nv x nv``.

Parameter ``Jn``:
    The normal separation velocities Jacobian, of size ``nc x nv``.

Parameter ``Jt``:
    The tangential velocities Jacobian, of size ``2nc x nv``.

Parameter ``p_star``:
    The generalized momentum the system would have at ``s + 1`` if
    contact forces were zero.

Parameter ``fn``:
    A vector of size ``nc`` containing the normal force at each
    contact point.

Parameter ``mu``:
    A vector of size ``nc`` containing the friction coefficient at
    each contact point. The solver makes no distinction between static
    and dynamic coefficients of friction or, similarly, the solver
    assumes the static and dynamic coefficients of friction are the
    same.

Warning:
    This method stores constant references to the matrices and vectors
    passed as arguments. Therefore 1. they must outlive this class
    and, 2. changes to the problem data invalidate any solution
    performed by this solver. In such a case,
    SetOneWayCoupledProblemData() and SolveWithGuess() must be invoked
    again.

Raises:
    RuntimeError if any of the data pointers are nullptr.

Raises:
    RuntimeError if the problem data sizes are not consistent as
    described above.

Raises:
    RuntimeError if SetTwoWayCoupledProblemData() was ever called on
    ``this`` solver.)""";
        } SetOneWayCoupledProblemData;
        // Symbol: drake::multibody::TamsiSolver::SetTwoWayCoupledProblemData
        struct /* SetTwoWayCoupledProblemData */ {
          // Source: drake/multibody/plant/tamsi_solver.h
          const char* doc =
R"""(Sets the problem data to solve the problem outlined in Eq. (10) in
this class's documentation using a two-way coupled approach:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    (10)  M(qˢ) vˢ⁺¹ = p* + δt [Jₙᵀ(qˢ) fₙ(vˢ⁺¹) + Jₜᵀ(qˢ) fₜ(vˢ⁺¹)]

.. raw:: html

    </details>

Refer to this class's documentation for further details on the
structure of the problem and the solution strategy. In the documented
parameters below, ``nv`` is the number of generalized velocities and
``nc`` is the number of contact points.

Parameter ``M``:
    The mass matrix of the system, of size ``nv x nv``.

Parameter ``Jn``:
    The normal separation velocities Jacobian, of size ``nc x nv``.

Parameter ``Jt``:
    The tangential velocities Jacobian, of size ``2nc x nv``.

Parameter ``p_star``:
    The generalized momentum the system would have at ``n + 1`` if
    contact forces were zero.

Parameter ``fn0``:
    Normal force at the previous time step. Always positive since
    bodies cannot attract each other.

Parameter ``stiffness``:
    A vector of size ``nc`` storing at each ith entry the stiffness
    coefficient for the ith contact pair.

Parameter ``dissipation``:
    A vector of size ``nc`` storing at each ith entry the dissipation
    coefficient for the ith contact pair.

Parameter ``mu``:
    A vector of size ``nc`` containing the friction coefficient at
    each contact point. The solver makes no distinction between static
    and dynamic coefficients of friction or, similarly, the solver
    assumes the static and dynamic coefficients of friction are the
    same.

Warning:
    This method stores constant references to the matrices and vectors
    passed as arguments. Therefore 1. they must outlive this class
    and, 2. changes to the problem data invalidate any solution
    performed by this solver. In such a case,
    SetOneWayCoupledProblemData() and SolveWithGuess() must be invoked
    again.

Raises:
    RuntimeError if any of the data pointers are nullptr.

Raises:
    RuntimeError if the problem data sizes are not consistent as
    described above.

Raises:
    RuntimeError if SetOneWayCoupledProblemData() was ever called on
    ``this`` solver.)""";
        } SetTwoWayCoupledProblemData;
        // Symbol: drake::multibody::TamsiSolver::SolveWithGuess
        struct /* SolveWithGuess */ {
          // Source: drake/multibody/plant/tamsi_solver.h
          const char* doc =
R"""(Given an initial guess ``v_guess``, this method uses a Newton-Raphson
iteration to find a solution for the generalized velocities satisfying
either Eq. (3) when one-way coupling is used or Eq. (10) when two-way
coupling is used. See this class's documentation for further details.
To retrieve the solution, please refer to retrieving_the_solution.

Returns:
    kSuccess if the iteration converges. All other values of
    TamsiSolverResult report different failure modes. Uses ``this``
    solver accessors to retrieve the last computed solution.

Warning:
    Always verify that the return value indicates success before
    retrieving the computed solution.

Parameter ``dt``:
    The time step used advance the solution in time.

Parameter ``v_guess``:
    The initial guess used in by the Newton-Raphson iteration.
    Typically, the previous time step velocities.

Raises:
    RuntimeError if ``v_guess`` is not of size ``nv``, the number of
    generalized velocities specified at construction.)""";
        } SolveWithGuess;
        // Symbol: drake::multibody::TamsiSolver::TamsiSolver<T>
        struct /* ctor */ {
          // Source: drake/multibody/plant/tamsi_solver.h
          const char* doc =
R"""(Instantiates a solver for a problem with ``nv`` generalized
velocities.

Raises:
    RuntimeError if nv is non-positive.)""";
        } ctor;
        // Symbol: drake::multibody::TamsiSolver::get_friction_forces
        struct /* get_friction_forces */ {
          // Source: drake/multibody/plant/tamsi_solver.h
          const char* doc =
R"""(Returns a constant reference to the most recent vector of friction
forces. These friction forces are defined in accordance to the
tangential velocities Jacobian Jₜ as documented in tamsi_class_intro
"this class's documentation".)""";
        } get_friction_forces;
        // Symbol: drake::multibody::TamsiSolver::get_generalized_contact_forces
        struct /* get_generalized_contact_forces */ {
          // Source: drake/multibody/plant/tamsi_solver.h
          const char* doc =
R"""(Returns a constant reference to the most recent vector of generalized
contact forces, including both friction and normal forces.)""";
        } get_generalized_contact_forces;
        // Symbol: drake::multibody::TamsiSolver::get_generalized_friction_forces
        struct /* get_generalized_friction_forces */ {
          // Source: drake/multibody/plant/tamsi_solver.h
          const char* doc =
R"""(Returns a constant reference to the most recent vector of generalized
friction forces.)""";
        } get_generalized_friction_forces;
        // Symbol: drake::multibody::TamsiSolver::get_generalized_velocities
        struct /* get_generalized_velocities */ {
          // Source: drake/multibody/plant/tamsi_solver.h
          const char* doc =
R"""(Returns a constant reference to the most recent vector of generalized
velocities.)""";
        } get_generalized_velocities;
        // Symbol: drake::multibody::TamsiSolver::get_iteration_statistics
        struct /* get_iteration_statistics */ {
          // Source: drake/multibody/plant/tamsi_solver.h
          const char* doc =
R"""(Returns statistics recorded during the last call to SolveWithGuess().
See IterationStats for details.)""";
        } get_iteration_statistics;
        // Symbol: drake::multibody::TamsiSolver::get_normal_forces
        struct /* get_normal_forces */ {
          // Source: drake/multibody/plant/tamsi_solver.h
          const char* doc =
R"""(Returns a constant reference to the most recent vector of (repulsive)
forces in the normal direction. That is, the normal force is positive
when the bodies push each other apart. Otherwise the normal force is
zero, since contact forces can only be repulsive.)""";
        } get_normal_forces;
        // Symbol: drake::multibody::TamsiSolver::get_normal_velocities
        struct /* get_normal_velocities */ {
          // Source: drake/multibody/plant/tamsi_solver.h
          const char* doc =
R"""(Returns a constant reference to the most recent solution vector for
normal separation velocities. This method returns an
``Eigen::VectorBlock`` referencing a vector of size ``nc``.)""";
        } get_normal_velocities;
        // Symbol: drake::multibody::TamsiSolver::get_solver_parameters
        struct /* get_solver_parameters */ {
          // Source: drake/multibody/plant/tamsi_solver.h
          const char* doc =
R"""(Returns the current set of parameters controlling the iteration
process. See Parameters for details.)""";
        } get_solver_parameters;
        // Symbol: drake::multibody::TamsiSolver::get_tangential_velocities
        struct /* get_tangential_velocities */ {
          // Source: drake/multibody/plant/tamsi_solver.h
          const char* doc =
R"""(Returns a constant reference to the most recent vector of tangential
forces. This method returns an ``Eigen::VectorBlock`` referencing a
vector of size ``nc``.)""";
        } get_tangential_velocities;
        // Symbol: drake::multibody::TamsiSolver::set_solver_parameters
        struct /* set_solver_parameters */ {
          // Source: drake/multibody/plant/tamsi_solver.h
          const char* doc =
R"""(Sets the parameters to be used by the solver. See Parameters for
details.)""";
        } set_solver_parameters;
      } TamsiSolver;
      // Symbol: drake::multibody::TamsiSolverIterationStats
      struct /* TamsiSolverIterationStats */ {
        // Source: drake/multibody/plant/tamsi_solver.h
        const char* doc =
R"""(Struct used to store information about the iteration process performed
by TamsiSolver.)""";
        // Symbol: drake::multibody::TamsiSolverIterationStats::Reset
        struct /* Reset */ {
          // Source: drake/multibody/plant/tamsi_solver.h
          const char* doc =
R"""((Internal) Used by TamsiSolver to reset statistics.)""";
        } Reset;
        // Symbol: drake::multibody::TamsiSolverIterationStats::Update
        struct /* Update */ {
          // Source: drake/multibody/plant/tamsi_solver.h
          const char* doc =
R"""((Internal) Used by TamsiSolver to update statistics.)""";
        } Update;
        // Symbol: drake::multibody::TamsiSolverIterationStats::num_iterations
        struct /* num_iterations */ {
          // Source: drake/multibody/plant/tamsi_solver.h
          const char* doc =
R"""(The number of iterations performed by the last TamsiSolver solve.)""";
        } num_iterations;
        // Symbol: drake::multibody::TamsiSolverIterationStats::residuals
        struct /* residuals */ {
          // Source: drake/multibody/plant/tamsi_solver.h
          const char* doc =
R"""((Advanced) Residual in the tangential velocities, in m/s. The k-th
entry in this vector corresponds to the residual for the k-th
Newton-Raphson iteration performed by the solver. After TamsiSolver
solved a problem, this vector will have size num_iterations. The last
entry in this vector, ``residuals[num_iterations-1]``, corresponds to
the residual upon completion of the solver, i.e. vt_residual.)""";
        } residuals;
        // Symbol: drake::multibody::TamsiSolverIterationStats::vt_residual
        struct /* vt_residual */ {
          // Source: drake/multibody/plant/tamsi_solver.h
          const char* doc =
R"""(Returns the residual in the tangential velocities, in m/s. Upon
convergence of the solver this value should be smaller than
Parameters::tolerance times Parameters::stiction_tolerance.)""";
        } vt_residual;
      } TamsiSolverIterationStats;
      // Symbol: drake::multibody::TamsiSolverParameters
      struct /* TamsiSolverParameters */ {
        // Source: drake/multibody/plant/tamsi_solver.h
        const char* doc =
R"""(These are the parameters controlling the iteration process of the
TamsiSolver solver.)""";
        // Symbol: drake::multibody::TamsiSolverParameters::max_iterations
        struct /* max_iterations */ {
          // Source: drake/multibody/plant/tamsi_solver.h
          const char* doc =
R"""(The maximum number of iterations allowed for the Newton-Raphson
iterative solver.)""";
        } max_iterations;
        // Symbol: drake::multibody::TamsiSolverParameters::relative_tolerance
        struct /* relative_tolerance */ {
          // Source: drake/multibody/plant/tamsi_solver.h
          const char* doc =
R"""(The tolerance to monitor the convergence of the tangential velocities.
This number specifies a tolerance relative to the value of the
stiction_tolerance and thus it is dimensionless. Using a tolerance
relative to the value of the stiction_tolerance is necessary in order
to capture transitions to stiction that would require an accuracy in
the value of the tangential velocities smaller than that of the
"stiction region" (the circle around the origin with radius
stiction_tolerance). A value close to one could cause the solver to
miss transitions from/to stiction. Small values approaching zero will
result in a higher number of iterations needed to attain the desired
level of convergence. Typical values lie within the 10⁻³ - 10⁻² range.)""";
        } relative_tolerance;
        // Symbol: drake::multibody::TamsiSolverParameters::stiction_tolerance
        struct /* stiction_tolerance */ {
          // Source: drake/multibody/plant/tamsi_solver.h
          const char* doc =
R"""(The stiction tolerance vₛ for the slip velocity in the regularized
friction function, in m/s. Roughly, for an externally applied
tangential forcing fₜ and normal force fₙ, under "stiction", the slip
velocity will be approximately vₜ ≈ vₛ fₜ/(μfₙ). In other words, the
maximum slip error of the regularized friction approximation occurs at
the edge of the friction cone when fₜ = μfₙ and vₜ = vₛ. The default
of 0.1 mm/s is a very tight value that for most problems of interest
in robotics will result in simulation results with negligible slip
velocities introduced by regularizing friction when in stiction.)""";
        } stiction_tolerance;
        // Symbol: drake::multibody::TamsiSolverParameters::theta_max
        struct /* theta_max */ {
          // Source: drake/multibody/plant/tamsi_solver.h
          const char* doc =
R"""((Advanced) TamsiSolver limits large angular changes between tangential
velocities at two successive iterations vₜᵏ⁺¹ and vₜᵏ. This change is
measured by the angle θ = acos(vₜᵏ⁺¹⋅vₜᵏ/(‖vₜᵏ⁺¹‖‖vₜᵏ‖)). To aid
convergence, TamsiSolver, limits this angular change to ``theta_max``.
Please refer to the documentation for TamsiSolver for further details.

Small values of ``theta_max`` will result in a larger number of
iterations of the solver for situations in which large angular changes
occur (sudden transients or impacts). Values of ``theta_max`` close to
π/2 allow for a faster convergence for problems with sudden
transitions to/from stiction. Large values of ``theta_max`` however
might lead to non-convergence of the solver. We choose a conservative
number by default that we found to work well in most practical
problems of interest.)""";
        } theta_max;
      } TamsiSolverParameters;
      // Symbol: drake::multibody::TamsiSolverResult
      struct /* TamsiSolverResult */ {
        // Source: drake/multibody/plant/tamsi_solver.h
        const char* doc =
R"""(The result from TamsiSolver::SolveWithGuess() used to report the
success or failure of the solver.)""";
        // Symbol: drake::multibody::TamsiSolverResult::kAlphaSolverFailed
        struct /* kAlphaSolverFailed */ {
          // Source: drake/multibody/plant/tamsi_solver.h
          const char* doc =
R"""(Could not solve for the α coefficient for per-iteration angle change.)""";
        } kAlphaSolverFailed;
        // Symbol: drake::multibody::TamsiSolverResult::kLinearSolverFailed
        struct /* kLinearSolverFailed */ {
          // Source: drake/multibody/plant/tamsi_solver.h
          const char* doc =
R"""(The linear solver used within the Newton-Raphson loop failed. This
might be caused by a divergent iteration that led to an invalid
Jacobian matrix.)""";
        } kLinearSolverFailed;
        // Symbol: drake::multibody::TamsiSolverResult::kMaxIterationsReached
        struct /* kMaxIterationsReached */ {
          // Source: drake/multibody/plant/tamsi_solver.h
          const char* doc =
R"""(The maximum number of iterations was reached.)""";
        } kMaxIterationsReached;
        // Symbol: drake::multibody::TamsiSolverResult::kSuccess
        struct /* kSuccess */ {
          // Source: drake/multibody/plant/tamsi_solver.h
          const char* doc = R"""(Successful computation.)""";
        } kSuccess;
      } TamsiSolverResult;
      // Symbol: drake::multibody::Wing
      struct /* Wing */ {
        // Source: drake/multibody/plant/wing.h
        const char* doc =
R"""(A System that connects to the MultibodyPlant in order to model the
simplified dynamics of an airfoil (or hydrofoil). Currently it only
supports a particular model of flat-plate aerodynamics with lift
coefficient = 2 sinα cosα, drag coefficient = 2 sin²α, moment
coefficient = 0, as documented for a number of NACA airfoils with
Reynolds number ≤ 10^5 in:

S. F. Hoerner and H. V. Borst, “Fluid-dynamic lift: practical
information on aerodynamic and hydrodynamic lift,” 1985 (Ch. 4,
Section 6).

This model was also empirically validated for a bird-scale UAV with
flat-plate wings (Reynolds number below 53000) in:

Rick Cory and Russ Tedrake, "Experiments in Fixed-Wing UAV Perching",
Proceedings of the AIAA Guidance, Navigation, and Control Conference ,
pp. 1-12, 2008.

and may generalize well as a model for other wings in the post-stall
regime, but it should only be viewed as a simple / coarse
approximation. We aim to generalize this class to general
lift/drag/moment coefficients soon.

.. pydrake_system::

    name: Wing
    input_ports:
    - body_poses
    - body_spatial_velocities
    - wind_velocity_at_aerodynamic_center (optional)
    - fluid_density (optional)
    output_ports:
    - spatial_force
    - aerodynamic_center

- The optional wind velocity input is a three-element BasicVector<T>
representing the translational velocity of the wind in world coordinates at
the aerodynamic center relative to the world origin.  See
get_aerodynamic_center_output_port() for more details.
- It is expected that the body_poses input should be connected to the
MultibodyPlant::get_body_poses_output_port() "MultibodyPlant body_poses
output port" and that body_spatial_velocities input should be connected to
the MultibodyPlant::get_body_spatial_velocities_output_port()
"MultibodyPlant body_spatial_velocities output port"
- The output is of type std::vector<ExternallyAppliedSpatialForce<T>>; it is
expected that this output will be connected to the MultibodyPlant::get_applied_spatial_force_input_port()
"externally_applied_spatial_force input port" of the MultibodyPlant.)""";
        // Symbol: drake::multibody::Wing::AddToBuilder
        struct /* AddToBuilder */ {
          // Source: drake/multibody/plant/wing.h
          const char* doc =
R"""(Helper method that constructs a Wing and connects the input and output
ports to the MultibodyPlant.

Parameter ``builder``:
    is a DiagramBuilder that the Wing will be added to.

Parameter ``plant``:
    is the MultibodyPlant containing the body referenced by
    ``body_index``, which the wing ports will be connected to.

See the Wing constructor for details on the remaining parameters.)""";
        } AddToBuilder;
        // Symbol: drake::multibody::Wing::Wing<T>
        struct /* ctor */ {
          // Source: drake/multibody/plant/wing.h
          const char* doc =
R"""(Constructs a system describing a single wing using flat-plate
aerodynamics as described in the class documentation.

Parameter ``body_index``:
    indicates the body on which the aerodynamic forces are applied.

Parameter ``surface_area``:
    is the total surface area of the wing in meters squared.

Parameter ``X_BodyWing``:
    is the pose of wing frame relative to the body frame, whose origin
    is at the aerodynamic center of the wing, the positive x-axis
    points along the chord towards the leading edge (e.g. towards the
    nose of the plane), the positive y-axis points along the span, and
    the z-axis points up. According to thin airfoil theory, the
    aerodynamic center of a symmetric wing (like this flat plate), is
    located at a quarter-chord position behind the leading edge.

Parameter ``fluid_density``:
    is the density of the fluid in kg/m^3. The default value is the
    density of dry air at 20 deg Celsius at sea-level. This value is
    only used if the optional fluid_density input port is not
    connected.)""";
          // Source: drake/multibody/plant/wing.h
          const char* doc_copyconvert =
R"""(Scalar-converting copy constructor. See system_scalar_conversion.)""";
        } ctor;
        // Symbol: drake::multibody::Wing::get_aerodynamic_center_output_port
        struct /* get_aerodynamic_center_output_port */ {
          // Source: drake/multibody/plant/wing.h
          const char* doc =
R"""(Returns a 3-element position of the aerodynamic center of the wing in
world coordinates. This output port does not depend on the optional
wind velocity input port, so it may be used to compute the wind
velocity at the aerodynamic center without causing any algebraic loops
in the Diagram. For instance, the following (sub-)Diagram could be
used to implement a wind field: ┌────────────┐ ┌──┤ Wind Field │◄─┐ │
└────────────┘ │ │ ┌──────────┐ │ └──►│ Wing ├───┘
wind_velocity_at_└──────────┘aerodynamic_center aerodynamic_center)""";
        } get_aerodynamic_center_output_port;
        // Symbol: drake::multibody::Wing::get_body_poses_input_port
        struct /* get_body_poses_input_port */ {
          // Source: drake/multibody/plant/wing.h
          const char* doc =
R"""(Returns a reference to the body_poses input port. It is anticipated
that this port will be connected the body_poses output port of a
MultibodyPlant.)""";
        } get_body_poses_input_port;
        // Symbol: drake::multibody::Wing::get_body_spatial_velocities_input_port
        struct /* get_body_spatial_velocities_input_port */ {
          // Source: drake/multibody/plant/wing.h
          const char* doc =
R"""(Returns a reference to the body_spatial_velocities input port. It is
anticipated that this port will be connected the
body_spatial_velocities output port of a MultibodyPlant.)""";
        } get_body_spatial_velocities_input_port;
        // Symbol: drake::multibody::Wing::get_fluid_density_input_port
        struct /* get_fluid_density_input_port */ {
          // Source: drake/multibody/plant/wing.h
          const char* doc =
R"""(Returns a reference to the optional fluid_density input port, which
accepts a scalar vector in units kg/m^3. This port is provided to
support vehicles which must take into account variations in
atmospheric density; such as a spacecraft during re-entry. If left
unconnected, the aerodynamic forces will be calculated using the
default fluid density passed in the constructor.)""";
        } get_fluid_density_input_port;
        // Symbol: drake::multibody::Wing::get_spatial_force_output_port
        struct /* get_spatial_force_output_port */ {
          // Source: drake/multibody/plant/wing.h
          const char* doc =
R"""(Returns a reference to the spatial_forces output port. It is
anticipated that this port will be connected to the
MultibodyPlant::get_applied_spatial_force_input_port()
"applied_spatial_force" input port of a MultibodyPlant.)""";
        } get_spatial_force_output_port;
        // Symbol: drake::multibody::Wing::get_wind_velocity_input_port
        struct /* get_wind_velocity_input_port */ {
          // Source: drake/multibody/plant/wing.h
          const char* doc =
R"""(Returns a reference to the input port for the optional three-element
BasicVector<T> representing the translational velocity of the wind in
world coordinates at the aerodynamic center relative to the world
origin. If this port is not connected, then the wind velocity is taken
to be zero.)""";
        } get_wind_velocity_input_port;
      } Wing;
      // Symbol: drake::multibody::contact_solvers
      struct /* contact_solvers */ {
        // Symbol: drake::multibody::contact_solvers::icf
        struct /* icf */ {
        } icf;
      } contact_solvers;
    } multibody;
  } drake;
} pydrake_doc_multibody_plant;

#if defined(__GNUG__)
#pragma GCC diagnostic pop
#endif
