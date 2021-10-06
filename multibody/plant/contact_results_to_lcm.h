#pragma once

#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/scene_graph.h"
#include "drake/lcmt_contact_results_for_viz.hpp"
#include "drake/multibody/plant/contact_results.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/lcm/lcm_publisher_system.h"

namespace drake {
namespace multibody {
namespace internal {

/* Stores the "full" name of a body *in contact*: its model instance name, body
 name, and geometry name. This assumes that `model` name is guaranteed to be
 unique within MBP. So, for two bodies which may be identically named (body
 name), they *must* differ by model name. For a body that has multiple collision
 geometries, we rely on the fact that every collision geometry for a single
 frame must be uniquely named.

 This includes further information to allow visualizers to make streamlining
 decisions. It reports if the body name is unique across the entire plant
 (body_name_is_unique) and the number of collision geometries associated
 with the body.

 If the body name is unique, the visualizer can display only the body
 name without fear of introducing ambiguity. Furthermore, with the number
 collision geometries per body, the visualizer can draw conclusions about
 the possible number of contact patches between bodies and streamline
 accordingly. */
struct FullBodyName {
  std::string model;
  std::string body;
  std::string geometry;
  bool body_name_is_unique;
  int geometry_count;
};

/* Facilitate unit testing. See ContactResultsToLcmSystem::Equals(). */
bool operator==(const FullBodyName& n1, const FullBodyName& n2);

}  // namespace internal

/** A System that encodes ContactResults into a lcmt_contact_results_for_viz
 message. It has a single input port with type ContactResults<T> and a single
 output port with lcmt_contact_results_for_viz.

 Although this class can be instantiated on all default scalars, its
 functionality will be limited for `T` = symbolic::Expression. If there are any
 symbolic::Variable instances in the expression, attempting to evaluate the
 output port will throw an exception. The support is sufficient that a
 systems::Diagram with a %ContactResultsToLcmSystem can be scalar converted to
 symbolic::Expression without error, but not necessarily evaluated.

 <h3>Constructing instances</h3>

 Generally, you shouldn't construct %ContactResultsToLcmSystem instances
 directly. We recommend using one of the overloaded
 @ref contact_result_vis_creation "ConnectContactResultsToDrakeVisualizer()"
 methods to add contact visualization to your diagram.

 <h3>How contacts are described in visualization</h3>

 In the visualizer, each contact between two bodies is uniquely characterized
 by two triples of names: (model instance name, body name, geometry name).
 These triples help distinguish contacts which might otherwise be ambiguous
 (e.g., contact with two bodies, both called "box" but part of different model
 instances).

 %ContactResultsToLcmSystem gets the model instance and body names from an
 instance of MultibodyPlant, but *geometry* names are not available from the
 plant. By default, %ContactResultsToLcmSystem will *generate* a unique name
 based on a geometry's unique id (e.g., "Id(7)"). For many applications
 (those cases where each body has only a single collision geometry), this is
 perfectly acceptable. However, in cases where a body has multiple collision
 geometries, those default names may not be helpful when viewing the visualized
 results. Instead, %ContactResultsToLcmSystem can use the names associated with
 the id in a geometry::SceneGraph instance. The only method for doing this is
 via the @ref contact_result_vis_creation
 "ConnectContactResultsToDrakeVisualizer()" methods and requires the diagram
 to be instantiated as double valued. If a diagram with a different scalar
 type is required, it should subsequently be scalar converted.

 @tparam_default_scalar
 @ingroup visualization */
template <typename T>
class ContactResultsToLcmSystem final : public systems::LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ContactResultsToLcmSystem)

  /** Constructs an instance with *default* geometry names (e.g., "Id(7)").

   @param plant   The MultibodyPlant that the ContactResults are generated from.
   @pre The `plant` parameter (or a fully equivalent plant) connects to `this`
        system's input port.
   @pre The `plant` parameter is finalized. */
  explicit ContactResultsToLcmSystem(const MultibodyPlant<T>& plant);

  /** Scalar-converting copy constructor. */
  template <typename U>
  explicit ContactResultsToLcmSystem(const ContactResultsToLcmSystem<U>& other)
      : ContactResultsToLcmSystem<T>(true) {
    geometry_id_to_body_name_map_ = other.geometry_id_to_body_name_map_;
    body_names_ = other.body_names_;
  }

  const systems::InputPort<T>& get_contact_result_input_port() const;
  const systems::OutputPort<T>& get_lcm_message_output_port() const;

 private:
  friend class ContactResultsToLcmTester;
  // The connection function gets friend access so it can call the "name lookup
  // functor" constructor.
  friend systems::lcm::LcmPublisherSystem* ConnectWithNameLookup(
      systems::DiagramBuilder<double>*, const MultibodyPlant<double>&,
      const systems::OutputPort<double>&,
      const std::function<std::string(geometry::GeometryId)>&,
      lcm::DrakeLcmInterface*, std::optional<double>);

  // Allow different specializations to access each other's private data for
  // scalar conversion.
  template <typename U> friend class ContactResultsToLcmSystem;

  // Special constructor that handles configuring ports. Used by both public
  // constructor and scalar-converting copy constructor.
  explicit ContactResultsToLcmSystem(bool);

  // Constructs the system using a "name lookup functor" (mapping geometry ids
  // to geometry names).
  ContactResultsToLcmSystem(
      const MultibodyPlant<T>& plant,
      const std::function<std::string(geometry::GeometryId)>&
          geometry_name_lookup);

  void CalcLcmContactOutput(const systems::Context<T>& context,
                            lcmt_contact_results_for_viz* output) const;

  // Reports if the other system is equivalent to this one. This can be used to
  // make sure the scalar copy converter has been updated. Every instance member
  // should be included in this function.
  template <typename U = T>
  bool Equals(const ContactResultsToLcmSystem<U>& other) const {
    return this->get_name() == other.get_name() &&
           contact_result_input_port_index_ ==
               other.contact_result_input_port_index_ &&
           message_output_port_index_ == other.message_output_port_index_ &&
           geometry_id_to_body_name_map_ ==
               other.geometry_id_to_body_name_map_ &&
           body_names_ == other.body_names_;
  }

  // Named indices for the i/o ports.
  systems::InputPortIndex contact_result_input_port_index_;
  systems::OutputPortIndex message_output_port_index_;

  // TODO(SeanCurtis-TRI): There is some incoherence in how body names are
  //  stored based on contact type (point vs hydro).
  //  geometry_id_to_body_name_map_ is exclusively used by hydro, and
  //  body_names_ is exclusively used for point contact. They should be
  //  reconciled.

  // A mapping from geometry IDs to per-body name data.
  std::unordered_map<geometry::GeometryId, internal::FullBodyName>
      geometry_id_to_body_name_map_;

  // A mapping from body index values to body names.
  std::vector<std::string> body_names_;
};

/** @name Visualizing contact results
 @anchor contact_result_vis_creation

 These methods extend a Diagram with the required components to publish contact
 results (as reported by MultibodyPlant) to drake_visualizer. We recommend
 using these methods instead of assembling the requisite components by hand.

 These must be called _during_ Diagram building. Each method makes modifications
 to the diagram being constructed by `builder` including the following changes:

 - adds systems multibody::ContactResultsToLcmSystem and LcmPublisherSystem to
   the Diagram and connects the draw message output to the publisher input,
 - connects a ContactResults<double>-valued output port to the
   ContactResultsToLcmSystem system, and
 - sets the publishing rate based on publish_period.

 The four variants differ in the following ways:

  - Two overloads take a SceneGraph and two don't. Those that do will ensure
    that the geometry names communicated in the lcm messages match the names
    used in SceneGraph. Those that don't are deprecated.
  - Two overloads take an OutputPort and two don't. This determines what is
    connected to the ContactResultsToLcmSystem input port. The overloads that
    specify an OutputPort will attempt to connect that port. Those that don't
    will connect the given plant's contact results output port.

 The parameters have the following semantics:

 @param builder                The diagram builder being used to construct the
                               Diagram. Systems will be added to this builder.
 @param plant                  The System in `builder` containing the plant
                               whose contact results are to be visualized.
 @param scene_graph            The SceneGraph that will determine how the
                               geometry names will appear in the lcm message.
 @param publish_period         An optional period to pass along to the
                               LcmPublisherSystem constructor; when null, a
                               reasonable default period will be used.
 @param lcm                    An optional lcm interface through which lcm
                               messages will be dispatched. Will be allocated
                               internally if none is supplied. If one is given,
                               it must remain alive at least as long as the
                               diagram built from `builder`.
 @param contact_results_port   The optional port that will be connected to the
                               ContactResultsToLcmSystem (as documented above).

 @returns (for all overloads) the LcmPublisherSystem (in case callers, e.g.,
          need to change the default publishing rate).

 @pre `plant` is contained within the supplied `builder`.
 @pre `scene_graph` (if given) is contained with the supplied `builder`.
 @pre `contact_results_port` (if given) belongs to a system that is an immediate
      child of `builder`. */
//@{

/** MultibodyPlant-connecting, default-named geometry overload.
 @ingroup visualization */
DRAKE_DEPRECATED("2022-01-01", "Provide a SceneGraph as the third argument.")
systems::lcm::LcmPublisherSystem* ConnectContactResultsToDrakeVisualizer(
    systems::DiagramBuilder<double>* builder,
    const MultibodyPlant<double>& plant,
    lcm::DrakeLcmInterface* lcm = nullptr,
    std::optional<double> publish_period = std::nullopt);

/** MultibodyPlant-connecting, SceneGraph-named geometry overload.
 @ingroup visualization */
systems::lcm::LcmPublisherSystem* ConnectContactResultsToDrakeVisualizer(
    systems::DiagramBuilder<double>* builder,
    const MultibodyPlant<double>& plant,
    const geometry::SceneGraph<double>& scene_graph,
    lcm::DrakeLcmInterface* lcm = nullptr,
    std::optional<double> publish_period = std::nullopt);

/** OutputPort-connecting, default-named geometry overload.
 @ingroup visualization */
DRAKE_DEPRECATED("2022-01-01", "Provide a SceneGraph as the third argument.")
systems::lcm::LcmPublisherSystem* ConnectContactResultsToDrakeVisualizer(
    systems::DiagramBuilder<double>* builder,
    const MultibodyPlant<double>& plant,
    const systems::OutputPort<double>& contact_results_port,
    lcm::DrakeLcmInterface* lcm = nullptr,
    std::optional<double> publish_period = std::nullopt);

/** OutputPort-connecting, SceneGraph-named geometry overload.
 @ingroup visualization */
systems::lcm::LcmPublisherSystem* ConnectContactResultsToDrakeVisualizer(
    systems::DiagramBuilder<double>* builder,
    const MultibodyPlant<double>& plant,
    const geometry::SceneGraph<double>& scene_graph,
    const systems::OutputPort<double>& contact_results_port,
    lcm::DrakeLcmInterface* lcm = nullptr,
    std::optional<double> publish_period = std::nullopt);

//@}

}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class drake::multibody::ContactResultsToLcmSystem)
