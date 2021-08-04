#pragma once

#include <functional>
#include <memory>
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
 name, and geometry name. This assumes that `model_name` is guaranteed to be
 unique within MBP. So, for two bodies which may be identically named (body
 name), they *must* differ by model name. For a body that has multiple collision
 geometries, we rely on the fact that every collision geometry for a single
 frame must be uniquely named. */
struct FullBodyName {
  std::string model;
  std::string body;
  std::string geometry;
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

 @tparam_default_scalar */
template <typename T>
class ContactResultsToLcmSystem final : public systems::LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ContactResultsToLcmSystem)

  /** Constructs a system that serialized ContactResults into an lcm message
   for visualization.

   In the visualizer, each contact between two bodies is uniquely characterized
   by two triples of names: (model instance name, body name, geometry name).
   In the visualizer, these triples will help distinguish contacts which might
   otherwise be ambiguous (i.e., contact with two bodies, both called "box" but
   part of different model instances).

   The name of each collision geometry is determined by the
   `geometry_name_lookup` functor. If omitted, each geometry will have a named
   based on the stringified version of its GeometryId (i.e., "Id(7)") in the
   visualizer. This behavior can be modified by providing a lookup functor. The
   most common source would be by passing in SceneGraph::geometry_name_lookup()
   as the value for `geometry_name_lookup` (which will guarantee the geometry
   names in visualization match the names in SceneGraph).

   @param plant                 The MultibodyPlant that the ContactResults are
                                generated from.
   @param geometry_name_lookup  An *optional* functor for mapping GeometryId
                                values to names.
   @throws std::exception if a GeometryId found in `plant` is not recognized by
                          the functor `geometry_name_lookup` and it chooses to
                          throw.
   @pre The `plant` parameter connects to `this` system's input port (or, a
        fully equivalent plant).
   @pre The `plant` parameter is finalized. */
  ContactResultsToLcmSystem(
      const MultibodyPlant<T>& plant,
      const std::function<std::string(geometry::GeometryId)>&
          geometry_name_lookup = nullptr);

  /** Scalar-converting copy constructor.  */
  template <typename U>
  explicit ContactResultsToLcmSystem(const ContactResultsToLcmSystem<U>& other)
      : ContactResultsToLcmSystem<T>(true) {
    geometry_id_to_body_name_map_ = other.geometry_id_to_body_name_map_;
    body_names_ = other.body_names_;
    /* While we set a default name, in case the user has set it, we want to
     persist it. */
    this->set_name(other.get_name());
  }

  const systems::InputPort<T>& get_contact_result_input_port() const;
  const systems::OutputPort<T>& get_lcm_message_output_port() const;

 private:
  friend class ContactResultsToLcmTester;

  // Allow different specializations to access each other's private data for
  // scalar conversion.
  template <typename U> friend class ContactResultsToLcmSystem;

  // Special constructor that handles configuring ports. Used by both public
  // constructor and scalar-converting copy constructor.
  explicit ContactResultsToLcmSystem(bool);

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

  // A mapping from geometry IDs to body indices.
  std::unordered_map<geometry::GeometryId, internal::FullBodyName>
      geometry_id_to_body_name_map_;

  // A mapping from body index values to body names.
  std::vector<std::string> body_names_;
};

// TODO(SeanCurtis-TRI): This is, strictly speaking, a styleguide infraction.
//  https://drake.mit.edu/styleguide/cppguide.html#Inputs_and_Outputs
//  "put all input-only parameters before any output parameters". However,
//  because of the nature of default parameters, by appending the optional
//  functor to the end, all current call sites will continue to work without
//  any deprecation actions. Consider going through deprecation to change the
//  order to match the style.
/** Extends a Diagram with the required components to publish contact results
 to drake_visualizer. This must be called _during_ Diagram building and
 uses the given `builder` to add relevant subsystems and connections.

 This is a convenience method to simplify some common boilerplate for adding
 contact results visualization capability to a Diagram. What it does is:

 - adds systems ContactResultsToLcmSystem and LcmPublisherSystem to
   the Diagram and connects the draw message output to the publisher input,
 - connects the `multibody_plant` contact results output to the
   ContactResultsToLcmSystem system, and
 - sets the publishing rate to 1/60 of a second (simulated time).

 @param builder                The diagram builder being used to construct the
                               Diagram.
 @param multibody_plant        The System in `builder` containing the plant
                               whose contact results are to be visualized.
 @param lcm                    An optional lcm interface through which lcm
                               messages will be dispatched. Will be allocated
                               internally if none is supplied.
 @param geometry_name_lookup   The *optional* functor for mapping GeometryId
                               values to names. See the
                               ContactResultsToLcmSystem constructor for more
                               details.

 @pre The given `multibody_plant` must be contained within the supplied
      DiagramBuilder.

 @returns the LcmPublisherSystem (in case callers, e.g., need to change the
 default publishing rate).

 @ingroup visualization */
systems::lcm::LcmPublisherSystem* ConnectContactResultsToDrakeVisualizer(
    systems::DiagramBuilder<double>* builder,
    const MultibodyPlant<double>& multibody_plant,
    lcm::DrakeLcmInterface* lcm = nullptr,
    const std::function<std::string(geometry::GeometryId)>&
        geometry_name_lookup = nullptr);

/** Implements ConnectContactResultsToDrakeVisualizer, but using
 explicitly specified `contact_results_port` and `geometry_input_port`
 arguments.  This call is required, for instance, when the MultibodyPlant is
 inside a Diagram, and the Diagram exports the pose bundle port.

 @pre contact_results_port must be connected to the contact_results_port of
 @p multibody_plant.

 @see ConnectContactResultsToDrakeVisualizer().

 @ingroup visualization */
systems::lcm::LcmPublisherSystem* ConnectContactResultsToDrakeVisualizer(
    systems::DiagramBuilder<double>* builder,
    const MultibodyPlant<double>& multibody_plant,
    const systems::OutputPort<double>& contact_results_port,
    lcm::DrakeLcmInterface* lcm = nullptr,
    const std::function<std::string(geometry::GeometryId)>&
        geometry_name_lookup = nullptr);

}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class drake::multibody::ContactResultsToLcmSystem)
