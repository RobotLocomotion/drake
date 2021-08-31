#pragma once

#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/lcmt_contact_results_for_viz.hpp"
#include "drake/multibody/plant/contact_results.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/lcm/lcm_publisher_system.h"

namespace drake {
namespace multibody {

/** A System that encodes ContactResults into a lcmt_contact_results_for_viz
 message. It has a single input port with type ContactResults<T> and a single
 output port with lcmt_contact_results_for_viz.

 Although this class can be instantiated on all default scalars, its
 functionality will be limited for `T` = symbolic::Expression. If there are any
 `symbolic::Variable`s in the expression, attempting to evaluate the output port
 will throw an exception. The support is sufficient that a systems::Diagram with
 a %ContactResultsToLcmSystem can be scalar converted to symbolic::Expression
 without error, but not necessarily evaluated.

 @tparam_default_scalar
 */
template <typename T>
class ContactResultsToLcmSystem final : public systems::LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ContactResultsToLcmSystem)

  /** Constructs a ContactResultsToLcmSystem.
   @param plant The MultibodyPlant that the ContactResults are generated from.
   @pre The `plant` must be finalized already. The input port of this system
        must be connected to the corresponding output port of `plant`
        (either directly or from an exported port in a Diagram).
  */
  explicit ContactResultsToLcmSystem(const MultibodyPlant<T>& plant);

  /** Scalar-converting copy constructor.  */
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

  // A mapping from geometry IDs to body indices.
  std::unordered_map<geometry::GeometryId, std::string>
      geometry_id_to_body_name_map_;

  // A mapping from body index values to body names.
  std::vector<std::string> body_names_;
};

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

 @param builder          The diagram builder being used to construct the
                         Diagram.
 @param multibody_plant  The System in `builder` containing the plant whose
                         contact results are to be visualized.
 @param lcm              An optional lcm interface through which lcm messages
                         will be dispatched. Will be allocated internally if
                         none is supplied.

 @pre The given `multibody_plant` must be contained within the supplied
      DiagramBuilder.

 @returns the LcmPublisherSystem (in case callers, e.g., need to change the
 default publishing rate).

 @ingroup visualization
 */
systems::lcm::LcmPublisherSystem* ConnectContactResultsToDrakeVisualizer(
    systems::DiagramBuilder<double>* builder,
    const MultibodyPlant<double>& multibody_plant,
    lcm::DrakeLcmInterface* lcm = nullptr);

/** Implements ConnectContactResultsToDrakeVisualizer, but using
 explicitly specified `contact_results_port` and `geometry_input_port`
 arguments.  This call is required, for instance, when the MultibodyPlant is
 inside a Diagram, and the Diagram exports the pose bundle port.

 @pre contact_results_port must be connected to the contact_results_port of
 @p multibody_plant.

 @see ConnectContactResultsToDrakeVisualizer().

 @ingroup visualization
 */
systems::lcm::LcmPublisherSystem* ConnectContactResultsToDrakeVisualizer(
    systems::DiagramBuilder<double>* builder,
    const MultibodyPlant<double>& multibody_plant,
    const systems::OutputPort<double>& contact_results_port,
    lcm::DrakeLcmInterface* lcm = nullptr);

}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class drake::multibody::ContactResultsToLcmSystem)
