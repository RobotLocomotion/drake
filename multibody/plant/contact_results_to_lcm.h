#pragma once

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/drake_deprecated.h"
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

 @tparam T The scalar type. Must be a valid Eigen scalar.

 Instantiated templates for the following kinds of T's are provided:

 - double
 - AutoDiffXd

 They are already available to link against in the containing library. No other
 values for T are currently supported.
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
      : systems::LeafSystem<T>(), body_names_(other.body_names_) {}

  const systems::InputPort<T>& get_contact_result_input_port() const;
  const systems::OutputPort<T>& get_lcm_message_output_port() const;

 private:
  // Allow different specializations to access each other's private data for
  // scalar conversion.
  template <typename U> friend class ContactResultsToLcmSystem;

  void CalcLcmContactOutput(const systems::Context<T>& context,
                            lcmt_contact_results_for_viz* output) const;

  // Named indices for the i/o ports.
  static constexpr int contact_result_input_port_index_{0};
  static constexpr int message_output_port_index_{0};

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

/** Implements ConnectContactResultsToDrakeVisualizer, but using @p
 contact_results_port to explicitly specify the output port used to get
 contact results for @p multibody_plant.  This is required, for instance,
 when the MultibodyPlant is inside a Diagram, and the Diagram exports the
 pose bundle port.

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

#ifndef DRAKE_DOXYGEN_CXX
// TODO(#9314) Remove this transitional namespace on or about 2019-03-01.
namespace multibody_plant {

template <typename T>
using ContactResultsToLcmSystem
    DRAKE_DEPRECATED("Spell as drake::multibody::Contact...System instead.")
    = ::drake::multibody::ContactResultsToLcmSystem<T>;

DRAKE_DEPRECATED("Spell as drake::multibody::Connect...Visualizer instead.")
inline systems::lcm::LcmPublisherSystem* ConnectContactResultsToDrakeVisualizer(
    systems::DiagramBuilder<double>* builder,
    const multibody::MultibodyPlant<double>& multibody_plant,
    const systems::OutputPort<double>& contact_results_port,
    lcm::DrakeLcmInterface* lcm = nullptr) {
  return multibody::ConnectContactResultsToDrakeVisualizer(
      builder, multibody_plant, contact_results_port, lcm);
}

}  // namespace multibody_plant
#endif  // DRAKE_DOXYGEN_CXX

}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class drake::multibody::ContactResultsToLcmSystem)
