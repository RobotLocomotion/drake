#pragma once

#include <optional>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/drake_deprecated.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/scene_graph.h"
#include "drake/lcmt_contact_results_for_viz.hpp"
#include "drake/multibody/plant/contact_results.h"
#include "drake/multibody/plant/internal_geometry_names.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/lcm/lcm_publisher_system.h"

namespace drake {
namespace multibody {

/** A System that encodes ContactResults into an LCM message. It has input ports
 for ContactResults and geometry::QueryObject, and an output port for
 lcmt_contact_results_for_viz.

 @system
 name: ContactResultsToLcmSystem
 input_ports:
 - u0
 - query_object (optional)
 output_ports:
 - y0
 @endsystem

 @note
 Generally, you shouldn't construct %ContactResultsToLcmSystem instances
 directly. We recommend using one of the overloaded
 @ref contact_result_vis_creation "ConnectContactResultsToDrakeVisualizer()"
 functions to add contact visualization to your diagram.

 @tparam_default_scalar
 @ingroup visualization */
template <typename T>
class ContactResultsToLcmSystem final : public systems::LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ContactResultsToLcmSystem)

  /** (Advanced) Bare constructor that does not connect any input ports.
  For convenience, prefer to use ConnectContactResultsToDrakeVisualizer()
  instead, which does connect the ports. */
  ContactResultsToLcmSystem();

  /** Scalar-converting copy constructor. */
  template <typename U>
  explicit ContactResultsToLcmSystem(const ContactResultsToLcmSystem<U>&);

  DRAKE_DEPRECATED("2022-06-01",
      "There is no longer any need to pass a plant into the constructor. "
      "Use the default (no-argument) constructor instead.")
  explicit ContactResultsToLcmSystem(const MultibodyPlant<T>&);

  const systems::InputPort<T>& get_contact_result_input_port() const;
  const systems::InputPort<T>& get_query_object_input_port() const;
  const systems::OutputPort<T>& get_lcm_message_output_port() const;

 private:
  void CalcLcmContactOutput(const systems::Context<T>& context,
                            lcmt_contact_results_for_viz* output) const;

  systems::InputPortIndex contact_result_input_port_index_;
  systems::InputPortIndex query_object_input_port_index_;
  systems::OutputPortIndex message_output_port_index_;
  systems::CacheIndex geometry_names_scratch_index_;
};

/** @name Visualizing contact results
 @anchor contact_result_vis_creation

 These functions extend a Diagram with the required components to publish
 contact results (as reported by MultibodyPlant) to a visualizer (either meldis
 or drake_visualizer). We recommend using these functions instead of assembling
 the requisite components by hand.

 These must be called _during_ Diagram building. Each function modifies the
 diagram being constructed by `builder` by adding and connecting two new systems
 to the diagram: ContactResultsToLcmSystem and systems::lcm::LcmPublisherSystem.

 The two overloads differ in the following way:

  -- XXX rewrite this
  - One overload takes an OutputPort and one doesn't. This determines what is
    connected to the ContactResultsToLcmSystem input port. The overload that
    specifies an OutputPort will attempt to connect that port. The one that
    doesn't will connect the given plant's contact results output port.

 The parameters have the following semantics:

 @param builder                The diagram builder being used to construct the
                               Diagram. Systems will be added to this builder.
 @param plant                  The System in `builder` containing the plant
                               whose contact results are to be visualized.
 @param scene_graph            (Ignored) Kept for backwards compatibility only.
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
 @pre `contact_results_port` (if given) belongs to a system that is an immediate
      child of `builder`. */
//@{

/** Recommended overload.
 @ingroup visualization */
systems::lcm::LcmPublisherSystem* ConnectContactResultsToDrakeVisualizer(
    systems::DiagramBuilder<double>* builder,
    const MultibodyPlant<double>& plant,
    lcm::DrakeLcmInterface* lcm = nullptr,
    std::optional<double> publish_period = std::nullopt);

/** Advanced overload, connecting to bespoke output ports instead of directly
 to a MultibodyPlant.
 @ingroup visualization */
systems::lcm::LcmPublisherSystem* ConnectContactResultsToDrakeVisualizer(
      systems::DiagramBuilder<double>* builder,
      const systems::OutputPort<double>& contact_results_port,
      const systems::OutputPort<double>& query_object_port,
      lcm::DrakeLcmInterface* lcm = nullptr,
      std::optional<double> publish_period = std::nullopt);

//@}

/** This overload is not useful; it will be deprecated in the future.
<!--
TODO(jwnimmer-tri) After the "Recommended" overload has been in place for a
while (perhaps around 2022-05-01 or thereabouts), begin formal deprecation.
--> */
systems::lcm::LcmPublisherSystem* ConnectContactResultsToDrakeVisualizer(
    systems::DiagramBuilder<double>* builder,
    const MultibodyPlant<double>& plant,
    const geometry::SceneGraph<double>& scene_graph,
    lcm::DrakeLcmInterface* lcm = nullptr,
    std::optional<double> publish_period = std::nullopt);

DRAKE_DEPRECATED("2022-06-01",
    "Use the overload that accepts the scene_graph.get_query_output_port() "
    "instead of the whole scene_graph. Note that the contact_results_port "
    "should precede the query_object_port in the list of arguments.")
systems::lcm::LcmPublisherSystem* ConnectContactResultsToDrakeVisualizer(
    systems::DiagramBuilder<double>* builder,
    const MultibodyPlant<double>& plant,
    const geometry::SceneGraph<double>& scene_graph,
    const systems::OutputPort<double>& contact_results_port,
    lcm::DrakeLcmInterface* lcm = nullptr,
    std::optional<double> publish_period = std::nullopt);

namespace internal {

/* Converts `contact_results` into an `output` message.
 @tparam_default_scalar */
template <typename T>
void ContactResultsToLcm(const GeometryNames& geometry_names,
                         const ContactResults<T>& contact_results,
                         lcmt_contact_results_for_viz* output);

}  // namespace internal
}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class drake::multibody::ContactResultsToLcmSystem)
