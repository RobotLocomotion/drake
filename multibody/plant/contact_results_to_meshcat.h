#pragma once

#include <map>
#include <memory>
#include <string>
#include <utility>

#include "drake/geometry/meshcat.h"
#include "drake/multibody/plant/contact_results_to_meshcat_params.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace multibody {

/** ContactResultsToMeshcat is a system that publishes a ContactResults to
geometry::Meshcat; it draws double-sided arrows at the location of the contact
force with length scaled by the magnitude of the contact force.  The most common
use of this system is to connect its input port to the contact results output
port of a MultibodyPlant.

 @system
 name: ContactResultsToMeshcat
 input_ports:
 - contact_results
 @endsystem

Note: This system current only visualizes "point pair" contacts.

@tparam_nonsymbolic_scalar
@ingroup visualization */
template <typename T>
class ContactResultsToMeshcat final : public systems::LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ContactResultsToMeshcat)

  /** Creates an instance of %ContactResultsToMeshcat */
  explicit ContactResultsToMeshcat(std::shared_ptr<geometry::Meshcat> meshcat,
                                   ContactResultsToMeshcatParams params = {});

  /** Scalar-converting copy constructor. See @ref system_scalar_conversion.
  It should only be used to convert _from_ double _to_ other scalar types. */
  template <typename U>
  explicit ContactResultsToMeshcat(const ContactResultsToMeshcat<U>& other);

  /** Calls geometry::Meshcat::Delete(std::string_view path), with the path set
  to `prefix`. Since this visualizer will only ever add geometry under this
  prefix, this will remove all geometry/transforms added by the visualizer, or
  by a previous instance of this visualizer using the same prefix.  Use the
  `delete_on_initialization_event` in the parameters to determine whether this
  should be called on initialization. */
  void Delete() const;

  /** Returns the multibody::ContactResults-valued input port. It should be
  connected to MultibodyPlant's multibody::ContactResults-valued output port.
  Failure to do so will cause a runtime error when attempting to broadcast
  messages. */
  const systems::InputPort<T>& contact_results_input_port() const {
    return this->get_input_port(contact_results_input_port_);
  }

  /** Adds a ContactResultsToMeshcat and connects it to the given
  MultibodyPlant's ContactResults-valued output port. */
  static const ContactResultsToMeshcat<T>& AddToBuilder(
      systems::DiagramBuilder<T>* builder, const MultibodyPlant<T>& plant,
      std::shared_ptr<geometry::Meshcat> meshcat,
      ContactResultsToMeshcatParams params = {});

  /** Adds a ContactResultsToMeshcat and connects it to the given
  multibody::ContactResults-valued output port. */
  static const ContactResultsToMeshcat<T>& AddToBuilder(
      systems::DiagramBuilder<T>* builder,
      const systems::OutputPort<T>& contact_results_port,
      std::shared_ptr<geometry::Meshcat> meshcat,
      ContactResultsToMeshcatParams params = {});

 private:
  /* ContactResultsToMeshcat of different scalar types can all access each
  other's data. */
  template <typename>
  friend class ContactResultsToMeshcat;

  /* The periodic event handler. It tests to see if the last scene description
  is valid (if not, sends the objects) and then sends the transforms. */
  systems::EventStatus UpdateMeshcat(const systems::Context<T>& context) const;

  /* Handles the initialization event. */
  systems::EventStatus OnInitialization(const systems::Context<T>&) const;

  /* The index of this System's QueryObject-valued input port. */
  int contact_results_input_port_{};

  /* Meshcat is mutable because we must send messages (a non-const operation)
  from a const System (e.g. during simulation).  We use shared_ptr instead of
  unique_ptr to facilitate sharing ownership through scalar conversion;
  creating a new Meshcat object during the conversion is not a viable option.
   */
  mutable std::shared_ptr<geometry::Meshcat> meshcat_{};

  /* Map of the published contact pairs to their boolean visible status. */
  mutable std::map<SortedPair<geometry::GeometryId>, bool> contacts_{};

  /* The parameters for the visualizer.  */
  ContactResultsToMeshcatParams params_;
};

/** A convenient alias for the ContactResultsToMeshcat class when using
the `double` scalar type. */
using ContactResultsToMeshcatd = ContactResultsToMeshcat<double>;

}  // namespace multibody

}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::ContactResultsToMeshcat)
