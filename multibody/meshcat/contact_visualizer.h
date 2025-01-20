#pragma once

#include <memory>
#include <vector>

#include "drake/geometry/meshcat.h"
#include "drake/multibody/meshcat/contact_visualizer_params.h"
#include "drake/multibody/plant/internal_geometry_names.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace multibody {
namespace meshcat {

namespace internal {
// Defined in point_contact_visualizer.h.
class PointContactVisualizer;
struct PointContactVisualizerItem;
// Defined in hydroelastic_contact_visualizer.h
class HydroelasticContactVisualizer;
struct HydroelasticContactVisualizerItem;
}  // namespace internal

/** ContactVisualizer is a system that publishes a ContactResults to
geometry::Meshcat; For point contact results, it draws double-sided arrows at
the location of the contact force with length scaled by the magnitude of the
contact force. For hydroelastic contact, it draws single-sided arrows at
the centroid of the contact patch, one for force and one for the moment of the
contact results. The length of these vectors are scaled by the magnitude of the
contact force/moment. The direction of the arrow is essentially arbitrary (based
on the GeometryIds) but is stable during a simulation. The most common use of
this system is to connect its input port to the contact results output port of a
MultibodyPlant.

 @system
 name: ContactVisualizer
 input_ports:
 - contact_results
 - query_object (optional)
 @endsystem

@warning In the current implementation, ContactVisualizer methods must be called
from the same thread where the class instance was constructed. For example,
running multiple simulations in parallel using the same ContactVisualizer
instance is not yet supported. We may generalize this in the future.

@tparam_nonsymbolic_scalar
@ingroup visualization */
template <typename T>
class ContactVisualizer final : public systems::LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ContactVisualizer);

  /** Creates an instance of %ContactVisualizer */
  explicit ContactVisualizer(std::shared_ptr<geometry::Meshcat> meshcat,
                             ContactVisualizerParams params = {});

  /** Scalar-converting copy constructor. See @ref system_scalar_conversion.
  It should only be used to convert _from_ double _to_ other scalar types. */
  template <typename U>
  explicit ContactVisualizer(const ContactVisualizer<U>& other);

  ~ContactVisualizer() final;

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

  /** Returns the geometry::QueryObject-valued input port. It should be
  (optionally) connected to SceneGraph's get_query_output_port(). Failure
  to do so will prevent the display from showing names for the geometry. */
  const systems::InputPort<T>& query_object_input_port() const {
    return this->get_input_port(query_object_input_port_);
  }

  /** Adds a ContactVisualizer and connects it to the given
  MultibodyPlant's multibody::ContactResults-valued output port and
  geometry::QueryObject-valued output port.
  The %ContactVisualizer's name (see systems::SystemBase::set_name) will be set
  to a sensible default value, unless the default name was already in use by
  another system. */
  static const ContactVisualizer<T>& AddToBuilder(
      systems::DiagramBuilder<T>* builder, const MultibodyPlant<T>& plant,
      std::shared_ptr<geometry::Meshcat> meshcat,
      ContactVisualizerParams params = {});

  /** Adds a ContactVisualizer and connects it to the given
  multibody::ContactResults-valued output port and the given
  geometry::QueryObject-valued output port.
  The %ContactVisualizer's name (see systems::SystemBase::set_name) will be set
  to a sensible default value, unless the default name was already in use by
  another system. */
  static const ContactVisualizer<T>& AddToBuilder(
      systems::DiagramBuilder<T>* builder,
      const systems::OutputPort<T>& contact_results_port,
      const systems::OutputPort<T>& query_object_port,
      std::shared_ptr<geometry::Meshcat> meshcat,
      ContactVisualizerParams params = {});

  /** Adds a ContactVisualizer and connects it to the given
  multibody::ContactResults-valued output port.
  The %ContactVisualizer's name (see systems::SystemBase::set_name) will be set
  to a sensible default value, unless the default name was already in use by
  another system.
  @warning This overload is dispreferred because it cannot show any geometry
  names in the visualizer. */
  static const ContactVisualizer<T>& AddToBuilder(
      systems::DiagramBuilder<T>* builder,
      const systems::OutputPort<T>& contact_results_port,
      std::shared_ptr<geometry::Meshcat> meshcat,
      ContactVisualizerParams params = {});

 private:
  /* ContactVisualizer of different scalar types can all access each
  other's data. */
  template <typename>
  friend class ContactVisualizer;

  /* The periodic event handler. It tests to see if the last scene description
  is valid (if not, sends the objects) and then sends the transforms. */
  systems::EventStatus UpdateMeshcat(const systems::Context<T>&) const;

  /* Obtains the geometry_names scratch entry. On the first call, queries
  @p plant and the attached QueryObject and fills the returned GeometryNames. */
  const multibody::internal::GeometryNames& GetGeometryNames(
      const systems::Context<T>&, const MultibodyPlant<T>*) const;

  /* Calc function for the cache entry of visualized point contacts. */
  void CalcPointContacts(
      const systems::Context<T>&,
      std::vector<internal::PointContactVisualizerItem>*) const;

  /* Calc function for the cache entry of visualized hydroelastic contacts. */
  void CalcHydroelasticContacts(
      const systems::Context<T>&,
      std::vector<internal::HydroelasticContactVisualizerItem>*) const;

  /* Handles the initialization event. */
  systems::EventStatus OnInitialization(const systems::Context<T>&) const;

  typename systems::LeafSystem<T>::GraphvizFragment DoGetGraphvizFragment(
      const typename systems::LeafSystem<T>::GraphvizFragmentParams& params)
      const final;

  /* Meshcat is mutable because we must send messages (a non-const operation)
  from a const System (e.g., during simulation). We use shared_ptr instead of
  unique_ptr to facilitate sharing ownership through scalar conversion;
  creating a new Meshcat object during the conversion is not a viable option.
  */
  const std::shared_ptr<geometry::Meshcat> meshcat_;

  /* The parameters for the visualizer. */
  const ContactVisualizerParams params_;

  /* This helper class is mutable because we must update it (a non-const
  operation) from a const System (e.g., during simulation). */
  std::unique_ptr<internal::PointContactVisualizer> point_visualizer_;

  /* This helper class is mutable because we must update it (a non-const
  operation) from a const System (e.g., during simulation). */
  std::unique_ptr<internal::HydroelasticContactVisualizer>
      hydroelastic_visualizer_;

  systems::InputPortIndex contact_results_input_port_;
  systems::InputPortIndex query_object_input_port_;
  systems::CacheIndex geometry_names_scratch_;
  systems::CacheIndex point_contacts_cache_;
  systems::CacheIndex hydroelastic_contacts_cache_;
};

/** A convenient alias for the ContactVisualizer class when using
the `double` scalar type. */
using ContactVisualizerd = ContactVisualizer<double>;

}  // namespace meshcat
}  // namespace multibody

namespace systems {
namespace scalar_conversion {
template <>
struct Traits<drake::multibody::meshcat::ContactVisualizer>
    : public NonSymbolicTraits {};
}  // namespace scalar_conversion
}  // namespace systems

}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::meshcat::ContactVisualizer);
