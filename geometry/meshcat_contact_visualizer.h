#pragma once

#include <map>
#include <memory>
#include <string>
#include <utility>

#include "drake/geometry/geometry_roles.h"
#include "drake/geometry/meshcat.h"
#include "drake/geometry/meshcat_contact_visualizer_params.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace geometry {

/**
 @system
 name: MeshcatContactVisualizer
 input_ports:
 - contact_results
 @endsystem

@tparam_nonsymbolic_scalar
*/
template <typename T>
class MeshcatContactVisualizer final : public systems::LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MeshcatContactVisualizer)

  /** Creates an instance of %MeshcatContactVisualizer */
  explicit MeshcatContactVisualizer(std::shared_ptr<Meshcat> meshcat,
                                    MeshcatContactVisualizerParams params = {});

  /** Scalar-converting copy constructor. See @ref system_scalar_conversion.
   It should only be used to convert _from_ double _to_ other scalar types.
   */
  template <typename U>
  explicit MeshcatContactVisualizer(const MeshcatContactVisualizer<U>& other);

  /** Calls Meschat::Delete(std::string path), with the path set to `prefix`.
   Since this visualizer will only ever add geometry under this prefix, this
   will remove all geometry/transforms added by the visualizer, or by a
   previous instance of this visualizer using the same prefix.  Use the
   `delete_on_initialization_event` in the constructor to determine whether
   this should be called on initialization. */
  void Delete() const;

  /** Returns the ContactResults-valued input port. It should be connected to
   MultibodyPlant's ContactResults-valued output port. Failure to do so will
   cause a runtime error when attempting to broadcast messages. */
  const systems::InputPort<T>& contact_results_input_port() const {
    return this->get_input_port(contact_results_input_port_);
  }

  /** Adds a MeshcatContactVisualizer and connects it to the given
   MultibodyPlant's ContactResults-valued output port. See
   MeshcatContactVisualizer::MeshcatContactVisualizer(MeshcatContactVisualizer*,
   MeshcatContactVisualizerParams) for details. */
  static const MeshcatContactVisualizer<T>& AddToBuilder(
      systems::DiagramBuilder<T>* builder,
      const multibody::MultibodyPlant<T>& plant,
      std::shared_ptr<Meshcat> meshcat,
      MeshcatContactVisualizerParams params = {});

  /** Adds a MeshcatContactVisualizer and connects it to the given
   multibody::ContactResults-valued output port. See
   MeshcatContactVisualizer::MeshcatContactVisualizer(MeshcatContactVisualizer*,
   MeshcatContactVisualizer) for details. */
  static const MeshcatContactVisualizer<T>& AddToBuilder(
      systems::DiagramBuilder<T>* builder,
      const systems::OutputPort<T>& contact_results_port,
      std::shared_ptr<Meshcat> meshcat,
      MeshcatContactVisualizerParams params = {});

 private:
  /* MeshcatContactVisualizer of different scalar types can all access each
   other's data. */
  template <typename>
  friend class MeshcatContactVisualizer;

  /* The periodic event handler. It tests to see if the last scene description
   is valid (if not, sends the objects) and then sends the transforms.  */
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
  mutable std::shared_ptr<Meshcat> meshcat_{};

  /* Map of the published contact pairs to their boolean visible status. */
  mutable std::map<std::pair<GeometryId, GeometryId>, bool> contacts_{};

  /* The parameters for the visualizer.  */
  MeshcatContactVisualizerParams params_;
};

/** A convenient alias for the MeshcatContactVisualizer class when using the
`double` scalar type. */
using MeshcatContactVisualizerd = MeshcatContactVisualizer<double>;

}  // namespace geometry

// Define the conversion trait to *only* allow double -> AutoDiffXd conversion.
// Symbolic is not supported yet, and AutoDiff -> double doesn't "make sense".
namespace systems {
namespace scalar_conversion {
template <>
struct Traits<geometry::MeshcatContactVisualizer> : public NonSymbolicTraits {};
}  // namespace scalar_conversion
}  // namespace systems

}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::geometry::MeshcatContactVisualizer)
