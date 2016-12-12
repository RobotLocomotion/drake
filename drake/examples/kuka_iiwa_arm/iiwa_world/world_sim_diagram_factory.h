#pragma once

#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/constant_vector_source.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
// TODO(naveenoid): These methods must be merged with those in
// /examples/toyota_hsrb/hsrb_diagram_factories.h and moved to a common
// library.

/// A custom `systems::Diagram` composed of a `systems::RigidBodyPlant`
/// and a `systems::DrakeVisualizer`. The diagram's output port zero is
/// connected to the `systems::DrakeVisualizer`'s input port zero. The
/// resulting diagram has the same input and output ports as the plant.
template <typename T>
class VisualizedPlant : public systems::Diagram<T> {
 public:
  /// Builds the VisualizedPlant.
  /// @p rigid_body_tree the tree to be used within the `RigidBodyPlant`
  /// @p penetration_stiffness, @p penetration_damping, and
  /// @p friction_coefficient define the penetration and friction parameters
  /// of the plant.
  /// @p lcm is a pointer to an externally created lcm object.
  VisualizedPlant(std::unique_ptr<RigidBodyTree<T>> rigid_body_tree,
                  double penetration_stiffness, double penetration_damping,
                  double friction_coefficient, lcm::DrakeLcmInterface* lcm);

 private:
  systems::RigidBodyPlant<T>* rigid_body_plant_{nullptr};
  systems::DrakeVisualizer* drake_visualizer{nullptr};
};

/// A custom `Diagram` consisting of a `ConstantVectorSource` of 0 magnitude
/// connected to a `VisualizedPlant`'s input port zero. The resulting diagram
/// has no input ports and the same output ports as the `VisualizedPlant`.
template <typename T>
class PassiveVisualizedPlant : public systems::Diagram<T> {
 public:
  /// Builds the PassiveVisualizedPlant.
  /// \param visualized_plant a unique pointer to the `VisualizedPlant`.
  explicit PassiveVisualizedPlant(
      std::unique_ptr<VisualizedPlant<T>> visualized_plant);

 private:
  VisualizedPlant<T>* visualized_plant_{nullptr};
  systems::ConstantVectorSource<T>* constant_vector_source_{nullptr};
};

}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
