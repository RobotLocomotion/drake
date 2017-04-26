#pragma once

#include <memory>
#include <string>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/system.h"
#include "drake/systems/framework/system_output.h"
#include "drake/systems/framework/system_port_descriptor.h"

namespace drake {
namespace systems {
namespace sensors {

/// An ideal sensor that measures collisions between any two bodies in the
/// world.
///
/// <B>%System Input Port:</B>
///
/// This system has one input port that is accessible via the following
/// accessor:
///
///  - get_input_port(): Contains `x`, the generalized joint position and
///    velocity state vector of the world.
///
/// <B>%System Output Port:</B>
///
/// This system has one output port that is accessible via the following
/// accessor:
///
///  - get_output_port(): Contains the sensed collision data in this sensor's
///    frame. It's an abstract port containing a `ContactResults<double>`.
///
/// @ingroup sensor_systems
///
class GlobalCollisionSensor : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(GlobalCollisionSensor);

  /// A constructor that initializes an GlobalCollisionSensor.
  ///
  /// @param[in] name The name of the GlobalCollisionSensor. This can be any
  /// value.
  ///
  /// @param[in] tree The RigidBodyTree that models the world being sensed.
  ///
  GlobalCollisionSensor(const std::string& name,
      std::unique_ptr<RigidBodyTree<double>> tree);

  /// Instantiates and attaches a GlobalCollisionSensor to a RigidBodyPlant. It
  /// connects the GlobalCollisionSensor's plant state input port.
  ///
  /// @param[in] name The name of the sensor. This can be any value.
  ///
  /// @param[in] tree The RigidBodyTree that describes the world.
  ///
  /// @param[in] plant_state_port A descriptor of the port containing the
  /// plant's generalized joint state vector.
  ///
  /// @param[out] builder A pointer to the DiagramBuilder to which the newly
  /// instantiated GlobalCollisionSensor is added. This must not be `nullptr`.
  ///
  /// @return A pointer to the newly instantiated and added
  /// GlobalCollisionSensor. The GlobalCollisionSensor is initially owned by the
  /// builder. Ownership will subsequently be transferred to the Diagram that is
  /// built by the builder.
  static GlobalCollisionSensor* AttachGlobalCollisionSensor(
      const std::string& name,
      std::unique_ptr<RigidBodyTree<double>> tree,
      const OutputPortDescriptor<double>& plant_state_port,
      DiagramBuilder<double>* builder);

  /// Returns the name of this sensor.
  const std::string& get_name() const { return name_; }

  const RigidBodyTree<double>& get_tree() {
    return plant_.get_rigid_body_tree();
  }

  /// Returns a descriptor of the input port that contains `x`, the generalized
  /// position and velocity vector of the world.
  const InputPortDescriptor<double>& get_input_port() const {
    return System<double>::get_input_port(input_port_index_);
  }

  /// Returns a descriptor of the output port that contains the sensed
  /// ContactResults<double> value.
  const OutputPortDescriptor<double>& get_output_port() const {
    return System<double>::get_output_port(output_port_index_);
  }

 protected:
  /// Allocates the data for the abstract-valued output port specified by
  /// @p descriptor.
  std::unique_ptr<AbstractValue> AllocateOutputAbstract(
      const OutputPortDescriptor<double>& descriptor) const override;

  /// Computes the "sensed" collisions.
  void DoCalcOutput(const systems::Context<double>& context,
                    systems::SystemOutput<double>* output) const override;

 private:
  const std::string name_;
  const RigidBodyPlant<double> plant_;

  int input_port_index_{};
  int output_port_index_{};
};

}  // namespace sensors
}  // namespace systems
}  // namespace drake
