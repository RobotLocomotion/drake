#pragma once

#include <memory>

#include "drake/automotive/gen/euler_floating_joint_state.h"
#include "drake/automotive/gen/maliput_railcar_config.h"
#include "drake/automotive/gen/maliput_railcar_state.h"
#include "drake/automotive/maliput/api/lane.h"
#include "drake/automotive/maliput/api/lane_data.h"
#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace automotive {

/// Converts MaliputRailcarState to a full 6-DOF EulerFloatingJointState.
///
/// Instantiated templates for the following kinds of T's are provided:
/// - double
///
/// They are already available to link against in the containing library.
template <typename T>
class MaliputRailcarToEulerFloatingJoint : public systems::LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MaliputRailcarToEulerFloatingJoint)

  MaliputRailcarToEulerFloatingJoint(const maliput::api::Lane& lane,
      const MaliputRailcarConfig<T>& config);

  int input_port_index() const { return input_port_index_; }
  int output_port_index() const { return output_port_index_; }

  const systems::InputPortDescriptor<T>& input_port() const {
    return this->get_input_port(input_port_index_);
  }

  void DoCalcOutput(const systems::Context<T>& context,
                    systems::SystemOutput<T>* output) const override;

 protected:
  std::unique_ptr<systems::BasicVector<T>> AllocateOutputVector(
      const systems::OutputPortDescriptor<T>& descriptor) const override {
    return std::make_unique<EulerFloatingJointState<T>>();
  }

 private:
  const maliput::api::Lane& lane_;
  MaliputRailcarConfig<T> config_;
  int input_port_index_;
  int output_port_index_;
};

}  // namespace automotive
}  // namespace drake
