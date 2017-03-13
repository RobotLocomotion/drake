#pragma once

#include <memory>

#include <Eigen/Dense>

#include "drake/common/eigen_types.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace systems {
namespace rendering {

/// PoseBundleToDrawMessage converts a PoseBundle on its single abstract-valued
/// input port to a Drake Visualizer Interface LCM draw message,
/// lcmt_viewer_draw, on its single abstract-valued output port.
///
/// The draw message will contain one link for each pose in the PoseBundle. The
/// name of the link will be the name of the corresponding pose. The robot_num
/// will always be 0. Because of this restriction, only one instance of a
/// model can be visualized, and no two models can have overlapping link names.
/// TODO(david-german-tri): Lift this restriction.
class PoseBundleToDrawMessage : public LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PoseBundleToDrawMessage)

  PoseBundleToDrawMessage();
  ~PoseBundleToDrawMessage() override;

 protected:
  /// Copies the input poses into the draw message.
  void DoCalcOutput(const Context<double>& context,
                    SystemOutput<double>* output) const override;

  /// Allocates a draw message.
  std::unique_ptr<AbstractValue> AllocateOutputAbstract(
      const OutputPortDescriptor<double>& descriptor) const override;
};

}  // namespace rendering
}  // namespace systems
}  // namespace drake
